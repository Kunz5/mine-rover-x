/**
 * MineRoverX — Firmware v3
 * ESP32 + L298N + HC-SR04 + metal-detector module + marker servo
 *
 * v3 is a rewrite of v2, not a patch. v2 worked on the bench but had three
 * defects that made it unsafe to drive over live ground. See
 * docs/firmware-postmortem.md for the full diagnosis. In short:
 *
 *   1. pulseIn() returns 0 on timeout, and v2 tested `distance <= STOP_CM`,
 *      so *no echo at all* was read as *obstacle at 0 cm*. Clear ground
 *      triggered an emergency stop.
 *   2. The obstacle branch ran stop(); delay(100); return; which skipped the
 *      serial read, so once triggered the rover could not receive any command,
 *      including reverse. It had to be physically moved.
 *   3. Motion commands latched the motors on with no timeout. A dropped serial
 *      link meant the rover drove until it hit something or the battery died.
 *
 * v3 is a non-blocking state machine. There is no delay() in loop(). Serial is
 * always serviced. Motion requires a live command or it stops on its own.
 */

#include <ESP32Servo.h>

// ─────────────────────────────────────────────────────────────────────────────
// Pin map — these match the physical build
// ─────────────────────────────────────────────────────────────────────────────
static const uint8_t PIN_ENA  = 27;   // L298N ENA — right side PWM
static const uint8_t PIN_IN1  = 26;
static const uint8_t PIN_IN2  = 25;
static const uint8_t PIN_ENB  = 5;    // L298N ENB — left side PWM
static const uint8_t PIN_IN3  = 18;
static const uint8_t PIN_IN4  = 19;

static const uint8_t PIN_TRIG = 17;   // HC-SR04
static const uint8_t PIN_ECHO = 16;

static const uint8_t PIN_IR     = 4;  // IR obstacle module (active LOW)
static const uint8_t PIN_DETECT = 34; // metal detector BUZZER line, tapped (input-only pin)
static const uint8_t PIN_SERVO  = 13; // marker-drop / head-lift servo

// ─────────────────────────────────────────────────────────────────────────────
// Tunables
// ─────────────────────────────────────────────────────────────────────────────
static const uint16_t PWM_FREQ_HZ   = 20000;  // 20 kHz — above audible range,
                                              // stops the gearmotors whining
static const uint8_t  PWM_BITS      = 8;      // 0..255, keeps v2's speed scale

static const float    MAX_RANGE_CM  = 400.0f; // HC-SR04 spec ceiling
static const uint32_t ECHO_TIMEOUT_US = 25000; // ~4.3 m round trip
static const float    STOP_CM       = 15.0f;  // obstacle hold threshold
static const float    CLEAR_CM      = 22.0f;  // must exceed this to resume
                                              // (hysteresis: stops the rover
                                              //  oscillating at the boundary)

static const uint32_t CMD_TIMEOUT_MS  = 1200; // deadman — no command, no motion
static const uint32_t RANGE_PERIOD_MS = 50;   // 20 Hz ranging
static const uint32_t CTRL_PERIOD_MS  = 20;   // 50 Hz control tick
static const uint32_t TELEM_PERIOD_MS = 250;  // 4 Hz telemetry

static const uint8_t  RAMP_STEP     = 12;     // PWM counts per control tick.
                                              // Soft-start: full torque from
                                              // standstill strips N20 gears.
static const uint8_t  DEFAULT_SPEED = 180;

// ── Metal detector: envelope detection on a tapped buzzer line ─────────────
// The detector module has no data output of any kind. Its only indication that
// it has found something is an audible alarm, so its buzzer terminal is
// soldered directly to a GPIO. That line carries an audio-frequency waveform
// while the alarm sounds, and sits idle otherwise.
//
// This means a level test is the wrong instrument. digitalRead() on an active
// buzzer line returns a value that flips hundreds of times a second, so
// `digitalRead(pin) == HIGH` would be wrong roughly half the times it sampled.
// The question is not "is the line high" but "is the line oscillating".
//
// So: count edges with an interrupt, integrate them over a fixed window, and
// treat a sufficiently high edge rate as "alarm sounding". A detection is only
// committed once the alarm has sounded continuously for DETECT_CONFIRM_MS,
// which rejects the short chirps the module emits on power-up and when it is
// swung quickly over uneven ground.
static const uint32_t DETECT_WINDOW_MS  = 100;   // envelope integration window
static const uint16_t DETECT_MIN_EDGES  = 20;    // >= 20 edges/100ms => >= 100 Hz
static const uint32_t DETECT_CONFIRM_MS = 2000;  // sustained alarm before commit
static const uint32_t DETECT_RELEASE_MS = 400;   // silence before the hold resets

static const int SERVO_STOWED = 20;           // marker arm retracted
static const int SERVO_DROP   = 110;          // marker released
static const uint32_t DROP_HOLD_MS = 400;

// ─────────────────────────────────────────────────────────────────────────────
// PWM compatibility. arduino-esp32 3.x replaced the ledcSetup/ledcAttachPin
// pair with a single ledcAttach(pin, freq, bits). v2 used analogWrite(), which
// only exists as a shim on 3.x — on 2.x it silently does nothing on these pins.
// ─────────────────────────────────────────────────────────────────────────────
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  static inline void pwmInit(uint8_t pin, uint8_t) { ledcAttach(pin, PWM_FREQ_HZ, PWM_BITS); }
  static inline void pwmWrite(uint8_t pin, uint8_t, uint8_t duty) { ledcWrite(pin, duty); }
#else
  static inline void pwmInit(uint8_t pin, uint8_t ch) {
    ledcSetup(ch, PWM_FREQ_HZ, PWM_BITS);
    ledcAttachPin(pin, ch);
  }
  static inline void pwmWrite(uint8_t, uint8_t ch, uint8_t duty) { ledcWrite(ch, duty); }
#endif
static const uint8_t CH_A = 0, CH_B = 1;

// ─────────────────────────────────────────────────────────────────────────────
// State
// ─────────────────────────────────────────────────────────────────────────────
enum class Motion : uint8_t { STOP, FWD, REV, LEFT, RIGHT };
enum class State  : uint8_t { IDLE, DRIVING, OBSTACLE_HOLD, TARGET_FOUND, ESTOP };

static State   state        = State::IDLE;
static Motion  motion       = Motion::STOP;
static bool    motorsArmed  = false;
static uint8_t targetSpeed  = DEFAULT_SPEED;
static uint8_t currentSpeed = 0;          // ramped toward targetSpeed
static float   rangeCm      = MAX_RANGE_CM;
static bool    detectLatched = false;

static volatile uint32_t edgeCount = 0;   // written from ISR
static uint16_t lastEdgeRate  = 0;        // edges in the most recent window
static uint32_t toneHeldMs    = 0;        // how long the alarm has sounded
static uint32_t silenceMs     = 0;
static bool     toneConfirmed = false;
static uint32_t lastDetectMs  = 0;

static uint32_t lastCommandMs = 0;
static uint32_t lastRangeMs   = 0;
static uint32_t lastCtrlMs    = 0;
static uint32_t lastTelemMs   = 0;

static Servo markerServo;
static String rxBuffer;

// ─────────────────────────────────────────────────────────────────────────────
// Ranging
// ─────────────────────────────────────────────────────────────────────────────

/** One HC-SR04 ping. Returns MAX_RANGE_CM when nothing echoes back.
 *  This is the fix for defect 1: "no echo" means "nothing is there",
 *  which is the *safest* reading, not the most dangerous one. */
static float pingOnce() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  uint32_t us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return MAX_RANGE_CM;             // timeout → out of range
  float cm = (us * 0.0343f) * 0.5f;
  if (cm < 2.0f || cm > MAX_RANGE_CM) return MAX_RANGE_CM;  // below spec floor
  return cm;
}

/** Median of three pings. Ultrasonic returns are noisy over grass and gravel —
 *  a single spurious short reading would otherwise brake the rover mid-sweep.
 *  Median rejects one outlier at a cost of ~2 extra pings. */
static float pingMedian3() {
  float a = pingOnce(), b = pingOnce(), c = pingOnce();
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

/** ISR: one increment per transition on the tapped buzzer line. */
static void IRAM_ATTR onDetectorEdge() { edgeCount++; }

/** Integrate edges over DETECT_WINDOW_MS and decide whether the alarm is
 *  sounding. Returns true on the window in which a detection is first
 *  confirmed. */
static bool serviceDetector(uint32_t now) {
  if (now - lastDetectMs < DETECT_WINDOW_MS) return false;
  lastDetectMs = now;

  noInterrupts();
  uint32_t edges = edgeCount;
  edgeCount = 0;
  interrupts();
  lastEdgeRate = (edges > 65535) ? 65535 : (uint16_t)edges;

  bool wasConfirmed = toneConfirmed;
  if (lastEdgeRate >= DETECT_MIN_EDGES) {
    toneHeldMs += DETECT_WINDOW_MS;
    silenceMs = 0;
  } else {
    silenceMs += DETECT_WINDOW_MS;
    // A brief dropout mid-alarm should not reset the accumulated hold; only
    // sustained silence does.
    if (silenceMs >= DETECT_RELEASE_MS) { toneHeldMs = 0; toneConfirmed = false; }
  }
  toneConfirmed = (toneHeldMs >= DETECT_CONFIRM_MS);
  return toneConfirmed && !wasConfirmed;
}

// ─────────────────────────────────────────────────────────────────────────────
// Motor primitives
// ─────────────────────────────────────────────────────────────────────────────
static void applyMotion(Motion m, uint8_t duty) {
  switch (m) {
    case Motion::FWD:
      digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
      digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW); break;
    case Motion::REV:
      digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
      digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH); break;
    case Motion::LEFT:
      digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
      digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH); break;
    case Motion::RIGHT:
      digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
      digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);  break;
    case Motion::STOP:
    default:
      digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
      digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
      duty = 0; break;
  }
  pwmWrite(PIN_ENA, CH_A, duty);
  pwmWrite(PIN_ENB, CH_B, duty);
}

static void halt(const char* why) {
  motion = Motion::STOP;
  currentSpeed = 0;
  applyMotion(Motion::STOP, 0);
  if (why) { Serial.print(F("# halt: ")); Serial.println(why); }
}

// ─────────────────────────────────────────────────────────────────────────────
// Marker drop — non-blocking, driven from the control tick
// ─────────────────────────────────────────────────────────────────────────────
static bool     dropActive = false;
static uint32_t dropStartMs = 0;

static void beginMarkerDrop() {
  markerServo.write(SERVO_DROP);
  dropActive  = true;
  dropStartMs = millis();
  Serial.println(F("# marker: released"));
}

static void serviceMarker(uint32_t now) {
  if (dropActive && now - dropStartMs >= DROP_HOLD_MS) {
    markerServo.write(SERVO_STOWED);
    dropActive = false;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Command handling — this ALWAYS runs, in every state. Defect 2 was caused by
// a control path that could skip it.
// ─────────────────────────────────────────────────────────────────────────────
static void printStatus() {
  Serial.print(F("# state="));
  switch (state) {
    case State::IDLE:          Serial.print(F("IDLE")); break;
    case State::DRIVING:       Serial.print(F("DRIVING")); break;
    case State::OBSTACLE_HOLD: Serial.print(F("OBSTACLE_HOLD")); break;
    case State::TARGET_FOUND:  Serial.print(F("TARGET_FOUND")); break;
    case State::ESTOP:         Serial.print(F("ESTOP")); break;
  }
  Serial.print(F(" armed="));  Serial.print(motorsArmed);
  Serial.print(F(" range="));  Serial.print(rangeCm, 1);
  Serial.print(F(" speed="));  Serial.print(currentSpeed);
  Serial.print(F("/"));        Serial.print(targetSpeed);
  Serial.print(F(" detect=")); Serial.print(detectLatched);
  Serial.print(F(" edges/100ms=")); Serial.print(lastEdgeRate);
  Serial.print(F(" toneHeld=")); Serial.println(toneHeldMs);
}

/** Commands set *intent* only. The state machine is derived from sensors on
 *  every control tick by updateState().
 *
 *  Keeping those two things separate is what fixes a bug the host test suite
 *  caught before this ever reached hardware: when setMotion() assigned state
 *  directly, reversing away from an obstacle overwrote OBSTACLE_HOLD with
 *  DRIVING, and forward immediately became legal again while the obstacle was
 *  still 10 cm ahead. State must be a function of the world, not of the last
 *  thing the operator typed. */
static void setMotion(Motion m) {
  if (state == State::ESTOP) { Serial.println(F("! estop engaged")); return; }
  if (!motorsArmed)          { Serial.println(F("! motors disarmed - send ARM")); return; }
  // Gate forward on the live range reading, not on a latched state flag.
  if (m == Motion::FWD && rangeCm <= CLEAR_CM) {
    Serial.println(F("! obstacle ahead - reverse or turn first"));
    return;                   // reverse and turns stay available, unlike v2
  }
  motion = m;
}

/** State is a pure function of (estop, detection, range, motion).
 *  ESTOP and TARGET_FOUND are latched and need an explicit CLEAR. */
static void updateState() {
  if (state == State::ESTOP || state == State::TARGET_FOUND) return;
  if (rangeCm <= STOP_CM) {
    state = State::OBSTACLE_HOLD;
  } else if (state == State::OBSTACLE_HOLD) {
    // Inside the hysteresis band (STOP_CM..CLEAR_CM) the hold persists, so the
    // rover cannot chatter in and out of it at the threshold.
    if (rangeCm >= CLEAR_CM)
      state = (motion == Motion::STOP) ? State::IDLE : State::DRIVING;
  } else {
    state = (motion == Motion::STOP) ? State::IDLE : State::DRIVING;
  }
}

static void handleCommand(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;
  cmd.toUpperCase();
  lastCommandMs = millis();

  // Split "SPD 200" into verb + argument
  int sp = cmd.indexOf(' ');
  String verb = (sp < 0) ? cmd : cmd.substring(0, sp);
  String arg  = (sp < 0) ? ""  : cmd.substring(sp + 1);

  if      (verb == "F" || verb == "FWD"   || verb == "FORWARD") setMotion(Motion::FWD);
  else if (verb == "B" || verb == "REV"   || verb == "BACK")    setMotion(Motion::REV);
  else if (verb == "L" || verb == "LEFT")                       setMotion(Motion::LEFT);
  else if (verb == "R" || verb == "RIGHT")                      setMotion(Motion::RIGHT);
  else if (verb == "S" || verb == "STOP")                       { setMotion(Motion::STOP); halt(nullptr); }
  else if (verb == "ARM"    || verb == "ENABLE")  { motorsArmed = true;  Serial.println(F("# armed")); }
  else if (verb == "DISARM" || verb == "DISABLE") { motorsArmed = false; halt("disarmed"); }
  else if (verb == "ESTOP") { state = State::ESTOP; motorsArmed = false; halt("ESTOP"); }
  else if (verb == "CLEAR") {
    if (state == State::ESTOP)        { state = State::IDLE; Serial.println(F("# estop cleared - still disarmed")); }
    if (state == State::TARGET_FOUND) { state = State::IDLE; }
    detectLatched = false;
  }
  else if (verb == "SPD" || verb == "SPEED") {
    int v = arg.toInt();
    if (v < 0) v = 0; if (v > 255) v = 255;
    targetSpeed = (uint8_t)v;
    Serial.print(F("# speed → ")); Serial.println(targetSpeed);
  }
  else if (verb == "MARK") beginMarkerDrop();
  else if (verb == "CAL") {   // field calibration: watch the raw envelope
    Serial.print(F("# edges/100ms=")); Serial.print(lastEdgeRate);
    Serial.print(F(" threshold="));    Serial.print(DETECT_MIN_EDGES);
    Serial.print(F(" toneHeld="));     Serial.print(toneHeldMs);
    Serial.print(F("/"));              Serial.println(DETECT_CONFIRM_MS);
  }
  else if (verb == "?" || verb == "STATUS") printStatus();
  else if (verb == "HELP") {
    Serial.println(F("# F B L R S | ARM DISARM | SPD <0-255> | MARK | CAL | ESTOP CLEAR | ? HELP"));
  }
  else { Serial.print(F("! unknown: ")); Serial.println(verb); }
}

/** Drain the serial buffer without blocking. readStringUntil() in v2 blocked
 *  for up to the stream timeout on a partial line. */
static void serviceSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (rxBuffer.length()) { handleCommand(rxBuffer); rxBuffer = ""; }
    } else if (rxBuffer.length() < 64) {
      rxBuffer += ch;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Control tick
// ─────────────────────────────────────────────────────────────────────────────
static void serviceControl(uint32_t now) {
  if (toneConfirmed && !detectLatched) {
    detectLatched = true;
    state = State::TARGET_FOUND;
    halt("metal detected");
    beginMarkerDrop();
    Serial.println(F("# TARGET — send CLEAR to acknowledge"));
  }

  State before = state;
  updateState();
  if (state == State::OBSTACLE_HOLD && motion == Motion::FWD)
    halt("obstacle within threshold");
  if (before == State::OBSTACLE_HOLD && state != State::OBSTACLE_HOLD)
    Serial.println(F("# path clear"));

  // Deadman. Defect 3: v2 latched motors on forever.
  if (motion != Motion::STOP && (now - lastCommandMs) > CMD_TIMEOUT_MS) {
    setMotion(Motion::STOP);
    halt("command timeout");
  }

  // Speed ramp toward target
  uint8_t want = (motion == Motion::STOP || !motorsArmed) ? 0 : targetSpeed;
  if (currentSpeed < want)      currentSpeed = min<int>(want, currentSpeed + RAMP_STEP);
  else if (currentSpeed > want) currentSpeed = max<int>(want, currentSpeed - RAMP_STEP);

  applyMotion(motion, currentSpeed);
}

/** Fixed-rate CSV telemetry so a host can log and plot a sweep.
 *  v2 printed on every loop iteration, flooding the link at several kHz. */
static void serviceTelemetry(uint32_t now) {
  Serial.print(F("T,")); Serial.print(now);
  Serial.print(',');     Serial.print((int)state);
  Serial.print(',');     Serial.print(rangeCm, 1);
  Serial.print(',');     Serial.print(currentSpeed);
  Serial.print(',');     Serial.print(detectLatched ? 1 : 0);
  Serial.print(',');     Serial.print(lastEdgeRate);
  Serial.print(',');     Serial.println(digitalRead(PIN_IR) == LOW ? 1 : 0);
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pwmInit(PIN_ENA, CH_A);
  pwmInit(PIN_ENB, CH_B);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_IR, INPUT);
  pinMode(PIN_DETECT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_DETECT), onDetectorEdge, CHANGE);

  markerServo.setPeriodHertz(50);
  markerServo.attach(PIN_SERVO, 500, 2400);
  markerServo.write(SERVO_STOWED);

  halt(nullptr);
  lastCommandMs = millis();

  Serial.println(F("# MineRoverX v3 ready. Motors DISARMED."));
  Serial.println(F("# F B L R S | ARM DISARM | SPD <0-255> | MARK | CAL | ESTOP CLEAR | ? HELP"));
  Serial.println(F("# telemetry: T,ms,state,range_cm,speed,detect,edges,ir"));
}

void loop() {
  uint32_t now = millis();

  serviceSerial();                       // never gated — always runs

  if (now - lastRangeMs >= RANGE_PERIOD_MS) {
    lastRangeMs = now;
    rangeCm = pingMedian3();
  }
  serviceDetector(now);

  if (now - lastCtrlMs >= CTRL_PERIOD_MS) {
    lastCtrlMs = now;
    serviceControl(now);
    serviceMarker(now);
  }
  if (now - lastTelemMs >= TELEM_PERIOD_MS) {
    lastTelemMs = now;
    serviceTelemetry(now);
  }
}

// Host-side unit tests for MineRoverX firmware v3.
// Build:  g++ -std=c++17 -I. -o test_v3 test_v3.cpp && ./test_v3
//
// Each test targets a defect found in v2 (see docs/firmware-postmortem.md) or a
// safety property v3 is supposed to guarantee.
#include "arduino_stub.h"
#include "../firmware/v3/v3.ino"
#include <iostream>

static int passed = 0, failed = 0;
#define CHECK(cond, name) do { \
    if (cond) { ++passed; std::cout << "  pass  " << name << "\n"; } \
    else { ++failed; std::cout << "  FAIL  " << name << "  (" << __LINE__ << ")\n"; } \
} while (0)

static void reset(uint32_t t = 1000) {
    sim() = SimState{};
    sim().millis = t;
    Serial.rx.clear();
    state = State::IDLE; motion = Motion::STOP; motorsArmed = false;
    targetSpeed = DEFAULT_SPEED; currentSpeed = 0;
    rangeCm = MAX_RANGE_CM; detectLatched = false; dropActive = false;
    lastCommandMs = t; lastRangeMs = lastCtrlMs = lastTelemMs = t;
    edgeCount = 0; lastEdgeRate = 0; toneHeldMs = 0; silenceMs = 0;
    toneConfirmed = false; lastDetectMs = t;
    rxBuffer = "";
}
static void send(const std::string& cmd) { Serial.feed(cmd); serviceSerial(); }
static void tick(uint32_t dt) { sim().millis += dt; serviceControl(sim().millis); }
// microseconds of echo for a given distance in cm
// one envelope window with `edges` transitions counted on the buzzer line
static void window(uint16_t edges) {
    edgeCount = edges;
    sim().millis += DETECT_WINDOW_MS;
    serviceDetector(sim().millis);
}
static uint32_t usFor(float cm) { return (uint32_t)(cm * 2.0f / 0.0343f); }

int main() {
    std::cout << "MineRoverX v3 — firmware logic tests\n\n";

    // ── Defect 1: no echo must read as "clear", never as "obstacle at 0 cm" ──
    std::cout << "defect 1 — ultrasonic out-of-range handling\n";
    reset();
    sim().echoQueue = {0};
    CHECK(pingOnce() == MAX_RANGE_CM, "pulseIn timeout returns MAX_RANGE, not 0");
    reset();
    sim().echoQueue = {usFor(1.0f)};
    CHECK(pingOnce() == MAX_RANGE_CM, "below-spec reading (<2cm) rejected");
    reset();
    sim().echoQueue = {usFor(30.0f)};
    CHECK(pingOnce() > 29.0f && pingOnce() == MAX_RANGE_CM, "valid reading passes through, then queue empties to MAX_RANGE");

    // ── Median filter rejects a single spurious return ───────────────────────
    std::cout << "\nnoise rejection\n";
    reset();
    sim().echoQueue = {usFor(80.0f), usFor(3.0f), usFor(82.0f)};
    float m = pingMedian3();
    CHECK(m > 70.0f, "median-of-3 rejects one spurious short return");
    reset();
    sim().echoQueue = {usFor(40.0f), usFor(41.0f), usFor(40.5f)};
    CHECK(pingMedian3() > 39.0f && pingMedian3() <= MAX_RANGE_CM, "median of consistent readings is stable");

    // ── Command parsing ──────────────────────────────────────────────────────
    std::cout << "\ncommand parsing\n";
    reset();
    send("ARM");   CHECK(motorsArmed, "ARM arms the motors");
    send("SPD 200"); CHECK(targetSpeed == 200, "SPD sets target speed");
    send("SPD 999"); CHECK(targetSpeed == 255, "SPD clamps above 255");
    send("SPD -50"); CHECK(targetSpeed == 0,   "SPD clamps below 0");
    send("spd 120"); CHECK(targetSpeed == 120, "commands are case-insensitive");
    reset(); send("F"); CHECK(motion == Motion::STOP, "motion refused while disarmed");
    reset(); send("ARM"); send("FORWARD");
    CHECK(motion == Motion::FWD, "v2 long-form command words still accepted");

    // ── Defect 3: deadman switch ─────────────────────────────────────────────
    std::cout << "\ndefect 3 — command timeout (deadman)\n";
    reset(); send("ARM"); send("F");
    CHECK(motion == Motion::FWD, "rover is driving");
    tick(CMD_TIMEOUT_MS - 100);
    CHECK(motion == Motion::FWD, "still driving before timeout elapses");
    tick(200);
    CHECK(motion == Motion::STOP, "motors stop when no command arrives in time");
    CHECK(sim().pwmA == 0 && sim().pwmB == 0, "PWM driven to zero on timeout");

    // ── Defect 2: rover must stay commandable while holding for an obstacle ──
    std::cout << "\ndefect 2 — remains commandable during obstacle hold\n";
    reset(); send("ARM"); send("F");
    rangeCm = 10.0f; tick(CTRL_PERIOD_MS);
    CHECK(state == State::OBSTACLE_HOLD, "obstacle within 15cm triggers hold");
    send("B");
    CHECK(motion == Motion::REV, "reverse IS accepted while holding (v2 could not)");
    send("F");
    CHECK(motion != Motion::FWD, "forward is still refused into the obstacle");

    // ── Hysteresis ───────────────────────────────────────────────────────────
    std::cout << "\nobstacle hysteresis\n";
    reset(); send("ARM"); send("F");
    rangeCm = 10.0f; tick(CTRL_PERIOD_MS);
    rangeCm = 18.0f; send("S"); send("ARM"); tick(CTRL_PERIOD_MS);
    CHECK(state == State::OBSTACLE_HOLD, "18cm is above STOP but below CLEAR — still held");
    rangeCm = 25.0f; tick(CTRL_PERIOD_MS);
    CHECK(state == State::IDLE, "25cm clears the hold");

    // ── Metal detector: envelope detection on the tapped buzzer line ────────
    std::cout << "\nmetal detector - envelope detection\n";
    reset();
    for (int i = 0; i < 40; ++i) window(0);
    CHECK(!toneConfirmed, "silent line never confirms a detection");

    reset();
    for (int i = 0; i < 5; ++i) window(60);        // 500 ms of alarm
    CHECK(!toneConfirmed, "0.5s of alarm is not enough to commit");
    for (int i = 0; i < 14; ++i) window(60);       // total 1.9 s
    CHECK(!toneConfirmed, "1.9s still below the 2s confirm threshold");
    window(60);                                    // 2.0 s
    CHECK(toneConfirmed, "2.0s of sustained alarm confirms the detection");

    reset();
    for (int i = 0; i < 25; ++i) window(60);
    CHECK(toneConfirmed, "detection confirmed");
    window(0); window(0);                          // 200 ms dropout
    CHECK(toneConfirmed, "a brief dropout mid-alarm does not drop the detection");
    window(0); window(0);                          // 400 ms total silence
    CHECK(!toneConfirmed, "sustained silence releases the detection");

    reset();
    for (int i = 0; i < 25; ++i) window(19);       // just under threshold
    CHECK(!toneConfirmed, "edge rate below threshold is treated as idle");
    reset();
    for (int i = 0; i < 25; ++i) window(20);       // exactly at threshold
    CHECK(toneConfirmed, "edge rate at threshold counts as alarm");

    // ── Detection drives the rover ──────────────────────────────────────────
    std::cout << "\ndetection response\n";
    reset(); send("ARM"); send("F");
    for (int i = 0; i < 25; ++i) { lastCommandMs = sim().millis; window(60); }
    serviceControl(sim().millis);
    CHECK(state == State::TARGET_FOUND, "confirmed detection enters TARGET_FOUND");
    CHECK(motion == Motion::STOP,       "rover halts on detection");
    CHECK(sim().servoAngle == SERVO_DROP, "marker servo actuates to drop position");
    sim().millis += DROP_HOLD_MS + 10; serviceMarker(sim().millis);
    CHECK(sim().servoAngle == SERVO_STOWED, "marker servo restows after hold");
    CHECK(detectLatched, "detection latches until acknowledged");
    send("CLEAR");
    CHECK(!detectLatched && state == State::IDLE, "CLEAR acknowledges and resets state");

    // ── Speed ramp ───────────────────────────────────────────────────────────
    std::cout << "\nsoft-start ramp\n";
    reset(); send("ARM"); send("SPD 240"); send("F");
    tick(CTRL_PERIOD_MS);
    CHECK(currentSpeed == RAMP_STEP, "first tick applies one ramp step, not full power");
    CHECK(currentSpeed < 240, "does not jump straight to target");
    for (int i = 0; i < 40; ++i) { lastCommandMs = sim().millis; tick(CTRL_PERIOD_MS); }
    CHECK(currentSpeed == 240, "reaches target speed after ramping");

    // ── E-stop ───────────────────────────────────────────────────────────────
    std::cout << "\nemergency stop\n";
    reset(); send("ARM"); send("F"); send("ESTOP");
    CHECK(state == State::ESTOP && motion == Motion::STOP, "ESTOP halts and latches");
    send("ARM"); send("F");
    CHECK(motion == Motion::STOP, "motion refused while ESTOP is latched");
    send("CLEAR"); send("ARM"); send("F");
    CHECK(motion == Motion::FWD, "CLEAR then ARM restores control");

    std::cout << "\n" << passed << " passed, " << failed << " failed\n";
    return failed == 0 ? 0 : 1;
}

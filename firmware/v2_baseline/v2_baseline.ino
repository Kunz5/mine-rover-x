// === CONFIGURATION ===

// Motor driver pins (L298N - DC Gear Motors)
const int enA = 27;    // ENA -> PWM (Motor A speed) Right side motor
const int in1 = 26;    // IN1
const int in2 = 25;    // IN2
const int enB = 5;     // ENB -> PWM (Motor B speed)  Left side motor
const int in3 = 18;    // IN3
const int in4 = 19;    // IN4

// Ultrasonic Sensor Pins
const int trigPin = 17;
const int echoPin = 16;

// Motor parameters
int motorSpeed = 180;      // PWM value (0-255)
bool motorsEnabled = true;
const int stopDistance = 15; // cm

// === SETUP ===

void setup() {
  Serial.begin(115200);

  // Setup L298N pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  disableMotors();

  Serial.println("ESP32 + L298N + Ultrasonic Sensor Bot Initialized.");
  Serial.println("Commands: enable, disable, forward, back, left, right, stop");
}

// === MAIN LOOP ===

void loop() {
  float distance = getDistance();

  if (distance <= stopDistance) {
    stopMotors();
    Serial.println("⚠️ Object detected within 15 cm! Stopping...");
    delay(100);
    return;
  } else {
    Serial.println("✅ Path is clear.");
    Serial.println("💬 Waiting for command...");
    // ⚠️ DO NOT return here
  }

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();

    if (command == "enable") {
      motorsEnabled = true;
      Serial.println("Motors enabled.");
    }
    else if (command == "disable") {
      motorsEnabled = false;
      disableMotors();
      Serial.println("Motors disabled.");
    }
    else if (motorsEnabled) {
      if (command == "forward") {
        moveForward();
        Serial.println("Moving forward...");
      }
      else if (command == "back") {
        moveBackward();
        Serial.println("Moving backward...");
      }
      else if (command == "left") {
        turnLeft();
        Serial.println("Turning left...");
      }
      else if (command == "right") {
        turnRight();
        Serial.println("Turning right...");
      }
      else if (command == "stop") {
        stopMotors();
        Serial.println("Motors stopped.");
      }
      else {
        Serial.println("Unknown command.");
      }
    }
    else {
      Serial.println("Motors are disabled. Type 'enable' to activate.");
    }
  }
}


// === ULTRASONIC SENSOR FUNCTION ===

float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000); // 20ms timeout
  float distance = duration * 0.0343 / 2; // cm
  return distance;
}

// === MOTOR CONTROL FUNCTIONS ===

void moveForward() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}

void moveBackward() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}

void turnLeft() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}

void turnRight() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}

void stopMotors() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void disableMotors() {
  stopMotors();
  digitalWrite(enA, LOW);
  digitalWrite(enB, LOW);
}

// This sketch implements the requested luring strategy for the sumo robot.
// The comments deliberately explain each idea in plain English so that someone
// with limited coding experience can follow along and adjust the behavior.
//
// IMPORTANT: if you are copying this file out of a GitHub diff or e-mail,
// make sure you copy only the actual code below (or download the "Raw" file).
// Accidentally pasting diff markers such as leading '+' signs will cause the
// Arduino IDE to show errors like "too many decimal points".

#include <Arduino.h>
#include <math.h>

// ------------------------------------------------------------
// Pin assignments
// ------------------------------------------------------------
// Ultrasonic "trigger" pins send the ping, "echo" pins listen for the return.
const int LEFT_TRIG_PIN = 2;
const int LEFT_ECHO_PIN = 4;
const int RIGHT_TRIG_PIN = 7;
const int RIGHT_ECHO_PIN = 8;

const int FRONT_IR_PIN = A0;

// Five downward-facing IR sensors look for the white border of the arena.
const int LINE_SENSOR_PINS[] = {A1, A2, A3, A4, A5};
const int NUM_LINE_SENSORS = sizeof(LINE_SENSOR_PINS) / sizeof(LINE_SENSOR_PINS[0]);

// Motor driver pins. One PWM pin drives each direction of each motor.
const int LEFT_FWD_PIN = 5;
const int LEFT_REV_PIN = 6;
const int RIGHT_FWD_PIN = 9;
const int RIGHT_REV_PIN = 10;

// ------------------------------------------------------------
// Motion tuning constants
// ------------------------------------------------------------
// Speed settings (0-255). These can be tuned for your specific robot.
const int SEARCH_SPIN_SPEED = 150;
const int LURE_BASE_SPEED = 120;
const int LURE_STEER_OFFSET = 40;
const int PRESENTATION_ROTATE_SPEED = 180;
const unsigned long PRESENTATION_ROTATE_TIME = 180; // ms
const int HIT_BACKUP_SPEED = 200;
const unsigned long HIT_BACKUP_DURATION = 300; // ms
const int EDGE_BACKUP_SPEED = 200;
const unsigned long EDGE_BACKUP_DURATION = 450; // ms
const unsigned long EDGE_TURN_DURATION = 250; // ms
const int EDGE_TURN_SPEED = 180;

// ------------------------------------------------------------
// Sensor thresholds & logic tuning
// ------------------------------------------------------------
// Distances are measured in centimeters. Keep everything together for clarity.
const int ENEMY_DETECTION_DISTANCE = 60;          // Start reacting when enemy is closer than this
const int PRESENTATION_TRIGGER_DISTANCE = 15;     // Rotate to present an arm when enemy is near edge
const int HIT_DISTANCE_THRESHOLD = 12;            // If the opponent gets extremely close we treat it as a hit
const int HIT_DROP_THRESHOLD = 20;                // Sudden drop of 20 cm between readings also counts as a hit
const unsigned long WAIT_FOR_HIT_TIMEOUT = 1000;  // Give the opponent 1 second to strike
const unsigned long ENEMY_LOST_TIMEOUT = 600;     // If we lose contact for this long, return to searching
const int STABLE_DETECTION_COUNT = 3;             // Require 3 good readings to avoid reacting to noise
const int EDGE_WHITE_THRESHOLD = 700;             // Analog value indicating the white border

// Ultrasonic debounce settings
const int ULTRASONIC_ATTEMPTS = 3;

// ------------------------------------------------------------
// Enumerations and state tracking
// ------------------------------------------------------------
// Each state describes a simple behavior we can explain in plain language.
enum RobotState {
  SEARCHING,
  LURE_LEFT,
  LURE_RIGHT,
  PRESENT_LEFT_ARM,
  PRESENT_RIGHT_ARM,
  WAIT_FOR_HIT,
  BACKING_UP
};

// Track which arm is currently being presented.
enum ArmSelection {
  ARM_NONE,
  ARM_LEFT,
  ARM_RIGHT
};

RobotState currentState = SEARCHING;
ArmSelection presentedArm = ARM_NONE;
unsigned long stateStartTime = 0;  // Used for timing actions in each state

// Sensor history for hit detection
float lastLeftUltrasonic = 999.0f;
float lastRightUltrasonic = 999.0f;

// Stable detection bookkeeping. These counters confirm that we really saw the
// opponent and not just a random sensor glitch.
int leftStableCount = 0;
int rightStableCount = 0;
unsigned long lastLeftSeenTime = 0;
unsigned long lastRightSeenTime = 0;

// ------------------------------------------------------------
// Forward declarations
// ------------------------------------------------------------
float readUltrasonic(int trigPin, int echoPin);
float readFrontIR();
void setMotors(int leftSpeed, int rightSpeed);
void stopMotors();
bool detectHit(float currentDistance, float &previousDistance);
bool checkEdge();
void handleEdge();
void transitionTo(RobotState newState);
void resetDetectionCounters();
const char *stateToString(RobotState state);

// ------------------------------------------------------------
// Arduino setup
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);

  pinMode(LEFT_FWD_PIN, OUTPUT);
  pinMode(LEFT_REV_PIN, OUTPUT);
  pinMode(RIGHT_FWD_PIN, OUTPUT);
  pinMode(RIGHT_REV_PIN, OUTPUT);

  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    pinMode(LINE_SENSOR_PINS[i], INPUT);
  }

  stopMotors();
  transitionTo(SEARCHING);
}

// ------------------------------------------------------------
// Main control loop
// ------------------------------------------------------------
void loop() {
  // 1) Always prioritize edge detection. Staying in the ring is the top priority.
  if (checkEdge()) {
    handleEdge();
    return;
  }

  // 2) Read all sensors so the current state can use the latest information.
  float leftDistance = readUltrasonic(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  float rightDistance = readUltrasonic(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
  float frontDistance = readFrontIR();

  unsigned long now = millis();

  // 3) Stable detection logic. Increase a counter when a sensor repeatedly sees
  // the opponent, reset the counter when the reading is far away again.
  if (leftDistance < ENEMY_DETECTION_DISTANCE) {
    if (leftStableCount < STABLE_DETECTION_COUNT) {
      leftStableCount++;
    }
    lastLeftSeenTime = now;
  } else {
    leftStableCount = 0;
  }

  if (rightDistance < ENEMY_DETECTION_DISTANCE) {
    if (rightStableCount < STABLE_DETECTION_COUNT) {
      rightStableCount++;
    }
    lastRightSeenTime = now;
  } else {
    rightStableCount = 0;
  }

  // 4) Print out the raw data so a human can see what is happening in real time.
  Serial.print(F("State: "));
  Serial.print(stateToString(currentState));
  Serial.print(F(" | L: "));
  Serial.print(leftDistance);
  Serial.print(F("cm | R: "));
  Serial.print(rightDistance);
  Serial.print(F("cm | Front IR: "));
  Serial.print(frontDistance);
  Serial.println(F("cm"));

  switch (currentState) {
    case SEARCHING:
      // Spin in place until one of the ultrasonic sensors sees an opponent.
      setMotors(SEARCH_SPIN_SPEED, -SEARCH_SPIN_SPEED);
      if (leftStableCount >= STABLE_DETECTION_COUNT) {
        transitionTo(LURE_LEFT);
      } else if (rightStableCount >= STABLE_DETECTION_COUNT) {
        transitionTo(LURE_RIGHT);
      }
      break;

    case LURE_LEFT:
      // Slowly angle toward the right so the opponent follows us toward the left edge.
      setMotors(LURE_BASE_SPEED - LURE_STEER_OFFSET, LURE_BASE_SPEED + LURE_STEER_OFFSET);
      if (leftDistance <= PRESENTATION_TRIGGER_DISTANCE) {
        transitionTo(PRESENT_RIGHT_ARM);
      } else if (now - lastLeftSeenTime > ENEMY_LOST_TIMEOUT) {
        transitionTo(SEARCHING);
      }
      break;

    case LURE_RIGHT:
      // Mirror of the previous state: lure the opponent toward the right edge.
      setMotors(LURE_BASE_SPEED + LURE_STEER_OFFSET, LURE_BASE_SPEED - LURE_STEER_OFFSET);
      if (rightDistance <= PRESENTATION_TRIGGER_DISTANCE) {
        transitionTo(PRESENT_LEFT_ARM);
      } else if (now - lastRightSeenTime > ENEMY_LOST_TIMEOUT) {
        transitionTo(SEARCHING);
      }
      break;

    case PRESENT_LEFT_ARM:
      // Rotate so the right arm faces the opponent. We time this action instead of
      // using sensors so that the motion is quick and consistent.
      setMotors(-PRESENTATION_ROTATE_SPEED, PRESENTATION_ROTATE_SPEED);
      if (now - stateStartTime >= PRESENTATION_ROTATE_TIME) {
        stopMotors();
        presentedArm = ARM_LEFT;
        transitionTo(WAIT_FOR_HIT);
      }
      break;

    case PRESENT_RIGHT_ARM:
      // Same idea, just opposite rotation.
      setMotors(PRESENTATION_ROTATE_SPEED, -PRESENTATION_ROTATE_SPEED);
      if (now - stateStartTime >= PRESENTATION_ROTATE_TIME) {
        stopMotors();
        presentedArm = ARM_RIGHT;
        transitionTo(WAIT_FOR_HIT);
      }
      break;

    case WAIT_FOR_HIT: {
      // Sit still with the chosen arm pointing at the opponent and watch for a hit.
      stopMotors();
      bool hitDetected = false;
      if (presentedArm == ARM_LEFT) {
        hitDetected = detectHit(leftDistance, lastLeftUltrasonic);
      } else if (presentedArm == ARM_RIGHT) {
        hitDetected = detectHit(rightDistance, lastRightUltrasonic);
      }

      if (hitDetected) {
        // Back away briefly to avoid being pushed out, then resume searching.
        transitionTo(BACKING_UP);
        setMotors(-HIT_BACKUP_SPEED, -HIT_BACKUP_SPEED);
      } else if (now - stateStartTime >= WAIT_FOR_HIT_TIMEOUT) {
        transitionTo(SEARCHING);
      }
      break;
    }

    case BACKING_UP:
      // After the short reverse timer expires, re-enter SEARCHING.
      if (now - stateStartTime >= HIT_BACKUP_DURATION) {
        stopMotors();
        transitionTo(SEARCHING);
      }
      break;
  }
}

// ------------------------------------------------------------
// Sensor helpers
// ------------------------------------------------------------
float readUltrasonic(int trigPin, int echoPin) {
  // Take several readings and average them to remove noise from the sensors.
  long validSum = 0;
  int validCount = 0;

  for (int i = 0; i < ULTRASONIC_ATTEMPTS; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 25000); // 25 ms timeout (~4 m)
    if (duration > 0) {
      long distance = duration / 58; // Convert to cm
      if (distance > 0 && distance < 400) {
        validSum += distance;
        validCount++;
      }
    }
    delay(5);
  }

  if (validCount == 0) {
    // Use 999 to represent "nothing detected". This is easier to compare later.
    return 999.0f;
  }
  return (float)validSum / validCount;
}

float readFrontIR() {
  // Convert the analog voltage from the Sharp IR sensor into centimeters.
  int raw = analogRead(FRONT_IR_PIN);
  float voltage = raw * (5.0f / 1023.0f);
  if (voltage <= 0.42f) {
    return 80.0f; // beyond sensor range
  }
  float distance = 27.86f / (voltage - 0.42f);
  if (distance < 10.0f) distance = 10.0f;
  if (distance > 80.0f || isnan(distance)) distance = 80.0f;
  return distance;
}

// ------------------------------------------------------------
// Motor helpers
// ------------------------------------------------------------
void setMotors(int leftSpeed, int rightSpeed) {
  // Positive values drive forward, negative values drive backward.
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed >= 0) {
    analogWrite(LEFT_FWD_PIN, leftSpeed);
    analogWrite(LEFT_REV_PIN, 0);
  } else {
    analogWrite(LEFT_FWD_PIN, 0);
    analogWrite(LEFT_REV_PIN, -leftSpeed);
  }

  if (rightSpeed >= 0) {
    analogWrite(RIGHT_FWD_PIN, rightSpeed);
    analogWrite(RIGHT_REV_PIN, 0);
  } else {
    analogWrite(RIGHT_FWD_PIN, 0);
    analogWrite(RIGHT_REV_PIN, -rightSpeed);
  }
}

void stopMotors() {
  // Helper that explicitly turns everything off.
  analogWrite(LEFT_FWD_PIN, 0);
  analogWrite(LEFT_REV_PIN, 0);
  analogWrite(RIGHT_FWD_PIN, 0);
  analogWrite(RIGHT_REV_PIN, 0);
}

// ------------------------------------------------------------
// Hit detection and safety
// ------------------------------------------------------------
bool detectHit(float currentDistance, float &previousDistance) {
  // The logic looks at how the ultrasonic distance changed between readings.
  bool hit = false;
  if (currentDistance >= 999.0f) {
    previousDistance = currentDistance;
    return false;
  }

  if (previousDistance < 999.0f &&
      (previousDistance - currentDistance) >= HIT_DROP_THRESHOLD) {
    hit = true;
  }

  if (currentDistance <= HIT_DISTANCE_THRESHOLD) {
    hit = true;
  }

  previousDistance = currentDistance;
  return hit;
}

bool checkEdge() {
  // If ANY of the downward sensors see the white line we must bail out.
  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    int reading = analogRead(LINE_SENSOR_PINS[i]);
    if (reading > EDGE_WHITE_THRESHOLD) {
      return true;
    }
  }
  return false;
}

void handleEdge() {
  // Emergency routine: back away, pivot, then go back to searching.
  Serial.println(F("EDGE DETECTED - ESCAPE"));
  stopMotors();
  setMotors(-EDGE_BACKUP_SPEED, -EDGE_BACKUP_SPEED);
  delay(EDGE_BACKUP_DURATION);
  setMotors(EDGE_TURN_SPEED, -EDGE_TURN_SPEED);
  delay(EDGE_TURN_DURATION);
  stopMotors();
  presentedArm = ARM_NONE;
  resetDetectionCounters();
  transitionTo(SEARCHING);
}

// ------------------------------------------------------------
// State management helpers
// ------------------------------------------------------------
void transitionTo(RobotState newState) {
  // Central helper to keep all state changes consistent and well-logged.
  currentState = newState;
  stateStartTime = millis();
  Serial.print(F("Transition -> "));
  Serial.println(stateToString(currentState));

  if (newState == SEARCHING) {
    presentedArm = ARM_NONE;
    resetDetectionCounters();
  }

  if (newState == BACKING_UP) {
    presentedArm = ARM_NONE;
  }
}

void resetDetectionCounters() {
  // Bring all the timing and detection helpers back to a known starting point.
  leftStableCount = 0;
  rightStableCount = 0;
  lastLeftSeenTime = millis();
  lastRightSeenTime = millis();
  lastLeftUltrasonic = 999.0f;
  lastRightUltrasonic = 999.0f;
}

const char *stateToString(RobotState state) {
  switch (state) {
    case SEARCHING:
      return "SEARCHING";
    case LURE_LEFT:
      return "LURE_LEFT";
    case LURE_RIGHT:
      return "LURE_RIGHT";
    case PRESENT_LEFT_ARM:
      return "PRESENT_LEFT_ARM";
    case PRESENT_RIGHT_ARM:
      return "PRESENT_RIGHT_ARM";
    case WAIT_FOR_HIT:
      return "WAIT_FOR_HIT";
    case BACKING_UP:
      return "BACKING_UP";
  }
  return "UNKNOWN";
}

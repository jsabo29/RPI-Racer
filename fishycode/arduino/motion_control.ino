#include <Servo.h>

Servo steering_servo;
Servo drive_motor;

const int STEERING_PIN = 14;
const int MOTOR_PIN = 9;

const int FRAME_WIDTH  = 640;
const int FRAME_HEIGHT = 480;

const int MOTOR_SPEED = 18; //18; //40;

void setup() {
  steering_servo.attach(STEERING_PIN);
  drive_motor.attach(MOTOR_PIN);
  Serial.begin(19200);
  while (!Serial);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int commaIdx = input.indexOf(',');
    if (commaIdx == -1) return;

    int x = input.substring(0, commaIdx).toInt();
    int y = input.substring(commaIdx + 1).toInt();

    // Determine region and act
    handleMovement(x, y);
  }
}

void handleMovement(int x, int y) {
  int col = (x < FRAME_WIDTH / 3) ? 0 : (x < 2 * FRAME_WIDTH / 3) ? 1 : 2;
  int row = (y < FRAME_HEIGHT / 3) ? 0 : (y < 2 * FRAME_HEIGHT / 3) ? 1 : 2;

  if (row == 1 && col == 1) {
    // Center region → Idle
    setDrivePower(0);
    steering_servo.write(90);
    Serial.println(101); 
    return;
  }

  // Movement mapping
  switch (col) {
    case 0:  // Top row
      setDrivePower(-MOTOR_SPEED);
      steerByRow(row, true); // true = forward turn
      Serial.println(1); 
      break;
    case 1:  // Middle row
      setDrivePower(0);
      steerByRow(row, false);
      Serial.println(0); 
      break;
    case 2:  // Bottom row
      setDrivePower(MOTOR_SPEED);
      steerByRow(row, false);
      Serial.println(-1); 
      break;
  }
}

void steerByRow(int row, bool topRow) {
  switch (row) {
    case 0: steering_servo.write(60); break;  // Left
    case 1: steering_servo.write(90); break;  // Center
    case 2: steering_servo.write(120); break; // Right
  }
}

/* void setDrivePower(int power) {
  power = constrain(power, -100, 100);
  int signal = map(power, -100, 100, 1470, 1530);
  drive_motor.writeMicroseconds(signal);
  Serial.println(signal);
} */

// Calibrated thresholds you measured:
const int ESC_NEUTRAL   = 1500;
const int ESC_FWD_START = 1550;
const int ESC_REV_START = 1450;

// Limit top speed (raise slowly once it feels controllable)
const int ESC_FWD_MAX = 1650;   // try 1600–1750
const int ESC_REV_MAX = 1350;   // try 1400–1250

// Slew limit (prevents instant jumps)
int esc_us = ESC_NEUTRAL;

void setDrivePower(int power) {
  power = constrain(power, -100, 100);

  int target = ESC_NEUTRAL;

  if (power > 0) {
    target = map(power, 1, 100, ESC_FWD_START, ESC_FWD_MAX);
  } else if (power < 0) {
    target = map(power, -1, -100, ESC_REV_START, ESC_REV_MAX);
  }

  // Ramp (smooth acceleration/deceleration)
  const int step = 10; // smaller = smoother/slower response
  if (target > esc_us) esc_us = min(esc_us + step, target);
  else if (target < esc_us) esc_us = max(esc_us - step, target);

  drive_motor.writeMicroseconds(esc_us);
}
#include <Servo.h>
#include <SharpIR.h>

Servo steering_servo;
Servo drive_motor;

const int STEERING_PIN = 14;
const int MOTOR_PIN = 9;

const int FRAME_WIDTH  = 640;
const int FRAME_HEIGHT = 480;

// Your “base” speed in percent (we multiply by speed_factor)
const int MOTOR_SPEED = 8; //18;

// ESC calibration you measured:
const int ESC_NEUTRAL   = 1500;
const int ESC_FWD_START = 1550;
const int ESC_REV_START = 1450;

const int ESC_FWD_MAX = 1560; //1650;
const int ESC_REV_MAX = 1440; //1350;

// Slew limit (prevents instant jumps)
int esc_us = ESC_NEUTRAL;

// random variables
char lineBuf[96];
size_t lineLen = 0; 
const unsigned long CONTROL_PERIOD_MS = 20; //Hz
unsigned long lastControlMs = 0; 
const unsigned long FAILSAFE_MS = 300;   // 0.3s; adjust 200–500ms
unsigned long lastCmdMs = 0;
// -------------------------
// AVOIDANCE STATE MACHINE
// -------------------------
enum Mode { FOLLOW, AVOID_NEUTRAL1, AVOID_REVERSE_TURN, AVOID_NEUTRAL2 };
Mode mode = FOLLOW;

unsigned long modeStartMs = 0;

// Tunable avoidance timings
const unsigned long AVOID_NEUTRAL1_MS    = 250;
const unsigned long AVOID_REVERSE_TURN_MS = 600;
const unsigned long AVOID_NEUTRAL2_MS    = 200;

// Tunable avoidance powers/steering
const int AVOID_REVERSE_POWER = -35;   // percent (-100..100)
const int AVOID_STEER_LEFT    = 60;
const int AVOID_STEER_RIGHT   = 120;

// Alternate which way we dodge each obstacle
bool avoidTurnRight = true;

// Latest command values from Pi
int   cmd_x = FRAME_WIDTH / 2;
int   cmd_y = FRAME_HEIGHT / 2;
float cmd_speed_factor = 0.0;
int   cmd_obstacle = 0;

// IR Sensor settings
#define IR_PIN A1
#define MIN_READINGS_TO_DETERMINE_OBJECT 5
SharpIR sensor(SharpIR::GP2Y0A21YK0F, IR_PIN)
int lastReadings[MIN_READINGS_TO_DETERMINE_OBJECT];

// -------------------------
// Helpers
// -------------------------
void setDrivePower(int power) {
  power = constrain(power, -100, 100);

  int target = ESC_NEUTRAL;

  if (power > 0) {
    target = map(power, 1, 100, ESC_FWD_START, ESC_FWD_MAX);
  } else if (power < 0) {
    target = map(power, -1, -100, ESC_REV_START, ESC_REV_MAX);
  }

  const int step = 10; // smaller = smoother
  if (target > esc_us) esc_us = min(esc_us + step, target);
  else if (target < esc_us) esc_us = max(esc_us - step, target);

  drive_motor.writeMicroseconds(esc_us);
  //Serial.println(esc_us); 
}

bool rearObjectDetected() {
  int currentDistance = sensor.getDisance();
  bool isObject = currentDistance < 50;
  for (int i = 0; i < MIN_READINGS_TO_DETERMINE_OBJECT; i++) {
    if (lastReadings[i] > 50) {
      isObject = false;
      break;
    }
  }

  // shift readings down one
  for (int i = 0; i < MIN_READINGS_TO_DETERMINE_OBJECT-1; i++) {
    lastReadings[i] = lastReadings[i+1];
  }
  lastReadings[MIN_READINGS_TO_DETERMINE_OBJECT-1] = currentDistance;

  return isObject;
}

void steerByRow(int row) {
  switch (row) {
    case 0: steering_servo.write(60);  break; // left
    case 1: steering_servo.write(90);  break; // center
    case 2: steering_servo.write(120); break; // right
  }
}

void enterMode(Mode m) {
  mode = m;
  modeStartMs = millis();
}

void startAvoidance() {
  // flip direction each obstacle event
  avoidTurnRight = !avoidTurnRight;
  enterMode(AVOID_NEUTRAL1);
}

// -------------------------
// CSV parsing: x,y,speed_factor,boundary_factor,obstacle_flag
// -------------------------

bool parseCommandLineC(const char *buf) {
  // Make a writable copy for strtok
  char tmp[96];
  strncpy(tmp, buf, sizeof(tmp));
  tmp[sizeof(tmp)-1] = '\0';

  char *tok = strtok(tmp, ",");
  if (!tok) return false;
  cmd_x = atoi(tok);

  tok = strtok(NULL, ",");
  if (!tok) return false;
  cmd_y = atoi(tok);

  tok = strtok(NULL, ",");
  if (!tok) return false;
  cmd_speed_factor = atof(tok);

  tok = strtok(NULL, ","); // boundary_factor (ignored)
  if (!tok) return false;

  tok = strtok(NULL, ",");
  if (!tok) return false;
  cmd_obstacle = atoi(tok);

  cmd_x = constrain(cmd_x, 0, FRAME_WIDTH);
  cmd_y = constrain(cmd_y, 0, FRAME_HEIGHT);
  cmd_speed_factor = constrain(cmd_speed_factor, 0.0, 1.0);
  cmd_obstacle = (cmd_obstacle != 0) ? 1 : 0;

  return true;
}

// -------------------------
// Normal fish-follow mapping
// -------------------------
void handleFollow(int x, int y, float speedFactor) {
  int col = (x < FRAME_WIDTH / 3) ? 0 : (x < 2 * FRAME_WIDTH / 3) ? 1 : 2;
  int row = (y < FRAME_HEIGHT / 3) ? 0 : (y < 2 * FRAME_HEIGHT / 3) ? 1 : 2;

  // Center region → Idle
  if (row == 1 && col == 1) {
    setDrivePower(0);
    steering_servo.write(90);
    return;
  }

  // Scale your base speed by speedFactor from Python
  int scaled = (int)round(MOTOR_SPEED * speedFactor);
  scaled = constrain(scaled, 0, 100);

  // Your original mapping (col decides forward/reverse/stop)
  switch (col) {
    case 0:  // left third -> reverse
      setDrivePower(-scaled);
      steerByRow(row);
      break;

    case 1:  // middle third -> stop (steer still follows row)
      setDrivePower(0);
      steerByRow(row);
      break;

    case 2:  // right third -> forward
      setDrivePower(scaled);
      steerByRow(row);
      break;
  }
}

// -------------------------
// Avoidance machine update
// -------------------------
void updateAvoidance() {
  unsigned long now = millis();
  unsigned long elapsed = now - modeStartMs;

  if (mode == AVOID_NEUTRAL1) {
    setDrivePower(0);
    steering_servo.write(90);
    if (elapsed >= AVOID_NEUTRAL1_MS) {
      enterMode(AVOID_REVERSE_TURN);
    }
    return;
  }

  if (mode == AVOID_REVERSE_TURN) {
    // reverse + steer left/right
    setDrivePower(AVOID_REVERSE_POWER);
    steering_servo.write(avoidTurnRight ? AVOID_STEER_RIGHT : AVOID_STEER_LEFT);

    if (elapsed >= AVOID_REVERSE_TURN_MS) {
      enterMode(AVOID_NEUTRAL2);
    }
    return;
  }

  if (mode == AVOID_NEUTRAL2) {
    setDrivePower(0);
    steering_servo.write(90);
    if (elapsed >= AVOID_NEUTRAL2_MS) {
      // If obstacle still present, do it again; otherwise go back to follow
      if (cmd_obstacle == 1) startAvoidance();
      else enterMode(FOLLOW);
    }
    return;
  }
}

void setup() {
  steering_servo.attach(STEERING_PIN);
  drive_motor.attach(MOTOR_PIN);
  Serial.begin(19200);
  while (!Serial);

  // Start safe
  steering_servo.write(90);
  drive_motor.writeMicroseconds(ESC_NEUTRAL);
  esc_us = ESC_NEUTRAL;
  lastCmdMs = millis(); 

  // IR_SENSOR
  for (int i = 0; i < MIN_READINGS_TO_DETERMINE_OBJECT; i++) {
    lastReadings[i] = 100;
  }
  Serial.begin(19200);
  Serial.println("SharpIR Ready")
}

void loop() {
  // 1) Read serial line if present (non-blocking-ish)

  while (Serial.available()) {
  char c = (char)Serial.read();
  if (c == '\n') {
    lineBuf[lineLen] = '\0';
    if (lineLen > 0) {
      if (parseCommandLineC(lineBuf)){
        lastCmdMs = millis(); // new function below
      }
    }
    lineLen = 0;
  } else if (c != '\r') {
    if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      // overflow, reset
      lineLen = 0;
    }
  }
}

  // FAILSAFE: stop if Python stops sending commands
 
  if (millis() - lastCmdMs > FAILSAFE_MS) {
    setDrivePower(0);
    steering_servo.write(90);
    enterMode(FOLLOW);
    cmd_obstacle = 0;
    return;}

  // 2) Highest priority: obstacle avoidance
  if (mode == FOLLOW && cmd_obstacle == 1) {
    startAvoidance();
  }

  if (mode != FOLLOW) {
    updateAvoidance();
    return;
  }

  // 3) Normal follow mode
  unsigned long now = millis(); 
  if(now - lastControlMs < CONTROL_PERIOD_MS) return; 
  lastControlMs = now; 
  handleFollow(cmd_x, cmd_y, cmd_speed_factor);
  }// 600,240, 1.0, 0.00, 0// 320,240,1.0,0.00,0
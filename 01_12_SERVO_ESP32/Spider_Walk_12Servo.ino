#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ESP32 + PCA9685 + 12-servo quadruped spider robot
// No sensors, display, Wi-Fi, or web controller.

#define SDA_PIN 21
#define SCL_PIN 22
#define PCA9685_ADDR 0x40

Adafruit_PWMServoDriver pwm(PCA9685_ADDR);

// Logical servo order: FL coxa/femur/tibia, FR, BL, BR
const uint8_t servoChannel[12] = {
  0, 1, 2,       // Front Left
  4, 5, 6,       // Front Right
  8, 9, 10,      // Back Left
  12, 13, 14     // Back Right
};

// Calibrate these to your actual servos. Keep conservative at first.
const int SERVO_MIN_TICK = 100;
const int SERVO_MAX_TICK = 520;
const int ANGLE_MIN = 70;
const int ANGLE_MAX = 110;
const int CENTER_ANGLE = 90;

const int COXA_SWING = 10;
const int FEMUR_LIFT = 10;
const int TIBIA_ADJUST = 8;
const int STEP_DELAY_MS = 350;

int currentAngle[12];

int clampAngle(int angle) {
  return constrain(angle, ANGLE_MIN, ANGLE_MAX);
}

uint16_t angleToTick(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVO_MIN_TICK, SERVO_MAX_TICK);
}

void setServo(int logicalIndex, int angle) {
  if (logicalIndex < 0 || logicalIndex >= 12) return;
  angle = clampAngle(angle);
  currentAngle[logicalIndex] = angle;
  pwm.setPWM(servoChannel[logicalIndex], 0, angleToTick(angle));
}

void setLeg(int leg, int coxa, int femur, int tibia) {
  int base = leg * 3;
  setServo(base, coxa);
  setServo(base + 1, femur);
  setServo(base + 2, tibia);
}

void standPose() {
  for (int i = 0; i < 4; i++) {
    setLeg(i, CENTER_ANGLE, CENTER_ANGLE, CENTER_ANGLE);
  }
  delay(600);
}

// First diagonal: Front Left + Back Right move together.
// Second diagonal: Front Right + Back Left move together.
// This is a basic demo gait; adjust directions/angles for your linkage.
void walkForwardStep() {
  // Phase A: lift and swing diagonal FL + BR forward
  setLeg(0, CENTER_ANGLE + COXA_SWING, CENTER_ANGLE - FEMUR_LIFT,
         CENTER_ANGLE + TIBIA_ADJUST);
  setLeg(3, CENTER_ANGLE - COXA_SWING, CENTER_ANGLE - FEMUR_LIFT,
         CENTER_ANGLE + TIBIA_ADJUST);
  // Other diagonal supports the body
  setLeg(1, CENTER_ANGLE, CENTER_ANGLE + FEMUR_LIFT / 2,
         CENTER_ANGLE - TIBIA_ADJUST / 2);
  setLeg(2, CENTER_ANGLE, CENTER_ANGLE + FEMUR_LIFT / 2,
         CENTER_ANGLE - TIBIA_ADJUST / 2);
  delay(STEP_DELAY_MS);

  // Phase B: lower FL + BR; swing FR + BL
  setLeg(0, CENTER_ANGLE - COXA_SWING, CENTER_ANGLE + FEMUR_LIFT / 2,
         CENTER_ANGLE - TIBIA_ADJUST / 2);
  setLeg(3, CENTER_ANGLE + COXA_SWING, CENTER_ANGLE + FEMUR_LIFT / 2,
         CENTER_ANGLE - TIBIA_ADJUST / 2);
  setLeg(1, CENTER_ANGLE - COXA_SWING, CENTER_ANGLE - FEMUR_LIFT,
         CENTER_ANGLE + TIBIA_ADJUST);
  setLeg(2, CENTER_ANGLE + COXA_SWING, CENTER_ANGLE - FEMUR_LIFT,
         CENTER_ANGLE + TIBIA_ADJUST);
  delay(STEP_DELAY_MS);

  // Return toward center for the next cycle
  standPose();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setPWMFreq(50);  // Standard hobby-servo frequency
  delay(300);

  // Initialize all channels to center before moving.
  for (int i = 0; i < 12; i++) {
    currentAngle[i] = CENTER_ANGLE;
    pwm.setPWM(servoChannel[i], 0, angleToTick(CENTER_ANGLE));
  }
  delay(1000);
  standPose();
  Serial.println("Spider ready. Send W to walk, S to stand, X to stop walking.");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'W' || command == 'w') {
      walkForwardStep();
    } else if (command == 'S' || command == 's') {
      standPose();
    } else if (command == 'X' || command == 'x') {
      standPose();
      Serial.println("Stopped at stand pose.");
    }
  }
}

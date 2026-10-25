#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// QUADROPAD / Spider robot
// ESP32 + PCA9685 (I2C address 0x40)
// ESP32 SDA=GPIO21, SCL=GPIO22
// CH0=R1, CH1=R2, CH2=L1, CH3=L2
// CH4=R4, CH5=R3, CH6=L3, CH7=L4
//
// SAFETY:
// 1) Test with the robot lifted off the ground.
// 2) Start with neutral angles and calibrate each channel.
// 3) If any servo buzzes, stalls, or binds, cut servo power.
// 4) This is a calibration starter, not a guaranteed gait for every assembly.

Adafruit_PWMServoDriver pwm(0x40);

const uint8_t SERVO_COUNT = 8;
const uint16_t SERVO_MIN_TICK = 110;  // Tune for your servos; do not assume endpoints are safe
const uint16_t SERVO_MAX_TICK = 510;  // Tune for your servos; do not assume endpoints are safe

const uint8_t R1 = 0;
const uint8_t R2 = 1;
const uint8_t L1 = 2;
const uint8_t L2 = 3;
const uint8_t R4 = 4;
const uint8_t R3 = 5;
const uint8_t L3 = 6;
const uint8_t L4 = 7;

// Neutral reference. Since 90 degrees works on your robot, keep this as the baseline.
int neutral[SERVO_COUNT] = {90, 90, 90, 90, 90, 90, 90, 90};

// Per-channel direction: set to -1 for a servo whose mechanical direction is reversed.
// Leave at +1 until you test the channel. This is a software angle mapping only.
int8_t direction[SERVO_COUNT] = {1, 1, 1, 1, 1, 1, 1, 1};

const int ANGLE_LIMIT_LOW = 60;
const int ANGLE_LIMIT_HIGH = 120;
const unsigned long STEP_MS = 350;
const uint8_t WALK_CYCLES = 2;

void setServoAngle(uint8_t channel, int logicalAngle) {
  if (channel >= SERVO_COUNT) return;

  // Conservative range while calibrating; prevents accidental 0/180 commands.
  logicalAngle = constrain(logicalAngle, ANGLE_LIMIT_LOW, ANGLE_LIMIT_HIGH);

  int physicalAngle = neutral[channel] +
                      direction[channel] * (logicalAngle - 90);
  physicalAngle = constrain(physicalAngle, ANGLE_LIMIT_LOW, ANGLE_LIMIT_HIGH);

  uint16_t tick = map(physicalAngle, 0, 180,
                      SERVO_MIN_TICK, SERVO_MAX_TICK);
  pwm.setPWM(channel, 0, tick);
}

void setNeutralPose() {
  for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
    // Direct neutral command, using the same PCA9685 pulse mapping.
    uint16_t tick = map(90, 0, 180, SERVO_MIN_TICK, SERVO_MAX_TICK);
    pwm.setPWM(ch, 0, tick);
  }
  delay(700);
}

void setPose(int a0, int a1, int a2, int a3,
             int a4, int a5, int a6, int a7) {
  setServoAngle(0, a0);
  setServoAngle(1, a1);
  setServoAngle(2, a2);
  setServoAngle(3, a3);
  setServoAngle(4, a4);
  setServoAngle(5, a5);
  setServoAngle(6, a6);
  setServoAngle(7, a7);
}

void runRestPose() {
  setNeutralPose();
  Serial.println("Neutral pose: all channels at 90 degrees.");
}

void runStandPose() {
  // Your reported stable position is all servos at 90 degrees.
  // Keep this until the mechanical stand angles are measured and calibrated.
  setNeutralPose();
  Serial.println("Stand reference pose: neutral. Calibrate leg geometry before walking.");
}

// Slow alternating leg test, NOT a guaranteed ground-walking gait.
// Run with the robot lifted. Observe which leg each channel controls.
void runWalkCalibrationTest() {
  Serial.println("Lift robot: running low-range alternating leg test.");
  for (uint8_t i = 0; i < WALK_CYCLES; i++) {
    // Diagonal group A: R1/L2, then group B: R2/L1.
    setPose(105, 90, 90, 105, 90, 105, 90, 90);
    delay(STEP_MS);

    setPose(90, 90, 90, 90, 90, 90, 90, 90);
    delay(STEP_MS);

    setPose(90, 105, 105, 90, 90, 90, 90, 90);
    delay(STEP_MS);

    setPose(90, 90, 90, 90, 90, 90, 90, 90);
    delay(STEP_MS);

    // Other diagonal group: R4/L3 and R3/L4.
    setPose(90, 90, 90, 90, 105, 90, 105, 90);
    delay(STEP_MS);

    setPose(90, 90, 90, 90, 90, 90, 90, 90);
    delay(STEP_MS);

    setPose(90, 90, 90, 90, 90, 105, 90, 105);
    delay(STEP_MS);

    setPose(90, 90, 90, 90, 90, 90, 90, 90);
    delay(STEP_MS);
  }
  runRestPose();
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  rest         - all servos to 90 degrees");
  Serial.println("  stand        - neutral reference pose");
  Serial.println("  test         - low-range alternating channel test (robot lifted)");
  Serial.println("  servo N A    - set channel N to angle A, e.g. servo 0 95");
  Serial.println("  help         - show commands");
  Serial.println();
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "rest") {
    runRestPose();
  } else if (cmd == "stand") {
    runStandPose();
  } else if (cmd == "test") {
    runWalkCalibrationTest();
  } else if (cmd == "help") {
    printHelp();
  } else if (cmd.startsWith("servo ")) {
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    if (secondSpace < 0) {
      Serial.println("Use: servo N A   (example: servo 0 95)");
      return;
    }

    int channel = cmd.substring(firstSpace + 1, secondSpace).toInt();
    int angle = cmd.substring(secondSpace + 1).toInt();

    if (channel < 0 || channel >= SERVO_COUNT) {
      Serial.println("Channel must be 0..7");
      return;
    }
    if (angle < ANGLE_LIMIT_LOW || angle > ANGLE_LIMIT_HIGH) {
      Serial.println("Calibration range is limited to 60..120 degrees.");
      return;
    }

    setServoAngle((uint8_t)channel, angle);
    Serial.print("Channel ");
    Serial.print(channel);
    Serial.print(" commanded to ");
    Serial.println(angle);
  } else {
    Serial.println("Unknown command. Type help.");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(500);

  runRestPose();
  printHelp();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }
}

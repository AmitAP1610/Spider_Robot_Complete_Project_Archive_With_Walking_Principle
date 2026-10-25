#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// QUADROPAD calibrated servo controller
// ESP32: SDA=GPIO21, SCL=GPIO22
// PCA9685 I2C address: 0x40
// CH0=R1, CH1=R2, CH2=L1, CH3=L2,
// CH4=R4, CH5=R3, CH6=L3, CH7=L4

Adafruit_PWMServoDriver pwm(0x40);

const uint8_t SERVO_COUNT = 8;
const uint8_t R1 = 0, R2 = 1, L1 = 2, L2 = 3;
const uint8_t R4 = 4, R3 = 5, L3 = 6, L4 = 7;

// Initial MG90S pulse range at 50 Hz. Adjust only after careful testing.
int pulseMin = 110;
int pulseMax = 510;

// All servos use 90 degrees as the initial neutral reference.
int neutralAngle[SERVO_COUNT] = {90, 90, 90, 90, 90, 90, 90, 90};

// Direction measured by the user:
// Inward on increasing requested angle: channels 0, 2, 4, 6
// Outward on increasing requested angle: channels 1, 3, 5, 7
int direction[SERVO_COUNT] = {
  -1,  // CH0: R1
   1,  // CH1: R2
  -1,  // CH2: L1
   1,  // CH3: L2
  -1,  // CH4: R4
   1,  // CH5: R3
  -1,  // CH6: L3
   1   // CH7: L4
};

const int SAFE_MIN_ANGLE = 70;
const int SAFE_MAX_ANGLE = 110;
const int MOVE_DELAY_MS = 350;

int lastAngle[SERVO_COUNT] = {90, 90, 90, 90, 90, 90, 90, 90};

uint16_t angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return (uint16_t)map(angle, 0, 180, pulseMin, pulseMax);
}

void writePhysicalAngle(uint8_t channel, int angle) {
  if (channel >= SERVO_COUNT) return;
  angle = constrain(angle, 0, 180);
  pwm.setPWM(channel, 0, angleToPulse(angle));
}

void setCalibratedAngle(uint8_t channel, int requestedAngle) {
  if (channel >= SERVO_COUNT) {
    Serial.println("ERROR: channel must be 0..7");
    return;
  }

  requestedAngle = constrain(requestedAngle, SAFE_MIN_ANGLE, SAFE_MAX_ANGLE);

  int physicalAngle = neutralAngle[channel]
                    + direction[channel] * (requestedAngle - 90);

  physicalAngle = constrain(physicalAngle, SAFE_MIN_ANGLE, SAFE_MAX_ANGLE);
  writePhysicalAngle(channel, physicalAngle);
  lastAngle[channel] = physicalAngle;
}

void moveAllNeutral() {
  for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
    writePhysicalAngle(ch, neutralAngle[ch]);
    lastAngle[ch] = neutralAngle[ch];
  }
  delay(500);
}

void printHelp() {
  Serial.println();
  Serial.println("Commands (Serial Monitor: 115200 baud, Newline):");
  Serial.println("  neutral           -> all servos to their neutral angles");
  Serial.println("  servo <ch> <deg>  -> test one channel, e.g. servo 0 95");
  Serial.println("  dir <ch> <1|-1>   -> reverse direction in software");
  Serial.println("  offset <ch> <deg> -> set channel neutral angle");
  Serial.println("  test              -> small alternating motion test; robot lifted");
  Serial.println("  help              -> show this menu");
  Serial.println("Calibration angle limits are currently 70..110 degrees.");
  Serial.println();
}

void runSmallAlternatingTest() {
  Serial.println("TEST: keep robot lifted and stop if a servo binds.");

  for (int cycle = 0; cycle < 2; cycle++) {
    setCalibratedAngle(R1, 105);
    setCalibratedAngle(L2, 105);
    delay(MOVE_DELAY_MS);
    moveAllNeutral();

    setCalibratedAngle(R2, 105);
    setCalibratedAngle(L1, 105);
    delay(MOVE_DELAY_MS);
    moveAllNeutral();

    setCalibratedAngle(R4, 105);
    setCalibratedAngle(L3, 105);
    delay(MOVE_DELAY_MS);
    moveAllNeutral();

    setCalibratedAngle(R3, 105);
    setCalibratedAngle(L4, 105);
    delay(MOVE_DELAY_MS);
    moveAllNeutral();
  }

  Serial.println("Test complete. This is a calibration test, not a walking gait.");
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "neutral" || cmd == "rest" || cmd == "stand") {
    moveAllNeutral();
    Serial.println("Neutral pose applied.");
    return;
  }

  if (cmd == "test") {
    runSmallAlternatingTest();
    return;
  }

  if (cmd == "help") {
    printHelp();
    return;
  }

  if (cmd.startsWith("servo ")) {
    int p1 = cmd.indexOf(' ');
    int p2 = cmd.indexOf(' ', p1 + 1);

    if (p2 < 0) {
      Serial.println("Use: servo <channel> <angle>, e.g. servo 0 95");
      return;
    }

    int ch = cmd.substring(p1 + 1, p2).toInt();
    int angle = cmd.substring(p2 + 1).toInt();

    if (ch < 0 || ch >= SERVO_COUNT) {
      Serial.println("Channel must be 0..7");
      return;
    }

    if (angle < SAFE_MIN_ANGLE || angle > SAFE_MAX_ANGLE) {
      Serial.println("For safety, use 70..110 degrees during calibration.");
      return;
    }

    setCalibratedAngle((uint8_t)ch, angle);
    Serial.print("Channel ");
    Serial.print(ch);
    Serial.print(" set to requested angle ");
    Serial.println(angle);
    return;
  }

  if (cmd.startsWith("offset ")) {
    int p1 = cmd.indexOf(' ');
    int p2 = cmd.indexOf(' ', p1 + 1);

    if (p2 < 0) {
      Serial.println("Use: offset <channel> <neutral angle>");
      return;
    }

    int ch = cmd.substring(p1 + 1, p2).toInt();
    int angle = cmd.substring(p2 + 1).toInt();

    if (ch < 0 || ch >= SERVO_COUNT ||
        angle < SAFE_MIN_ANGLE || angle > SAFE_MAX_ANGLE) {
      Serial.println("Use channel 0..7 and neutral angle 70..110.");
      return;
    }

    neutralAngle[ch] = angle;
    writePhysicalAngle((uint8_t)ch, angle);
    lastAngle[ch] = angle;

    Serial.print("Neutral angle for channel ");
    Serial.print(ch);
    Serial.print(" is now ");
    Serial.println(angle);
    return;
  }

  if (cmd.startsWith("dir ")) {
    int p1 = cmd.indexOf(' ');
    int p2 = cmd.indexOf(' ', p1 + 1);

    if (p2 < 0) {
      Serial.println("Use: dir <channel> <1|-1>");
      return;
    }

    int ch = cmd.substring(p1 + 1, p2).toInt();
    int dir = cmd.substring(p2 + 1).toInt();

    if (ch < 0 || ch >= SERVO_COUNT || (dir != 1 && dir != -1)) {
      Serial.println("Use channel 0..7 and direction 1 or -1.");
      return;
    }

    direction[ch] = dir;
    Serial.print("Direction for channel ");
    Serial.print(ch);
    Serial.print(" set to ");
    Serial.println(dir);
    return;
  }

  Serial.println("Unknown command. Type help.");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(500);

  moveAllNeutral();
  Serial.println("QUADROPAD servo calibration controller ready.");
  printHelp();
}

void loop() {
  if (Serial.available()) {
    processCommand(Serial.readStringUntil('\n'));
  }
}

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BluetoothSerial.h"

// QUADROPAD mobile control over Bluetooth Classic.
// For original ESP32 / ESP32-WROOM boards; not ESP32-S2/S3/C3.
// SDA=GPIO21, SCL=GPIO22, PCA9685 address=0x40
// CH0=R1, CH1=R2, CH2=L1, CH3=L2, CH4=R4, CH5=R3, CH6=L3, CH7=L4

Adafruit_PWMServoDriver pwm(0x40);
BluetoothSerial SerialBT;

const uint8_t SERVO_COUNT = 8;
const uint8_t R1=0, R2=1, L1=2, L2=3, R4=4, R3=5, L3=6, L4=7;
int pulseMin = 110;
int pulseMax = 510;
int neutralAngle[SERVO_COUNT] = {90,90,90,90,90,90,90,90};

// User-observed directions: inward channels 0,2,4,6; outward 1,3,5,7.
int direction[SERVO_COUNT] = {-1,1,-1,1,-1,1,-1,1};
const int SAFE_MIN_ANGLE = 70;
const int SAFE_MAX_ANGLE = 110;
const int MOVE_DELAY_MS = 350;
int lastAngle[SERVO_COUNT] = {90,90,90,90,90,90,90,90};

uint16_t angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return (uint16_t)map(angle, 0, 180, pulseMin, pulseMax);
}

void writePhysicalAngle(uint8_t channel, int angle) {
  if (channel >= SERVO_COUNT) return;
  pwm.setPWM(channel, 0, angleToPulse(constrain(angle, 0, 180)));
}

void sendMessage(const String &message) {
  Serial.println(message);
  if (SerialBT.hasClient()) SerialBT.println(message);
}

void setCalibratedAngle(uint8_t channel, int requestedAngle) {
  if (channel >= SERVO_COUNT) {
    sendMessage("ERROR: channel must be 0..7");
    return;
  }
  requestedAngle = constrain(requestedAngle, SAFE_MIN_ANGLE, SAFE_MAX_ANGLE);
  int physicalAngle = neutralAngle[channel] + direction[channel] * (requestedAngle - 90);
  physicalAngle = constrain(physicalAngle, SAFE_MIN_ANGLE, SAFE_MAX_ANGLE);
  writePhysicalAngle(channel, physicalAngle);
  lastAngle[channel] = physicalAngle;
}

void moveAllNeutral() {
  for (uint8_t ch=0; ch<SERVO_COUNT; ch++) {
    writePhysicalAngle(ch, neutralAngle[ch]);
    lastAngle[ch] = neutralAngle[ch];
  }
  delay(500);
}

void printHelp() {
  sendMessage("QUADROPAD Bluetooth commands:");
  sendMessage("neutral or rest or stop");
  sendMessage("servo <channel 0-7> <angle 70-110>, e.g. servo 0 100");
  sendMessage("dir <channel 0-7> <1 or -1>");
  sendMessage("offset <channel 0-7> <angle 70-110>");
  sendMessage("test (robot lifted only)");
  sendMessage("help");
}

void runSmallAlternatingTest() {
  sendMessage("TEST: lift robot; stop if any servo binds.");
  for (int cycle=0; cycle<2; cycle++) {
    setCalibratedAngle(R1,105); setCalibratedAngle(L2,105);
    delay(MOVE_DELAY_MS); moveAllNeutral();
    setCalibratedAngle(R2,105); setCalibratedAngle(L1,105);
    delay(MOVE_DELAY_MS); moveAllNeutral();
    setCalibratedAngle(R4,105); setCalibratedAngle(L3,105);
    delay(MOVE_DELAY_MS); moveAllNeutral();
    setCalibratedAngle(R3,105); setCalibratedAngle(L4,105);
    delay(MOVE_DELAY_MS); moveAllNeutral();
  }
  sendMessage("Test complete. Calibration test only, not a walking gait.");
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (!cmd.length()) return;

  if (cmd=="neutral" || cmd=="rest" || cmd=="stand" || cmd=="stop") {
    moveAllNeutral();
    sendMessage("Neutral pose applied.");
    return;
  }
  if (cmd=="test") { runSmallAlternatingTest(); return; }
  if (cmd=="help") { printHelp(); return; }

  if (cmd.startsWith("servo ")) {
    int p1=cmd.indexOf(' ');
    int p2=cmd.indexOf(' ',p1+1);
    if (p2<0) { sendMessage("Use: servo <channel> <angle>"); return; }
    int ch=cmd.substring(p1+1,p2).toInt();
    int angle=cmd.substring(p2+1).toInt();
    if (ch<0 || ch>=SERVO_COUNT) { sendMessage("Channel must be 0..7"); return; }
    if (angle<SAFE_MIN_ANGLE || angle>SAFE_MAX_ANGLE) {
      sendMessage("Angle must be 70..110 during calibration."); return;
    }
    setCalibratedAngle((uint8_t)ch,angle);
    sendMessage("Channel "+String(ch)+" set to requested angle "+String(angle));
    return;
  }

  if (cmd.startsWith("offset ")) {
    int p1=cmd.indexOf(' ');
    int p2=cmd.indexOf(' ',p1+1);
    if (p2<0) { sendMessage("Use: offset <channel> <neutral angle>"); return; }
    int ch=cmd.substring(p1+1,p2).toInt();
    int angle=cmd.substring(p2+1).toInt();
    if (ch<0 || ch>=SERVO_COUNT || angle<SAFE_MIN_ANGLE || angle>SAFE_MAX_ANGLE) {
      sendMessage("Use channel 0..7 and angle 70..110."); return;
    }
    neutralAngle[ch]=angle;
    writePhysicalAngle((uint8_t)ch,angle);
    lastAngle[ch]=angle;
    sendMessage("Neutral angle for channel "+String(ch)+" is now "+String(angle));
    return;
  }

  if (cmd.startsWith("dir ")) {
    int p1=cmd.indexOf(' ');
    int p2=cmd.indexOf(' ',p1+1);
    if (p2<0) { sendMessage("Use: dir <channel> <1|-1>"); return; }
    int ch=cmd.substring(p1+1,p2).toInt();
    int dir=cmd.substring(p2+1).toInt();
    if (ch<0 || ch>=SERVO_COUNT || (dir!=1 && dir!=-1)) {
      sendMessage("Use channel 0..7 and direction 1 or -1."); return;
    }
    direction[ch]=dir;
    sendMessage("Direction for channel "+String(ch)+" set to "+String(dir));
    return;
  }
  sendMessage("Unknown command. Send help.");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  SerialBT.begin("QUADROPAD");
  Wire.begin(21,22);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(500);
  moveAllNeutral();
  sendMessage("QUADROPAD ready. Pair phone with Bluetooth device QUADROPAD.");
  printHelp();
}

void loop() {
  if (Serial.available()) processCommand(Serial.readStringUntil('\n'));
  if (SerialBT.available()) processCommand(SerialBT.readStringUntil('\n'));
}

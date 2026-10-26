#include "ServoDriver.h"

ServoDriver::ServoDriver()
    : m_pwm(0x40),
      m_i2cMutex(nullptr),
      m_i2cAddr(0x40),
      m_sdaPin(21),
      m_sclPin(22),
      m_connected(false),
      m_lastReconnectAttemptMs(0)
{
    for (int i = 0; i < 12; i++) {
        m_currentAngleDeg[i] = g_servoCalibration[i].centerAngleDeg;
        m_targetAngleDeg[i]  = g_servoCalibration[i].centerAngleDeg;
    }
}

bool ServoDriver::init(uint8_t sdaPin, uint8_t sclPin, uint8_t i2cAddr) {
    m_sdaPin = sdaPin;
    m_sclPin = sclPin;
    m_i2cAddr = i2cAddr;

    Wire.begin(m_sdaPin, m_sclPin);
    Wire.setTimeOut(50);   // Prevent bus freeze if jumper wires disconnect
    Wire.setClock(400000); // 400 kHz Fast-Mode I2C for fast OLED and PCA9685 updates

    // Multi-attempt probe on configured address
    bool found = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        Wire.beginTransmission(m_i2cAddr);
        if (Wire.endTransmission() == 0) {
            found = true;
            break;
        }
        delay(40);
    }

    // If primary address failed, scan PCA9685 address range (0x40 to 0x47)
    if (!found) {
        for (uint8_t addr = 0x40; addr <= 0x47; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                m_i2cAddr = addr;
                found = true;
                Serial.printf(">> [I2C AUTO-DETECT] PCA9685 found at alternate address 0x%02X!\n", m_i2cAddr);
                break;
            }
        }
    }

    if (!found) {
        m_connected = false;
        Serial.printf("\n>> [I2C ERROR] PCA9685 not detected at address 0x%02X on SDA=%u, SCL=%u!\n",
                      i2cAddr, m_sdaPin, m_sclPin);
        scanI2CBus();
        return false;
    }

    m_pwm = Adafruit_PWMServoDriver(m_i2cAddr);
    m_pwm.begin();
    m_pwm.setPWMFreq(ServoCalibration::PWM_FREQUENCY_HZ);
    m_pwm.setOutputMode(true); // Force MODE2 OUTDRV=1 (Totem-Pole active high push-pull drive)
    m_pwm.wakeup();            // Ensure oscillator SLEEP bit is cleared
    delay(20);

    // Read and verify PCA9685 hardware configuration registers directly over I2C
    Wire.beginTransmission(m_i2cAddr);
    Wire.write(0x00); // MODE1 register
    Wire.endTransmission();
    if (Wire.requestFrom((uint8_t)m_i2cAddr, (uint8_t)2) == 2) {
        uint8_t mode1 = Wire.read();
        uint8_t mode2 = Wire.read();
        Serial.printf(">> [PCA9685 HARDWARE] MODE1: 0x%02X (%s), MODE2: 0x%02X (%s)\n",
                      mode1, (mode1 & 0x10) ? "SLEEP - Clearing..." : "OSC RUNNING",
                      mode2, (mode2 & 0x04) ? "TOTEM-POLE ACTIVE HIGH (OK)" : "OPEN-DRAIN (Warning!)");

        // Extra guarantee: if SLEEP bit is still set, clear it explicitly
        if (mode1 & 0x10) {
            Wire.beginTransmission(m_i2cAddr);
            Wire.write(0x00);
            Wire.write(mode1 & ~0x10);
            Wire.endTransmission();
            delay(10);
        }
    }

    m_connected = true;

    // Attempt to load saved calibration from ESP32 NVS flash
    if (loadCalibration()) {
        Serial.println(">> [NVS] Loaded saved servo calibration from flash.");
    } else {
        Serial.println(">> [NVS] Using factory default calibration.");
    }

    // Staggered soft stand-up sequence to prevent inrush current brownouts
    commandStandPoseSmooth(600);

    return true;
}

void ServoDriver::update(float dt, const LegJointAngles targetAngles[4]) {
    if (!m_connected) return;

    // Slew-rate maximum angle change for this timestep
    float maxDeltaDeg = GaitConfig::MAX_JOINT_SPEED_DEG_PER_SEC * dt;
    if (maxDeltaDeg < 0.1f) maxDeltaDeg = 0.1f;

    // Acquire shared I2C bus mutex before writing to PCA9685
    if (m_i2cMutex != nullptr) {
        if (xSemaphoreTake(m_i2cMutex, pdMS_TO_TICKS(15)) != pdTRUE) {
            return; // Bus temporarily occupied by OLED; skip this 20ms slice
        }
    }

    for (uint8_t leg = 0; leg < 4; leg++) {
        float modelAngles[3] = {
            targetAngles[leg].coxaDeg,
            targetAngles[leg].femurDeg,
            targetAngles[leg].tibiaDeg
        };

        for (uint8_t joint = 0; joint < 3; joint++) {
            uint8_t idx = leg * 3 + joint;

            // Convert kinematic model angle to physical servo command angle
            float cmdTarget = ServoCalibration::modelAngleToCommandAngle(idx, modelAngles[joint]);
            m_targetAngleDeg[idx] = cmdTarget;

            // Slew-rate rate limiting
            float current = m_currentAngleDeg[idx];
            float diff = cmdTarget - current;

            if (fabsf(diff) <= maxDeltaDeg) {
                current = cmdTarget;
            } else if (diff > 0.0f) {
                current += maxDeltaDeg;
            } else {
                current -= maxDeltaDeg;
            }

            m_currentAngleDeg[idx] = current;
            writeServoHardware(idx, current);
        }
    }

    if (m_i2cMutex != nullptr) {
        xSemaphoreGive(m_i2cMutex);
    }
}

void ServoDriver::setJointAngleDirect(uint8_t jointIndex, float angleDeg) {
    if (jointIndex >= 12) return;
    const auto& cal = g_servoCalibration[jointIndex];

    float safeAngle = constrain(angleDeg, cal.minAngleDeg, cal.maxAngleDeg);
    m_targetAngleDeg[jointIndex] = safeAngle;
    m_currentAngleDeg[jointIndex] = safeAngle;
    writeServoHardware(jointIndex, safeAngle);
}

void ServoDriver::commandStandPoseDirect() {
    commandStandPoseSmooth(300);
}

void ServoDriver::commandStandPoseSmooth(uint16_t durationMs) {
    if (!m_connected) return;

    // Stagger activation leg-by-leg with small pauses to prevent simultaneous inrush current spike
    for (uint8_t leg = 0; leg < 4; leg++) {
        for (uint8_t j = 0; j < 3; j++) {
            uint8_t idx = leg * 3 + j;
            m_targetAngleDeg[idx]  = g_servoCalibration[idx].centerAngleDeg;
            m_currentAngleDeg[idx] = g_servoCalibration[idx].centerAngleDeg;
            writeServoHardware(idx, m_currentAngleDeg[idx]);
        }
        delay(35); // 35 ms pause between legs spreads the current draw safely
    }
}

void ServoDriver::writeAll16Channels(float angleDeg) {
    if (!m_connected) return;
    float safeAngle = constrain(angleDeg, 0.0f, 180.0f);
    float pulseUs = 500.0f + (safeAngle / 180.0f) * 2000.0f;
    uint16_t tick = ServoCalibration::pulseUsToTicks(pulseUs);

    for (uint8_t ch = 0; ch < 16; ch++) {
        m_pwm.setPWM(ch, 0, tick);
    }
}

bool ServoDriver::autoReconnect() {
    if (m_connected) return true;

    uint32_t now = millis();
    if (now - m_lastReconnectAttemptMs < 2000) {
        return false;
    }
    m_lastReconnectAttemptMs = now;

    // Check primary address or scan 0x40..0x47
    Wire.beginTransmission(m_i2cAddr);
    if (Wire.endTransmission() == 0) {
        Serial.printf("\n>> [I2C RECONNECT] PCA9685 detected at address 0x%02X! Reinitializing...\n", m_i2cAddr);
        return init(m_sdaPin, m_sclPin, m_i2cAddr);
    }

    for (uint8_t addr = 0x40; addr <= 0x47; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("\n>> [I2C RECONNECT] PCA9685 detected at address 0x%02X! Reinitializing...\n", addr);
            return init(m_sdaPin, m_sclPin, addr);
        }
    }

    return false;
}

void ServoDriver::scanI2CBus() {
    Serial.println();
    Serial.println("==================================================================");
    Serial.printf("             I2C BUS SCAN (SDA=GPIO %u, SCL=GPIO %u)              \n", m_sdaPin, m_sclPin);
    Serial.println("==================================================================");

    uint8_t devicesFound = 0;
    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.printf("  [FOUND] I2C Device at 0x%02X", address);
            if (address >= 0x40 && address <= 0x47) {
                Serial.print("  <-- PCA9685 PWM Servo Driver");
            } else if (address == 0x70) {
                Serial.print("  <-- PCA9685 All-Call Address");
            }
            Serial.println();
            devicesFound++;
        }
    }

    if (devicesFound == 0) {
        Serial.println("  [ERROR] NO I2C DEVICES FOUND ON THE BUS!");
        Serial.println("  Please inspect hardware connections:");
        Serial.println("    1) PCA9685 VCC (logic pin) must connect to ESP32 3.3V.");
        Serial.println("    2) PCA9685 GND must connect to ESP32 GND (common ground).");
        Serial.println("    3) PCA9685 SDA must connect to ESP32 GPIO 21.");
        Serial.println("    4) PCA9685 SCL must connect to ESP32 GPIO 22.");
        Serial.println("    5) Make sure SDA and SCL jumper wires are not swapped or loose.");
    } else {
        Serial.printf("Scan complete. Total devices detected: %u\n", devicesFound);
    }
    Serial.println("==================================================================");
    Serial.println();
}

void ServoDriver::runHardwareDiagnosticTest() {
    if (!m_connected) {
        Serial.println(">> [HARDWARE TEST FAILED] PCA9685 is not connected! Run 'SCAN' to check I2C.");
        return;
    }

    Serial.println();
    Serial.println("==================================================================");
    Serial.println("               SEQUENTIAL SERVO HARDWARE MOTION TEST              ");
    Serial.println("==================================================================");
    Serial.println("Testing all 12 servos one-by-one with clear +/-20 degree motion.");
    Serial.println("Watch each joint move to confirm wiring and mechanical assembly:");

    for (uint8_t i = 0; i < 12; i++) {
        const auto& cal = g_servoCalibration[i];
        float center = cal.centerAngleDeg;
        float aPlus  = constrain(center + 20.0f, cal.minAngleDeg, cal.maxAngleDeg);
        float aMinus = constrain(center - 20.0f, cal.minAngleDeg, cal.maxAngleDeg);
        uint16_t tickPlus  = ServoCalibration::angleToTicks(i, aPlus);
        uint16_t tickMinus = ServoCalibration::angleToTicks(i, aMinus);
        uint16_t tickCenter = ServoCalibration::angleToTicks(i, center);

        Serial.printf("  Testing Joint %2u (Ch %2u, %-8s): [Center: %.0f deg / %u ticks] -> [Plus: %.0f deg / %u ticks] -> [Minus: %.0f deg / %u ticks] ... ",
                      i, cal.channel, cal.name, center, tickCenter, aPlus, tickPlus, aMinus, tickMinus);

        writeServoHardware(i, aPlus);
        delay(300);
        writeServoHardware(i, aMinus);
        delay(300);
        writeServoHardware(i, center);
        delay(150);

        Serial.println("[DONE]");
    }

    Serial.println("------------------------------------------------------------------");
    Serial.println("Hardware test complete. All 12 servos returned to center.");
    Serial.println();
    Serial.println(">> CRITICAL HARDWARE CHECK (If NO servos moved at all):");
    Serial.println("   1. PCA9685 'OE' PIN: Connect a jumper wire between 'OE' and 'GND' pins");
    Serial.println("      on the PCA9685 header. If OE is floating or HIGH, ALL outputs are disabled!");
    Serial.println("   2. SERVO PLUG ORIENTATION (Check for upside-down plugs!):");
    Serial.println("      - Black / Brown wire  --> GND (bottom row)");
    Serial.println("      - Red wire            --> V+  (center row)");
    Serial.println("      - Yellow / Orange wire --> PWM Signal (top row)");
    Serial.println("   3. SERVO BATTERY POWER:");
    Serial.println("      - Verify external 5V-6V battery connected to PCA9685 screw terminal.");
    Serial.println("      - Ensure screw is tightened onto bare metal wire, not plastic insulation.");
    Serial.println("==================================================================");
    Serial.println();
}

float ServoDriver::getCurrentAngle(uint8_t jointIndex) const {
    if (jointIndex >= 12) return 90.0f;
    return m_currentAngleDeg[jointIndex];
}

float ServoDriver::getTargetAngle(uint8_t jointIndex) const {
    if (jointIndex >= 12) return 90.0f;
    return m_targetAngleDeg[jointIndex];
}

void ServoDriver::writeServoHardware(uint8_t jointIndex, float angleDeg) {
    if (jointIndex >= 12 || !m_connected) return;

    uint16_t tick = ServoCalibration::angleToTicks(jointIndex, angleDeg);
    m_pwm.setPWM(g_servoCalibration[jointIndex].channel, 0, tick);
}

void ServoDriver::jogJoint(uint8_t jointIndex, float deltaDeg) {
    if (jointIndex >= 12) return;
    float newAngle = m_currentAngleDeg[jointIndex] + deltaDeg;
    setJointAngleDirect(jointIndex, newAngle);
}

void ServoDriver::setJointCenterFromCurrent(uint8_t jointIndex) {
    if (jointIndex >= 12) return;
    g_servoCalibration[jointIndex].centerAngleDeg = m_currentAngleDeg[jointIndex];
}

void ServoDriver::setJointCenterAngle(uint8_t jointIndex, float centerAngleDeg) {
    if (jointIndex >= 12) return;
    const auto& cal = g_servoCalibration[jointIndex];
    float safeCenter = constrain(centerAngleDeg, cal.minAngleDeg, cal.maxAngleDeg);
    g_servoCalibration[jointIndex].centerAngleDeg = safeCenter;
}

void ServoDriver::captureAllCenters() {
    for (uint8_t i = 0; i < 12; i++) {
        g_servoCalibration[i].centerAngleDeg = m_currentAngleDeg[i];
    }
}

bool ServoDriver::saveCalibration() {
    bool ok = ServoCalibration::saveToNVS();
    if (ok) {
        Serial.println(">> [NVS SAVE SUCCESS] All 12 servo calibration parameters saved to ESP32 flash memory!");
    } else {
        Serial.println(">> [NVS SAVE FAILED] Error writing calibration data to flash memory!");
    }
    return ok;
}

bool ServoDriver::loadCalibration() {
    bool ok = ServoCalibration::loadFromNVS();
    if (ok) {
        // Sync RAM target and current angles with loaded centers
        for (uint8_t i = 0; i < 12; i++) {
            m_targetAngleDeg[i]  = g_servoCalibration[i].centerAngleDeg;
            m_currentAngleDeg[i] = g_servoCalibration[i].centerAngleDeg;
            if (m_connected) {
                writeServoHardware(i, m_currentAngleDeg[i]);
            }
        }
    }
    return ok;
}

void ServoDriver::restoreDefaultCalibration() {
    ServoCalibration::restoreDefaults();
    for (uint8_t i = 0; i < 12; i++) {
        m_targetAngleDeg[i]  = g_servoCalibration[i].centerAngleDeg;
        m_currentAngleDeg[i] = g_servoCalibration[i].centerAngleDeg;
        if (m_connected) {
            writeServoHardware(i, m_currentAngleDeg[i]);
        }
    }
}

void ServoDriver::toggleJointDirection(uint8_t jointIndex) {
    if (jointIndex >= 12) return;
    g_servoCalibration[jointIndex].direction = -g_servoCalibration[jointIndex].direction;
}

void ServoDriver::performJointSweepTest(uint8_t jointIndex) {
    if (jointIndex >= 12 || !m_connected) return;
    const auto& cal = g_servoCalibration[jointIndex];

    Serial.print("Testing safe sweep on joint ");
    Serial.print(jointIndex);
    Serial.print(" (");
    Serial.print(cal.name);
    Serial.println(")...");

    float orig = m_currentAngleDeg[jointIndex];

    // Sweep from current to max in gentle steps
    for (float a = orig; a <= cal.maxAngleDeg; a += 1.0f) {
        setJointAngleDirect(jointIndex, a);
        delay(20);
    }
    delay(200);

    // Sweep from max to min
    for (float a = cal.maxAngleDeg; a >= cal.minAngleDeg; a -= 1.0f) {
        setJointAngleDirect(jointIndex, a);
        delay(20);
    }
    delay(200);

    // Return to original angle
    for (float a = cal.minAngleDeg; a <= orig; a += 1.0f) {
        setJointAngleDirect(jointIndex, a);
        delay(20);
    }

    Serial.println("Sweep test complete. Returned to original pose.");
}

void ServoDriver::printCalibrationTable() {
    Serial.println();
    Serial.println("==========================================================================");
    Serial.println("                     SERVO CALIBRATION TABLE                              ");
    Serial.println("==========================================================================");
    Serial.println("Idx | Ch | Joint Name | Center | Dir | Safe Min | Safe Max | CurAngle | Tick");
    Serial.println("----+----+------------+--------+-----+----------+----------+----------+-----");

    for (uint8_t i = 0; i < 12; i++) {
        const auto& c = g_servoCalibration[i];
        uint16_t tick = ServoCalibration::angleToTicks(i, m_currentAngleDeg[i]);

        char buf[128];
        snprintf(buf, sizeof(buf), " %2u | %2u | %-10s | %6.1f | %+2d  |  %6.1f  |  %6.1f  |  %6.1f  | %4u",
                 i, c.channel, c.name, c.centerAngleDeg, c.direction,
                 c.minAngleDeg, c.maxAngleDeg, m_currentAngleDeg[i], tick);
        Serial.println(buf);
    }
    Serial.println("--------------------------------------------------------------------------");
    bool hasSaved = ServoCalibration::hasSavedCalibration();
    Serial.printf(" Storage Status: %s in ESP32 Flash (NVS)\n",
                  hasSaved ? "VALID CALIBRATION STORED" : "NO SAVED DATA (Using Defaults)");
    Serial.println(" Commands: 'SAVE' to persist | 'LOAD' to reload | 'DEFAULT' to reset");
    Serial.println("==========================================================================");
}


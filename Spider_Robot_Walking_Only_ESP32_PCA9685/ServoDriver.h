#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "CalibrationConfig.h"
#include "GaitConfig.h"
#include "Kinematics.h"

/**
 * ============================================================================
 * SERVO DRIVER MODULE
 * ============================================================================
 * Interfaces the ESP32 with the PCA9685 16-channel PWM driver at 0x40 over I2C.
 * 
 * Pinout:
 *   ESP32 SDA : GPIO 21
 *   ESP32 SCL : GPIO 22
 *   PCA9685 I2C Address : 0x40
 *   PWM Frequency : 50 Hz
 * 
 * Safety & Features:
 *   - Per-joint rate limiting (slew rate limiter) prevents mechanical shock.
 *   - Clamping to calibrated physical angle limits and hardware PWM tick limits.
 *   - Interactive single-servo jog and calibration testing.
 *   - I2C communication verification.
 * ============================================================================
 */

class ServoDriver {
public:
    ServoDriver();

    /**
     * Initialize Wire I2C bus and PCA9685 servo driver.
     * @return true if PCA9685 responded at 0x40, false if communication failed.
     */
    bool init(uint8_t sdaPin = 21, uint8_t sclPin = 22, uint8_t i2cAddr = 0x40);

    /**
     * Update all 12 servos toward their target kinematic angles using slew-rate limiting.
     * @param dt Time elapsed since last update in seconds.
     * @param targetAngles Array of 4 LegJointAngles computed from IK.
     */
    void update(float dt, const LegJointAngles targetAngles[4]);

    /**
     * Set a single servo directly to an angle (used during calibration mode).
     * @param jointIndex 0..11
     * @param angleDeg Commanded angle in degrees
     */
    void setJointAngleDirect(uint8_t jointIndex, float angleDeg);

    /**
     * Command all servos to their calibrated standing center pose.
     */
    void commandStandPoseDirect();

    /**
     * Force all 16 PCA9685 channels (0 through 15) to a specific angle (e.g. 90 deg).
     */
    void writeAll16Channels(float angleDeg = 90.0f);

    /**
     * Get current filtered angle of a joint.
     */
    float getCurrentAngle(uint8_t jointIndex) const;

    /**
     * Get current target angle of a joint.
     */
    float getTargetAngle(uint8_t jointIndex) const;

    /**
     * Check if PCA9685 is detected on the I2C bus.
     */
    bool isConnected() const { return m_connected; }
    uint8_t getI2CAddress() const { return m_i2cAddr; }

    /**
     * Automatic re-connection handler (call periodically if !isConnected()).
     * @return true if reconnected, false if still waiting for hardware.
     */
    bool autoReconnect();

    /**
     * Run full I2C bus scan across all 127 addresses and print report over Serial.
     */
    void scanI2CBus();

    /**
     * Command all servos to their calibrated standing center pose smoothly (staggered to avoid inrush current).
     * @param durationMs Duration in ms for smooth transition.
     */
    void commandStandPoseSmooth(uint16_t durationMs = 800);

    /**
     * Run sequential hardware motion diagnostic test across all 12 servos.
     */
    void runHardwareDiagnosticTest();

    /**
     * Calibration & Permanent NVS Storage helpers
     */
    void printCalibrationTable();
    void jogJoint(uint8_t jointIndex, float deltaDeg);
    void setJointCenterFromCurrent(uint8_t jointIndex);
    void toggleJointDirection(uint8_t jointIndex);
    void performJointSweepTest(uint8_t jointIndex);

    bool saveCalibration();
    bool loadCalibration();
    void restoreDefaultCalibration();
    void captureAllCenters();
    void setJointCenterAngle(uint8_t jointIndex, float centerAngleDeg);

    /**
     * Set FreeRTOS mutex for thread-safe shared I2C bus arbitration.
     */
    void setI2CMutex(SemaphoreHandle_t mutex) { m_i2cMutex = mutex; }

private:
    Adafruit_PWMServoDriver m_pwm;
    SemaphoreHandle_t m_i2cMutex;
    uint8_t m_i2cAddr;
    uint8_t m_sdaPin;
    uint8_t m_sclPin;
    bool m_connected;
    uint32_t m_lastReconnectAttemptMs;

    float m_currentAngleDeg[12];
    float m_targetAngleDeg[12];

    void writeServoHardware(uint8_t jointIndex, float angleDeg);
};

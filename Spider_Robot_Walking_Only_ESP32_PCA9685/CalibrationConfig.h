#pragma once
#include <Arduino.h>
#include <Preferences.h>

/**
 * ============================================================================
 * SERVO CALIBRATION CONFIGURATION & PERMANENT NVS STORAGE
 * ============================================================================
 * Individual calibration parameters for all 12 servos on the quadruped robot.
 * 
 * Channels on PCA9685:
 *   Leg 0 (Front Left)  : Coxa = 0,  Femur = 1,  Tibia = 2
 *   Leg 1 (Front Right) : Coxa = 4,  Femur = 5,  Tibia = 6
 *   Leg 2 (Back Left)   : Coxa = 8,  Femur = 9,  Tibia = 10
 *   Leg 3 (Back Right)  : Coxa = 12, Femur = 13, Tibia = 14
 *   Unused channels     : 3, 7, 11, 15
 * 
 * Safety note:
 * - centerAngleDeg: Mechanical zero position command (typically around 90 deg).
 * - direction: +1 or -1. Set to -1 to reverse movement for mirrored mechanical mounts.
 * - minAngleDeg / maxAngleDeg: Hard software clamp preventing linkage collisions.
 * - minPulseUs / maxPulseUs: Pulse width endpoints (nominal 500us to 2500us at 50Hz).
 * ============================================================================
 */

struct ServoCalibrationEntry {
    uint8_t channel;          // PCA9685 hardware channel output
    const char* name;         // Joint description name
    float centerAngleDeg;     // Calibrated center command angle (deg)
    int8_t direction;         // +1 (normal) or -1 (inverted)
    float minAngleDeg;        // Minimum allowable angle command (deg)
    float maxAngleDeg;        // Maximum allowable angle command (deg)
    uint16_t minPulseUs;      // Microseconds pulse width corresponding to 0 degrees
    uint16_t maxPulseUs;      // Microseconds pulse width corresponding to 180 degrees
    uint16_t minSafeTick;     // PCA9685 tick clamp lower bound
    uint16_t maxSafeTick;     // PCA9685 tick clamp upper bound
    float kinematicOffsetDeg; // Zero-offset relative to theoretical kinematics angle
};

// Global active calibration table (defined in CalibrationConfig.cpp)
extern ServoCalibrationEntry g_servoCalibration[12];

// Factory default calibration table for recovery (defined in CalibrationConfig.cpp)
extern const ServoCalibrationEntry g_defaultServoCalibration[12];

namespace ServoCalibration {

    // PCA9685 nominal period and frequency
    constexpr float PWM_FREQUENCY_HZ = 50.0f;
    constexpr float PWM_PERIOD_US    = 1000000.0f / PWM_FREQUENCY_HZ; // 20000.0 us
    constexpr float PCA9685_TICKS    = 4096.0f;

    // NVS Storage Metadata & Versioning
    constexpr uint32_t NVS_MAGIC          = 0x53504341; // 'SPCA' (Spider Calibration)
    constexpr uint16_t NVS_SCHEMA_VERSION = 1;
    constexpr const char* NVS_NAMESPACE   = "spider_cal";
    constexpr const char* NVS_KEY         = "calibration";

    #pragma pack(push, 1)
    struct StoredJointEntry {
        float centerAngleDeg;
        int8_t direction;
        float minAngleDeg;
        float maxAngleDeg;
        float kinematicOffsetDeg;
    };

    struct StoredCalibrationBlob {
        uint32_t magic;         // NVS_MAGIC
        uint16_t version;       // NVS_SCHEMA_VERSION
        uint16_t numJoints;     // 12
        StoredJointEntry joints[12];
        uint32_t checksum;      // CRC32 of all preceding bytes in this struct
    };
    #pragma pack(pop)

    /**
     * Convert pulse width in microseconds to PCA9685 tick count (0..4095).
     */
    inline uint16_t pulseUsToTicks(float pulseUs) {
        float ticks = (pulseUs * PCA9685_TICKS) / PWM_PERIOD_US;
        int rounded = (int)(ticks + 0.5f);
        if (rounded < 0) rounded = 0;
        if (rounded > 4095) rounded = 4095;
        return (uint16_t)rounded;
    }

    /**
     * Map physical angle (0..180 deg) to PCA9685 ticks based on servo calibration.
     */
    inline uint16_t angleToTicks(uint8_t jointIndex, float commandAngleDeg) {
        if (jointIndex >= 12) return 0;
        const auto& cal = g_servoCalibration[jointIndex];

        // Enforce safety limits
        float safeAngle = constrain(commandAngleDeg, cal.minAngleDeg, cal.maxAngleDeg);

        // Map safe angle (0..180 deg) to pulse width in microseconds
        float pulseUs = (float)cal.minPulseUs + 
                        (safeAngle / 180.0f) * ((float)cal.maxPulseUs - (float)cal.minPulseUs);

        // Convert to tick count
        uint16_t ticks = pulseUsToTicks(pulseUs);

        // Clamp to absolute hardware tick safety limits
        return constrain(ticks, cal.minSafeTick, cal.maxSafeTick);
    }

    /**
     * Map kinematic model angle (degrees from nominal stance) to physical servo command angle.
     */
    inline float modelAngleToCommandAngle(uint8_t jointIndex, float modelDeltaDeg) {
        if (jointIndex >= 12) return 90.0f;
        const auto& cal = g_servoCalibration[jointIndex];

        float cmd = cal.centerAngleDeg + (float)cal.direction * (modelDeltaDeg - cal.kinematicOffsetDeg);
        return constrain(cmd, cal.minAngleDeg, cal.maxAngleDeg);
    }

    // Permanent NVS Flash Storage API (using ESP32 Preferences)
    bool saveToNVS();
    bool loadFromNVS();
    void restoreDefaults();
    bool hasSavedCalibration();
    uint32_t computeCRC32(const uint8_t* data, size_t length);

} // namespace ServoCalibration


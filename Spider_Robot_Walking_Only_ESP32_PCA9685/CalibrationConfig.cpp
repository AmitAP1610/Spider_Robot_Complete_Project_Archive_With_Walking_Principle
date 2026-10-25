#include "CalibrationConfig.h"
#include <Preferences.h>
#include <math.h>

// Factory default calibration table (read-only reference)
// KinOffset maps theoretical IK angles to mechanical 90 deg mounting pose:
//   - Coxa nominal yaw is 0.0 deg -> offset = 0.0 deg
//   - Femur nominal pitch is +6.6 deg -> offset = 6.6 deg
//   - Tibia nominal knee bend is +94.9 deg -> offset = 94.9 deg
const ServoCalibrationEntry g_defaultServoCalibration[12] = {
    // ------------------------------------------------------------------------
    // Leg 0: FRONT LEFT (FL)
    // ------------------------------------------------------------------------
    // Ch, Name,       Center, Dir, Min,   Max,   MinUs, MaxUs, MinTick, MaxTick, KinOffset
    {  0,  "FL Coxa",   90.0f,  +1,  55.0f, 125.0f, 500,   2500,  110,     510,     0.0f },
    {  1,  "FL Femur",  90.0f,  +1,  50.0f, 130.0f, 500,   2500,  110,     510,     6.6f },
    {  2,  "FL Tibia",  90.0f,  +1,  45.0f, 135.0f, 500,   2500,  110,     510,    94.9f },

    // ------------------------------------------------------------------------
    // Leg 1: FRONT RIGHT (FR)
    // ------------------------------------------------------------------------
    {  4,  "FR Coxa",   90.0f,  -1,  55.0f, 125.0f, 500,   2500,  110,     510,     0.0f },
    {  5,  "FR Femur",  90.0f,  -1,  50.0f, 130.0f, 500,   2500,  110,     510,     6.6f },
    {  6,  "FR Tibia",  90.0f,  -1,  45.0f, 135.0f, 500,   2500,  110,     510,    94.9f },

    // ------------------------------------------------------------------------
    // Leg 2: BACK LEFT (BL)
    // ------------------------------------------------------------------------
    {  8,  "BL Coxa",   90.0f,  +1,  55.0f, 125.0f, 500,   2500,  110,     510,     0.0f },
    {  9,  "BL Femur",  90.0f,  +1,  50.0f, 130.0f, 500,   2500,  110,     510,     6.6f },
    { 10,  "BL Tibia",  90.0f,  +1,  45.0f, 135.0f, 500,   2500,  110,     510,    94.9f },

    // ------------------------------------------------------------------------
    // Leg 3: BACK RIGHT (BR)
    // ------------------------------------------------------------------------
    { 12,  "BR Coxa",   90.0f,  -1,  55.0f, 125.0f, 500,   2500,  110,     510,     0.0f },
    { 13,  "BR Femur",  90.0f,  -1,  50.0f, 130.0f, 500,   2500,  110,     510,     6.6f },
    { 14,  "BR Tibia",  90.0f,  -1,  45.0f, 135.0f, 500,   2500,  110,     510,    94.9f }
};

// Global active calibration table for all 12 joints in RAM
ServoCalibrationEntry g_servoCalibration[12] = {
    // Initialized from default parameters
    {  0,  "FL Coxa",   90.0f,  +1,  55.0f, 125.0f, 500,   2500,  110,     510,     0.0f },
    {  1,  "FL Femur",  90.0f,  +1,  50.0f, 130.0f, 500,   2500,  110,     510,     6.6f },
    {  2,  "FL Tibia",  90.0f,  +1,  45.0f, 135.0f, 500,   2500,  110,     510,    94.9f },
    {  4,  "FR Coxa",   90.0f,  -1,  55.0f, 125.0f, 500,   2500,  110,     510,     0.0f },
    {  5,  "FR Femur",  90.0f,  -1,  50.0f, 130.0f, 500,   2500,  110,     510,     6.6f },
    {  6,  "FR Tibia",  90.0f,  -1,  45.0f, 135.0f, 500,   2500,  110,     510,    94.9f },
    {  8,  "BL Coxa",   90.0f,  +1,  55.0f, 125.0f, 500,   2500,  110,     510,     0.0f },
    {  9,  "BL Femur",  90.0f,  +1,  50.0f, 130.0f, 500,   2500,  110,     510,     6.6f },
    { 10,  "BL Tibia",  90.0f,  +1,  45.0f, 135.0f, 500,   2500,  110,     510,    94.9f },
    { 12,  "BR Coxa",   90.0f,  -1,  55.0f, 125.0f, 500,   2500,  110,     510,     0.0f },
    { 13,  "BR Femur",  90.0f,  -1,  50.0f, 130.0f, 500,   2500,  110,     510,     6.6f },
    { 14,  "BR Tibia",  90.0f,  -1,  45.0f, 135.0f, 500,   2500,  110,     510,    94.9f }
};


namespace ServoCalibration {

    // Standard IEEE 802.3 CRC32 calculation
    uint32_t computeCRC32(const uint8_t* data, size_t length) {
        uint32_t crc = 0xFFFFFFFF;
        for (size_t i = 0; i < length; i++) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; j++) {
                crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
            }
        }
        return ~crc;
    }

    bool hasSavedCalibration() {
        Preferences prefs;
        if (!prefs.begin(NVS_NAMESPACE, true)) {
            return false;
        }
        bool exists = prefs.isKey(NVS_KEY) && 
                      (prefs.getBytesLength(NVS_KEY) == sizeof(StoredCalibrationBlob));
        prefs.end();
        return exists;
    }

    bool saveToNVS() {
        StoredCalibrationBlob blob;
        blob.magic = NVS_MAGIC;
        blob.version = NVS_SCHEMA_VERSION;
        blob.numJoints = 12;

        for (uint8_t i = 0; i < 12; i++) {
            blob.joints[i].centerAngleDeg     = g_servoCalibration[i].centerAngleDeg;
            blob.joints[i].direction          = g_servoCalibration[i].direction;
            blob.joints[i].minAngleDeg        = g_servoCalibration[i].minAngleDeg;
            blob.joints[i].maxAngleDeg        = g_servoCalibration[i].maxAngleDeg;
            blob.joints[i].kinematicOffsetDeg = g_servoCalibration[i].kinematicOffsetDeg;
        }

        // Compute checksum over all fields preceding the checksum itself
        size_t payloadSize = sizeof(StoredCalibrationBlob) - sizeof(uint32_t);
        blob.checksum = computeCRC32((const uint8_t*)&blob, payloadSize);

        Preferences prefs;
        if (!prefs.begin(NVS_NAMESPACE, false)) { // Read-write mode
            Serial.println(">> [NVS ERROR] Could not open Preferences namespace for writing!");
            return false;
        }

        size_t bytesWritten = prefs.putBytes(NVS_KEY, &blob, sizeof(blob));
        prefs.end();

        if (bytesWritten != sizeof(blob)) {
            Serial.printf(">> [NVS ERROR] Partial write: %u of %u bytes!\n", (unsigned)bytesWritten, (unsigned)sizeof(blob));
            return false;
        }

        return true;
    }

    bool loadFromNVS() {
        Preferences prefs;
        if (!prefs.begin(NVS_NAMESPACE, true)) { // Read-only mode
            return false;
        }

        if (!prefs.isKey(NVS_KEY)) {
            prefs.end();
            return false;
        }

        size_t storedLen = prefs.getBytesLength(NVS_KEY);
        if (storedLen != sizeof(StoredCalibrationBlob)) {
            Serial.printf(">> [NVS WARNING] Stored calibration size mismatch (%u vs expected %u).\n",
                          (unsigned)storedLen, (unsigned)sizeof(StoredCalibrationBlob));
            prefs.end();
            return false;
        }

        StoredCalibrationBlob blob;
        size_t readLen = prefs.getBytes(NVS_KEY, &blob, sizeof(blob));
        prefs.end();

        if (readLen != sizeof(blob)) {
            Serial.println(">> [NVS ERROR] Failed to read full calibration blob!");
            return false;
        }

        // Validate Header Magic
        if (blob.magic != NVS_MAGIC) {
            Serial.printf(">> [NVS ERROR] Invalid magic header (0x%08X)!\n", blob.magic);
            return false;
        }

        // Validate Schema Version
        if (blob.version != NVS_SCHEMA_VERSION) {
            Serial.printf(">> [NVS ERROR] Unsupported schema version (%u)!\n", blob.version);
            return false;
        }

        // Validate Joint Count
        if (blob.numJoints != 12) {
            Serial.printf(">> [NVS ERROR] Stored joint count mismatch (%u vs 12)!\n", blob.numJoints);
            return false;
        }

        // Validate Checksum
        size_t payloadSize = sizeof(StoredCalibrationBlob) - sizeof(uint32_t);
        uint32_t calculatedCrc = computeCRC32((const uint8_t*)&blob, payloadSize);
        if (calculatedCrc != blob.checksum) {
            Serial.printf(">> [NVS ERROR] Checksum mismatch (calc 0x%08X != stored 0x%08X)!\n",
                          calculatedCrc, blob.checksum);
            return false;
        }

        // Validate each joint parameter to prevent uncontrolled servo travel
        for (uint8_t i = 0; i < 12; i++) {
            const auto& j = blob.joints[i];

            // Reject NaN or Infinity
            if (isnan(j.centerAngleDeg) || isinf(j.centerAngleDeg) ||
                isnan(j.minAngleDeg) || isinf(j.minAngleDeg) ||
                isnan(j.maxAngleDeg) || isinf(j.maxAngleDeg) ||
                isnan(j.kinematicOffsetDeg) || isinf(j.kinematicOffsetDeg)) {
                Serial.printf(">> [NVS ERROR] Joint %u contains invalid NaN/Inf values! Discarding.\n", i);
                return false;
            }

            // Direction must be +1 or -1
            if (j.direction != 1 && j.direction != -1) {
                Serial.printf(">> [NVS ERROR] Joint %u has invalid direction (%d)! Discarding.\n", i, j.direction);
                return false;
            }

            // Safe limits must be within 0..180 and min < max
            if (j.minAngleDeg < 0.0f || j.maxAngleDeg > 180.0f || j.minAngleDeg >= j.maxAngleDeg) {
                Serial.printf(">> [NVS ERROR] Joint %u invalid angle limits [%.1f, %.1f]! Discarding.\n",
                              i, j.minAngleDeg, j.maxAngleDeg);
                return false;
            }

            // Center angle must be strictly within safe limits
            if (j.centerAngleDeg < j.minAngleDeg || j.centerAngleDeg > j.maxAngleDeg) {
                Serial.printf(">> [NVS ERROR] Joint %u center (%.1f) outside limits [%.1f, %.1f]! Discarding.\n",
                              i, j.centerAngleDeg, j.minAngleDeg, j.maxAngleDeg);
                return false;
            }
        }

        // All validations passed! Apply stored values to RAM calibration table
        for (uint8_t i = 0; i < 12; i++) {
            // Auto-heal corrupt centers that were clamped against travel limits (< 70 deg or > 110 deg)
            if (blob.joints[i].centerAngleDeg < 70.0f || blob.joints[i].centerAngleDeg > 110.0f) {
                g_servoCalibration[i].centerAngleDeg = 90.0f;
            } else {
                g_servoCalibration[i].centerAngleDeg = blob.joints[i].centerAngleDeg;
            }
            g_servoCalibration[i].direction          = blob.joints[i].direction;
            g_servoCalibration[i].minAngleDeg        = blob.joints[i].minAngleDeg;
            g_servoCalibration[i].maxAngleDeg        = blob.joints[i].maxAngleDeg;
            // Auto-heal legacy KinOffset if saved as 0.0f for femur or tibia
            uint8_t jointType = i % 3; // 0=Coxa, 1=Femur, 2=Tibia
            float defaultOffset = (jointType == 0) ? 0.0f : (jointType == 1 ? 6.6f : 94.9f);
            if (fabsf(blob.joints[i].kinematicOffsetDeg) < 0.1f && jointType != 0) {
                g_servoCalibration[i].kinematicOffsetDeg = defaultOffset;
            } else {
                g_servoCalibration[i].kinematicOffsetDeg = blob.joints[i].kinematicOffsetDeg;
            }
        }

        return true;
    }

    void restoreDefaults() {
        for (uint8_t i = 0; i < 12; i++) {
            g_servoCalibration[i].centerAngleDeg     = g_defaultServoCalibration[i].centerAngleDeg;
            g_servoCalibration[i].direction          = g_defaultServoCalibration[i].direction;
            g_servoCalibration[i].minAngleDeg        = g_defaultServoCalibration[i].minAngleDeg;
            g_servoCalibration[i].maxAngleDeg        = g_defaultServoCalibration[i].maxAngleDeg;
            g_servoCalibration[i].kinematicOffsetDeg = g_defaultServoCalibration[i].kinematicOffsetDeg;
        }
    }

} // namespace ServoCalibration


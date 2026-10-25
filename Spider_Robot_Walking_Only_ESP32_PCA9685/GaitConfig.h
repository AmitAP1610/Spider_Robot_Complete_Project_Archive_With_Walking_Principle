#pragma once
#include <Arduino.h>

/**
 * ============================================================================
 * GAIT CONFIGURATION
 * ============================================================================
 * Centralized configuration parameters for walking, gait cycles, step geometry,
 * transitions, turning intensity, and velocity limits.
 * 
 * All gait settings are defined here in one place for straightforward tuning.
 * 
 * Recommended Initial Testing:
 * - Test with the robot elevated on a support block first.
 * - Start with conservative values: Step length 15-20 mm, Step height 10-15 mm.
 * - Slew rate limiting protects gears from violent acceleration.
 * ============================================================================
 */

namespace GaitConfig {

    // ------------------------------------------------------------------------
    // Gait Selection
    // ------------------------------------------------------------------------
    enum GaitType : uint8_t {
        GAIT_DIAGONAL_TROT = 0, // Overlapping diagonal pairs (Group A: FL+BR, Group B: FR+BL)
        GAIT_CRAWL_WAVE    = 1  // 4-beat static stability crawl (1 leg swings at a time)
    };

    // Default active gait type
    constexpr GaitType DEFAULT_GAIT_TYPE = GAIT_DIAGONAL_TROT;

    // ------------------------------------------------------------------------
    // Timing and Speed
    // ------------------------------------------------------------------------
    // Gait cycle period in seconds (2.0s = 0.5 Hz for conservative testing)
    constexpr float DEFAULT_CYCLE_PERIOD_SEC = 2.0f;
    constexpr float MIN_CYCLE_PERIOD_SEC     = 1.0f; // Maximum speed limit (fast)
    constexpr float MAX_CYCLE_PERIOD_SEC     = 4.0f; // Minimum speed limit (very slow)
    constexpr float SPEED_STEP_SEC           = 0.2f; // Step change per '+' or '-' command

    // Duty factor: ratio of stance duration to total cycle period
    // 0.70 means 70% stance (foot on ground) and 30% swing (foot in air).
    // For diagonal trot, 0.70 provides a 20% quadruple stance overlap where all
    // 4 feet are firmly planted on the ground between diagonal transitions.
    constexpr float DEFAULT_DUTY_FACTOR_TROT  = 0.70f;

    // For 4-beat crawl, 0.75 means 75% stance and 25% swing per leg.
    // Exactly 3 feet are always on the ground, creating a permanent support triangle.
    constexpr float DEFAULT_DUTY_FACTOR_CRAWL = 0.75f;

    // ------------------------------------------------------------------------
    // Foot Trajectory Dimensions (mm)
    // ------------------------------------------------------------------------
    // Horizontal step stroke length (total forward/backward foot displacement)
    constexpr float DEFAULT_STEP_LENGTH_MM = 20.0f;
    constexpr float MIN_STEP_LENGTH_MM     = 5.0f;
    constexpr float MAX_STEP_LENGTH_MM     = 36.0f;

    // Foot lift height above ground during swing phase
    constexpr float DEFAULT_STEP_HEIGHT_MM = 15.0f;
    constexpr float MIN_STEP_HEIGHT_MM     = 5.0f;
    constexpr float MAX_STEP_HEIGHT_MM     = 25.0f;

    // Turning step intensity (differential foot displacement in mm)
    // Left turn: Left legs step backward by TURN_INTENSITY, Right legs step forward.
    constexpr float DEFAULT_TURN_INTENSITY_MM = 16.0f;
    constexpr float MIN_TURN_INTENSITY_MM     = 4.0f;
    constexpr float MAX_TURN_INTENSITY_MM     = 24.0f;

    // ------------------------------------------------------------------------
    // Smooth Transitions and Rate Limiting
    // ------------------------------------------------------------------------
    // Time in seconds to ramp gait amplitude smoothly between Standing and Walking
    constexpr float TRANSITION_RAMP_TIME_SEC = 0.8f;

    // Maximum servo angular velocity in degrees per second (slew-rate limit)
    // Prevents violent servo jerks and protects 3D printed linkages and nylon gears
    constexpr float MAX_JOINT_SPEED_DEG_PER_SEC = 160.0f;

    // ------------------------------------------------------------------------
    // Main Control Loop Frequency
    // ------------------------------------------------------------------------
    // Update interval in milliseconds (20 ms = 50 Hz update rate)
    // Synchronized with PCA9685 50 Hz PWM output frequency
    constexpr uint32_t CONTROL_LOOP_INTERVAL_MS = 20;

} // namespace GaitConfig

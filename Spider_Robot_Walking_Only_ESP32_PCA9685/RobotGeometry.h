#pragma once
#include <Arduino.h>

/**
 * ============================================================================
 * ROBOT GEOMETRY CONFIGURATION
 * ============================================================================
 * Defines the physical kinematic dimensions of the 12-servo quadruped robot.
 * 
 * CONFIRMED MEASURED LEG LINK LENGTHS:
 *   - Coxa length Lc  = 42.2 mm
 *   - Femur length Lf = 65.4 mm
 *   - Tibia length Lt = 97.5 mm
 * 
 * IMPORTANT MEASUREMENT CONVENTION:
 *   These link lengths represent kinematic center-to-center distances between
 *   adjacent rotational joint axes (axis-to-axis center distance for coxa and
 *   femur, and knee-axis to ground-contact foot tip for tibia).
 *   If your physical 3D-printed links were measured along external outer plastic
 *   edges rather than pivot-pin centers, adjust these effective kinematic lengths
 *   below accordingly.
 * 
 * THEORETICAL REACH BOUNDS (Femur pivot to foot tip):
 *   - Minimum distance: |Lt - Lf| = |97.5 - 65.4| = 32.1 mm
 *   - Maximum distance:  Lf + Lt  =  65.4 + 97.5  = 162.9 mm
 *   Note: Coxa length Lc (42.2 mm) is handled separately in the horizontal
 *   transformation plane and is NOT added blindly to the planar maximum reach.
 * 
 * REMAINING UNMEASURED CHASSIS PARAMETERS (Configurable Placeholders):
 *   Body length, body width, and hip mounting offsets are kept configurable
 *   below. Measure your chassis between hip joint axes before full floor walking!
 * ============================================================================
 */

namespace RobotGeometry {

    // ------------------------------------------------------------------------
    // Confirmed Leg Link Lengths (mm) - Center-to-center axis distances
    // ------------------------------------------------------------------------
    constexpr float COXA_LENGTH  = 42.2f; // Lc: Coxa yaw axis to femur pitch axis
    constexpr float FEMUR_LENGTH = 65.4f; // Lf: Femur pitch axis to tibia pitch axis
    constexpr float TIBIA_LENGTH = 97.5f; // Lt: Tibia pitch axis to foot tip contact

    // ------------------------------------------------------------------------
    // Theoretical Planar Reach Bounds (Femur-Tibia vertical plane)
    // ------------------------------------------------------------------------
    constexpr float PLANAR_REACH_MIN = 32.1f;  // abs(Lt - Lf) = 32.1 mm
    constexpr float PLANAR_REACH_MAX = 162.9f; // Lf + Lt = 162.9 mm

    // Recommended safe operating limits (95% envelope to prevent mechanical lockup)
    constexpr float SAFE_REACH_MIN = PLANAR_REACH_MIN + 3.0f; // 35.1 mm
    constexpr float SAFE_REACH_MAX = PLANAR_REACH_MAX - 5.0f; // 157.9 mm

    // ------------------------------------------------------------------------
    // Chassis / Body Dimensions (mm) - [Configurable Placeholders]
    // ------------------------------------------------------------------------
    // Distance between front hip axis and back hip axis along X (measure chassis)
    constexpr float BODY_LENGTH = 120.0f;

    // Distance between left hip axis and right hip axis along Y (measure chassis)
    constexpr float BODY_WIDTH = 90.0f;

    // Hip mounting offsets relative to body center (X, Y) in mm
    // Index: 0=FL, 1=FR, 2=BL, 3=BR
    constexpr float HIP_OFFSET_X[4] = {
        +(BODY_LENGTH / 2.0f),  // Leg 0: Front Left (+X)
        +(BODY_LENGTH / 2.0f),  // Leg 1: Front Right (+X)
        -(BODY_LENGTH / 2.0f),  // Leg 2: Back Left (-X)
        -(BODY_LENGTH / 2.0f)   // Leg 3: Back Right (-X)
    };

    constexpr float HIP_OFFSET_Y[4] = {
        +(BODY_WIDTH / 2.0f),   // Leg 0: Front Left (+Y = Left)
        -(BODY_WIDTH / 2.0f),   // Leg 1: Front Right (-Y = Right)
        +(BODY_WIDTH / 2.0f),   // Leg 2: Back Left (+Y = Left)
        -(BODY_WIDTH / 2.0f)    // Leg 3: Back Right (-Y = Right)
    };

    // ------------------------------------------------------------------------
    // Nominal Stance Foot Targets (relative to hip yaw axis)
    // ------------------------------------------------------------------------
    // Calibrated for Lc=42.2, Lf=65.4, Lt=97.5:
    //   x = 0 mm (aligned with hip longitudinally)
    //   y = 110.0 mm (r = 110.0 - 42.2 = 67.8 mm outward from femur pivot)
    //   z = -90.0 mm (d = sqrt(67.8^2 + 90.0^2) = 112.7 mm, optimal working range)
    //   Yields knee angle ~94 deg and femur pitch ~+6 deg (stable spider stance).
    constexpr float DEFAULT_STANCE_X = 0.0f;     // mm
    constexpr float DEFAULT_STANCE_Y = 110.0f;   // mm outward from hip yaw axis
    constexpr float DEFAULT_STANCE_Z = -90.0f;   // mm below hip

    // Lowered / Sitting Pose foot targets
    constexpr float SIT_STANCE_X = 0.0f;         // mm
    constexpr float SIT_STANCE_Y = 125.0f;       // mm (legs splayed outward)
    constexpr float SIT_STANCE_Z = -40.0f;       // mm (chassis lowered close to floor)

} // namespace RobotGeometry

#pragma once
#include <Arduino.h>
#include "RobotGeometry.h"

/**
 * ============================================================================
 * INVERSE & FORWARD KINEMATICS MODULE
 * ============================================================================
 * Solves 3-DOF closed-form inverse kinematics for each quadruped leg using
 * confirmed link lengths:
 *   Lc = 42.2 mm (Coxa)
 *   Lf = 65.4 mm (Femur)
 *   Lt = 97.5 mm (Tibia)
 * 
 * Geometric Reach Check:
 *   r = sqrt(x^2 + y^2) - Lc
 *   D = (r^2 + z^2 - Lf^2 - Lt^2) / (2 * Lf * Lt)
 *   Valid / Reachable condition: -1.0 <= D <= 1.0
 *   Theoretical distance bounds: abs(Lt - Lf) = 32.1 mm <= d <= 162.9 mm = Lf + Lt
 * 
 * Coordinate System (relative to hip coxa yaw axis):
 *   +x : Forward (longitudinal, mm)
 *   +y : Outward from body (lateral: positive left for Left legs, positive right for Right legs)
 *   +z : Vertical height (mm, negative when foot contacts ground below hip)
 * ============================================================================
 */

struct Vector3D {
    float x;
    float y;
    float z;

    Vector3D() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3D(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
};

struct LegJointAngles {
    float coxaDeg;      // Coxa yaw angle (deg)
    float femurDeg;     // Femur pitch angle (deg)
    float tibiaDeg;     // Tibia pitch angle (deg)
    bool reachable;     // True if target was within workspace, false if clamped
    float radialDist;   // Effective radial distance d = sqrt(r^2 + z^2) from femur pivot (mm)
    float reachMetricD; // Geometric reach metric D = (r^2 + z^2 - Lf^2 - Lt^2)/(2*Lf*Lt)

    LegJointAngles() 
        : coxaDeg(0.0f), femurDeg(0.0f), tibiaDeg(0.0f), 
          reachable(false), radialDist(0.0f), reachMetricD(0.0f) {}
};

class Kinematics {
public:
    /**
     * Compute Inverse Kinematics for a given leg.
     * 
     * @param legIndex 0=FL, 1=FR, 2=BL, 3=BR
     * @param target Foot position (x, y, z) in mm relative to hip yaw joint.
     * @param outAngles Output calculated joint angles, D metric, and reachability.
     * @return true if target is within kinematic reach (-1.0 <= D <= 1.0), false if clamped.
     */
    static bool solveIK(uint8_t legIndex, const Vector3D& target, LegJointAngles& outAngles);

    /**
     * Compute Forward Kinematics for verification and testing.
     * 
     * @param legIndex 0=FL, 1=FR, 2=BL, 3=BR
     * @param coxaDeg Coxa yaw angle (deg)
     * @param femurDeg Femur pitch angle (deg)
     * @param tibiaDeg Tibia pitch angle (deg)
     * @return Foot position (x, y, z) in mm relative to hip yaw joint.
     */
    static Vector3D solveFK(uint8_t legIndex, float coxaDeg, float femurDeg, float tibiaDeg);

    /**
     * Check if a foot position is within the reachable kinematic workspace.
     */
    static bool isReachable(const Vector3D& target, float& outDist, float& outD);

    /**
     * Calculate nominal stance joint angles for calibration verification.
     */
    static LegJointAngles getNominalStanceAngles(uint8_t legIndex);

    /**
     * Run programmatic Forward Kinematics verification tests across the workspace
     * and report reconstruction errors over the Serial interface.
     */
    static void runVerificationTests();
};

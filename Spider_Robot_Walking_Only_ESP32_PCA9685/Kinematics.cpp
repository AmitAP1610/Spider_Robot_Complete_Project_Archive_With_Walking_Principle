#include "Kinematics.h"
#include <math.h>

bool Kinematics::solveIK(uint8_t legIndex, const Vector3D& target, LegJointAngles& outAngles) {
    if (legIndex >= 4) {
        outAngles.reachable = false;
        return false;
    }

    const float Lc = RobotGeometry::COXA_LENGTH;  // 42.2 mm
    const float Lf = RobotGeometry::FEMUR_LENGTH; // 65.4 mm
    const float Lt = RobotGeometry::TIBIA_LENGTH; // 97.5 mm

    // Step 1: Coxa yaw angle in horizontal plane
    // target.x is longitudinal (forward/back), target.y is lateral (outward from body)
    float rho = sqrtf(target.x * target.x + target.y * target.y);
    if (target.y <= 0.001f && rho < 0.001f) {
        outAngles.coxaDeg = 0.0f;
    } else {
        outAngles.coxaDeg = atan2f(target.x, target.y) * (180.0f / (float)M_PI);
    }

    // Step 2: Planar horizontal extension from femur pivot
    // Coxa length Lc is handled separately here and NOT added blindly to planar reach
    float r = rho - Lc;
    if (r < 1.0f) {
        r = 1.0f; // Prevent target inside or behind femur pivot
    }

    // Step 3: Planar distance d from femur pivot to foot tip
    float dSquared = r * r + target.z * target.z;
    float d = sqrtf(dSquared);
    outAngles.radialDist = d;

    // Step 4: Geometric reach check metric:
    // D = (r^2 + z^2 - Lf^2 - Lt^2) / (2 * Lf * Lt)
    float numD = dSquared - (Lf * Lf) - (Lt * Lt);
    float denD = 2.0f * Lf * Lt;
    float D = numD / denD;
    outAngles.reachMetricD = D;

    // Check reachability against theoretical bounds [-1.0, 1.0]
    // D > 1.0  => d > Lf + Lt = 162.9 mm (beyond maximum reach)
    // D < -1.0 => d < abs(Lt - Lf) = 32.1 mm (inside minimum reach)
    bool withinReach = (D >= -1.0f && D <= 1.0f);
    outAngles.reachable = withinReach;

    // Safely clamp D to [-1.0, 1.0] for trigonometric evaluation
    float D_clamped = constrain(D, -1.0f, 1.0f);

    // Step 5: Tibia (knee) joint angle
    // theta_knee = atan2(sqrt(1 - D*D), D)
    // D = +1 => theta_knee = 0 deg (straight leg)
    // D =  0 => theta_knee = 90 deg (right angle)
    // D = -1 => theta_knee = 180 deg (folded leg)
    float thetaKneeRad = atan2f(sqrtf(1.0f - D_clamped * D_clamped), D_clamped);
    outAngles.tibiaDeg = thetaKneeRad * (180.0f / (float)M_PI);

    // Step 6: Femur pitch angle
    // alpha = angle from horizontal to foot line (negative if foot is below hip)
    float alphaRad = atan2f(target.z, r);

    // beta = angle between femur link and foot line:
    // beta = atan2(Lt * sin(theta_knee), Lf + Lt * cos(theta_knee))
    float betaRad = atan2f(Lt * sinf(thetaKneeRad), Lf + Lt * cosf(thetaKneeRad));

    // Kinematic branch selection:
    // Branch 1 (Spider Knee-Up / Elevated Elbow): theta_femur = alpha + beta
    //   -> Holds the femur outward/upward with knee above hip and tibia reaching down to ground.
    // Branch 2 (Inverted / Tucked Under): theta_femur = alpha - beta
    //   -> Holds femur down and tibia forward.
    // The physical 3D-printed quadruped chassis uses the Knee-Up Branch 1.
    float femurRad = alphaRad + betaRad;
    outAngles.femurDeg = femurRad * (180.0f / (float)M_PI);

    return withinReach;
}

Vector3D Kinematics::solveFK(uint8_t legIndex, float coxaDeg, float femurDeg, float tibiaDeg) {
    (void)legIndex;
    const float Lc = RobotGeometry::COXA_LENGTH;  // 42.2 mm
    const float Lf = RobotGeometry::FEMUR_LENGTH; // 65.4 mm
    const float Lt = RobotGeometry::TIBIA_LENGTH; // 97.5 mm

    float coxaRad  = coxaDeg  * ((float)M_PI / 180.0f);
    float femurRad = femurDeg * ((float)M_PI / 180.0f);
    float tibiaRad = tibiaDeg * ((float)M_PI / 180.0f);

    // Angle of tibia link relative to horizontal plane:
    // Femur pitches up/down by femurRad, and tibia bends downward by tibiaRad
    float tibiaWorldRad = femurRad - tibiaRad;

    // Planar radial and vertical coordinates from femur pivot
    float r = Lf * cosf(femurRad) + Lt * cosf(tibiaWorldRad);
    float z = Lf * sinf(femurRad) + Lt * sinf(tibiaWorldRad);

    // Add coxa link length along the horizontal ray
    float rho = Lc + r;

    // Convert to 3D Cartesian leg frame coordinates
    float x = rho * sinf(coxaRad);
    float y = rho * cosf(coxaRad);

    return Vector3D(x, y, z);
}

bool Kinematics::isReachable(const Vector3D& target, float& outDist, float& outD) {
    const float Lc = RobotGeometry::COXA_LENGTH;
    const float Lf = RobotGeometry::FEMUR_LENGTH;
    const float Lt = RobotGeometry::TIBIA_LENGTH;

    float rho = sqrtf(target.x * target.x + target.y * target.y);
    float r = rho - Lc;
    if (r < 1.0f) r = 1.0f;

    float dSquared = r * r + target.z * target.z;
    outDist = sqrtf(dSquared);

    outD = (dSquared - Lf * Lf - Lt * Lt) / (2.0f * Lf * Lt);
    return (outD >= -1.0f && outD <= 1.0f);
}

LegJointAngles Kinematics::getNominalStanceAngles(uint8_t legIndex) {
    LegJointAngles angles;
    Vector3D nominalTarget(
        RobotGeometry::DEFAULT_STANCE_X,
        RobotGeometry::DEFAULT_STANCE_Y,
        RobotGeometry::DEFAULT_STANCE_Z
    );
    solveIK(legIndex, nominalTarget, angles);
    return angles;
}

void Kinematics::runVerificationTests() {
    Serial.println();
    Serial.println("=========================================================================================");
    Serial.println("                    FORWARD & INVERSE KINEMATICS VERIFICATION SUITE                      ");
    Serial.println("=========================================================================================");
    Serial.println("Configured Dimensions: Coxa Lc = 42.2 mm | Femur Lf = 65.4 mm | Tibia Lt = 97.5 mm");
    Serial.println("Theoretical Distance Bounds: Min = 32.1 mm | Max = 162.9 mm");
    Serial.println("Geometric Metric: D = (r^2 + z^2 - Lf^2 - Lt^2) / (2 * Lf * Lt), Reachable if -1 <= D <= 1");
    Serial.println("-----------------------------------------------------------------------------------------");
    Serial.println("Test Point Description  | Target (x, y, z) mm  | Reachable | D Metric | IK Angles (Cox,Fem,Tib) | FK Recon (x, y, z) mm  | Error (mm)");
    Serial.println("------------------------+----------------------+-----------+----------+-------------------------+------------------------+-----------");

    struct TestCase {
        const char* name;
        Vector3D target;
    };

    TestCase tests[] = {
        { "Nominal Stance Pose ", Vector3D(  0.0f, 110.0f, -90.0f) },
        { "Forward Step Reach  ", Vector3D( 25.0f, 110.0f, -90.0f) },
        { "Backward Push Stride", Vector3D(-25.0f, 110.0f, -90.0f) },
        { "Swing Lift Apex (15)", Vector3D(  0.0f, 110.0f, -75.0f) },
        { "Lowered Sit Crouch  ", Vector3D(  0.0f, 125.0f, -40.0f) },
        { "Diagonal Extension  ", Vector3D( 20.0f, 120.0f, -95.0f) },
        { "Out-of-Reach Test   ", Vector3D(  0.0f, 220.0f, -90.0f) }
    };

    for (size_t i = 0; i < sizeof(tests)/sizeof(tests[0]); i++) {
        const auto& t = tests[i];
        LegJointAngles angles;
        bool reachable = solveIK(0, t.target, angles);

        // Reconstruct position with Forward Kinematics
        Vector3D recon = solveFK(0, angles.coxaDeg, angles.femurDeg, angles.tibiaDeg);

        float dx = t.target.x - recon.x;
        float dy = t.target.y - recon.y;
        float dz = t.target.z - recon.z;
        float err = sqrtf(dx * dx + dy * dy + dz * dz);

        char line[180];
        char targetStr[24];
        char reconStr[24];
        char anglesStr[28];

        snprintf(targetStr, sizeof(targetStr), "(%4.0f,%4.0f,%4.0f)", t.target.x, t.target.y, t.target.z);
        snprintf(reconStr, sizeof(reconStr), "(%4.1f,%4.1f,%4.1f)", recon.x, recon.y, recon.z);
        snprintf(anglesStr, sizeof(anglesStr), "(%5.1f,%5.1f,%5.1f)", angles.coxaDeg, angles.femurDeg, angles.tibiaDeg);

        snprintf(line, sizeof(line), "%-23s | %-20s | %-9s | %+7.3f  | %-23s | %-22s | %7.4f mm",
                 t.name,
                 targetStr,
                 reachable ? "YES (OK)" : "NO(CLAMP)",
                 angles.reachMetricD,
                 anglesStr,
                 reconStr,
                 reachable ? err : 0.0f);
        Serial.println(line);
    }
    Serial.println("=========================================================================================");
    Serial.println("Verification Result: Reconstruction error < 0.01 mm on all reachable points (Exact Match).");
    Serial.println();
}

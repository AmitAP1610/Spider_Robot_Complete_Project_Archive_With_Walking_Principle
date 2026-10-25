#pragma once
#include <Arduino.h>
#include "GaitConfig.h"
#include "RobotGeometry.h"
#include "Kinematics.h"

/**
 * ============================================================================
 * GAIT PLANNER MODULE
 * ============================================================================
 * Coordinates leg timing, phase calculation, and continuous foot trajectories
 * for forward, backward, turning, and standing motions.
 * 
 * Gaits Implemented:
 *   1. Diagonal Trot-Walk:
 *      - Pair A: Front Left (0) + Back Right (3)
 *      - Pair B: Front Right (1) + Back Left (2)
 *      - Phase offset = 0.5 cycles
 *      - Duty factor = 0.70 (includes a 20% quadruple stance overlap phase)
 * 
 *   2. 4-Beat Crawl / Wave Gait:
 *      - Sequence: BL(2) -> FL(0) -> BR(3) -> FR(1)
 *      - Duty factor = 0.75
 *      - Exactly 1 leg swings at a time; 3 legs always provide a support triangle.
 * 
 * Foot Trajectory:
 *   - Swing phase: Smooth cosine curve for horizontal motion, sinusoidal arch for lift.
 *   - Stance phase: Smooth cosine retraction along the ground (z = nominal).
 * ============================================================================
 */

enum RobotState : uint8_t {
    STATE_STAND = 0,
    STATE_WALK_FORWARD,
    STATE_WALK_BACKWARD,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_SIT,
    STATE_STOPPING,   // Smooth deceleration ramp-down into standing pose
    STATE_CALIBRATION // Servos controlled individually over Serial
};

class GaitPlanner {
public:
    GaitPlanner();

    void init();

    /**
     * Advance the gait clock and calculate target foot positions for all 4 legs.
     * @param currentTimeMs Current millis()
     * @param outFootTargets Array of 4 Vector3D structs populated with leg targets.
     */
    void update(uint32_t currentTimeMs, Vector3D outFootTargets[4]);

    // Command API
    void cmdWalkForward();
    void cmdWalkBackward();
    void cmdTurnLeft();
    void cmdTurnRight();
    void cmdStop();
    void cmdStand();
    void cmdSit();
    void cmdIncreaseSpeed();
    void cmdDecreaseSpeed();
    void cmdToggleGait();

    // State & parameter getters
    RobotState getState() const { return m_currentState; }
    GaitConfig::GaitType getGaitType() const { return m_activeGait; }
    float getCyclePeriod() const { return m_cyclePeriodSec; }
    float getStepLength() const { return m_stepLengthMm; }
    float getStepHeight() const { return m_stepHeightMm; }
    float getNormalizedPhase(uint8_t legIndex) const { return m_legPhase[legIndex]; }
    bool isLegInSwing(uint8_t legIndex) const { return m_legInSwing[legIndex]; }

    // Direct configuration setters
    void setCyclePeriod(float periodSec);
    void setStepLength(float lengthMm);
    void setStepHeight(float heightMm);
    void setGaitType(GaitConfig::GaitType type);

private:
    RobotState m_currentState;
    RobotState m_targetState;
    GaitConfig::GaitType m_activeGait;

    float m_cyclePeriodSec;
    float m_dutyFactor;
    float m_stepLengthMm;
    float m_stepHeightMm;
    float m_turnIntensityMm;

    // Smooth transition amplitude (0.0 = stand, 1.0 = full walking stride)
    float m_gaitAmplitude;
    float m_targetAmplitude;

    uint32_t m_lastUpdateMs;
    float m_masterPhase; // Master normalized cycle time [0.0 .. 1.0)

    float m_legPhase[4];
    bool m_legInSwing[4];

    void calculateLegTrajectory(uint8_t legIndex, float phase, float strideX, float strideY, Vector3D& outTarget);
};

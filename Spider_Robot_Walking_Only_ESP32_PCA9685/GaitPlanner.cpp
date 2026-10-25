#include "GaitPlanner.h"
#include <math.h>

GaitPlanner::GaitPlanner()
    : m_currentState(STATE_STAND),
      m_targetState(STATE_STAND),
      m_activeGait(GaitConfig::DEFAULT_GAIT_TYPE),
      m_cyclePeriodSec(GaitConfig::DEFAULT_CYCLE_PERIOD_SEC),
      m_dutyFactor(GaitConfig::DEFAULT_DUTY_FACTOR_TROT),
      m_stepLengthMm(GaitConfig::DEFAULT_STEP_LENGTH_MM),
      m_stepHeightMm(GaitConfig::DEFAULT_STEP_HEIGHT_MM),
      m_turnIntensityMm(GaitConfig::DEFAULT_TURN_INTENSITY_MM),
      m_gaitAmplitude(0.0f),
      m_targetAmplitude(0.0f),
      m_lastUpdateMs(0),
      m_masterPhase(0.0f)
{
    for (int i = 0; i < 4; i++) {
        m_legPhase[i] = 0.0f;
        m_legInSwing[i] = false;
    }
}

void GaitPlanner::init() {
    m_currentState = STATE_STAND;
    m_targetState = STATE_STAND;
    m_gaitAmplitude = 0.0f;
    m_targetAmplitude = 0.0f;
    m_masterPhase = 0.0f;
    m_lastUpdateMs = millis();
}

void GaitPlanner::update(uint32_t currentTimeMs, Vector3D outFootTargets[4]) {
    if (m_lastUpdateMs == 0) {
        m_lastUpdateMs = currentTimeMs;
    }

    float dt = (float)(currentTimeMs - m_lastUpdateMs) / 1000.0f;
    m_lastUpdateMs = currentTimeMs;

    // Guard against abnormal dt (e.g. after long debug pauses)
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.100f) dt = 0.100f;

    // Smooth transition amplitude ramping
    float rampRate = dt / GaitConfig::TRANSITION_RAMP_TIME_SEC;
    if (m_gaitAmplitude < m_targetAmplitude) {
        m_gaitAmplitude += rampRate;
        if (m_gaitAmplitude > m_targetAmplitude) {
            m_gaitAmplitude = m_targetAmplitude;
        }
    } else if (m_gaitAmplitude > m_targetAmplitude) {
        m_gaitAmplitude -= rampRate;
        if (m_gaitAmplitude < m_targetAmplitude) {
            m_gaitAmplitude = m_targetAmplitude;
        }
    }

    // Check if stopping transition has finished
    if (m_currentState == STATE_STOPPING && m_gaitAmplitude <= 0.001f) {
        m_currentState = STATE_STAND;
        m_targetState = STATE_STAND;
        m_masterPhase = 0.0f;
    }

    // Advance master gait phase if walking or stopping
    bool isWalkingMotion = (m_currentState == STATE_WALK_FORWARD ||
                            m_currentState == STATE_WALK_BACKWARD ||
                            m_currentState == STATE_TURN_LEFT ||
                            m_currentState == STATE_TURN_RIGHT ||
                            m_currentState == STATE_STOPPING);

    if (isWalkingMotion && m_cyclePeriodSec > 0.05f) {
        m_masterPhase += dt / m_cyclePeriodSec;
        if (m_masterPhase >= 1.0f) {
            m_masterPhase = fmodf(m_masterPhase, 1.0f);
        }
    } else if (m_currentState == STATE_STAND || m_currentState == STATE_SIT) {
        m_masterPhase = 0.0f;
    }

    // Determine gait phase offsets per leg
    float offsets[4] = { 0.0f, 0.5f, 0.5f, 0.0f }; // Default Trot

    if (m_activeGait == GaitConfig::GAIT_DIAGONAL_TROT) {
        m_dutyFactor = GaitConfig::DEFAULT_DUTY_FACTOR_TROT;
        // Group A: Leg 0 (FL) + Leg 3 (BR) -> Offset 0.0
        // Group B: Leg 1 (FR) + Leg 2 (BL) -> Offset 0.5
        offsets[0] = 0.0f;
        offsets[1] = 0.5f;
        offsets[2] = 0.5f;
        offsets[3] = 0.0f;
    } else {
        m_dutyFactor = GaitConfig::DEFAULT_DUTY_FACTOR_CRAWL;
        // 4-beat crawl: BL(2) -> FL(0) -> BR(3) -> FR(1)
        offsets[2] = 0.00f; // BL
        offsets[0] = 0.25f; // FL
        offsets[3] = 0.50f; // BR
        offsets[1] = 0.75f; // FR
    }

    // Calculate individual leg phases: stance is [0, beta), swing is [beta, 1.0)
    for (int i = 0; i < 4; i++) {
        m_legPhase[i] = fmodf(m_masterPhase + offsets[i], 1.0f);
        m_legInSwing[i] = (m_legPhase[i] >= m_dutyFactor && m_gaitAmplitude > 0.01f);
    }

    // Calculate base forward stride and turn differential
    float baseStride = 0.0f;
    float turnDiff = 0.0f;

    switch (m_currentState) {
        case STATE_WALK_FORWARD:
            baseStride = +m_stepLengthMm * m_gaitAmplitude;
            break;
        case STATE_WALK_BACKWARD:
            baseStride = -m_stepLengthMm * m_gaitAmplitude;
            break;
        case STATE_TURN_LEFT:
            turnDiff = +m_turnIntensityMm * m_gaitAmplitude;
            break;
        case STATE_TURN_RIGHT:
            turnDiff = -m_turnIntensityMm * m_gaitAmplitude;
            break;
        case STATE_STOPPING:
            // Maintain stride direction while ramping down amplitude
            if (m_targetState == STATE_WALK_BACKWARD) {
                baseStride = -m_stepLengthMm * m_gaitAmplitude;
            } else if (m_targetState == STATE_TURN_LEFT) {
                turnDiff = +m_turnIntensityMm * m_gaitAmplitude;
            } else if (m_targetState == STATE_TURN_RIGHT) {
                turnDiff = -m_turnIntensityMm * m_gaitAmplitude;
            } else {
                baseStride = +m_stepLengthMm * m_gaitAmplitude;
            }
            break;
        default:
            baseStride = 0.0f;
            turnDiff = 0.0f;
            break;
    }

    // Left legs: Leg 0 (FL) and Leg 2 (BL)
    // Right legs: Leg 1 (FR) and Leg 3 (BR)
    float stride[4];
    stride[0] = baseStride - turnDiff; // FL
    stride[1] = baseStride + turnDiff; // FR
    stride[2] = baseStride - turnDiff; // BL
    stride[3] = baseStride + turnDiff; // BR

    // Generate foot targets for all 4 legs
    for (int i = 0; i < 4; i++) {
        if (m_currentState == STATE_SIT) {
            outFootTargets[i] = Vector3D(
                RobotGeometry::SIT_STANCE_X,
                RobotGeometry::SIT_STANCE_Y,
                RobotGeometry::SIT_STANCE_Z
            );
        } else if (m_currentState == STATE_STAND && m_gaitAmplitude <= 0.001f) {
            outFootTargets[i] = Vector3D(
                RobotGeometry::DEFAULT_STANCE_X,
                RobotGeometry::DEFAULT_STANCE_Y,
                RobotGeometry::DEFAULT_STANCE_Z
            );
        } else {
            calculateLegTrajectory(i, m_legPhase[i], stride[i], 0.0f, outFootTargets[i]);
        }
    }
}

void GaitPlanner::calculateLegTrajectory(uint8_t legIndex, float phase, float strideX, float strideY, Vector3D& outTarget) {
    (void)legIndex;
    (void)strideY;

    // Master phase: u in [0.0 .. 1.0)
    float u = fmodf(phase, 1.0f);
    if (u < 0.0f) u += 1.0f;

    const float beta = m_dutyFactor; // Stance duty factor (default 0.70)
    const float L = strideX;         // Horizontal step length in mm
    const float H = m_stepHeightMm * m_gaitAmplitude; // Foot lift height in mm

    const float x0 = RobotGeometry::DEFAULT_STANCE_X;
    const float y0 = RobotGeometry::DEFAULT_STANCE_Y;
    const float z0 = RobotGeometry::DEFAULT_STANCE_Z;

    outTarget.y = y0;

    if (u < beta) {
        // --------------------------------------------------------------------
        // STANCE PHASE (0 <= u < beta):
        // Foot supports body, moving backward relative to chassis from +L/2 to -L/2
        // Normalized stance progress: s = u / beta in [0.0 .. 1.0]
        // --------------------------------------------------------------------
        float s = u / beta;

        // Linear ground retraction matching body forward advance:
        // x(s) = +L/2 - L*s
        // At s=0.0: x = x0 + L/2 (touchdown forward placement)
        // At s=0.5: x = x0       (mid-stance under hip)
        // At s=1.0: x = x0 - L/2 (liftoff rear extension)
        outTarget.x = x0 + (L / 2.0f) - L * s;
        outTarget.z = z0; // Ground plane contact

    } else {
        // --------------------------------------------------------------------
        // SWING PHASE (beta <= u < 1.0):
        // Foot lifts, travels forward, and touches down smoothly
        // Normalized swing progress: q = (u - beta) / (1 - beta) in [0.0 .. 1.0]
        // --------------------------------------------------------------------
        float q = (u - beta) / (1.0f - beta);

        // Cubic polynomial smoothstep horizontal trajectory:
        // x(q) = -L/2 + L * (3*q^2 - 2*q^3)
        // At q=0.0: x = x0 - L/2 (smooth liftoff from rear stance position)
        // At q=0.5: x = x0       (passing hip center at maximum clearance)
        // At q=1.0: x = x0 + L/2 (smooth landing at forward touchdown position)
        // Zero-velocity endpoints: dx/dq = 6*L*q*(1 - q) = 0 at q=0 and q=1
        float cubicProgress = 3.0f * q * q - 2.0f * q * q * q;
        outTarget.x = x0 - (L / 2.0f) + L * cubicProgress;

        // Smooth sinusoidal lift arch:
        // z(q) = z0 + H * sin(pi * q)
        // At q=0.0: z = z0 (liftoff)
        // At q=0.5: z = z0 + H (peak clearance apex)
        // At q=1.0: z = z0 (soft ground contact)
        outTarget.z = z0 + H * sinf((float)M_PI * q);
    }
}

void GaitPlanner::cmdWalkForward() {
    m_currentState = STATE_WALK_FORWARD;
    m_targetState = STATE_WALK_FORWARD;
    m_targetAmplitude = 1.0f;
}

void GaitPlanner::cmdWalkBackward() {
    m_currentState = STATE_WALK_BACKWARD;
    m_targetState = STATE_WALK_BACKWARD;
    m_targetAmplitude = 1.0f;
}

void GaitPlanner::cmdTurnLeft() {
    m_currentState = STATE_TURN_LEFT;
    m_targetState = STATE_TURN_LEFT;
    m_targetAmplitude = 1.0f;
}

void GaitPlanner::cmdTurnRight() {
    m_currentState = STATE_TURN_RIGHT;
    m_targetState = STATE_TURN_RIGHT;
    m_targetAmplitude = 1.0f;
}

void GaitPlanner::cmdStop() {
    if (m_currentState != STATE_STAND && m_currentState != STATE_SIT) {
        m_targetState = m_currentState; // Remember previous direction while ramping down
        m_currentState = STATE_STOPPING;
        m_targetAmplitude = 0.0f; // Ramp down gait amplitude to zero
    }
}

void GaitPlanner::cmdStand() {
    m_currentState = STATE_STAND;
    m_targetState = STATE_STAND;
    m_targetAmplitude = 0.0f;
    m_gaitAmplitude = 0.0f;
    m_masterPhase = 0.0f;
}

void GaitPlanner::cmdSit() {
    m_currentState = STATE_SIT;
    m_targetState = STATE_SIT;
    m_targetAmplitude = 0.0f;
    m_gaitAmplitude = 0.0f;
}

void GaitPlanner::cmdIncreaseSpeed() {
    // Decrease cycle period
    float newPeriod = m_cyclePeriodSec - GaitConfig::SPEED_STEP_SEC;
    if (newPeriod < GaitConfig::MIN_CYCLE_PERIOD_SEC) {
        newPeriod = GaitConfig::MIN_CYCLE_PERIOD_SEC;
    }
    m_cyclePeriodSec = newPeriod;
}

void GaitPlanner::cmdDecreaseSpeed() {
    // Increase cycle period
    float newPeriod = m_cyclePeriodSec + GaitConfig::SPEED_STEP_SEC;
    if (newPeriod > GaitConfig::MAX_CYCLE_PERIOD_SEC) {
        newPeriod = GaitConfig::MAX_CYCLE_PERIOD_SEC;
    }
    m_cyclePeriodSec = newPeriod;
}

void GaitPlanner::cmdToggleGait() {
    if (m_activeGait == GaitConfig::GAIT_DIAGONAL_TROT) {
        m_activeGait = GaitConfig::GAIT_CRAWL_WAVE;
        m_dutyFactor = GaitConfig::DEFAULT_DUTY_FACTOR_CRAWL;
    } else {
        m_activeGait = GaitConfig::GAIT_DIAGONAL_TROT;
        m_dutyFactor = GaitConfig::DEFAULT_DUTY_FACTOR_TROT;
    }
}

void GaitPlanner::setCyclePeriod(float periodSec) {
    m_cyclePeriodSec = constrain(periodSec, GaitConfig::MIN_CYCLE_PERIOD_SEC, GaitConfig::MAX_CYCLE_PERIOD_SEC);
}

void GaitPlanner::setStepLength(float lengthMm) {
    m_stepLengthMm = constrain(lengthMm, GaitConfig::MIN_STEP_LENGTH_MM, GaitConfig::MAX_STEP_LENGTH_MM);
}

void GaitPlanner::setStepHeight(float heightMm) {
    m_stepHeightMm = constrain(heightMm, GaitConfig::MIN_STEP_HEIGHT_MM, GaitConfig::MAX_STEP_HEIGHT_MM);
}

void GaitPlanner::setGaitType(GaitConfig::GaitType type) {
    m_activeGait = type;
    m_dutyFactor = (type == GaitConfig::GAIT_DIAGONAL_TROT) ? 
                   GaitConfig::DEFAULT_DUTY_FACTOR_TROT : 
                   GaitConfig::DEFAULT_DUTY_FACTOR_CRAWL;
}

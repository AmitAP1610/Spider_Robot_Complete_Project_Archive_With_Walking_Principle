# Technical Inspection Report: Original Code & Mechanical Hardware

**Project:** 12-Servo ESP32 + PCA9685 Quadruped / Spider Robot  
**Document Type:** Pre-Implementation Architectural & Hardware Audit  

---

## 1. Executive Summary

An in-depth engineering inspection was performed across all legacy codebase variants (`Spider_Walk_12Servo.ino`, `QUADROPAD_calibrated_servo_code.ino`, `QUADROPAD_12Servo_Integrated.ino`, and original Sesame archives) and the physical robot assembly documentation (photos, wiring diagrams, and CAD renders).

The original software stack suffered from critical limitations:
1. **Zero Inverse Kinematics**: Leg motion relied on hardcoded open-loop servo angles (`90°`, `100°`, `80°`) rather than Cartesian foot trajectories in Euclidean space.
2. **Blocking Delays Throughout**: Movement functions executed blocking `delay(350)` and `delay(600)` calls, freezing microcontroller execution and rendering stop commands unresponsive during walking cycles.
3. **Flawed Gait Mechanics**: At the end of every step cycle, the legacy sketch invoked `standPose()`, resetting all servos to 90°. This erased net stride progression, causing the robot to rock in place and drag its feet backward.
4. **Incorrect Hardware Assumptions**: Servos were commanded to 90° under the assumption that 90° corresponds to mechanical neutral across all joints. In reality, spline tooth misalignments, horn mounting orientations, and mirrored left/right linkages produce severe asymmetric stance errors unless individually calibrated.
5. **Overloaded Extraneous Peripherals**: Integrating Wi-Fi, web servers, OLED animators, and distance sensors onto the same ESP32 caused I2C bus contention with the PCA9685 and unpredictable loop timing.

---

## 2. Detailed Inspection of Legacy Code Variants

### 2.1 `Spider_Walk_12Servo.ino` (12-Servo Open-Loop Baseline)
- **Strengths**: Established the correct PCA9685 I2C address (`0x40`), I2C pins (SDA 21, SCL 22), and channel mapping for 12 servos:
  - FL: 0, 1, 2
  - FR: 4, 5, 6
  - BL: 8, 9, 10
  - BR: 12, 13, 14
- **Critical Limitations**:
  - **No Inverse Kinematics**: Used fixed angle heuristics:
    ```cpp
    setLeg(0, CENTER_ANGLE + COXA_SWING, CENTER_ANGLE - FEMUR_LIFT, CENTER_ANGLE + TIBIA_ADJUST);
    ```
  - **Blocking Architecture**: Used sequential `delay(STEP_DELAY_MS)` (350 ms). The main loop cannot process user stop commands while executing a step.
  - **Zero Velocity Discontinuity**: Commanded sudden step jumps between poses with no slew-rate acceleration limiting, subjecting small nylon servo gears to severe shock torque.
  - **Destructive Stride Reset**: Called `standPose()` at line 93 after every diagonal step, forcing all feet back to center and scrubbing the feet along the floor.

---

### 2.2 `QUADROPAD_calibrated_servo_code.ino` (Calibration Variant)
- **Strengths**: Introduced per-channel direction multipliers (`servoDirection[channel] = {1, 1, 1...}`) and neutral offset arrays.
- **Critical Limitations**:
  - Designed for an **8-servo quadruped** (CH0–CH7), omitting the 4 coxa yaw servos.
  - Hardcoded tick mapping `pulseMin = 110` and `pulseMax = 510` without verifying microsecond pulse width endpoints.
  - Serial monitor interface required interactive channel typing but lacked real-time trajectory coordination.

---

### 2.3 `QUADROPAD_12Servo_Integrated.ino` & Sesame Firmware Stack
- **Inspected Functions**:
  - `angleToPulse()` / `angleToPcaTicks()`: Linearly mapped 0–180° to ticks without per-servo endpoint calibration.
  - `setServo()` / `setServoAngle()`: Directly updated PCA9685 registers with an inline blocking `delayWithFace()`.
  - `standPosition()` / `runStandPose()`: Commanded all servos to 90°.
  - `sitPosition()` / `runSitPose()`: Commanded coxa to 90°, femur to 78° (or 50°), tibia to 78° (or 40°).
  - `shakeFrontLegs()` / `runShakePose()`: Looped femur between 100° and 80° with 250 ms delays.
  - `moveForward()` / `runWalkPose()`: Looped `stepPair()` diagonally with fixed angles (100° lift, 80°/100° swing).
- **Critical Limitations**:
  - High memory overhead and task contention caused by OLED frame buffering (SSD1306 128x64) over the same I2C bus as the PCA9685.
  - Wi-Fi Access Point and WebServer interrupts introduced jitter into servo timing.
  - Fixed-angle gait was incapable of differential turning; turning was simulated by arbitrary coxa angle offsets that slipped on high-friction surfaces.

---

## 3. Physical Mechanical Assembly & Photograph Audit

Photographs (`d43f40c3a1a075fa...png`, `949785b7a762...png`) and wiring schematics were inspected:

### 3.1 3D-Printed Brackets & Linkage Kinematics
1. **Coxa Joint (Hip Yaw)**:
   - Rotates around a vertical axis normal to the chassis top plate.
   - Sweep range is limited mechanically by the chassis corner cutout. Excessive yaw sweeps ($> \pm 35^\circ$) cause the coxa horn to strike the chassis frame.
   - Measured coxa link length: **$L_c = 42.2\text{ mm}$**.
2. **Femur Joint (Hip Pitch)**:
   - Rotates around a horizontal axis perpendicular to the coxa link.
   - Elevated elbow arrangement: in neutral stance, the femur is nearly horizontal (sloping $+5^\circ$ to $+10^\circ$ upward).
   - Measured femur link length: **$L_f = 65.4\text{ mm}$**.
3. **Tibia Joint (Knee Pitch)**:
   - Rotates around a horizontal axis parallel to the femur pitch axis.
   - Points the lower leg downward to the ground with a neutral knee bend of approximately $90^\circ$.
   - Measured tibia link length: **$L_t = 97.5\text{ mm}$**.

### 3.2 Mirrored Mounting Geometry
- Servos on the **Left Side** (Legs 0 and 2) and **Right Side** (Legs 1 and 3) are mounted as physical mirror images.
- A positive rotation on a Left Coxa swings the foot **forward**, whereas the identical positive PWM command on a Right Coxa swings the foot **backward**.
- **Requirement**: The software must maintain independent direction multipliers (`+1` for normal, `-1` for inverted) for every joint, rather than assuming uniform angular behavior.

### 3.3 Electrical & Power Distribution Audit
- Inspection of the custom PCB (`sesame_robot_report.docx`) revealed an onboard DC-DC buck converter footprint rated for **3A continuous**.
- **Current Demand Analysis**:
  - A single MG90S / SG90 servo under mechanical load draws $400\text{ mA} - 700\text{ mA}$, with a stall current of up to $0.85\text{A} - 1.0\text{A}$.
  - During walking, at least 6 to 8 servos are simultaneously accelerating and bearing body mass:
    $$I_{\text{active}} \approx 6 \times 0.6\text{A} = 3.6\text{A} \quad (\text{transient peaks exceed } 5.0\text{A})$$
  - A 3A regulator (or a standard LM2596 module) operates at or beyond its thermal threshold. Under peak acceleration, the 5V rail collapses, triggering the ESP32 brownout detector.
- **Mandatory Hardware Requirement**:
  - The servo power rail (PCA9685 `V+`) must be powered by a dedicated high-current switching regulator rated for at least **$5\text{A} - 8\text{A}$ continuous**.
  - A $1000\,\mu\text{F} - 2200\,\mu\text{F}$ low-ESR buffer capacitor must be installed directly across `V+` and `GND`.
  - ESP32 GND and PCA9685 GND must share a heavy-gauge common ground.

---

## 4. Architectural Improvements in the Replacement Controller

| Engineering Dimension | Original Legacy Implementation | Replacement Controller Architecture |
|---|---|---|
| **Locomotion Paradigm** | Open-loop fixed angles (`setServo(ch, 90)`) | Closed-form 3-DOF analytical Inverse Kinematics ($L_c, L_f, L_t$) |
| **Foot Trajectory** | None (instantaneous pose jumping) | Continuous cubic smoothstep swing with sinusoidal lift arch |
| **Loop Timing** | Blocking `delay(350)` calls ($< 3\text{ Hz}$ effective) | Non-blocking $50\text{ Hz}$ ($20\text{ ms}$) deterministic state machine |
| **Stride Continuity** | Cancelled by `standPose()` after each step | Smooth cyclic progression; stance retracts foot linearly along ground |
| **Turning Kinematics** | Arbitrary coxa offsets | True differential stride kinematics ($S_{\text{left}}$ vs $S_{\text{right}}$) |
| **Acceleration Control**| None (causes gear stripping and chassis wobble) | Configurable slew-rate limiter (`MAX_JOINT_SPEED_DEG_PER_SEC`) |
| **Calibration Architecture**| Global 90° assumption; hardcoded ticks | 12-joint calibration records (center, direction, pulse limits, ticks) |
| **Reachability Safety** | None (can pass out-of-range targets to servos) | Geometric reach check $D \in [-1, 1]$ with automatic safe projection |
| **Scope Discipline** | Bloated with Wi-Fi, WebServer, OLED, Sonar | Strict walking-only; zero sensor dependencies |

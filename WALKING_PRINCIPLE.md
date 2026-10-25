# Spider Robot Walking Principle — Technical Reference and Calculation Workbook

**Project target:** ESP32 + PCA9685 + 12-servo quadruped spider robot  
**Document status:** Engineering reference / preliminary calculations. This is not a claim that the current robot's gait is validated.  
**Important data limitation:** Actual coxa offset, femur and tibia lengths, chassis dimensions, mass, centre of mass, exact servo model, measured servo pulse endpoints, and per-joint neutral angles were not supplied. Therefore, robot-specific numeric IK, torque, balance, and power sizing cannot be completed honestly yet. Equations below are general, and worked values are explicitly illustrative. Replace assumptions with measurements before use.

## 1. Project configuration currently recorded

- Controller: ESP32 DevKit.
- Servo driver: PCA9685, expected I2C address `0x40`.
- I2C pins: SDA GPIO 21, SCL GPIO 22.
- Servos: 12, three per leg: coxa, femur, tibia.
- Logical leg order in current sketch: 0 = Front Left (FL), 1 = Front Right (FR), 2 = Back Left (BL), 3 = Back Right (BR).
- PCA9685 channels: FL 0/1/2; FR 4/5/6; BL 8/9/10; BR 12/13/14. Channels 3, 7, 11, 15 unused.
- Nominal PWM frequency in sketch: 50 Hz.
- I2C logic: PCA9685 VCC to ESP32 3V3; SDA/SCL to GPIO 21/22; common ground required.
- Servo rail: regulated supply, typically 5 V only if compatible with the specific servos; supply positive to PCA9685 V+, negative to servo-power GND. Common ground between ESP32, PCA9685 and servo supply.
- Current standalone sketch commands `W` = one cycle, `S` = center/stand pose, `X` = return to center. `X` is not a power cutoff.
- Current sketch uses an assumed tick range of 100–520 and starting centre angles of 90°. These values are not calibrated or guaranteed safe for all servos.

## 2. What “walking” means

A gait is a time-coordinated pattern of stance and swing phases for each leg. During stance, a foot contacts the ground and can transmit force to the body. During swing, the foot lifts, travels to a new placement, and lands. Servo movement alone does not guarantee foot lift, traction, forward displacement, or balance.

A robust control chain is:
1. Desired body motion (forward/backward/turn/stand).
2. Gait planner assigns each leg a phase and stance/swing state.
3. Foot trajectory planner calculates desired foot position.
4. Inverse kinematics (IK) converts foot position to joint angles.
5. Per-servo calibration converts model angles to servo commands.
6. Joint limits and reachability checks reject unsafe targets.
7. ESP32 sends commands over I2C to PCA9685; PCA9685 outputs PWM.
8. Optional feedback (IMU, contact sensors, current sensing) measures errors and supports closed-loop control.

The current fixed-angle sketch implements only a basic open-loop pose sequence. It does not implement trajectory-based IK, ground-contact detection, IMU balance, or closed-loop gait correction.

## 3. Leg anatomy, degrees of freedom and coordinates

- Coxa: usually controls horizontal leg sweep/yaw.
- Femur: upper-leg pitch, depending on mounting orientation.
- Tibia: lower-leg/knee pitch, depending on linkage.
- Nominal total DOF = 4 legs × 3 joints/leg = 12 independently commanded joint variables.

Adopt a body frame for calculations: x forward, y left, z upward. Each leg can use a local frame at its coxa/hip reference. Transformations between the body frame and each local leg frame must account for the position and orientation of that hip. Those offsets and rotations are not yet measured for this robot.

Joint vector: q = [theta_FL_coxa, theta_FL_femur, theta_FL_tibia, ..., theta_BR_tibia]^T.

## 4. Gait patterns and diagonal timing

Common quadruped gaits include crawl (one leg at a time), trot (diagonal pairs), pace (same-side pairs), and bound (front pair/rear pair). For a basic diagonal trot:
- Pair A = {FL, BR}
- Pair B = {FR, BL}
- Ideal phase offset between pairs = 180 degrees = pi radians = 0.5 cycles.

Normalized phase for leg i:

`phase_i(t) = frac(t / T + delta_i)`

where `T` is gait period in seconds, `delta_i` is phase offset in cycles, and `frac(x) = x - floor(x)`.

Example offsets: FL 0.0, BR 0.0, FR 0.5, BL 0.5. These define timing only; they do not prove that the body remains stable. The exact trot support pattern can be dynamically unstable depending on duty factor, timing, speed and mass distribution.

Duty factor:

`beta = T_stance / T`

`T = T_stance + T_swing`

`T_stance = beta * T`; `T_swing = (1 - beta) * T`.

**Illustrative calculation only:** let T = 2.0 s and beta = 0.75. Then T_stance = 1.5 s, T_swing = 0.5 s, and gait frequency f = 1/T = 0.5 Hz. These are example settings, not measured or validated values for the robot.

## 5. Foot trajectory calculations

A desired foot position is p = [x, y, z]^T. During stance, the foot is approximately stationary relative to the ground, so relative to a forward-moving body it moves backward. Under ideal no-slip conditions, a simplified body-relative trajectory is:

`x_f(t) = x_front - v * t`, for `0 <= t <= T_stance`.

For a stance stroke length L:

`v_average ≈ L / T_stance`.

During swing, define normalized progress `u = t / T_swing`, with 0 <= u <= 1:

`x_f(u) = x_rear + (x_front - x_rear) * u`

`z_f(u) = z0 + h * sin(pi * u)`

where h is foot lift height and z0 is the nominal ground-relative height in the selected coordinate convention. This trajectory begins and ends at z0 and reaches z0+h at u=0.5. It is an example trajectory; smoother landing profiles may be needed to reduce impact.

**Illustrative example:** L = 40 mm, h = 20 mm, T_stance = 1.5 s gives idealized v_average = 40/1.5 = 26.7 mm/s during stance. The 40 mm and 20 mm values are not measurements for this chassis and must not be copied without checking reachability and collisions.

## 6. Inverse kinematics (IK): simplified 3-DOF leg

IK maps desired foot position to joint angles. It depends on actual joint-axis orientation, link lengths, coordinate conventions, and servo zero offsets. Measure these before implementing the equations in firmware.

Let:
- L0 = horizontal coxa offset/length (if applicable).
- L1 = femur length measured between joint axes.
- L2 = tibia length measured between joint axes.
- (x, y, z) = target foot position relative to the relevant hip reference.

For a conventional coxa yaw joint:

`theta1 = atan2(y, x)`

`rho = sqrt(x*x + y*y)`

`r = rho - L0`

`d = sqrt(r*r + z*z)`

For the planar femur/tibia triangle, the law of cosines gives:

`Ck = (L1*L1 + L2*L2 - d*d) / (2*L1*L2)`

`gamma = acos(Ck)`

The target is unreachable in this planar model if d > L1+L2 or d < abs(L1-L2). Numerically clamp Ck to [-1, 1] only after checking reachability and floating-point tolerance; clamping must not disguise an unreachable target.

Define:

`alpha = atan2(z, r)`

`Cf = (L1*L1 + d*d - L2*L2) / (2*L1*d)`

`delta = acos(Cf)`

One possible femur solution is `theta2 = alpha + delta`; another is `alpha - delta`. The knee/servo angle may be `gamma`, `pi - gamma`, or a transformed angle depending on whether the model uses internal triangle angle or relative joint angle. Verify the correct branch and signs against the actual assembly.

Trigonometric functions in Arduino C/C++ use radians. Conversion: `degrees = radians * 180 / PI`; `radians = degrees * PI / 180`.

### Reachability checks

For the planar two-link portion:

`abs(L1 - L2) <= d <= L1 + L2`.

The usable workspace is smaller after joint-angle limits, body collisions, servo horn geometry and cable constraints are considered. Reject unreachable targets instead of silently commanding a clamped joint pose.

### Forward kinematics check

Under one common planar convention, the radial and vertical coordinates are:

`r = L1*cos(theta2) + L2*cos(theta2 + theta3)`

`z = L1*sin(theta2) + L2*sin(theta2 + theta3)`

These equations assume theta3 is a relative knee angle and angles are measured from the same stated reference. They must be modified if the robot's joint convention differs. Forward kinematics can check the foot location implied by a set of joint angles.

## 7. Servo calibration and angle mapping

The model angle and the servo command angle are different quantities. For servo i:

`theta_command_i = center_i + direction_i * delta_theta_i`

where `center_i` is its measured neutral angle, `direction_i` is +1 or -1, and `delta_theta_i` is the model offset in degrees. Then clamp only to measured safe limits:

`theta_safe_i = min(max_i, max(min_i, theta_command_i))`.

**Illustrative example:** center = 92 degrees, direction = -1, offset = +8 degrees gives command = 92 + (-1*8) = 84 degrees. This is only valid if 84 degrees is within that servo's calibrated safe range.

Calibration steps:
1. Lift/support robot and remove mechanical load where possible.
2. Verify correct channel and servo orientation.
3. Use a conservative pulse/angle command.
4. Fit servo horn at neutral with power disconnected.
5. Move in small increments and observe direction.
6. Record centre, direction, safe min/max and notes per servo.
7. Check full linkage travel for frame/ground collisions.
8. Repeat all 12 joints individually.
9. Test the stand pose with the robot supported before gait tests.

Do not assume mirrored legs necessarily use the same direction pattern. Determine DIR empirically for each joint.

## 8. PCA9685 PWM calculations

The current sketch requests 50 Hz PWM, giving a nominal period:

`T_PWM = 1 / 50 = 0.020 s = 20 ms`.

PCA9685 has 4096 counts per PWM period. Approximate pulse width:

`t_pulse = ticks / 4096 * T_PWM`.

For the sketch's assumed 100 ticks: `100 / 4096 * 20 ms = 0.488 ms = 488 us`.
For 520 ticks: `520 / 4096 * 20 ms = 2.539 ms = 2539 us`.

These endpoints may be outside the recommended range for some servos. Do not treat them as universal. Check servo specifications and test conservatively. Linear angle-to-tick mapping assumes an approximate linear servo response; actual mechanical angle and pulse relationship can vary.

## 9. Smooth motion and timing

Gait frequency: `f_gait = 1 / T`.

Linear interpolation from angle theta0 to theta1 over a movement duration Tm:

`theta(t) = theta0 + (theta1 - theta0) * ((t - t0) / Tm)` for t0 <= t <= t0+Tm.

This reduces abrupt position jumps but gives constant angular velocity and abrupt acceleration at segment boundaries. A cubic smoothstep `s(u)=3u^2-2u^3`, with `u=(t-t0)/Tm` clamped to [0,1], can be used as `theta=theta0+(theta1-theta0)*s(u)` for zero endpoint velocity. More advanced profiles may be required for heavy loads or high speeds.

The current sketch uses 350 ms waits for two phases, then returns all joints to centre. It is a single open-loop demonstration cycle, not a continuous trajectory planner. Returning all joints to centre may erase net stride and cause rocking or foot drag.

## 10. Balance and support polygon

The support polygon is the convex hull of ground-contact feet. For quasi-static stability, the vertical projection of the centre of mass should remain inside this polygon with a margin. This criterion alone does not guarantee dynamic stability during fast motion, impacts, or slipping.

Practical precautions: low speed, short steps, verified contact, wide support, low centre of mass where possible, no binding, and conservative load. A diagonal trot can have intervals with only two diagonal contacts; stability depends on foot placement and dynamics. A slow crawl may be a better first physical gait for a heavy or uncalibrated hobby-servo robot.

## 11. Power and current calculations

Servo rail uses a regulated supply appropriate for the servo model (often 5 V for common hobby servos). PCA9685 VCC is logic supply; V+ is servo rail. ESP32, PCA9685 and servo supply must share ground. Never route the servo load current through the ESP32 3.3 V pin.

Worst-case upper-bound supply estimate:

`I_supply >= sum(I_peak_i for i = 1..12)`.

This is a conservative sizing bound if all servos could reach peak current simultaneously, not a prediction of typical current. Obtain stall/peak current from each servo datasheet, then consider wire, connector, converter thermal rating, transient response, fuse/protection, and battery discharge capability. A small LM2596 board may be insufficient for 12 servos; verify its real continuous current and cooling rather than relying on advertised peak ratings.

Battery energy estimate if capacity is known:

`E_Wh = V_nominal * capacity_Ah`.

Ideal runtime estimate:

`runtime_hours ≈ E_Wh * efficiency / P_average_W`.

This is only an estimate; actual runtime depends on battery discharge limits, converter efficiency, servo load and duty cycle. Do not use it without real voltage, capacity, efficiency and measured/estimated average power.

## 12. Torque and mechanical load

For a simplified static load at a lever arm r, required joint torque is approximately:

`tau = F * r_perpendicular`.

If a supported mass m contributes force F = m*g, then `tau = m*g*r_perpendicular`, where g ≈ 9.81 m/s^2. This is a simplified static estimate; actual joint torque depends on linkage geometry, load sharing, leg posture, acceleration and friction. Dynamic and impact loads can be much larger. Use servo torque ratings at the actual supply voltage and maintain margin; do not design around stall torque as a continuous rating.

A robot mass and lever arms were not supplied, so no honest numeric torque sizing can be made yet.

## 13. Body speed and step length

Under ideal no-slip assumptions, if the effective stride length is L and each complete gait cycle advances the body by approximately L, then:

`v_average ≈ L / T`.

This is only an ideal relation. The actual displacement may be lower or zero if feet slip, drag, reverse, or the body does not shift as expected. Measure displacement over several cycles to estimate real speed:

`v_measured = distance_measured / elapsed_time`.

## 14. Firmware logic and pseudocode

```text
initialize I2C and PCA9685
load measured calibration values
command verified standing pose
wait for W/S/X command
while walking requested:
    update gait phase using elapsed time
    for each leg:
        determine stance or swing
        compute desired foot x,y,z
        check reachability and collision limits
        solve IK and select correct joint branch
        convert model angles using per-servo calibration
        send safe PWM targets
    check stop/safety condition
if stop requested:
    move to a known safe pose if safe to do so
```

Do not block indefinitely in delay loops if responsive stop commands are needed. A software stop command cannot cut power; provide a physical power switch or accessible battery disconnect for emergencies.

## 15. Current sketch: limitations that must be documented

- Fixed offsets are not IK.
- 90 degrees is only a default centre, not necessarily the mechanical standing angle.
- Direction multipliers are guesses until each joint is tested.
- Angle limits 75–105 or 70–110 are software bounds, not proof of mechanical safety.
- Tick range 100–520 is unverified for the specific servos.
- A two-phase diagonal pose sequence does not guarantee correct stance/swing or forward displacement.
- `standPose()` resets joints to centre at the end of a cycle and may reduce net travel.
- No foot-contact sensor, IMU, current sensor, body attitude feedback, slip detection or closed-loop correction is present.
- No successful compile or physical gait test is implied by this document.

## 16. Test and validation plan

| Test | Method | Pass criteria |
|---|---|---|
| I2C/PCA9685 | Scan I2C or run driver check | Address 0x40 responds |
| One servo | Test each channel individually | Correct servo moves smoothly, no binding |
| Calibration | Record centre, direction and limits | Repeatable safe range per joint |
| Stand pose | Support body while commanding pose | No obvious strain, all feet positioned appropriately |
| Foot swing | Test one leg lifted/supporting chassis | Foot clears ground and frame |
| Phase order | Observe diagonal pair timing | Intended pairs alternate |
| Slow gait | Short cycles with support available | No severe tipping, collision or stalls |
| Repeatability | Run multiple cycles and measure displacement | Consistent movement and no overheating |
| Power | Measure supply voltage/current during loaded motion | Voltage stays within component limits; no resets or hot wiring |

Record firmware version, servo model, centre angle, direction, min/max, pulse range, supply voltage, measured current, step length, lift height, gait period, and test observations.

## 17. Troubleshooting

- One whole leg does not move: verify its three plugs, channel mapping, servo rail, common ground, OE pin, and whether the problem follows the servo or channel after a safe swap test.
- One joint does not move: test the servo on a known-working channel and test the suspect channel with a known-working servo.
- Jitter/reset: investigate supply current capability, voltage drop, ground wiring, connector quality and electrical noise.
- Servo buzzes/stalls: disconnect servo power immediately; check mechanical binding and pulse endpoints.
- Foot scrapes: inspect joint direction, foot trajectory, link lengths, and calibration.
- Robot tips: reduce speed and step length; review contact sequence, support polygon, centre of mass and foot placement.
- Robot rocks but does not advance: likely foot timing/stance/trajectory problem; fixed-angle poses may return the foot to the starting position without net stride.

## 18. Required measurements before robot-specific calculations can be finalized

Fill in these values from the physical robot:

| Parameter | Value | How to measure |
|---|---|---|
| Coxa offset/length L0 for each leg | TBD mm | Measure joint-axis to femur-axis geometry |
| Femur length L1 for each leg | TBD mm | Centre-to-centre between joint axes |
| Tibia length L2 for each leg | TBD mm | Centre-to-centre between joint axes |
| Hip positions in body frame | TBD mm and angles | Measure from body-frame origin |
| Body length/width/height | TBD mm | Measure chassis |
| Robot mass | TBD kg | Weigh assembled robot |
| Centre of mass | TBD mm | Estimate or measure with supported balance method |
| Servo model | TBD | Read label/datasheet |
| Servo peak/stall current at target voltage | TBD A | Datasheet or current-limited measurement |
| Servo torque rating at target voltage | TBD kg·cm or N·m | Datasheet |
| Safe pulse width per servo | TBD us | Datasheet and cautious calibration |
| Centre angle per servo | TBD degrees | Calibration test |
| Direction multiplier per servo | +1 or -1 | Observe motion |
| Safe angle limits per servo | TBD degrees | Mechanical travel test |
| Battery voltage/capacity | TBD V/Ah | Battery label and measurement |
| Converter continuous rating | TBD A | Module specification and thermal test |

Until these measurements are filled in, any specific numeric IK angles, torque sizing, stability margin, or battery runtime would be speculative.

## 19. Calculation summary

The key equations gathered in this document are:

1. Degrees of freedom: `DOF = 4 * 3 = 12`.
2. Gait frequency: `f = 1/T`.
3. Duty factor: `beta = T_stance/T`.
4. Stance/swing durations: `T_stance=beta*T`, `T_swing=(1-beta)*T`.
5. Ideal stride speed: `v ≈ L/T` (only under stated assumptions).
6. Swing progress: `u=t/T_swing`.
7. Swing height: `z=z0+h*sin(pi*u)`.
8. Coxa yaw: `theta1=atan2(y,x)`.
9. Radial distance: `r=sqrt(x^2+y^2)-L0`.
10. Hip-to-foot distance: `d=sqrt(r^2+z^2)`.
11. Two-link reachability: `abs(L1-L2) <= d <= L1+L2`.
12. Knee law of cosines: `cos(gamma)=(L1^2+L2^2-d^2)/(2*L1*L2)`.
13. Servo mapping: `theta_cmd=center+direction*offset`.
14. PWM period at 50 Hz: `20 ms`.
15. Pulse width: `ticks/4096 * 20 ms`.
16. Static torque approximation: `tau=F*r_perpendicular`.
17. Power energy: `E_Wh=V*Ah`.
18. Ideal runtime: `runtime≈E_Wh*efficiency/P_average`.
19. Measured speed: `distance/time`.

## 20. Safety

- Disconnect servo power before rewiring or fitting servo horns.
- Verify buck converter output with a multimeter before attaching servos.
- Do not power servos from ESP32 3.3 V.
- Do not connect a raw 2S lithium pack directly to a 5 V servo rail.
- Use wiring, connectors, supply and protection rated for expected current.
- Lift/support the robot for initial tests.
- Keep hands away from linkages while powered.
- Stop and disconnect power if a servo binds, buzzes, stalls, overheats, or pushes against a mechanical stop.
- `X` is a software command, not an emergency disconnect.

---

**Document conclusion:** This file provides the walking principles, equations, assumptions, illustrative calculations, calibration process, electrical sizing methods, validation plan and outstanding measurement list. It is comprehensive as a project reference, but robot-specific numeric calculations cannot be finalized until the actual dimensions, mass, servo model, calibration values and power specifications are supplied.

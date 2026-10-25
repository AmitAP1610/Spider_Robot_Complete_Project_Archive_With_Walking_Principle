# Quadruped Spider Robot: Diagnostics & Troubleshooting Guide

This troubleshooting guide addresses the most common electrical, mechanical, kinematic, and firmware issues encountered when building and testing an ESP32 + PCA9685 12-servo quadruped robot.

---

## 1. Electrical & Power Issues

### 1.1 ESP32 Reboots or Resets Unexpectedly (Brownout Detector)
- **Symptom:** As soon as the robot begins walking or lifts multiple legs, the ESP32 resets, prints `Brownout detector was triggered`, and restarts into setup.
- **Cause:** When multiple hobby servos accelerate simultaneously, their combined instantaneous current draw can exceed $4\text{A} - 6\text{A}$. If the battery or 5V regulator cannot supply this transient surge, the 5V rail collapses momentarily. This voltage dip bleeds through to the ESP32's 3.3V LDO regulator, triggering the internal brownout detector.
- **Solutions:**
  1. **Strict Power Isolation:** Never draw servo power from the ESP32 5V or 3.3V pins. The PCA9685 `V+` screw terminal must receive power directly from an external regulated 5V power supply.
  2. **Add Bulk Capacitance:** Connect a $1000\,\mu\text{F}$ to $2200\,\mu\text{F}$ low-ESR electrolytic capacitor directly across the PCA9685 `V+` and `GND` screw terminals (observing correct polarity!). This reservoir capacitor buffers the instantaneous current spikes demanded by servo motor brushes.
  3. **Adequate Regulator Sizing:** A standard LM2596 module is rated for $2\text{A}$ continuous ($3\text{A}$ with heat sink). For 12 servos, replace the LM2596 with a high-current buck converter rated for $5\text{A} - 8\text{A}$ continuous (e.g. XL4015 or continuous synchronous step-down converter), or use a 2S LiPo battery through an 8A UBEC.
  4. **Wiring Gauge:** Use thick wire (at least 20 AWG or 18 AWG) for the servo power supply and ground lines to avoid resistive voltage drops.

---

### 1.2 PCA9685 Not Detected (`PCA9685 did not respond at address 0x40!`)
- **Symptom:** Boot log reports `WARNING: PCA9685 did not respond at address 0x40!`.
- **Cause:** I2C bus wiring fault, incorrect address, or missing logic power.
- **Solutions:**
  1. **Check SDA & SCL Lines:** Verify that ESP32 GPIO 21 connects to PCA9685 SDA, and GPIO 22 connects to PCA9685 SCL. Swapping SDA and SCL is the #1 most common wiring error.
  2. **Check Logic VCC:** Verify that PCA9685 `VCC` pin connects to ESP32 `3.3V`. (Do not confuse `VCC` with the `V+` servo power terminal).
  3. **Check Common Ground:** Confirm continuity between ESP32 GND and PCA9685 GND.
  4. **Check Address Solder Jumpers:** By default, all address solder pads on the PCA9685 (A0 to A5) are open, configuring the I2C address as `0x40`. If any solder bridges exist across A0..A5, the board will be at a different address.

---

## 2. Mechanical & Servo Linkage Issues

### 2.1 Servo Buzzes, Hums, or Gets Hot at Rest
- **Symptom:** In standing pose (`H`), one or more servos emit a constant high-pitched hum or buzz and warm up quickly.
- **Cause:** The servo is being commanded to an angle that it cannot physically reach because the mechanical horn or link is jammed against the chassis, another link, or the travel limit of the internal gearbox potentiometer.
- **Solutions:**
  1. **Switch Off Servo Power Immediately:** Do not leave a stalling servo energized; hobby servo motors (like SG90 or MG90S) will burn out their drive transistors or melt nylon gears within 60 seconds of stalling.
  2. **Recalibrate Center Angle:** Enter calibration mode (`C`) and select the buzzing joint. Jog the joint by $1^\circ - 3^\circ$ away from the bind point until the humming stops completely.
  3. **Check Horn Spline Alignment:** Unscrew the horn from the servo output spline and reposition it one tooth closer to true mechanical neutral before tightening.

---

### 2.2 Stripped Servo Gears or Missing Teeth
- **Symptom:** The servo motor spins rapidly or makes a clicking/grinding sound, but the leg link slips or does not move.
- **Cause:** High impact forces during foot landing, excessive body weight, or sudden direction reversals exceeding the mechanical shear strength of nylon gears.
- **Solutions:**
  1. **Metal-Gear Servos:** Standard nylon SG90 servos are extremely fragile. Upgrade to metal-gear servos (e.g. MG90S or MG996R) for all 12 joints, especially the femur and tibia pitch joints which bear the entire robot weight.
  2. **Enforce Rate Limiting:** In `GaitConfig.h`, keep `MAX_JOINT_SPEED_DEG_PER_SEC` at a conservative value ($120^\circ/\text{s} - 160^\circ/\text{s}$) to avoid shock torque spikes during direction changes.
  3. **Smooth Trajectories:** Ensure `DEFAULT_STEP_HEIGHT_MM` is kept moderate ($12\text{ mm} - 15\text{ mm}$) to reduce landing impact kinetic energy.

---

## 3. Kinematic & Locomotion Issues

### 3.1 Leg Moves in the Wrong Direction (Inverted Motion)
- **Symptom:** When walking forward, one leg pushes forward instead of backward, or a knee bends outward into an unnatural hyperextended pose.
- **Cause:** The mechanical servo horn was installed in an inverted orientation relative to the theoretical kinematic model, or the joint direction multiplier in `CalibrationConfig.h` is inverted.
- **Solutions:**
  1. Enter calibration mode (`C`).
  2. Select the malfunctioning joint (0 to 11).
  3. Press `I` to toggle the direction multiplier between `+1` and `-1`.
  4. Test the motion with `+` and `-`.
  5. Save the updated direction in `CalibrationConfig.h`.

---

### 3.2 Foot Drags on the Floor During Swing Phase
- **Symptom:** During swing, the foot fails to clear the floor, catching on the surface and causing the robot to stumble or pull its body downward.
- **Cause:**
  - Foot lift height is set too low.
  - Link lengths in `RobotGeometry.h` are smaller than the physical links on the robot.
  - Leg links are bending or flexing under load.
- **Solutions:**
  1. In `GaitConfig.h`, increase `DEFAULT_STEP_HEIGHT_MM` from $15.0\text{ mm}$ to $18.0\text{ mm}$ or $20.0\text{ mm}$.
  2. Measure your actual physical link lengths ($L_c, L_f, L_t$) using calipers and update `RobotGeometry.h`.
  3. Verify that the robot's default standing height (`DEFAULT_STANCE_Z = -90.0f`) provides adequate femur angle range.

---

### 3.3 Robot Turns or Drifts to One Side When Commanded to Walk Straight
- **Symptom:** Sending `W` causes the robot to curve noticeably to the left or right rather than advancing in a straight line.
- **Cause:** Asymmetric mechanical friction across legs, mismatched servo centers, or unequal traction on foot tips.
- **Solutions:**
  1. **Check Leg Stride Balance:** Verify that left-side and right-side servos share identical travel calibration.
  2. **Foot Traction:** Ensure all four foot tips have identical contact material. Bare 3D printed PLA foot tips slip unpredictably on smooth surfaces; add small silicone caps or rubber o-rings to all four feet for uniform traction.
  3. **Free Linkage Rotation:** Ensure pivot shoulder screws on all 4 legs are tightened to the exact same tension. A stiff hinge on one leg reduces its effective stroke length, causing the robot to turn toward the stiff side.

---

### 3.4 Kinematic Target Out-of-Reach Warnings
- **Symptom:** Serial debug reports unreachable target, or a leg jerks abruptly during high stride lengths.
- **Cause:** The commanded foot target $(x, y, z)$ exceeds the maximum geometric reach $d_{\max} = L_f + L_t = 162.9\text{ mm}$ or falls inside $d_{\min} = |L_t - L_f| = 32.1\text{ mm}$.
- **Solutions:**
  1. Reduce `DEFAULT_STEP_LENGTH_MM` (e.g. from $20\text{ mm}$ to $15\text{ mm} - 18\text{ mm}$).
  2. Adjust nominal stance height (`DEFAULT_STANCE_Z`) closer to the optimal working envelope of the leg ($z \approx -85\text{ mm}$ to $-95\text{ mm}$).
  3. Verify that the measured femur ($L_f = 65.4\text{ mm}$) and tibia ($L_t = 97.5\text{ mm}$) link lengths in `RobotGeometry.h` are accurate.

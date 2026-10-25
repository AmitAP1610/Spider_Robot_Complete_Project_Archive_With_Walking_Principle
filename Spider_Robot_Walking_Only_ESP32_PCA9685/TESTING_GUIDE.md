# Step-by-Step Testing & Verification Guide

Follow this progressive validation plan before placing your quadruped robot on the floor. Testing in stages prevents stripped servo gears, brownouts, and physical chassis damage.

---

## Stage 1: Power & Electrical Pre-Flight Check

Do NOT connect the PCA9685 to the ESP32 or servos until voltages have been measured with a multimeter.

1. **Verify Logic Power**:
   - Measure voltage between ESP32 `3V3` and `GND`: Should read $3.25\text{V} - 3.35\text{V}$.
   - PCA9685 `VCC` pin must connect to ESP32 `3V3`.
   - *Never connect PCA9685 VCC to 5V if communicating directly with an unbuffered 3.3V ESP32.*
2. **Verify Servo Power Rail**:
   - Power up the external 5V regulator (e.g., LM2596 or high-current buck converter).
   - Measure voltage across the PCA9685 screw terminals (`V+` and `GND`): Should read $5.0\text{V} - 5.2\text{V}$ DC under no load.
   - *Ensure common ground:* Measure continuity between ESP32 GND and PCA9685 GND with power disconnected. Resistance must be $< 0.5\,\Omega$.
3. **Current Capacity Check**:
   - 12 micro servos (e.g. SG90 or MG90S) can draw peak currents of up to $0.7\text{A} - 1.0\text{A}$ each when accelerating or under load.
   - A single standard LM2596 module is typically rated for only $2\text{A}$ continuous ($3\text{A}$ with a large heatsink). If all 12 servos move simultaneously, a 2A regulator will overheat or enter thermal shutdown, dropping the voltage and causing the ESP32 to brown out!
   - Use an adequately rated buck converter ($5\text{A} - 8\text{A}$ continuous) or separate the left and right servo banks if using small regulators.

---

## Stage 2: Elevated Stand Test (Robot Suspended)

Place the robot chassis on a wooden block, box, or cup so all four legs hang freely without touching the table surface.

1. Connect the ESP32 to your PC via USB cable.
2. Open Serial Monitor at **115200 baud**.
3. Verify that the boot banner appears:
   ```
   Connecting to PCA9685 at I2C 0x40 (SDA=21, SCL=22)... [OK]
   PCA9685 detected. PWM output configured to 50 Hz.
   ```
4. Turn on the external servo power switch.
5. All 12 servos should move smoothly to their calibrated standing center positions:
   - Coxa links aligned perpendicular to the chassis side ($0^\circ$ yaw).
   - Femur links aligned horizontally.
   - Tibia links bent downward at approximately $90^\circ$.
6. **Auditory Inspection**:
   - Listen closely to each servo.
   - A healthy servo at rest is completely silent.
   - If any servo hums loudly, buzzes, or feels hot to the touch, it is fighting mechanical resistance or jammed against the chassis. **Switch off power immediately** and adjust the horn or linkage.

---

## Stage 3: Single-Servo Calibration & Range Validation

With the robot still suspended on its stand:

1. Type `C` in the Serial Monitor to enter Calibration Mode.
2. Cycle through joints `0` through `11`:
   - Verify that joint `0` moves FL Coxa, joint `1` moves FL Femur, joint `2` moves FL Tibia.
   - Verify all channel mappings according to the table in `CALIBRATION_GUIDE.md`.
3. For each joint, run a safe sweep test by typing `T`.
   - Confirm the servo rotates smoothly throughout its configured safe angle range.
   - Confirm no cables are pulled taut or caught in joint pinch points.
4. Type `X` to exit calibration mode.

---

## Stage 4: Elevated Air-Walking Simulation

Before placing the robot on the floor, simulate full walking cycles while the robot remains elevated on the support block.

1. **Test Forward Locomotion (`W`)**:
   - Type `W` and press Enter.
   - Observe the diagonal coordination:
     - Group A: Front Left (Leg 0) and Back Right (Leg 3) should lift, swing forward in an arch, and land together.
     - Group B: Front Right (Leg 1) and Back Left (Leg 2) should move approximately half a cycle later.
   - Observe the stance motion:
     - While Group A swings forward in the air, Group B's feet should be moving backward at constant height (simulating pushing the ground backward to propel the body forward).
   - Verify that feet do not collide with adjacent legs or chassis corners during maximum swing.
2. **Test Stop / Deceleration (`S`)**:
   - Type `S` while the robot is in mid-stride.
   - Observe that the robot does not freeze instantly in mid-air; it smoothly ramps down stride amplitude and comes to rest with all four feet at nominal stance height.
3. **Test Backward Locomotion (`B`)**:
   - Type `B`. Verify feet swing backward in the air and push forward on the simulated ground.
4. **Test Turning (`A` and `D`)**:
   - Type `A` (Left Turn):
     - Left feet (FL, BL) should step backward.
     - Right feet (FR, BR) should step forward.
   - Type `D` (Right Turn):
     - Left feet should step forward.
     - Right feet should step backward.
5. **Test Speed Tuning (`+` and `-`)**:
   - Press `+` to verify that cadence speeds up smoothly.
   - Press `-` to return to a slow, conservative cadence ($2.0\text{ s}$ cycle period).
6. **Test Alternative Crawl Gait (`G`)**:
   - Press `G` to switch to 4-Beat Crawl mode.
   - Confirm that only one leg lifts at a time (BL $\to$ FL $\to$ BR $\to$ FR).

---

## Stage 5: Gentle Ground Touchdown Test

1. Set the gait to `STAND` (`H`).
2. Carefully place the robot on a flat, non-slip surface (such as a rubber cutting mat, foam mat, or low-pile carpet). Avoid slick polished tile or glass for initial tests.
3. Observe the chassis height:
   - Does the chassis maintain its commanded height of approximately $60\text{ mm}$ above the floor?
   - Do the legs splay outward under the weight of the battery and chassis?
   - If the legs sag or servos jitter under body weight, the servo supply voltage may be sagging or the servo torque rating may be insufficient for the robot's total weight.
4. Test sitting and standing:
   - Send `L` to lower / sit the chassis onto the ground.
   - Send `H` to stand back up to nominal height.

---

## Stage 6: First Floor Walking Test

1. Ensure the walking speed is set to conservative ($T_{\text{cycle}} = 2.0\text{ s}$ or slower).
2. Start with the **4-Beat Crawl Gait** (`G` command if not already in crawl mode) because it maintains 3 points of ground contact at all times.
3. Send `W` to begin walking forward.
4. Observe the displacement:
   - Does the robot move forward steadily?
   - Are the feet slipping backward on the floor surface? (If slipping occurs, add small silicone or rubber pads to the foot tips).
   - Does the body pitch or roll excessively?
5. Send `S` to stop and inspect servo temperatures.
6. Once the crawl gait is verified, toggle to the **Diagonal Trot Gait** (`G`) and test forward movement.

---

## Stage 7: Tuning & Optimization Matrix

| Symptom | Root Cause | Recommended Configuration Adjustment |
|---|---|---|
| **Feet drag along the ground during swing** | Insufficient foot clearance | In `GaitConfig.h`, increase `DEFAULT_STEP_HEIGHT_MM` from $15\text{ mm}$ to $18\text{ mm} - 20\text{ mm}$. |
| **Robot tips or wobbles side-to-side** | Stride too long or trot duty factor too low | 1) Reduce `DEFAULT_STEP_LENGTH_MM` to $15\text{ mm}$.<br>2) Increase `DEFAULT_DUTY_FACTOR_TROT` to $0.75$.<br>3) Switch to 4-Beat Crawl (`G`). |
| **Servos jerk violently on direction changes** | Angular acceleration too high | In `GaitConfig.h`, reduce `MAX_JOINT_SPEED_DEG_PER_SEC` from $160^\circ/\text{s}$ to $100^\circ/\text{s} - 120^\circ/\text{s}$. |
| **Robot stumbles when starting or stopping** | Transition ramp too abrupt | Increase `TRANSITION_RAMP_TIME_SEC` from $0.8\text{ s}$ to $1.2\text{ s}$. |
| **Legs struggle to lift off the floor** | Foot adhesion or excessive linkage friction | Check joint hinge screws for over-tightening. Lightly lubricate pivot points. |

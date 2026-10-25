# 12-Servo Calibration Guide for Quadruped Spider Robot

Every hobby servo exhibits small manufacturing variances in its internal potentiometer, gearbox backlash, and horn spline alignment. **Never assume all servos share the same center (90°), travel limits, or direction.**

Follow this systematic guide to calibrate all 12 servos safely without stripping gears or burning out motors.

---

## 1. Safety Rules Before Powering On

1. **Elevate the Robot**: Place the robot body on a stand or block so all four legs hang freely in the air. Never perform initial calibration with the feet resting on the floor.
2. **Independent Power**: Do not power the servos from the ESP32 3.3V or 5V pins! Connect a dedicated 5.0V regulated power supply (5A+ continuous) to the PCA9685 `V+` screw terminal.
3. **Common Ground**: Ensure the ESP32 GND, PCA9685 GND, and servo supply ground are connected together.
4. **Immediate Disconnect**: Keep the servo power switch within easy reach. If a servo buzzes, hums loudly, stalls, or hits the chassis, **switch off servo power immediately**.

---

## 2. Channel & Joint Mapping Table

| Logical Index | Leg | Joint | PCA9685 Channel | Default Name | Initial Center | Initial Dir | Safe Range |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **0** | Front Left (FL) | Coxa (Yaw) | **0** | `FL Coxa` | 90° | +1 | 55° – 125° |
| **1** | Front Left (FL) | Femur (Pitch) | **1** | `FL Femur` | 90° | +1 | 50° – 130° |
| **2** | Front Left (FL) | Tibia (Knee) | **2** | `FL Tibia` | 90° | +1 | 45° – 135° |
| **3** | Front Right (FR) | Coxa (Yaw) | **4** | `FR Coxa` | 90° | -1 | 55° – 125° |
| **4** | Front Right (FR) | Femur (Pitch) | **5** | `FR Femur` | 90° | -1 | 50° – 130° |
| **5** | Front Right (FR) | Tibia (Knee) | **6** | `FR Tibia` | 90° | -1 | 45° – 135° |
| **6** | Back Left (BL) | Coxa (Yaw) | **8** | `BL Coxa` | 90° | +1 | 55° – 125° |
| **7** | Back Left (BL) | Femur (Pitch) | **9** | `BL Femur` | 90° | +1 | 50° – 130° |
| **8** | Back Left (BL) | Tibia (Knee) | **10** | `BL Tibia` | 90° | +1 | 45° – 135° |
| **9** | Back Right (BR) | Coxa (Yaw) | **12** | `BR Coxa` | 90° | -1 | 55° – 125° |
| **10** | Back Right (BR) | Femur (Pitch) | **13** | `BR Femur` | 90° | -1 | 50° – 130° |
| **11** | Back Right (BR) | Tibia (Knee) | **14** | `BR Tibia` | 90° | -1 | 45° – 135° |

*(Channels 3, 7, 11, and 15 are unused.)*

---

## 3. Step-by-Step Mechanical Horn Installation

If you are assembling the robot from scratch:
1. Connect only the ESP32 and PCA9685 logic.
2. Upload the firmware.
3. With no servo horns attached to the splines, power up the servo supply. The firmware initializes all channels to their neutral pulse ($1500\,\mu\text{s}$, approximately 90°).
4. Power off the servo supply.
5. Attach each plastic servo horn as close as possible to the mechanical neutral orientation:
   - **Coxa**: Horn aligned perpendicular to the chassis side (leg pointing directly outwards).
   - **Femur**: Horn aligned horizontally or slightly tilted up ($5^\circ$ to $10^\circ$).
   - **Tibia**: Lower leg link attached at approximately a 90° knee bend relative to the femur.
6. Install the horn retaining screw gently without over-torquing.

---

## 4. Interactive USB Serial Calibration Mode

The firmware includes an interactive serial calibration suite that lets you jog one joint at a time in fine increments.

### Entering Calibration Mode
1. Open the Arduino IDE Serial Monitor at **115200 baud**.
2. Type `C` and press Enter.
3. The robot stops the walking engine, commands all joints to safe center angles, and displays the calibration interface.

### Calibration Menu Controls

| Command | Function |
|:---:|---|
| `0` .. `11` | Select joint index 0 through 11 directly (e.g. `0`, `10`, `11`) |
| `U` / `K` | Select Joint 8 (BL Tibia) / Joint 11 (BR Tibia) |
| `N` / `P` | Cycle to **Next** or **Previous** joint |
| `+` / `-` | Jog selected joint by **$\pm 1.0^\circ$** |
| `>` / `<` | Jog selected joint by **$\pm 5.0^\circ$** (or `++` / `--`) |
| `JOG <D>` | Jog selected joint by custom degree delta (e.g. `JOG 2.5`) |
| `SET <A>` | Set selected joint directly to angle `A` (e.g. `SET 95`) |
| `SET J A` | Set joint `J` to angle `A` (e.g. `SET 0 55`) |
| `Z` | **Capture Center**: Sets current commanded angle as the new center for selected joint |
| `ZALL` | **Capture All Centers**: Sets current angles as new centers for **all 12 joints** |
| `I` | **Toggle Direction**: Inverts direction multiplier ($+1 \leftrightarrow -1$) |
| `T` | **Safe Sweep Test**: Sweeps joint gently from min to max limit to verify travel |
| `SHOW` or `V` | **View Table**: Dumps complete calibration table and NVS flash status |
| `SAVE` | **Permanent Flash Save**: Persists all 12 calibrations to ESP32 NVS flash |
| `LOAD` | **Reload Flash**: Restores stored calibration parameters from ESP32 NVS flash |
| `DEFAULT` | **Factory Reset**: Restores original factory default calibration in RAM |
| `X` or `Q` | **Exit**: Leaves calibration mode and returns all servos to Stand pose |

---

## 5. Per-Joint Calibration Workflow

Repeat these steps for each of the 12 joints (from index 0 to 11):

### 5.1 Center Alignment
1. Select the joint (e.g. `0` for Front Left Coxa).
2. Look at the mechanical alignment of the leg.
3. Use `+` and `-` (or `SET <angle>`) to jog the joint until it reaches its ideal neutral stance angle:
   - Coxa: Perpendicular to chassis ($0^\circ$ yaw).
   - Femur: Horizontal ($0^\circ$ pitch relative to ground).
   - Tibia: $90^\circ$ right angle knee bend relative to femur.
4. Press `Z` to capture this angle as the new `centerAngleDeg` (or type `ZALL` once all 12 joints are aligned).

### 5.2 Direction Verification
1. Jog the joint positively by typing `>` ($+5^\circ$).
2. Observe the physical motion:
   - **Coxa**: Positive jog should swing the foot **forward**. If it swings backward, type `I` to invert.
   - **Femur**: Positive jog should pitch the leg **upward**. If it pitches downward, type `I` to invert.
   - **Tibia**: Positive jog should extend the lower leg outward/forward. If it bends further in, type `I` to invert.
3. Type `Z` again once neutral is confirmed.

### 5.3 Travel Limit Check
1. Type `T` to run a gentle sweep test.
2. Verify that the servo travels smoothly between its configured minimum and maximum limits without binding, whining, or contacting the robot chassis.
3. If binding occurs, record narrower limits (e.g. 60° to 120°).

---

## 6. Saving Calibration Permanently to ESP32 Flash (NVS)

You no longer need to recompile code to save calibration!

1. Type `SHOW` in the Serial Monitor to review the calibration table.
2. If all current angles match your desired neutral pose, type `ZALL` to capture all 12 centers at once.
3. Type `SAVE` and press Enter.
   - The firmware writes all 12 centers, directions, limits, and CRC32 checksum into the ESP32 NVS flash (`spider_cal` namespace).
   - The Serial Monitor confirms: `>> [NVS SAVE SUCCESS] All 12 servo calibration parameters saved to ESP32 flash memory!`
4. Power cycle or reset your ESP32.
   - During boot, the firmware detects the saved calibration, validates the checksum and safe bounds, and prints:
     `>> [NVS] Loaded saved servo calibration from flash.`
   - All 12 servos immediately hold your calibrated standing pose!


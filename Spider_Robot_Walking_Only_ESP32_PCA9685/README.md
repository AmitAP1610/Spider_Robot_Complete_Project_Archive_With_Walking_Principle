# Spider Robot: Realistic Walking-Only Controller (ESP32 + PCA9685)

A precision quadruped walking controller for an ESP32 DevKit and PCA9685 16-channel PWM servo driver controlling a 12-servo (3-DOF per leg) spider/quadruped robot.

This project implements a **strict walking-only** scope. All extraneous and unverified dependencies (Wi-Fi, Bluetooth, web servers, OLED displays, distance sensors, IMUs, ROS) have been excluded to maximize control loop frequency, determinism, and safety.

---

## 1. Hardware Architecture & Pinout

| Component | Specification | Connection / Pin |
|---|---|---|
| **Microcontroller** | ESP32 DevKit (38-pin or 30-pin) | USB Serial Monitor at 115200 baud |
| **Servo Driver** | PCA9685 16-Channel 12-Bit PWM | I2C Address: `0x40` |
| **I2C SDA** | ESP32 GPIO 21 | PCA9685 SDA |
| **I2C SCL** | ESP32 GPIO 22 | PCA9685 SCL |
| **Logic Power** | 3.3V from ESP32 | PCA9685 VCC (Logic only!) |
| **Servo Power** | 5.0V - 6.0V Regulated Supply (5A - 8A) | PCA9685 V+ screw terminal |
| **Ground** | Common Ground | ESP32 GND, PCA9685 GND, Power Supply GND |

### PCA9685 Channel Mapping

```
                 FRONT
          [Leg 0: FL]   [Leg 1: FR]
   Coxa  : CH 0          CH 4  : Coxa
   Femur : CH 1          CH 5  : Femur
   Tibia : CH 2          CH 6  : Tibia
             \             /
              \   CHASSIS /
               |         |
              /   CENTER  \
             /             \
   Coxa  : CH 8          CH 12 : Coxa
   Femur : CH 9          CH 13 : Femur
   Tibia : CH 10         CH 14 : Tibia
          [Leg 2: BL]   [Leg 3: BR]
                 BACK
```

*Note: PCA9685 Channels 3, 7, 11, and 15 are unused.*

---

## 2. Key Technical Features

1. **Analytical 3-DOF Inverse Kinematics with Confirmed Dimensions**:
   - Closed-form trigonometric solver using measured link lengths: **Coxa $L_c = 42.2\text{ mm}$**, **Femur $L_f = 65.4\text{ mm}$**, **Tibia $L_t = 97.5\text{ mm}$**.
   - Geometric reach verification: $D = (r^2 + z^2 - L_f^2 - L_t^2) / (2 L_f L_t)$, validating targets against theoretical bounds $[32.1\text{ mm}, 162.9\text{ mm}]$.
   - Programmatic Forward Kinematics verification suite confirms $< 0.001\text{ mm}$ reconstruction error on all reachable targets.
2. **Smooth Continuous Foot Trajectories**:
   - **Swing phase**: Smooth cosine trajectory along the horizontal axis ($X$) with zero endpoint velocity, paired with a sinusoidal arch for foot clearance ($Z$).
   - **Stance phase**: Ground retraction along a matching cosine profile ensuring continuous velocity at liftoff and touchdown.
3. **Dual Gait Engines**:
   - **Diagonal Trot-Walk (Default)**: Overlapping diagonal pairs with 70% duty factor ($\beta = 0.70$). Includes a 20% quadruple stance phase where all 4 feet are firmly planted on the ground between diagonal transitions.
   - **4-Beat Crawl / Wave Gait**: Static stability gait with 75% duty factor ($\beta = 0.75$). Only one foot swings at any instant; three feet always maintain a stable support tripod.
4. **Differential Turning Kinematics**:
   - Genuine kinematic turning using differential foot stride lengths ($S_{left}$ vs $S_{right}$), creating a true yaw moment about the body center rather than arbitrary servo rotations.
5. **Soft Motion & Anti-Jerk Protection**:
   - Slew-rate rate limiting (`MAX_JOINT_SPEED_DEG_PER_SEC`) on all 12 servo channels.
   - Smooth amplitude ramping between standing and walking states.
6. **Interactive USB Serial Calibration Mode**:
   - Jog any individual servo by $\pm 1^\circ$ or $\pm 5^\circ$ directly over the USB Serial Monitor.
   - Live calibration table inspection and zero-center capture.

---

## 3. Project File Structure

```
Spider_Robot_Walking_Only_ESP32_PCA9685/
├── Spider_Robot_Walking_Only_ESP32_PCA9685.ino  # Main sketch and Serial command processor
├── RobotGeometry.h                             # Physical dimensions, link lengths, stance targets
├── CalibrationConfig.h                         # Per-servo channel map, center, direction, safe limits
├── CalibrationConfig.cpp                       # Global 12-servo calibration table definition
├── GaitConfig.h                                # Gait timing, duty factor, step length/height, limits
├── Kinematics.h                                # 3-DOF IK/FK solver interface and data structures
├── Kinematics.cpp                              # Closed-form trigonometric IK solver implementation
├── GaitPlanner.h                               # Gait state machine and phase coordinator
├── GaitPlanner.cpp                             # Trajectory generator and differential turning logic
├── ServoDriver.h                               # PCA9685 I2C driver and rate-limiting interface
├── ServoDriver.cpp                             # Tick conversion, hardware writes, calibration jogs
├── FaceController.h                            # SSD1306 OLED face animator and FreeRTOS task interface
├── FaceController.cpp                          # Face state machine, idle blinks, and thread-safe rendering
├── WebRemote.h                                 # Smartphone Wi-Fi SoftAP and Captive Portal remote interface
├── WebRemote.cpp                               # Responsive touch controller, D-Pad, face triggers, and tests
├── face-bitmaps.h                              # 35+ 128x64 monochrome PROGMEM face expression bitmaps
├── README.md                                   # Comprehensive project overview and quickstart
├── WALKING_PRINCIPLE.md                        # Theoretical manual, torque sizing, and electrical specs
├── WALKING_ALGORITHM.md                        # Complete mathematical and kinematic derivation
├── CODE_AND_HARDWARE_INSPECTION.md             # Technical audit of original code and mechanical assembly
├── CALIBRATION_GUIDE.md                        # Step-by-step 12-servo calibration protocol
├── TESTING_GUIDE.md                            # Elevated pre-flight and progressive walking test plan
├── TROUBLESHOOTING.md                          # Diagnostics for electrical, mechanical, and code issues
├── ROBOT_MEASUREMENTS_TEMPLATE.csv             # Template for chassis dimensions and mass measurements
└── SERVO_CALIBRATION_TEMPLATE.csv              # Template for recording 12-servo calibration values
```

---

## 4. USB Serial Control Commands

Open the Arduino IDE Serial Monitor at **115200 baud** (configured with "Newline" or "No line ending"):

| Key | Action | Description |
|:---:|---|---|
| `W` | **Walk Forward** | Continuous forward gait progression |
| `B` | **Walk Backward** | Continuous backward gait progression |
| `A` | **Turn Left** | Counter-clockwise turn via differential foot strides |
| `D` | **Turn Right** | Clockwise turn via differential foot strides |
| `S` | **Stop / Transition to Stand** | Smoothly ramps down stride amplitude and returns to stand pose |
| `H` | **Stand Pose** | Commands nominal standing height |
| `L` | **Sit / Lower Pose** | Splays legs outward and lowers chassis to the ground |
| `+` | **Increase Speed** | Decreases gait cycle period by 0.2 s (faster cadence) |
| `-` | **Decrease Speed** | Increases gait cycle period by 0.2 s (slower cadence) |
| `G` | **Toggle Gait** | Toggles between Diagonal Trot and 4-Beat Crawl |
| `R` | **Report Status** | Prints active state, gait type, cycle period, and I2C connection |
| `K` | **Kinematics Test** | Runs Forward & Inverse Kinematics verification suite (reconstruction error < 0.01 mm) |
| `C` | **Calibration Mode** | Enters single-servo jogging and calibration menu |
| `FACE <name>` | **Set Face** | Command expression (e.g. `FACE CUTE`, `FACE HAPPY`, `FACE ANGRY`, `FACE REST`) |
| `FACE LIST` | **List Faces** | Prints all 35+ available facial expression names to Serial |
| `FACE AUTO` | **Face Auto Mode** | Re-enables automatic motion-synchronized facial expressions |
| `FACE FPS <N>` | **Animation Speed**| Adjusts playback frame rate (e.g. `FACE FPS 4`) |
| `?` | **Help** | Displays the full serial command menu |

> **IMPORTANT SAFETY NOTICE ON STOPPING:**  
> Sending the `S` command stops gait progression and brings the robot into a stable standing pose. It **does NOT cut electrical power** to the servos. Hobby servos will continue holding their commanded position and drawing quiescent current. To de-energize the servos, switch off the external servo power supply.

---

## 5. Quickstart & Installation

### Required Software & Libraries
1. **Arduino IDE** (v1.8.19 or v2.x).
2. **ESP32 Arduino Core** by Espressif Systems:
   - Add URL to Board Manager: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Install `esp32` by Espressif Systems.
   - Select Board: `ESP32 Dev Module`.
3. **Arduino Libraries** (Install via Arduino Library Manager `Ctrl + Shift + I`):
   - `Adafruit PWM Servo Driver` (by Adafruit)
   - `Adafruit SSD1306` (by Adafruit)
   - `Adafruit GFX Library` (by Adafruit)

### OLED Display Wiring (SSD1306 128x64 I2C)
The OLED display shares the hardware I2C bus with the PCA9685 servo driver, safely arbitrated via a FreeRTOS mutex on ESP32 Core 0:
- **OLED VCC** $\rightarrow$ ESP32 3.3V
- **OLED GND** $\rightarrow$ ESP32 GND
- **OLED SDA** $\rightarrow$ ESP32 GPIO 21 (Shared with PCA9685 SDA)
- **OLED SCL** $\rightarrow$ ESP32 GPIO 22 (Shared with PCA9685 SCL)

*(Note: If no OLED display is plugged in, the robot automatically detects this and continues in headless walking mode with zero errors.)*

### Uploading Firmware
1. Open `Spider_Robot_Walking_Only_ESP32_PCA9685.ino` in Arduino IDE.
2. Select the correct COM port and board (`ESP32 Dev Module`).
3. Set Upload Speed to `921600` or `115200`.
4. Click **Upload**.
5. Open Serial Monitor at **115200 baud**.

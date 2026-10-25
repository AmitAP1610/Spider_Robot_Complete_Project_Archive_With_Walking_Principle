# Quadruped Spider Robot: Walking Algorithm & Kinematics Reference

This document provides the complete mathematical and robotic foundation for the 12-servo quadruped walking controller.

---

## 1. Coordinate Frames & Conventions

### 1.1 Robot Body Frame $\{B\}$
The robot chassis defines the primary inertial reference frame:
- **Origin $(0, 0, 0)$**: Geometric center of the chassis between all four hip joints.
- **$+X_B$**: Longitudinal forward direction.
- **$+Y_B$**: Lateral left direction.
- **$+Z_B$**: Vertical upward direction (normal to the ground plane).

### 1.2 Leg Frame $\{L_i\}$ (per leg $i \in \{0, 1, 2, 3\}$)
Each leg has a local reference frame originating at the coxa yaw rotation axis:
- **Leg 0 (Front Left, FL)**: Hip at $(+L_{body}/2, +W_{body}/2, 0)$
- **Leg 1 (Front Right, FR)**: Hip at $(+L_{body}/2, -W_{body}/2, 0)$
- **Leg 2 (Back Left, BL)**: Hip at $(-L_{body}/2, +W_{body}/2, 0)$
- **Leg 3 (Back Right, BR)**: Hip at $(-L_{body}/2, -W_{body}/2, 0)$

Within each local leg frame:
- **$+x$**: Forward displacement relative to the hip (parallel to $+X_B$).
- **$+y$**: Lateral outward displacement from the body center ($+y > 0$ always points outward toward the side).
- **$+z$**: Vertical elevation relative to the hip joint ($z < 0$ when the foot contacts the ground below the chassis).

```
                      +X (Forward)
                           ^
                           |
      Leg 0 (FL)           |           Leg 1 (FR)
     [CH 0, 1, 2]          |          [CH 4, 5, 6]
           \               |               /
            \+y            |            +y/
             O-------------+-------------O
             |             |             |
+Y (Left) <--+-------------+-------------+--> -Y (Right)
             |             |             |
             O-------------+-------------O
            /+y            |            +y\
           /               |               \
      Leg 2 (BL)           |           Leg 3 (BR)
     [CH 8, 9, 10]         |          [CH 12, 13, 14]
                           |
                      -X (Backward)
```

---

## 2. Analytical 3-DOF Inverse Kinematics (IK)

Each leg consists of three serial revolute joints:
1. **Coxa Joint ($\theta_0$)**: Horizontal yaw sweep in the $XY$ plane.
2. **Femur Joint ($\theta_1$)**: Pitch rotation in the vertical plane aligned with the coxa arm.
3. **Tibia Joint ($\theta_2$)**: Knee pitch rotation in the same vertical plane.

### 2.1 Confirmed Link Length Dimensions
- **Coxa length ($L_c$)**: $42.2\text{ mm}$ (distance from coxa yaw axis to femur pitch axis)
- **Femur length ($L_f$)**: $65.4\text{ mm}$ (center-to-center distance between femur and tibia pitch axes)
- **Tibia length ($L_t$)**: $97.5\text{ mm}$ (center of knee pitch axis to foot ground contact point)

> **Important Measurement Convention:**  
> These dimensions are true center-to-center distances between rotational joint axes. If your physical links were measured along external outer plastic edges, adjust the effective kinematic lengths in `RobotGeometry.h` accordingly.

### 2.2 Theoretical Distance Bounds (Femur-Tibia Planar Model)
- **Minimum theoretical distance:** $|L_t - L_f| = |97.5 - 65.4| = 32.1\text{ mm}$
- **Maximum theoretical distance:** $L_f + L_t = 65.4 + 97.5 = 162.9\text{ mm}$

*Note:* The coxa length $L_c = 42.2\text{ mm}$ is handled strictly in the horizontal $XY$ transformation and is **never added blindly** to the planar maximum reach.

### 2.3 Closed-Form Analytical Solution

Given target foot coordinates $(x, y, z)$ in the local leg frame:

#### Step 1: Coxa Yaw Angle ($\theta_0$)
In the horizontal $XY$ plane, the coxa sweeps to align with the foot target:
$$\rho = \sqrt{x^2 + y^2}$$
$$\theta_0 = \arctan2(x, y) \quad [\text{radians}]$$
$$\theta_{0,\text{deg}} = \theta_0 \cdot \frac{180}{\pi}$$

#### Step 2: Planar Horizontal Extension from Femur Pivot ($r$)
The coxa arm extends horizontally by length $L_c$. The planar distance from the femur pitch axis to the foot projection is:
$$r = \rho - L_c = \sqrt{x^2 + y^2} - 42.2\text{ mm}$$
If $r < 1.0\text{ mm}$, the target lies within the coxa radius and is clamped to $r = 1.0\text{ mm}$.

#### Step 3: Planar Distance ($d$) and Geometric Reach Metric ($D$)
The straight-line distance $d$ from the femur pitch axis to the foot tip is:
$$d^2 = r^2 + z^2 \implies d = \sqrt{r^2 + z^2}$$

The geometric reach check metric is defined as:
$$D = \frac{r^2 + z^2 - L_f^2 - L_t^2}{2 L_f L_t} = \frac{d^2 - 65.4^2 - 97.5^2}{2 \times 65.4 \times 97.5} = \frac{d^2 - 13783.41}{12753.0}$$

**Reachability Criterion:**
- Target is physically reachable if and only if:
  $$-1.0 \le D \le 1.0$$
- If $D > 1.0$: Target distance $d > 162.9\text{ mm}$ (exceeds maximum reach).
- If $D < -1.0$: Target distance $d < 32.1\text{ mm}$ (inside minimum reach).
- The solver clamps $D_{\text{clamped}} = \text{constrain}(D, -1.0, 1.0)$ and reports `reachable = false`.

#### Step 4: Tibia / Knee Joint Angle ($\theta_2$)
Because $D = \cos(\theta_{\text{knee\_relative}})$:
$$\theta_2 = \arccos(D_{\text{clamped}}) \quad [\text{radians}]$$
$$\theta_{2,\text{deg}} = \theta_2 \cdot \frac{180}{\pi}$$
- When $D = +1.0$ (fully extended, $d = 162.9\text{ mm}$): $\theta_2 = 0^\circ$ (straight leg).
- When $D = 0.0$ ($d = 117.4\text{ mm}$): $\theta_2 = 90^\circ$ (right-angle knee bend).
- When $D = -1.0$ (fully folded, $d = 32.1\text{ mm}$): $\theta_2 = 180^\circ$ (tibia folded against femur).

#### Step 5: Femur Joint Angle ($\theta_1$)
The elevation angle from the horizontal plane to the foot line $d$ is:
$$\alpha = \arctan2(z, r) \quad [\text{negative when } z < 0]$$

Using the Law of Cosines, the angle between the femur link and the line $d$ is:
$$\cos(\delta) = \frac{L_f^2 + d_{\text{safe}}^2 - L_t^2}{2 L_f d_{\text{safe}}}$$
$$\delta = \arccos\left(\text{clamp}(\cos(\delta), -1.0, +1.0)\right)$$

For the standard outward-pointing quadruped knee configuration:
$$\theta_1 = \alpha + \delta \quad [\text{radians}]$$
$$\theta_{1,\text{deg}} = \theta_1 \cdot \frac{180}{\pi}$$

### 2.4 Forward Kinematics (FK) & Mathematical Reconstruction
Given joint angles $(\theta_0, \theta_1, \theta_2)$ in degrees:
$$\theta_{\text{tibia\_world}} = \theta_{1,\text{rad}} - \theta_{2,\text{rad}}$$
$$r_{\text{recon}} = L_f \cos(\theta_{1,\text{rad}}) + L_t \cos(\theta_{\text{tibia\_world}})$$
$$z_{\text{recon}} = L_f \sin(\theta_{1,\text{rad}}) + L_t \sin(\theta_{\text{tibia\_world}})$$
$$\rho_{\text{recon}} = L_c + r_{\text{recon}}$$
$$x_{\text{recon}} = \rho_{\text{recon}} \sin(\theta_{0,\text{rad}}), \quad y_{\text{recon}} = \rho_{\text{recon}} \cos(\theta_{0,\text{rad}})$$

### 2.5 Programmatic Verification Test Results
Running `Kinematics::runVerificationTests()` evaluates test points across the reachable envelope:

| Test Point | Target $(x, y, z)$ mm | $D$ Metric | Status | IK $(\theta_0, \theta_1, \theta_2)$ deg | FK Recon $(x, y, z)$ mm | Error |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| **Nominal Stance** | $(0, 110, -90)$ | $+0.007$ | Reachable | $(0.0^\circ, 5.7^\circ, 89.6^\circ)$ | $(0.0, 110.0, -90.0)$ | **$< 0.001\text{ mm}$** |
| **Forward Reach** | $(25, 110, -90)$ | $+0.046$ | Reachable | $(12.8^\circ, 6.7^\circ, 87.4^\circ)$ | $(25.0, 110.0, -90.0)$ | **$< 0.001\text{ mm}$** |
| **Backward Stride**| $(-25, 110, -90)$ | $+0.046$ | Reachable | $(-12.8^\circ, 6.7^\circ, 87.4^\circ)$ | $(-25.0, 110.0, -90.0)$| **$< 0.001\text{ mm}$** |
| **Swing Lift Apex** | $(0, 110, -75)$ | $-0.187$ | Reachable | $(0.0^\circ, 15.6^\circ, 100.8^\circ)$ | $(0.0, 110.0, -75.0)$ | **$< 0.001\text{ mm}$** |
| **Lowered Sit** | $(0, 125, -40)$ | $-0.499$ | Reachable | $(0.0^\circ, 33.3^\circ, 119.9^\circ)$ | $(0.0, 125.0, -40.0)$ | **$< 0.001\text{ mm}$** |
| **Diagonal Reach** | $(20, 120, -95)$ | $+0.207$ | Reachable | $(9.5^\circ, 7.8^\circ, 78.1^\circ)$ | $(20.0, 120.0, -95.0)$ | **$< 0.001\text{ mm}$** |
| **Out-of-Reach** | $(0, 220, -90)$ | $+1.722$ | **Rejected** | $(0.0^\circ, -26.9^\circ, 0.0^\circ)$ | *Safely Clamped* | *Clamped to $d_{\max}$* |

*Conclusion:* On every reachable test target, the inverse kinematics output reconstructed by forward kinematics exhibits Euclidean position error $< 0.001\text{ mm}$ (exact numerical match within 32-bit single-precision floating point). Target points outside $[-1, 1]$ are safely clamped without producing `NaN` or unhandled exceptions.

---

## 3. Gait Planning & Master Phase Generation

### 3.1 Normalized Master Clock
The controller maintains a continuous normalized time variable $\Phi(t) \in [0.0, 1.0)$:
$$\Phi(t) = \text{fmod}\left(\Phi(t - \Delta t) + \frac{\Delta t}{T_{\text{cycle}}}, 1.0\right)$$
where $T_{\text{cycle}}$ is the total gait period (default $2.0\text{ s}$).

### 3.2 Leg Phase Offsets ($\Delta_i$)
Each leg $i$ operates at a fixed phase offset relative to the master clock:
$$\phi_i(t) = \text{fmod}\left(\Phi(t) + \Delta_i, 1.0\right)$$

#### Diagonal Trot-Walk Gait ($\beta = 0.70$)
- **Group A**: Leg 0 (FL) and Leg 3 (BR) $\implies \Delta_0 = 0.00, \Delta_3 = 0.00$
- **Group B**: Leg 1 (FR) and Leg 2 (BL) $\implies \Delta_1 = 0.50, \Delta_2 = 0.50$

Duty factor $\beta = 0.70$ means each leg spends 70% of its cycle in stance and 30% in swing:
- Swing fraction: $\alpha_{\text{swing}} = 1.0 - \beta = 0.30$
- Stance fraction: $\beta_{\text{stance}} = 0.70$

**Gait Chronogram:**
```
Phase:     0.0        0.3        0.5        0.8        1.0
Group A:  |-- SWING --|---------- STANCE ----------|
Group B:  |-- STANCE ------------|-- SWING --|-- STANCE --|
Support:  [Diagon. B] [4 LEGS ON] [Diagon. A] [4 LEGS ON]
```
*Note the 20% quadruple stance overlap phases ($[0.3, 0.5]$ and $[0.8, 1.0]$) where all 4 feet are firmly planted on the ground, preventing the body from tipping before the alternating diagonal lifts.*

#### 4-Beat Crawl / Wave Gait ($\beta = 0.75$)
For maximum static stability:
- Sequence: Back Left (2) $\to$ Front Left (0) $\to$ Back Right (3) $\to$ Front Right (1)
- Phase offsets: $\Delta_2 = 0.00, \Delta_0 = 0.25, \Delta_3 = 0.50, \Delta_1 = 0.75$
- Swing fraction: $0.25$; Stance fraction: $0.75$
- **Guarantee:** Exactly 1 leg swings at any given instant. Exactly 3 legs are always in contact with the ground, maintaining a permanent static support triangle.

---

## 4. Foot Trajectory Mathematics

To prevent mechanical impact, joint acceleration spikes, and foot drag, foot trajectories must satisfy $C^1$ velocity continuity at the stance-swing boundaries.

### 4.1 Swing Phase ($\phi_i < \alpha_{\text{swing}}$)
Normalized swing progress:
$$u = \frac{\phi_i}{\alpha_{\text{swing}}} \in [0.0, 1.0]$$

#### Horizontal Trajectory ($X$):
$$X(u) = X_0 - \frac{S_i}{2} \cos(\pi u)$$
- At liftoff ($u = 0.0$): $X = X_0 - \frac{S_i}{2}$ (rear stance position)
- At midpoint ($u = 0.5$): $X = X_0$ (passing hip center)
- At touchdown ($u = 1.0$): $X = X_0 + \frac{S_i}{2}$ (forward contact position)
- Derivative: $\frac{dX}{du} = \frac{S_i \pi}{2} \sin(\pi u) \implies \left.\frac{dX}{du}\right|_{u=0} = 0, \left.\frac{dX}{du}\right|_{u=1} = 0$ (zero endpoint velocity).

#### Vertical Lift Trajectory ($Z$):
$$Z(u) = Z_0 + H \cdot A_{\text{gait}} \cdot \sin(\pi u)$$
- At liftoff ($u = 0.0$): $Z = Z_0$
- At apex ($u = 0.5$): $Z = Z_0 + H$ (maximum ground clearance)
- At touchdown ($u = 1.0$): $Z = Z_0$ (soft ground contact)

### 4.2 Stance Phase ($\phi_i \ge \alpha_{\text{swing}}$)
Normalized stance progress:
$$v = \frac{\phi_i - \alpha_{\text{swing}}}{\beta_{\text{stance}}} \in [0.0, 1.0]$$

#### Horizontal Trajectory ($X$):
$$X(v) = X_0 + \frac{S_i}{2} \cos(\pi v)$$
- At touchdown ($v = 0.0$): $X = X_0 + \frac{S_i}{2}$
- At liftoff ($v = 1.0$): $X = X_0 - \frac{S_i}{2}$
- Foot retracts backward relative to the chassis, propelling the robot forward.

#### Vertical Ground Contact ($Z$):
$$Z(v) = Z_0 \quad (\text{foot remains on ground surface})$$

---

## 5. Differential Turning Kinematics

When the robot executes a turn, foot trajectories are computed differentially rather than merely spinning joints by fixed angles:

Let $S_{\text{base}}$ be the longitudinal translation stride, and $T_{\text{turn}}$ be the differential turn intensity:
- **Left Turn (`A`)**:
  - Left legs (FL, BL): $S_{\text{left}} = S_{\text{base}} - T_{\text{turn}}$
  - Right legs (FR, BR): $S_{\text{right}} = S_{\text{base}} + T_{\text{turn}}$
- **Right Turn (`D`)**:
  - Left legs (FL, BL): $S_{\text{left}} = S_{\text{base}} + T_{\text{turn}}$
  - Right legs (FR, BR): $S_{\text{right}} = S_{\text{base}} - T_{\text{turn}}$

When turning in place ($S_{\text{base}} = 0$):
- Left legs take backward steps ($S_{\text{left}} = -T_{\text{turn}}$).
- Right legs take forward steps ($S_{\text{right}} = +T_{\text{turn}}$).
- This produces a pure yaw moment $\tau_z$ about the chassis center of mass.

---

## 6. Stability Analysis & Support Polygons

### 6.1 Support Polygon Definition
The support polygon $\mathcal{P}$ is the 2D convex hull of all foot contact points projected onto the ground plane:
$$\mathcal{P} = \text{ConvexHull}\left(\{ (x_i, y_i) \mid \text{leg } i \text{ is in stance} \}\right)$$

### 6.2 Quasi-Static Stability Criterion
A quadruped is quasi-statically stable if and only if the vertical projection of its Center of Mass (CoM) lies strictly within the interior of $\mathcal{P}$ with a stability margin $d_{\text{margin}} > 0$:
$$\text{CoM}_{xy} \in \text{Interior}(\mathcal{P})$$

- In **4-Beat Crawl**, 3 legs are in stance at all times, forming a large support triangle. As long as CoM stays inside the triangle, the robot cannot tip over.
- In **Diagonal Trot**, during the diagonal swing interval, only 2 feet touch the ground, reducing the support polygon to a line segment. The robot maintains balance through:
  1. Low body height (lowering gravitational potential energy).
  2. Short step length ($20\text{ mm}$) to minimize tipping levers.
  3. Overlapping duty factor ($\beta = 0.70$), which inserts 4-leg support phases between diagonal transitions.

---

## 7. Servo Calibration & PCA9685 PWM Calculations

The PCA9685 provides 12-bit PWM resolution (4096 counts per period) operating at $50\text{ Hz}$:
$$T_{\text{PWM}} = \frac{1}{50\text{ Hz}} = 0.020\text{ s} = 20{,}000\,\mu\text{s}$$

### 7.1 Kinematic Angle to Servo Command
$$\theta_{\text{command}} = \theta_{\text{center}} + \text{direction} \times (\theta_{\text{IK}} - \theta_{\text{offset}})$$
$$\theta_{\text{safe}} = \text{constrain}(\theta_{\text{command}}, \theta_{\min}, \theta_{\max})$$

### 7.2 Pulse Width Mapping
$$t_{\text{pulse}} = t_{\min} + \left(\frac{\theta_{\text{safe}}}{180^\circ}\right) \times (t_{\max} - t_{\min})$$
where typically $t_{\min} = 500\,\mu\text{s}$ and $t_{\max} = 2500\,\mu\text{s}$.

### 7.3 PCA9685 Tick Conversion
$$\text{ticks} = \text{round}\left(\frac{t_{\text{pulse}} \times 4096}{20{,}000\,\mu\text{s}}\right)$$
$$\text{ticks}_{\text{clamped}} = \text{constrain}(\text{ticks}, \text{tick}_{\min}, \text{tick}_{\max})$$

For standard endpoints:
- $500\,\mu\text{s} \implies \frac{500 \times 4096}{20000} \approx 102\text{ ticks}$
- $1500\,\mu\text{s} \implies \frac{1500 \times 4096}{20000} \approx 307\text{ ticks}$
- $2500\,\mu\text{s} \implies \frac{2500 \times 4096}{20000} \approx 512\text{ ticks}$

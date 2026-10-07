# Challenge 1: Robot Simulation & 3D Rotation Kinematics 🤖🚁

**Course:** Introduction to Robotics (ITR / ME 639) — IIT Gandhinagar  
**Author:** Prachi Jindal

---

## 📌 Challenge Overview

This project delivers a complete **MuJoCo physics simulation** featuring real-time keyboard teleoperation, dynamic coordinate frame tracking, and live $SO(3)$ rotation matrix visualization for two distinct robotic platforms:

1. **TurtleBot3 Waffle Pi:** 3-DOF planar ground mobile robot (differential drive kinematics).
2. **Quadcopter Drone:** 6-DOF aerial multi-rotor vehicle (flight dynamics and attitude kinematics).

---

## 🌟 Key Features

- **High-Fidelity MJCF Models:**
  - `model/turtlebot3_waffle_pi.xml` — Dual actuated wheels, caster supports, lidar turret, camera bracket, and RGB body axes.
  - `model/quadcopter.xml` — 4-rotor X-configuration multirotor with fuselage, prop discs, landing legs, and body axes.
- **Dynamic Coordinate Frame Visualization:**
  - **Fixed World / Space Frame $\{s\}$:** Displayed at $(0, 0, 0)$ as reference markers.
  - **Attached Moving Body Frame $\{b\}$:** Attached to the robot's center of mass, rendered with standard **RGB (Red = $+X$, Green = $+Y$, Blue = $+Z$)** axes that dynamically translate and rotate in 3D.
- **Live Rotation Matrix Readout ($SO(3)$ HUD):**
  - Instantaneous console display of the $3 \times 3$ rotation matrix $R_{sb} \in SO(3)$, position vector $\mathbf{p}$, Euler angles $(\phi, \theta, \psi)$, and numerical orthogonality checks ($R^T R = I, \det(R) = +1$).
- **Keyboard Teleoperation:** Smooth, responsive real-time control for driving and flight.

---

## 🎥 Demo Video

| Robot | Video | What it shows |
| :--- | :--- | :--- |
| TurtleBot3 Waffle Pi | [`videos/wafflepi.mp4`](videos/wafflepi.mp4) | Keyboard driving and turning, the body frame $\{b\}$ rotating about $+Z$, and the live $R_{sb}$ readout |
| Quadcopter | [`videos/quadcopter.mp4`](videos/quadcopter.mp4) | Climbing, pitching, rolling and yawing, the body frame tilting in 3D, and the live $R_{sb}$ readout |

---

## 🗂️ Project Directory Structure

```
Challenge_1/
├── model/
│   ├── turtlebot3_waffle_pi.xml       # MJCF scene for TurtleBot3 Waffle Pi
│   └── quadcopter.xml                 # MJCF scene for 6-DOF Quadcopter Drone
│
├── scripts/
│   ├── utils.py                       # SO(3) matrix, quaternion, and HUD formatting utilities
│   ├── teleop_turtlebot.py            # TurtleBot3 teleop + live rotation matrix HUD
│   ├── teleop_quadcopter.py           # Quadcopter flight teleop + live rotation matrix HUD
│   └── run_challenge1.py              # Interactive launcher menu
│
├── videos/
│   ├── wafflepi.mp4                   # TurtleBot3 Waffle Pi demo
│   └── quadcopter.mp4                 # Quadcopter demo
│
├── requirements.txt                   # Project dependencies (mujoco, numpy)
└── README.md                          # Challenge documentation & user guide
```

---

## 🚀 Setup & Execution Guide

### 1. Prerequisites & Virtual Environment
```bash
# Clone the repository (if not already done)
git clone https://github.com/prachiiii27/ITR_Prachi.git
cd ITR_Prachi/Challenges/Challenge_1

# Create and activate virtual environment
python -m venv venv
source venv/bin/activate       # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 2. Launch Interactive Menu
```bash
python scripts/run_challenge1.py
```

### 3. Or Run Either Simulation Directly:

#### 🚜 Option A: TurtleBot3 Waffle Pi
```bash
python scripts/teleop_turtlebot.py
```

| Key | Action |
| :---: | :--- |
| `W` / `↑` | Accelerate Forward |
| `S` / `↓` | Reverse / Decelerate |
| `A` / `←` | Steer Left (Counter-clockwise Yaw) |
| `D` / `→` | Steer Right (Clockwise Yaw) |
| `X` / `B` | Brake / Stop (zero velocity) |
| `R` | Reset to Origin Pose |
| `SPACE` | MuJoCo Pause / Unpause |

---

#### 🚁 Option B: 6-DOF Quadcopter Drone
```bash
python scripts/teleop_quadcopter.py
```

| Key | Action |
| :---: | :--- |
| `W` / `S` | Pitch Forward / Pitch Backward |
| `A` / `D` | Roll Left / Roll Right |
| `I` / `K` (or `↑`/`↓`) | Climb Altitude (+Z) / Descend (-Z) |
| `J` / `L` (or `←`/`→`) | Yaw Rotate Left / Yaw Rotate Right |
| `X` / `H` | Level Drone & Hold Current Altitude |
| `R` | Reset to Initial Hover Pose |
| `SPACE` | MuJoCo Pause / Unpause |

---

## 🧮 Mathematical Background

### 1. Spatial Rotation Representation ($SO(3)$)
The orientation of the moving body frame $\{b\}$ relative to the space frame $\{s\}$ is represented by the matrix $R_{sb} \in SO(3)$:

$$R_{sb} = \begin{bmatrix} r_{11} & r_{12} & r_{13} \\ r_{21} & r_{22} & r_{23} \\ r_{31} & r_{32} & r_{33} \end{bmatrix}, \quad R^T R = I, \quad \det(R) = +1$$

Converted from the unit quaternion $\mathbf{q} = [w, x, y, z]^T$:

$$R(\mathbf{q}) = \begin{bmatrix} 1 - 2(y^2 + z^2) & 2(xy - zw) & 2(xz + yw) \\ 2(xy + zw) & 1 - 2(x^2 + z^2) & 2(yz - xw) \\ 2(xz - yw) & 2(yz + xw) & 1 - 2(x^2 + y^2) \end{bmatrix}$$

### 2. Differential Drive Kinematics (TurtleBot3)
For forward velocity $v$ and yaw rate $\omega$, with wheel radius $r$ and track width $L$:

$$\dot{\theta}_L = \frac{v - \frac{\omega L}{2}}{r}, \quad \dot{\theta}_R = \frac{v + \frac{\omega L}{2}}{r}$$

### 3. 6-DOF Quadcopter Dynamics
Total thrust $T$ aligned along the body $+Z$ axis maps to spatial frame coordinates:

$$\mathbf{F}_s = R_{sb} \begin{bmatrix} 0 \\ 0 \\ T \end{bmatrix} + m \mathbf{g}$$

### 4. Control Laws Used
- **Altitude (PD + gravity feed-forward):** $T = mg + K_{p,z}(z_{ref} - z) - K_{d,z}\,\dot z$, with $m$ read from the model (`body_subtreemass`).
- **Roll / pitch (PD on Euler-angle error):** $\tau_x = K_p(\phi_{ref} - \phi) - K_d\,\omega_x$, $\tau_y = K_p(\theta_{ref} - \theta) - K_d\,\omega_y$.
- **Yaw (rate command):** $\tau_z = K_{p,\psi}\,\dot\psi_{ref} - K_{d,\psi}\,\omega_z$.
- Body-frame thrust and torques are rotated to the world frame with $R_{sb}$ before being applied via `xfrc_applied`.
- Sign convention: a positive pitch $\theta$ tips the thrust axis towards $+X$ (forward); a negative roll $\phi$ tips it towards $+Y$ (left).

# Challenge 2: MuJoCo Forward Kinematics (FK) & Robot Control Lab

This repository contains the complete implementation for **Challenge 2** of the MuJoCo Forward Kinematics & Robotics Lab, featuring interactive simulation, torque/position PD control, gravity compensation via Recursive Newton-Euler (RNE), and End-Effector (EE) tracking for **6-DOF**, **7-DOF**, **HEAL**, and **Franka Emika Panda** robot manipulators.

---

## 📌 Tasks Overview

### Task 1: 6-DOF & 7-DOF Kinematic Manipulators
- **6-DOF Industrial Manipulator** (`scripts/task1_6dof_manipulator.py`):
  - Custom self-contained XML string modeling a 6-DOF industrial arm with visual geoms, physical defaults, and torque (`motor`) actuators.
  - Interactive joint control with RNE gravity compensation and PD torque control.
  - End-effector site (`EE`) position from our own Product-of-Exponentials FK, printed next to MuJoCo's value as a check.
- **7-DOF Redundant Manipulator** (`scripts/task1_7dof_manipulator.py`):
  - Custom self-contained XML string modeling a 7-DOF Franka-style kinematically redundant arm.
  - Interactive 7-joint movement using keyboard controls.
  - Gravity compensation via Recursive Newton-Euler algorithm (`mujoco.mj_rne`).
  - End-effector position from our own PoE FK, checked against MuJoCo.

### Task 2: HEAL & Franka Robot XML Simulation
- **HEAL Robot Model** (`scripts/task2_heal_robot.py`):
  - Dynamically loads `robot_descriptions/single_arm_heal_effort_actuation_rs_mj_2.xml`.
  - Joint position target mapping and PD torque control with RNE gravity compensation, respecting actuator effort/ctrl ranges.
  - Real-time End-Effector (`right_center` site) tracking with our own PoE FK vs MuJoCo.
- **Franka Emika Panda Robot Model** (`scripts/task2_franka_robot.py`):
  - Dynamically loads `robot_descriptions/franka/panda.xml`.
  - Full 7-DOF arm joint control + interactive parallel-jaw gripper control (Open / Close).
  - Gravity bias force compensation + PD control loop.
  - The Menagerie `panda.xml` defines arm actuators 1–7 as position servos; the script switches them to torque motors at load time (limits 87 N·m for joints 1–4, 12 N·m for joints 5–7) so the PD + gravity-compensation torques are applied as torques.
  - Starts from the Panda home pose $q = [0, 0, 0, -\pi/2, 0, \pi/2, -\pi/4]$ (all-zero is outside joint 4's range); the gripper command in metres (0–0.04 m) is mapped to the actuator's 0–255 range.
  - End-effector (`hand` body) position from our own PoE FK, checked against MuJoCo.

---

## 📐 Mathematical Formulation of Forward Kinematics (FK)

Forward Kinematics (FK) is the mapping from the joint space configuration vector $\mathbf{q} = [q_1, q_2, \dots, q_n]^T \in \mathbb{R}^n$ to the end-effector pose $\mathbf{T}_{EE} \in \text{SE}(3)$ in 3D operational space:

$$\mathbf{T}_{EE}(\mathbf{q}) = \begin{bmatrix} \mathbf{R}_{EE}(\mathbf{q}) & \mathbf{p}_{EE}(\mathbf{q}) \\ \mathbf{0}_{1 \times 3} & 1 \end{bmatrix} \in \text{SE}(3)$$

where $\mathbf{R}_{EE}(\mathbf{q}) \in \text{SO}(3)$ represents the end-effector rotation matrix and $\mathbf{p}_{EE}(\mathbf{q}) = [x, y, z]^T \in \mathbb{R}^3$ represents the Cartesian position of the end-effector.

---

### 1. Homogeneous Transformations & Kinematic Chain

For an $n$-DOF spatial manipulator, the total transformation matrix from the base frame $\{0\}$ to the end-effector frame $\{EE\}$ is the ordered product of adjacent link transformation matrices:

$$\mathbf{T}_{EE}^0(\mathbf{q}) = \mathbf{T}_1^0(q_1) \cdot \mathbf{T}_2^1(q_2) \cdot \dots \cdot \mathbf{T}_n^{n-1}(q_n) \cdot \mathbf{T}_{EE}^n$$

Each adjacent transform $\mathbf{T}_i^{i-1}(q_i)$ accounts for both relative orientation $\mathbf{R}_i^{i-1}$ and translation $\mathbf{p}_i^{i-1}$:

$$\mathbf{T}_i^{i-1}(q_i) = \begin{bmatrix} \mathbf{R}_i^{i-1}(q_i) & \mathbf{p}_i^{i-1} \\ \mathbf{0} & 1 \end{bmatrix}$$

---

### 2. Denavit-Hartenberg (D-H) Convention

Under standard Denavit-Hartenberg convention, each link transformation matrix $\mathbf{A}_i(q_i)$ is determined by four link parameters $(\theta_i, d_i, a_i, \alpha_i)$:

$$\mathbf{A}_i(q_i) = \text{Rot}_z(\theta_i) \cdot \text{Trans}_z(d_i) \cdot \text{Trans}_x(a_i) \cdot \text{Rot}_x(\alpha_i)$$

$$\mathbf{A}_i(q_i) = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

For a revolute joint, $\theta_i = q_i + \theta_{i, \text{offset}}$, while $d_i$, $a_i$, and $\alpha_i$ are fixed link geometric parameters.

---

### 3. Product of Exponentials (PoE) / Lie Group Formulation

Alternatively, using the Product of Exponentials (PoE) formula in Lie algebra $\mathfrak{se}(3)$, the forward kinematics pose is expressed directly using joint screw axes $\mathbf{S}_i = [\boldsymbol{\omega}_i^T, \mathbf{v}_i^T]^T \in \mathbb{R}^6$:

$$\mathbf{T}(\mathbf{q}) = e^{[\mathbf{S}_1] q_1} e^{[\mathbf{S}_2] q_2} \cdots e^{[\mathbf{S}_n] q_n} \mathbf{M}$$

where:
- $\mathbf{M} \in \text{SE}(3)$ is the home configuration matrix when all joint angles $\mathbf{q} = \mathbf{0}$.
- $[\mathbf{S}_i] \in \mathfrak{se}(3)$ is the $4 \times 4$ matrix representation of the $i$-th joint screw axis:
  $$[\mathbf{S}_i] = \begin{bmatrix} [\boldsymbol{\omega}_i] & \mathbf{v}_i \\ \mathbf{0} & 0 \end{bmatrix}, \quad [\boldsymbol{\omega}_i] = \begin{bmatrix} 0 & -\omega_{z,i} & \omega_{y,i} \\ \omega_{z,i} & 0 & -\omega_{x,i} \\ -\omega_{y,i} & \omega_{x,i} & 0 \end{bmatrix}$$

---

### 4. Differential Kinematics & Geometric Jacobian

The mapping from joint velocity vector $\dot{\mathbf{q}}$ to spatial end-effector velocity twist $\mathbf{V}_{EE} = [\boldsymbol{\omega}_{EE}^T, \mathbf{v}_{EE}^T]^T$ is governed by the Geometric Jacobian matrix $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{6 \times n}$:

$$\mathbf{V}_{EE} = \begin{bmatrix} \boldsymbol{\omega}_{EE} \\ \mathbf{v}_{EE} \end{bmatrix} = \mathbf{J}(\mathbf{q}) \dot{\mathbf{q}} = \sum_{i=1}^n \mathbf{J}_i(\mathbf{q}) \dot{q}_i$$

For a revolute joint $i$ centered at position $\mathbf{p}_{i-1}$ with rotation axis $\mathbf{z}_{i-1}$:

$$\mathbf{J}_i(\mathbf{q}) = \begin{bmatrix} \mathbf{z}_{i-1} \\ \mathbf{z}_{i-1} \times (\mathbf{p}_{EE} - \mathbf{p}_{i-1}) \end{bmatrix}$$

---

### 5. Hierarchical Kinematic Forwarding in MuJoCo

In MuJoCo (`mj_forward`), spatial transformations for each body $i$ are computed recursively:

$$\mathbf{X}_i = \mathbf{X}_{\text{parent}(i)} \cdot \mathbf{T}_{\text{joint}(i)}(q_i)$$

The global Cartesian position of the end-effector site $\mathbf{p}_{EE}$ reported in the scripts is:

$$\mathbf{p}_{EE} = \mathbf{p}_{\text{body}(EE)} + \mathbf{R}_{\text{body}(EE)} \cdot \mathbf{r}_{\text{site\_offset}}$$

---

### 6. FK Verification in Code (`scripts/fk_utils.py`)

The PoE formula above is implemented directly in `fk_utils.py`:

1. At $\mathbf{q} = \mathbf{0}$, each hinge joint's world axis $\boldsymbol{\omega}_i$ and a point $\mathbf{p}_i$ on it are read from the model, giving $\mathbf{S}_i = [\boldsymbol{\omega}_i;\ -\boldsymbol{\omega}_i \times \mathbf{p}_i]$, and the end-effector home pose gives $\mathbf{M}$.
2. For any $\mathbf{q}$, $e^{[\mathbf{S}_i]q_i}$ is computed with Rodrigues' formula and the product $e^{[\mathbf{S}_1]q_1}\cdots e^{[\mathbf{S}_n]q_n}\mathbf{M}$ gives $\mathbf{T}_{EE}(\mathbf{q})$.
3. Every script prints this end-effector position next to MuJoCo's own value and the difference. Over 200 random joint configurations of all four robots the two agree to within $10^{-14}$ (machine precision).

### 7. Joint-Space Control Law

All four scripts use the same PD controller with gravity compensation:

$$\boldsymbol{\tau} = \mathbf{K}_p(\mathbf{q}_{ref} - \mathbf{q}) - \mathbf{K}_d\,\dot{\mathbf{q}} + \mathbf{c}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{g}(\mathbf{q})$$

where $\mathbf{c} + \mathbf{g}$ comes from `mujoco.mj_rne` with zero acceleration, and $\boldsymbol{\tau}$ is clipped to each actuator's torque limit.

---

## 🎥 Demo Video

▶️ **[Watch the Challenge 2 demo on YouTube](YOUTUBE_LINK_HERE)**

The video shows all four simulations (6-DOF arm, 7-DOF arm, HEAL robot, Franka Panda): moving joints with the keyboard, the arm holding each target under gravity, the Franka gripper opening and closing, and the terminal printing our own PoE FK end-effector position next to MuJoCo's.

---

## 🚀 Getting Started & Requirements

### Prerequisites
- Python 3.10+
- `mujoco` >= 3.1
- `numpy`

### Installation
Install all required dependencies using pip:
```bash
pip install -r requirements.txt
```

---

## 🎮 Interactive Keyboard Controls

Click inside the **MuJoCo Viewer** window to give it focus before using the keyboard:

| Key Pair | Action |
|---|---|
| **Q / A** | Joint 1 (+ / -) |
| **W / S** | Joint 2 (+ / -) |
| **E / D** | Joint 3 (+ / -) |
| **R / F** | Joint 4 (+ / -) |
| **T / G** | Joint 5 (+ / -) |
| **Y / H** | Joint 6 (+ / -) |
| **U / J** | Joint 7 (+ / -) *(7-DOF & Franka)* |
| **O / P** | Gripper Open / Close *(Franka)* |
| **SPACE** | Reset all joint targets (Franka: back to home pose, gripper open) |

---

## 💻 Running the Scripts

### Interactive Launcher Menu
Run all scripts from a single unified interactive prompt:
```bash
python scripts/run_all.py
```

### Individual Execution
- **Task 1 (6-DOF Manipulator):**
  ```bash
  python scripts/task1_6dof_manipulator.py
  ```
- **Task 1 (7-DOF Manipulator):**
  ```bash
  python scripts/task1_7dof_manipulator.py
  ```
- **Task 2 (HEAL Robot Model):**
  ```bash
  python scripts/task2_heal_robot.py
  ```
- **Task 2 (Franka Panda Model):**
  ```bash
  python scripts/task2_franka_robot.py
  ```

---

## 📁 Repository Structure

```
Challenge_2/
├── robot_descriptions/
│   ├── single_arm_heal_effort_actuation_rs_mj_2.xml
│   ├── franka/
│   │   ├── panda.xml
│   │   └── assets/
│   └── ...
├── scripts/
│   ├── run_all.py                 # Interactive Launcher
│   ├── task1_6dof_manipulator.py  # 6-DOF Industrial Arm Script
│   ├── task1_7dof_manipulator.py  # 7-DOF Franka-Style Arm Script
│   ├── task2_heal_robot.py        # HEAL XML Model Script
│   ├── task2_franka_robot.py      # Franka Panda XML Model Script
│   └── fk_utils.py                # Our PoE forward kinematics + RNE gravity torque helper
├── environment.yml
├── requirements.txt
└── README.md
```

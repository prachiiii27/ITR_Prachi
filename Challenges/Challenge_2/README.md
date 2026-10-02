# Challenge 2: MuJoCo Forward Kinematics (FK) & Robot Control Lab

This repository contains the complete implementation for **Challenge 2** of the MuJoCo Forward Kinematics & Robotics Lab, featuring interactive simulation, torque/position PD control, gravity compensation via Recursive Newton-Euler (RNE), and End-Effector (EE) tracking for **6-DOF**, **7-DOF**, **HEAL**, and **Franka Emika Panda** robot manipulators.

---

## 📌 Tasks Overview

### Task 1: 6-DOF & 7-DOF Kinematic Manipulators
- **6-DOF Industrial Manipulator** (`scripts/task1_6dof_manipulator.py`):
  - Custom self-contained XML string modeling a 6-DOF industrial arm with visual geoms, physical defaults, and actuators.
  - Interactive joint control with RNE gravity compensation and PD torque control.
  - End-effector site (`EE`) position calculation and real-time output.
- **7-DOF Redundant Manipulator** (`scripts/task1_7dof_manipulator.py`):
  - Custom self-contained XML string modeling a 7-DOF Franka-style kinematically redundant arm.
  - Interactive 7-joint movement using keyboard controls.
  - Gravity compensation via Recursive Newton-Euler algorithm (`mujoco.mj_rne`).

### Task 2: HEAL & Franka Robot XML Simulation
- **HEAL Robot Model** (`scripts/task2_heal_robot.py`):
  - Dynamically loads `robot_descriptions/single_arm_heal_effort_actuation_rs_mj_2.xml`.
  - Joint position target mapping and PD control respecting actuator effort/ctrl ranges.
  - Real-time End-Effector (`right_center` site) tracking.
- **Franka Emika Panda Robot Model** (`scripts/task2_franka_robot.py`):
  - Dynamically loads `robot_descriptions/franka/panda.xml`.
  - Full 7-DOF arm joint control + interactive parallel-jaw gripper control (Open / Close).
  - Gravity bias force compensation + PD control loop.

---

## 🚀 Getting Started & Requirements

### Prerequisites
- Python 3.10+
- `mujoco` >= 3.0
- `numpy`
- `scipy`

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
| **SPACE** | Reset all joints and targets to default |

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
ITR_mujoco_fk_lab/
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
│   └── task2_franka_robot.py      # Franka Panda XML Model Script
├── environment.yml
├── requirements.txt
└── README.md
```

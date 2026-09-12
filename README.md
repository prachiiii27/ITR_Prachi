# Introduction to Robotics (ITR / ME 639) 🤖

**Repository of Coursework, Homework Assignments, and Practical Challenges**  
*Department of Mechanical Engineering, Indian Institute of Technology Gandhinagar (IITGN)*  
**Author:** Prachi Jindal

---

## 📌 About the Course

**Introduction to Robotics** covers the foundational mathematics, mechanics, computation, and software architectures used in modern robotics engineering. Key topics include:

- **Rigid Body Kinematics:** Spatial rotations ($SO(3)$), homogeneous transformations ($SE(3)$), Euler angles, unit quaternions, and screw theory (Product of Exponentials).
- **Forward & Inverse Kinematics:** Analytical and numerical solvers for open-chain serial manipulators.
- **Differential Kinematics & Statics:** Geometric and analytical Jacobians, manipulability ellipsoids, and singularity analysis.
- **Robot Dynamics:** Lagrangian formulation, Newton-Euler recursive algorithm, and inertia modeling.
- **Motion Planning & Control:** Trajectory generation, operational space control, and impedance/force control.
- **Simulation & Software Frameworks:** MuJoCo physics engine, ROS2 (Robot Operating System), and RViz.

---

## 🗂️ Repository Structure

```
ITR_Prachi/
├── Homework/                       # Semester Homework Assignments
│   ├── HW1/                        # Homework 1: 3D Rotations & Kinematics
│   │   ├── Problem1_Rotation_Visualizer/  # WebGL/Three.js interactive rotation tool
│   │   ├── model/                  # MuJoCo asymmetric body model definition
│   │   ├── scripts/                # Python simulation and verification scripts
│   │   ├── ros_ws/                 # ROS2 TF broadcaster workspace
│   │   ├── results/                # Numerical plots and outputs
│   │   ├── videos/                 # Simulation recordings
│   │   ├── Hw1_part1.pdf           # Written solutions & analytical derivations
│   │   └── README.md               # Detailed HW1 documentation
│   └── README.md                   # Homework directory index
│
└── Challenges/                     # Hands-on Robotics Challenges & Mini-Projects
    └── README.md                   # Challenges directory index
```

---

## 🛠️ Environment & Prerequisites

- **Python:** 3.8+ (`numpy`, `scipy`, `matplotlib`, `mujoco`)
- **Web Browser:** Any modern browser with WebGL support (Chrome, Firefox, Safari, Edge)
- **ROS Version:** ROS 2 Humble / Iron / Jazzy (optional, for ROS workspace packages)

---

## 👤 Author

**Prachi Jindal**  
Junior Undergraduate, Mechanical Engineering  
Indian Institute of Technology Gandhinagar (IITGN)

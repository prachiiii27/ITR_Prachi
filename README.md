# Introduction to Robotics (ITR) 🤖

**Course Repository: Coursework, Homework Assignments & Practical Challenges**  
*Department of Mechanical Engineering, Indian Institute of Technology Gandhinagar (IITGN)*  
**Author:** Prachi Jindal

---

## 📚 Course Overview

This repository contains all academic assignments, interactive simulation tools, robotics codebases, and challenge projects developed as part of the **Introduction to Robotics (ITR)** course at IIT Gandhinagar.

---

## 🎯 Course Content & Topics

- **Introduction to Robotics:** Types, Terminology, Applications.
- **Review of Coordinate Frames, Vectors, and Transformations in 3D.**
- **Rotation Representations:** Matrices, Euler Angles, Axis-Angle, Quaternions.
- **Rigid Body Transformations:** Homogeneous Coordinates, $SE(3)$, Composition.
- **Denavit-Hartenberg (DH) Representation and Conventions.**
- **Forward and Inverse Kinematics for Serial Manipulators and Mobile Robots.**
- **Workspaces and Reachability Analysis.**
- **Velocity Kinematics; Manipulator Jacobians; Inverse Velocity Analysis.**
- **Singularities:** Identification and Implications for Motion.
- **Motion Planning and Trajectory Generation using Vision; Motion Control.**
- **Introduction to Robot Statics and Dynamics.**

---

## 🗂️ Repository Structure

```
ITR_Prachi/
├── README.md                            # Main Course Overview & Syllabus
│
├── Homework/                            # Homework Assignments & Problems
│   ├── README.md                        # Homework assignments index
│   │
│   ├── Homework_Problem_1/              # Homework Problem 1: Interactive 3D Frame Rotation Visualizer
│   │   ├── rigid_body_rotation_visualizer.html
│   │   ├── rigid_body_rotation_visualizer_1.html
│   │   └── README.md
│   │
│   └── HW1/                             # Homework 1: MuJoCo Rotation Sandbox & Skew Verification
│       ├── model/                       # MuJoCo MJCF model
│       ├── scripts/                     # Python simulation scripts
│       ├── ros_ws/                      # ROS2 TF broadcaster workspace
│       ├── results/                     # Numerical plots
│       ├── videos/                      # Simulation video demos
│       ├── Hw1_part1.pdf                # Analytical solutions & derivations
│       └── README.md
│
└── Challenges/                          # Robotics Challenges & Mini-Projects
    ├── README.md                        # Challenges index
    │
    └── Challenge_1/                     # Challenge 1: TurtleBot3 & Quadcopter Simulation
        ├── model/                       # MJCF XML models for TurtleBot3 & Quadcopter
        ├── scripts/                     # Teleop & Rotation Matrix HUD scripts
        ├── requirements.txt             # Dependencies
        └── README.md                    # Complete challenge documentation
```

---

## 🛠️ Software & Environment

- **Python:** 3.8+ (`numpy`, `scipy`, `matplotlib`, `mujoco`)
- **Web Browser:** Any modern browser with WebGL support for 3D visualizers
- **Robot Middleware:** ROS 2 (Humble / Iron / Jazzy)

---

## 👤 Author

**Prachi Jindal**  
Junior Undergraduate, Mechanical Engineering  
Indian Institute of Technology Gandhinagar (IITGN)

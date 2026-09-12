# Homework 1: MuJoCo 3D Rotation Sandbox & Skew Properties 🔄

**Course:** Introduction to Robotics (ITR) — IIT Gandhinagar  
**Author:** Prachi Jindal

---

## 📖 Overview

Homework 1 focuses on 3D rotation kinematics in physics simulation, contrasting **current-frame (intrinsic)** versus **fixed-frame (extrinsic)** rotation composition, and numerically verifying skew-symmetric matrix identities using MuJoCo and ROS2.

---

## 🗂️ Directory Structure

```
HW1/
├── model/
│   └── asymmetric_body.xml             # MuJoCo MJCF model: asymmetric free body + axis markers
├── scripts/
│   ├── utils.py                        # Rotation math: hat/vee operators, exp/log, quat ↔ R
│   ├── 01_rotation_sandbox.py          # Current-frame vs fixed-frame animation in MuJoCo
│   ├── 02_verify_skew_properties.py    # Numerical batch verification of skew-symmetric identities
│   └── 02_verify_skew_properties_live.py # Live viewer + residual error plot
├── ros_ws/                             # ROS2 workspace: TF coordinate frame broadcaster
│   ├── README.md
│   └── src/hw01_tf_demo/
├── results/
│   └── q8_residual_plot.png            # Numerical residual verification plot
├── videos/
│   └── Q7_rotation_demo.mp4            # MuJoCo rotation composition screen recording
├── Hw1_part1.pdf                       # Written derivations and solutions
├── AI_USE_NOTE.md                      # AI tools usage disclosure
└── requirements.txt                    # Python dependencies (mujoco, numpy, matplotlib)
```

---

## 🚀 Setup & Execution

### 1. MuJoCo Python Simulations
```bash
# Setup virtual environment
python -m venv venv
source venv/bin/activate    # On Windows: venv\Scripts\activate
pip install -r requirements.txt

# Run Rotation Sandbox (Current vs Fixed Frame composition)
cd scripts
python 01_rotation_sandbox.py

# Verify Skew-Symmetric Properties (Numerical Batch)
python 02_verify_skew_properties.py

# Live Verification & Residual Plotting
python 02_verify_skew_properties_live.py
```

### 2. ROS2 TF Broadcaster Demo
```bash
cd ros_ws
colcon build
source install/setup.bash
ros2 run hw01_tf_demo tf_broadcaster_node
```

---

## 🔑 Key Concepts Covered

- **Special Orthogonal Group $SO(3)$:** $R^T R = I, \det(R) = +1$
- **Composition Rules:**
  - *Fixed-frame (Space):* Pre-multiply ($R_{\text{new}} = R_{\text{step}} R$)
  - *Body-frame (Current):* Post-multiply ($R_{\text{new}} = R R_{\text{step}}$)
- **Lie Algebra $\mathfrak{so}(3)$:** Skew-symmetric representation $[\omega] \in \mathbb{R}^{3 \times 3}$ and matrix exponential.

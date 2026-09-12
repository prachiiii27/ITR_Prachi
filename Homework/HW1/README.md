# Homework 1: 3D Rigid Body Rotations & Kinematics 🔄

**Course:** Introduction to Robotics (ITR / ME 639) — IIT Gandhinagar  
**Student:** Prachi Jindal

---

## 📖 Overview

Homework 1 focuses on the foundations of 3D spatial rotations, representation of orientations ($SO(3)$), composition of rotations, skew-symmetric matrices, Lie algebra $\mathfrak{so}(3)$, and numerical simulation in physics engines and ROS2.

This assignment comprises:
1. **Problem 1:** Interactive 3D Frame Rotation Visualizer (Web-based with real-time sliders & $SO(3)$ matrix readout).
2. **Part 2:** MuJoCo 3D Rotation Sandbox & verification of skew-symmetric matrix identities.
3. **ROS2 Demonstration:** ROS2 TF broadcaster node visualizing coordinate frames in RViz.
4. **Analytical Solutions:** Complete written derivation and solutions in [`Hw1_part1.pdf`](Hw1_part1.pdf).

---

## 🗂️ Directory Structure

```
HW1/
├── Problem1_Rotation_Visualizer/       # Task 1: WebGL/Three.js interactive visualizer
│   ├── rigid_body_rotation_visualizer.html
│   ├── rigid_body_rotation_visualizer_1.html
│   └── README.md
├── model/
│   └── asymmetric_body.xml             # MuJoCo MJCF asymmetric free body model
├── scripts/
│   ├── utils.py                        # Spatial math: hat/vee operators, exp/log, quat ↔ R
│   ├── 01_rotation_sandbox.py          # Current-frame vs fixed-frame animation in MuJoCo
│   ├── 02_verify_skew_properties.py    # Batch verification of skew-symmetric identities
│   └── 02_verify_skew_properties_live.py # Live viewer + residual error plot
├── ros_ws/                             # ROS2 workspace for TF coordinate frame broadcaster
│   ├── README.md
│   └── src/hw01_tf_demo/
├── results/
│   └── q8_residual_plot.png            # Numerical residual verification plot
├── videos/
│   └── Q7_rotation_demo.mp4            # MuJoCo rotation composition video demo
├── Hw1_part1.pdf                       # Written solutions & analytical derivations
├── AI_USE_NOTE.md                      # Disclosure of AI tools usage
└── requirements.txt                    # Python dependencies (mujoco, numpy, matplotlib)
```

---

## 🚀 Setup & Execution

### 1. Interactive 3D Visualizer (Problem 1)
Open [`Problem1_Rotation_Visualizer/rigid_body_rotation_visualizer.html`](Problem1_Rotation_Visualizer/rigid_body_rotation_visualizer.html) in any web browser.

### 2. MuJoCo Python Simulations
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

### 3. ROS2 TF Broadcaster Demo
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
- **Lie Algebra $\mathfrak{so}(3)$:** Skew-symmetric representation $[\omega] \in \mathbb{R}^{3 \times 3}$ and Rodrigues' Formula / Matrix Exponential.

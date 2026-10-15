# Robotics Challenges & Mini-Projects 🏆

This directory hosts practical robotics challenges, simulation tasks, and control project.

---

## 📋 Challenge Tracks

| Track | Title | Status | Description |
| :--- | :--- | :---: | :--- |
| **[Challenge 1](Challenge_1/)** | TurtleBot3 & Quadcopter MuJoCo Simulation | ✅ Completed | Spawning TurtleBot3 Waffle Pi and 6-DOF Quadcopter in MuJoCo with keyboard teleoperation, dynamic body frame visualization, and live $SO(3)$ rotation matrix display. |
| **[Challenge 2](Challenge_2/)** | MuJoCo Forward Kinematics (FK) & Robot Control Lab | ✅ Completed | 6-DOF industrial arm, 7-DOF redundant arm, HEAL robot, and Franka Emika Panda with interactive keyboard controls, PD torque regulation, RNE gravity compensation, and real-time FK end-effector tracking. |
| **[Challenge 3 – Franka](Challenge_3Franka/)** | Pick and Place with IK – Franka Emika Panda (7-DoF) | ✅ Completed | Panda picks a randomly posed cube and places it in a tray. Workspace analysis and table design, randomized episodes with logging, collision-aware planning, and four IK solvers compared on identical seeds: Mink, pseudo-inverse, Damped Least Squares and QP with joint/velocity limits. |
| **[Challenge 3 – HEAL](Challenge_3Heal/)** | Pick and Place with IK – Addverb HEAL (6-DoF) | ✅ Completed | Same task with the HEAL arm from Challenge 2. Added a two-finger gripper and position servos, an IK seed library and joint-space transfers (a 6-DoF arm has no spare joint), and a new workspace-based table (H = 0.30 m). Same four IK solvers and experiments. |
---
## 🛠️ Common Setup
 
```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r <challenge folder>/requirements.txt
```
 
Each challenge folder has its own README with exact run commands.

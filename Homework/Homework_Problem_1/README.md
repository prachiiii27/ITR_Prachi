# Homework Problem 1: Interactive 3D Frame Rotation & Rotation Matrix Visualizer 🌐

**Course:** Introduction to Robotics (ITR / ME 639) — IIT Gandhinagar  
**Author:** Prachi Jindal

---

## 🎥 Video Demonstration

▶️ **Simulation Video:** [Watch on YouTube](https://youtu.be/2pB4u-ScafU)

Screen recording demonstrating real-time slider manipulation, dynamic body frame rotations, and live rotation matrix updates.

---

## 📌 Problem Statement

Build an interactive script/program to visualize rigid body rotations in 3D space. The tool must display:
- **Two Coordinate Frames:** One fixed reference frame (World/Space Frame $\{s\}$) and one rotatable body frame ($\{b\}$) controlled in real time using interactive sliders.
- **Live Rotation Matrix Readout:** Real-time numerical display of the corresponding $3 \times 3$ rotation matrix $R \in SO(3)$.
- **Interactive Controls:** Sliders for Euler rotation angles with instantaneous frame re-orientation and matrix updates.

---

## 🚀 How to Run

Simply open either visualizer file directly in any modern web browser (no installation or local server required):

- **[`rigid_body_rotation_visualizer.html`](rigid_body_rotation_visualizer.html)** — Interactive 3D visualization tool featuring fixed reference and body frames with live matrix calculation.
- **[`rigid_body_rotation_visualizer_1.html`](rigid_body_rotation_visualizer_1.html)** — Enhanced version with updated visual styling, axis rendering, and smooth slider response.

---

## 🧮 Mathematical Formulation

Elementary rotation matrices for roll ($\phi$), pitch ($\theta$), and yaw ($\psi$):

$$R_x(\phi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}$$

$$R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}$$

$$R_z(\psi) = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

- **Composition:** $R = R_z(\psi) R_y(\theta) R_x(\phi)$
- **Orthonormality Property:** $R^T R = I, \quad \det(R) = +1$

---

## 📁 Repository Files

| File | Description |
| :--- | :--- |
| `rigid_body_rotation_visualizer.html` | Core WebGL/Three.js interactive rotation tool |
| `rigid_body_rotation_visualizer_1.html` | Enhanced visualizer variant |

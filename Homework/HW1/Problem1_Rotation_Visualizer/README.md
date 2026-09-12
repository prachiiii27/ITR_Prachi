# Problem 1 — Interactive 3D Rigid Body Rotation Visualizer 🌐

## 📌 Problem Description
Build an interactive script/program to visualize rigid body rotations in 3D space. The tool must display:
- **Two Coordinate Frames:** One fixed reference frame (World/Space Frame $\{s\}$) and one rotatable body frame ($\{b\}$) controlled in real time using interactive sliders.
- **Real-Time Rotation Matrix:** Live mathematical readout of the $3 \times 3$ rotation matrix $R \in SO(3)$.
- **Interactive Controls:** Sliders for Euler angles (Roll $\phi$, Pitch $\theta$, Yaw $\psi$) with extrinsic (fixed-frame) and intrinsic (current-frame) rotation sequence toggles.

---

## 🚀 How to Run

Simply open either HTML visualizer directly in any modern web browser (no local server or installation required):

- **[`rigid_body_rotation_visualizer.html`](rigid_body_rotation_visualizer.html)** — Standard 3D visualizer with interactive sliders and live matrix computation.
- **[`rigid_body_rotation_visualizer_1.html`](rigid_body_rotation_visualizer_1.html)** — Enhanced interactive version with updated visual styling and axis tracers.

---

## 🧮 Mathematical Background

The rotation matrix $R$ is computed from the elemental rotation matrices:

$$R_x(\phi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}, \quad
R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}, \quad
R_z(\psi) = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

- **Extrinsic Composition (Fixed Frame):** $R = R_z(\psi) R_y(\theta) R_x(\phi)$ (pre-multiplication)
- **Intrinsic Composition (Current Frame):** $R = R_{x'}(\phi) R_{y''}(\theta) R_{z'''}(\psi)$ (post-multiplication)

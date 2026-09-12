import os
import sys
import time
import numpy as np

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print("[ERROR] MuJoCo is not installed in your active environment.")
    print("Run: pip install -r requirements.txt")
    sys.exit(1)

from utils import quat2mat, mat2euler, format_rotation_matrix_hud

# Model path resolution
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, "..", "model", "turtlebot3_waffle_pi.xml")

# Differential Drive Geometry
WHEEL_RADIUS = 0.033     # r = 33 mm
TRACK_WIDTH = 0.288      # L = 288 mm (distance between left & right wheels)

# Teleoperation state
linear_vel = 0.0         # v in m/s
angular_vel = 0.0        # omega in rad/s
MAX_LIN_VEL = 0.5        # max forward/backward speed
MAX_ANG_VEL = 2.0        # max turning speed
VEL_STEP_LIN = 0.05
VEL_STEP_ANG = 0.2

def print_controls():
    print("""
============================================================
  TURTLEBOT3 WAFFLE PI — KEYBOARD TELEOPERATION & SO(3) HUD
============================================================
  [W]        : Increase linear speed (Forward)
  [S]        : Decrease linear speed (Backward)
  [A]        : Turn Left (Positive Yaw rate)
  [D]        : Turn Right (Negative Yaw rate)
  [SPACE]    : Emergency Stop / Zero Velocity
  [R]        : Reset Robot to Origin Pose
  [Q / ESC]  : Quit Simulation
------------------------------------------------------------
  Visuals:
    - Faint axes at (0,0,0): Fixed Space Reference Frame {s}
    - Bright RGB axes on robot: Moving Body Frame {b}
      * RED   = +X (Forward)
      * GREEN = +Y (Left)
      * BLUE  = +Z (Upward)
============================================================
""")

def key_callback(keycode):
    global linear_vel, angular_vel
    # GLFW keycodes
    if keycode in [ord('W'), ord('w'), 265]:      # W or UP arrow
        linear_vel = min(linear_vel + VEL_STEP_LIN, MAX_LIN_VEL)
    elif keycode in [ord('S'), ord('s'), 264]:    # S or DOWN arrow
        linear_vel = max(linear_vel - VEL_STEP_LIN, -MAX_LIN_VEL)
    elif keycode in [ord('A'), ord('a'), 263]:    # A or LEFT arrow
        angular_vel = min(angular_vel + VEL_STEP_ANG, MAX_ANG_VEL)
    elif keycode in [ord('D'), ord('d'), 262]:    # D or RIGHT arrow
        angular_vel = max(angular_vel - VEL_STEP_ANG, -MAX_ANG_VEL)
    elif keycode == 32:                            # SPACE
        linear_vel = 0.0
        angular_vel = 0.0
    elif keycode in [ord('R'), ord('r')]:         # R (Reset)
        linear_vel = 0.0
        angular_vel = 0.0

def main():
    print_controls()
    
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"Model file not found: {MODEL_PATH}")

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    last_hud_time = 0.0
    hud_interval = 0.2  # Print matrix 5 times per second

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # Differential Drive Forward Kinematics:
            # v_l = (v - omega * L / 2) / r
            # v_r = (v + omega * L / 2) / r
            w_left = (linear_vel - (angular_vel * TRACK_WIDTH / 2.0)) / WHEEL_RADIUS
            w_right = (linear_vel + (angular_vel * TRACK_WIDTH / 2.0)) / WHEEL_RADIUS

            data.ctrl[0] = w_left
            data.ctrl[1] = w_right

            mujoco.mj_step(model, data)
            viewer.sync()

            # Extract Body Pose & SO(3) Rotation Matrix
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "turtlebot")
            pos = data.xpos[body_id]
            quat = data.xquat[body_id]  # [w, x, y, z]
            R = quat2mat(quat)
            rpy = mat2euler(R)
            rpy_deg = np.degrees(rpy)

            # Display real-time rotation matrix in terminal HUD
            cur_time = time.time()
            if cur_time - last_hud_time >= hud_interval:
                hud_text = format_rotation_matrix_hud(R, pos, rpy_deg, robot_name="TurtleBot3 Waffle Pi")
                print(hud_text)
                last_hud_time = cur_time

            # Maintain physics rate
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()

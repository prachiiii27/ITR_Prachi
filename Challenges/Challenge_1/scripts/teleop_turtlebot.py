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

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, "..", "model", "turtlebot3_waffle_pi.xml")

# Differential Drive Geometry
WHEEL_RADIUS = 0.033     # r = 33 mm
TRACK_WIDTH = 0.288      # L = 288 mm

# Teleop velocity commands
linear_vel = 0.0         # m/s
angular_vel = 0.0        # rad/s
MAX_LIN_VEL = 0.8        # max forward/backward speed
MAX_ANG_VEL = 2.5        # max yaw angular rate
VEL_STEP_LIN = 0.10
VEL_STEP_ANG = 0.35

def print_controls():
    print("""
===================================================================
  TURTLEBOT3 WAFFLE PI — KEYBOARD TELEOPERATION & SO(3) MATRIX
===================================================================
  1. Click ONCE inside the MuJoCo viewer window to focus it.
  2. Press keys on your keyboard:
     [W] / [Up Arrow]    : Drive Forward
     [S] / [Down Arrow]  : Drive Backward / Reverse
     [A] / [Left Arrow]  : Steer Left (Rotate +Yaw)
     [D] / [Right Arrow] : Steer Right (Rotate -Yaw)
     [X] or [B]          : Stop / Brake (Zero velocity)
     [R]                 : Reset Robot to Origin
     [Space]             : MuJoCo Pause / Unpause (ensure UNPAUSED!)
  3. Watch the moving RGB frame on the robot and the live matrix below!
===================================================================
""")

def key_callback(keycode):
    global linear_vel, angular_vel
    # Handle GLFW keycodes (both lowercase and uppercase + arrow keys)
    # W (87 / 119) or UP (265)
    if keycode in [ord('W'), ord('w'), 265]:
        linear_vel = min(linear_vel + VEL_STEP_LIN, MAX_LIN_VEL)
        print(f">> [FORWARD]  Linear={linear_vel:+.2f} m/s, Angular={angular_vel:+.2f} rad/s")
    # S (83 / 115) or DOWN (264)
    elif keycode in [ord('S'), ord('s'), 264]:
        linear_vel = max(linear_vel - VEL_STEP_LIN, -MAX_LIN_VEL)
        print(f">> [BACKWARD] Linear={linear_vel:+.2f} m/s, Angular={angular_vel:+.2f} rad/s")
    # A (65 / 97) or LEFT (263)
    elif keycode in [ord('A'), ord('a'), 263]:
        angular_vel = min(angular_vel + VEL_STEP_ANG, MAX_ANG_VEL)
        print(f">> [TURN LEFT]  Linear={linear_vel:+.2f} m/s, Angular={angular_vel:+.2f} rad/s")
    # D (68 / 100) or RIGHT (262)
    elif keycode in [ord('D'), ord('d'), 262]:
        angular_vel = max(angular_vel - VEL_STEP_ANG, -MAX_ANG_VEL)
        print(f">> [TURN RIGHT] Linear={linear_vel:+.2f} m/s, Angular={angular_vel:+.2f} rad/s")
    # X / B / Space (Brake)
    elif keycode in [ord('X'), ord('x'), ord('B'), ord('b')]:
        linear_vel = 0.0
        angular_vel = 0.0
        print(">> [BRAKE / STOP]")
    # R (Reset)
    elif keycode in [ord('R'), ord('r')]:
        linear_vel = 0.0
        angular_vel = 0.0
        print(">> [RESET TO ORIGIN]")

def main():
    global linear_vel, angular_vel
    print_controls()
    
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"Model file not found: {MODEL_PATH}")

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    last_hud_time = 0.0
    hud_interval = 0.25

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # Differential Drive Kinematic Mapping:
            # omega_L = (v - omega * L / 2) / r
            # omega_R = (v + omega * L / 2) / r
            w_left = (linear_vel - (angular_vel * TRACK_WIDTH / 2.0)) / WHEEL_RADIUS
            w_right = (linear_vel + (angular_vel * TRACK_WIDTH / 2.0)) / WHEEL_RADIUS

            data.ctrl[0] = w_left
            data.ctrl[1] = w_right

            mujoco.mj_step(model, data)
            viewer.sync()

            # Extract Robot Body Pose & SO(3) Rotation Matrix
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "turtlebot")
            pos = data.xpos[body_id]
            quat = data.xquat[body_id]  # [w, x, y, z]
            R = quat2mat(quat)
            rpy = mat2euler(R)
            rpy_deg = np.degrees(rpy)

            # Display real-time rotation matrix HUD in console
            cur_time = time.time()
            if cur_time - last_hud_time >= hud_interval:
                hud_text = format_rotation_matrix_hud(R, pos, rpy_deg, robot_name="TurtleBot3 Waffle Pi")
                print(hud_text)
                last_hud_time = cur_time

            # Step rate sync
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()

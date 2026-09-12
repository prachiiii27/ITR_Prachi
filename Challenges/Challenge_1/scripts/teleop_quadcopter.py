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
MODEL_PATH = os.path.join(SCRIPT_DIR, "..", "model", "quadcopter.xml")

# Flight command state
target_altitude = 1.0     # z target in meters
target_roll = 0.0         # phi in radians
target_pitch = 0.0        # theta in radians
target_yaw_rate = 0.0     # psi_dot in rad/s

DRONE_MASS = 1.35         # kg (fuselage + battery + 4 motors + avionics)
GRAVITY = 9.81            # m/s^2
HOVER_THRUST = DRONE_MASS * GRAVITY

def print_controls():
    print("""
============================================================
  QUADCOPTER 6-DOF DRONE — KEYBOARD FLIGHT & SO(3) HUD
============================================================
  [W] / [S]   : Pitch Forward (Tilt +X) / Pitch Backward (Tilt -X)
  [A] / [D]   : Roll Left (Tilt +Y)     / Roll Right (Tilt -Y)
  [I] / [K]   : Climb Altitude (+Z)     / Descend Altitude (-Z)
  [J] / [L]   : Yaw Rotate Left         / Yaw Rotate Right
  [SPACE]     : Level Drone / Stabilize Hover at Current Altitude
  [R]         : Reset Drone to Initial Takeoff Pose (x=0, y=0, z=1.0)
  [Q / ESC]   : Quit Simulation
------------------------------------------------------------
  Visuals:
    - Faint axes at (0,0,0): Fixed World Reference Frame {s}
    - Bright RGB axes on Quadcopter: Moving Body Frame {b}
      * RED   = +X (Forward arm)
      * GREEN = +Y (Left arm)
      * BLUE  = +Z (Thrust axis)
============================================================
""")

def key_callback(keycode):
    global target_altitude, target_roll, target_pitch, target_yaw_rate
    ANGLE_STEP = np.radians(3.0)
    MAX_TILT = np.radians(25.0)
    
    if keycode in [ord('W'), ord('w')]:          # Pitch Forward
        target_pitch = max(target_pitch - ANGLE_STEP, -MAX_TILT)
    elif keycode in [ord('S'), ord('s')]:        # Pitch Backward
        target_pitch = min(target_pitch + ANGLE_STEP, MAX_TILT)
    elif keycode in [ord('A'), ord('a')]:        # Roll Left
        target_roll = max(target_roll - ANGLE_STEP, -MAX_TILT)
    elif keycode in [ord('D'), ord('d')]:        # Roll Right
        target_roll = min(target_roll + ANGLE_STEP, MAX_TILT)
    elif keycode in [ord('I'), ord('i'), 265]:   # Climb Up
        target_altitude = min(target_altitude + 0.1, 5.0)
    elif keycode in [ord('K'), ord('k'), 264]:   # Descend Down
        target_altitude = max(target_altitude - 0.1, 0.1)
    elif keycode in [ord('J'), ord('j'), 263]:   # Yaw Left
        target_yaw_rate = 0.5
    elif keycode in [ord('L'), ord('l'), 262]:   # Yaw Right
        target_yaw_rate = -0.5
    elif keycode == 32:                           # SPACE (Hover / Level)
        target_roll = 0.0
        target_pitch = 0.0
        target_yaw_rate = 0.0
    elif keycode in [ord('R'), ord('r')]:        # R (Reset)
        target_altitude = 1.0
        target_roll = 0.0
        target_pitch = 0.0
        target_yaw_rate = 0.0

def main():
    global target_altitude, target_roll, target_pitch, target_yaw_rate
    print_controls()
    
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"Model file not found: {MODEL_PATH}")

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadcopter")

    # PD Controller Gains for Attitude & Altitude Stabilization
    Kp_z = 25.0;  Kd_z = 12.0
    Kp_att = 15.0; Kd_att = 3.5
    Kp_yaw = 8.0;  Kd_yaw = 2.0

    last_hud_time = 0.0
    hud_interval = 0.2

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # Current State
            pos = data.xpos[body_id]
            quat = data.xquat[body_id]
            R = quat2mat(quat)
            rpy = mat2euler(R)
            lin_vel = data.qvel[0:3]
            ang_vel = data.qvel[3:6]

            # Altitude Control (Thrust along world Z projected onto body Z)
            z_err = target_altitude - pos[2]
            thrust_z = HOVER_THRUST + Kp_z * z_err - Kd_z * lin_vel[2]
            thrust_z = np.clip(thrust_z, 0.0, 2.5 * HOVER_THRUST)

            # Attitude Torques (Roll, Pitch, Yaw)
            tau_x = Kp_att * (target_roll - rpy[0]) - Kd_att * ang_vel[0]
            tau_y = Kp_att * (target_pitch - rpy[1]) - Kd_att * ang_vel[1]
            tau_z = Kp_yaw * target_yaw_rate - Kd_yaw * ang_vel[2]

            # Apply generalized forces to freejoint
            # Forces in body frame mapped to spatial coordinates: F_space = R @ [0, 0, thrust_z]
            f_world = R @ np.array([0.0, 0.0, thrust_z])
            t_world = R @ np.array([tau_x, tau_y, tau_z])

            data.xfrc_applied[body_id, 0:3] = f_world
            data.xfrc_applied[body_id, 3:6] = t_world

            mujoco.mj_step(model, data)
            viewer.sync()

            # Periodic HUD Update in Terminal
            cur_time = time.time()
            if cur_time - last_hud_time >= hud_interval:
                rpy_deg = np.degrees(rpy)
                hud_text = format_rotation_matrix_hud(R, pos, rpy_deg, robot_name="6-DOF Quadcopter Drone")
                print(hud_text)
                last_hud_time = cur_time

            # Decay yaw command if key released
            target_yaw_rate *= 0.95

            # Maintain physics rate
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()

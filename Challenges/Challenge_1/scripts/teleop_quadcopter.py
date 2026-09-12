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

target_altitude = 1.0     # z in meters
target_roll = 0.0         # phi in rad
target_pitch = 0.0        # theta in rad
target_yaw_rate = 0.0     # psi_dot in rad/s

DRONE_MASS = 1.35         # kg
GRAVITY = 9.81
HOVER_THRUST = DRONE_MASS * GRAVITY

def print_controls():
    print("""
===================================================================
  6-DOF QUADCOPTER DRONE — KEYBOARD FLIGHT & SO(3) ROTATION MATRIX
===================================================================
  1. Click ONCE inside the MuJoCo viewer window to focus it.
  2. Press flight keys:
     [W] / [S]           : Pitch Forward (Tilt +X) / Pitch Backward
     [A] / [D]           : Roll Left (Tilt +Y) / Roll Right
     [I] / [K] (or ↑ / ↓): Climb Up (+Z) / Descend Down (-Z)
     [J] / [L] (or ← / →): Yaw Turn Left / Yaw Turn Right
     [X] or [H]          : Level Drone & Hover at Current Height
     [R]                 : Reset to Initial Takeoff Pose
     [Space]             : MuJoCo Pause / Unpause
  3. Watch the 3D RGB body frame tilt dynamically & live matrix below!
===================================================================
""")

def key_callback(keycode):
    global target_altitude, target_roll, target_pitch, target_yaw_rate
    ANGLE_STEP = np.radians(4.0)
    MAX_TILT = np.radians(25.0)

    # Pitch W (87) / S (83)
    if keycode in [ord('W'), ord('w')]:
        target_pitch = max(target_pitch - ANGLE_STEP, -MAX_TILT)
        print(f">> [PITCH FWD] Pitch={np.degrees(target_pitch):+.1f}°")
    elif keycode in [ord('S'), ord('s')]:
        target_pitch = min(target_pitch + ANGLE_STEP, MAX_TILT)
        print(f">> [PITCH BACK] Pitch={np.degrees(target_pitch):+.1f}°")
    # Roll A (65) / D (68)
    elif keycode in [ord('A'), ord('a')]:
        target_roll = max(target_roll - ANGLE_STEP, -MAX_TILT)
        print(f">> [ROLL LEFT] Roll={np.degrees(target_roll):+.1f}°")
    elif keycode in [ord('D'), ord('d')]:
        target_roll = min(target_roll + ANGLE_STEP, MAX_TILT)
        print(f">> [ROLL RIGHT] Roll={np.degrees(target_roll):+.1f}°")
    # Altitude I (73) / K (75) / Up (265) / Down (264)
    elif keycode in [ord('I'), ord('i'), 265]:
        target_altitude = min(target_altitude + 0.15, 6.0)
        print(f">> [CLIMB] Alt={target_altitude:+.2f} m")
    elif keycode in [ord('K'), ord('k'), 264]:
        target_altitude = max(target_altitude - 0.15, 0.1)
        print(f">> [DESCEND] Alt={target_altitude:+.2f} m")
    # Yaw J (74) / L (76) / Left (263) / Right (262)
    elif keycode in [ord('J'), ord('j'), 263]:
        target_yaw_rate = 0.6
        print(">> [YAW LEFT]")
    elif keycode in [ord('L'), ord('l'), 262]:
        target_yaw_rate = -0.6
        print(">> [YAW RIGHT]")
    # Level / Hover X / H
    elif keycode in [ord('X'), ord('x'), ord('H'), ord('h')]:
        target_roll = 0.0
        target_pitch = 0.0
        target_yaw_rate = 0.0
        print(">> [LEVEL HOVER]")
    # Reset R
    elif keycode in [ord('R'), ord('r')]:
        target_altitude = 1.0
        target_roll = 0.0
        target_pitch = 0.0
        target_yaw_rate = 0.0
        print(">> [RESET POSE]")

def main():
    global target_altitude, target_roll, target_pitch, target_yaw_rate
    print_controls()

    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"Model file not found: {MODEL_PATH}")

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadcopter")

    Kp_z = 30.0;  Kd_z = 14.0
    Kp_att = 18.0; Kd_att = 4.0
    Kp_yaw = 10.0; Kd_yaw = 2.5

    last_hud_time = 0.0
    hud_interval = 0.25

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            step_start = time.time()

            pos = data.xpos[body_id]
            quat = data.xquat[body_id]
            R = quat2mat(quat)
            rpy = mat2euler(R)
            lin_vel = data.qvel[0:3]
            ang_vel = data.qvel[3:6]

            # Altitude PID
            z_err = target_altitude - pos[2]
            thrust_z = HOVER_THRUST + Kp_z * z_err - Kd_z * lin_vel[2]
            thrust_z = np.clip(thrust_z, 0.0, 3.0 * HOVER_THRUST)

            # Attitude Stabilization Torques
            tau_x = Kp_att * (target_roll - rpy[0]) - Kd_att * ang_vel[0]
            tau_y = Kp_att * (target_pitch - rpy[1]) - Kd_att * ang_vel[1]
            tau_z = Kp_yaw * target_yaw_rate - Kd_yaw * ang_vel[2]

            f_world = R @ np.array([0.0, 0.0, thrust_z])
            t_world = R @ np.array([tau_x, tau_y, tau_z])

            data.xfrc_applied[body_id, 0:3] = f_world
            data.xfrc_applied[body_id, 3:6] = t_world

            mujoco.mj_step(model, data)
            viewer.sync()

            cur_time = time.time()
            if cur_time - last_hud_time >= hud_interval:
                rpy_deg = np.degrees(rpy)
                hud_text = format_rotation_matrix_hud(R, pos, rpy_deg, robot_name="6-DOF Quadcopter Drone")
                print(hud_text)
                last_hud_time = cur_time

            target_yaw_rate *= 0.96

            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()

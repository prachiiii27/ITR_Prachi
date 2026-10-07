"""
Task 2: Franka Emika Panda Robot MuJoCo Model Simulation
Loads Franka Panda XML model description, implements 7-DOF PD joint control + gripper actuation,
RNE gravity compensation, forward kinematics tracking, and interactive viewer.
"""

import os
import sys
import time
import traceback
import numpy as np
import mujoco

from fk_utils import PoEForwardKinematics, gravity_torque

# Optional viewer import with headless fallback
try:
    import mujoco.viewer
    HAS_VIEWER = True
except ImportError:
    HAS_VIEWER = False


# Home pose (from the Menagerie panda keyframe). All zeros is NOT valid for the
# Panda: joint 4 must stay between -3.07 and -0.07 rad.
HOME = np.array([0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, -0.7853])

# Dynamic Target State (7 arm joints + 1 gripper state)
q_target = HOME.copy()
gripper_target = 0.04  # finger opening in metres: 0.04 = fully open, 0 = closed
STEP = np.deg2rad(2.0)
GRIPPER_STEP = 0.005

# Franka Joint Limits
LOWER_ARM = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
UPPER_ARM = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])


def key_callback(key):
    global q_target, gripper_target

    # Q / A : Joint 1
    if key == ord('Q'):
        q_target[0] += STEP
    elif key == ord('A'):
        q_target[0] -= STEP
    # W / S : Joint 2
    elif key == ord('W'):
        q_target[1] += STEP
    elif key == ord('S'):
        q_target[1] -= STEP
    # E / D : Joint 3
    elif key == ord('E'):
        q_target[2] += STEP
    elif key == ord('D'):
        q_target[2] -= STEP
    # R / F : Joint 4
    elif key == ord('R'):
        q_target[3] += STEP
    elif key == ord('F'):
        q_target[3] -= STEP
    # T / G : Joint 5
    elif key == ord('T'):
        q_target[4] += STEP
    elif key == ord('G'):
        q_target[4] -= STEP
    # Y / H : Joint 6
    elif key == ord('Y'):
        q_target[5] += STEP
    elif key == ord('H'):
        q_target[5] -= STEP
    # U / J : Joint 7
    elif key == ord('U'):
        q_target[6] += STEP
    elif key == ord('J'):
        q_target[6] -= STEP
    # O / P : Gripper Open / Close
    elif key == ord('O'):
        gripper_target = min(0.04, gripper_target + GRIPPER_STEP)
    elif key == ord('P'):
        gripper_target = max(0.00, gripper_target - GRIPPER_STEP)
    # SPACE : Reset all
    elif key == 32:
        q_target[:] = HOME
        gripper_target = 0.04

    q_target[:] = np.clip(q_target, LOWER_ARM, UPPER_ARM)
    print("Target angles (deg):", np.round(np.rad2deg(q_target), 1), f"Gripper: {gripper_target:.3f}m")


def main():
    global q_target, gripper_target

    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(script_dir)
    xml_path = os.path.join(repo_root, "robot_descriptions", "franka", "panda.xml")

    if not os.path.exists(xml_path):
        alt_path = os.path.join(repo_root, "robot_descriptions", "franka_scene.xml")
        if os.path.exists(alt_path):
            xml_path = alt_path
        else:
            raise FileNotFoundError(f"Franka XML file not found at: {xml_path}")

    print(f"\nLoading Franka Emika Panda XML: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # The Menagerie panda.xml ships actuators 1-7 as POSITION servos (ctrl = target angle).
    # This lab does PD torque control, so switch them to plain torque motors here:
    #   force = gain * ctrl + bias   ->   gain = 1, bias = 0   =>   force = ctrl (N*m)
    # and limit ctrl to the real Panda torque limits (87 N*m joints 1-4, 12 N*m joints 5-7).
    for i in range(7):
        model.actuator_gaintype[i] = mujoco.mjtGain.mjGAIN_FIXED
        model.actuator_gainprm[i, :] = 0.0
        model.actuator_gainprm[i, 0] = 1.0
        model.actuator_biastype[i] = mujoco.mjtBias.mjBIAS_NONE
        model.actuator_biasprm[i, :] = 0.0
        model.actuator_ctrllimited[i] = 1
        model.actuator_ctrlrange[i] = model.actuator_forcerange[i]
    # Actuator 8 (gripper) stays a position servo; its ctrl runs 0 (closed) .. 255 (open).

    print("\n==========================================")
    print("       TASK 2: FRANKA PANDA ROBOT MODEL")
    print("==========================================")
    print(f"DOF       : {model.nv}")
    print(f"Joints    : {model.njnt}")
    print(f"Actuators : {model.nu}")

    print("\n---------- JOINTS ----------")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        print(f" Joint {i+1}: {name}")

    print("\n---------- ACTUATORS ----------")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f" Actuator {i+1}: {name}")

    print("\n------------------------------------------")
    print("KEYBOARD CONTROLS (Passive Viewer)")
    print("------------------------------------------")
    print(" Q / A  -> Joint 1 + / -")
    print(" W / S  -> Joint 2 + / -")
    print(" E / D  -> Joint 3 + / -")
    print(" R / F  -> Joint 4 + / -")
    print(" T / G  -> Joint 5 + / -")
    print(" Y / H  -> Joint 6 + / -")
    print(" U / J  -> Joint 7 + / -")
    print(" O / P  -> Gripper Open / Close")
    print(" SPACE  -> Reset to home pose, gripper open")
    print("------------------------------------------\n")

    # Gains for PD controller
    kp = np.array([600, 600, 600, 600, 250, 150, 50])
    kd = np.array([ 50,  50,  50,  50,  30,  20, 10])

    # End-effector: the "attachment_site" if the model has one, otherwise the "hand" body
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site") >= 0:
        fk = PoEForwardKinematics(model, "attachment_site", ee_is_site=True)
    else:
        fk = PoEForwardKinematics(model, "hand", ee_is_site=False)

    data.qpos[:7] = HOME
    data.qpos[7:9] = 0.04          # fingers open
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)

    def control():
        # tau = Kp (q_ref - q) - Kd qdot + g(q)   for the 7 arm joints
        tau = kp * (q_target - data.qpos[:7]) - kd * data.qvel[:7] + gravity_torque(model, data)[:7]
        data.ctrl[:7] = np.clip(tau, model.actuator_ctrlrange[:7, 0], model.actuator_ctrlrange[:7, 1])
        # Gripper: metres (0..0.04) -> actuator units (0..255)
        if model.nu > 7:
            data.ctrl[7] = gripper_target / 0.04 * 255.0

    try:
        with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -20
            viewer.cam.distance = 2.5
            viewer.cam.lookat[:] = [0, 0, 0.5]

            last_print = time.time()

            while viewer.is_running():
                step_start = time.time()
                control()

                mujoco.mj_step(model, data)
                viewer.sync()

                now = time.time()
                if now - last_print > 0.5:
                    joint_deg = np.rad2deg(data.qpos[:7])
                    print(f"Joints (deg): {np.round(joint_deg, 1)}")

                    print(f"Finger position: {data.qpos[7]:.3f} m each (target {gripper_target:.3f} m; 0.04 = open, 0 = closed)")
                    print(fk.report(data) + "\n")
                    last_print = now

                time.sleep(max(0.0, model.opt.timestep - (time.time() - step_start)))

    except Exception:
        traceback.print_exc()
        print("Viewer unavailable - running a short headless check instead...")
        for _ in range(500):
            control()
            mujoco.mj_step(model, data)
        print(fk.report(data))


if __name__ == "__main__":
    main()

"""
Task 2: HEAL Robot MuJoCo Model Simulation
Loads HEAL Robot XML description model dynamically, applies PD control with
RNE gravity compensation,
and provides interactive viewer with end-effector tracking.
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


# Dynamic Target State for HEAL Joints
q_target = None
STEP = np.deg2rad(2.0)


def key_callback(key):
    global q_target
    if q_target is None:
        return

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
    # SPACE : Reset all joints
    elif key == 32:
        q_target[:] = 0

    print("Target angles (deg):", np.round(np.rad2deg(q_target), 2))


def main():
    global q_target

    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(script_dir)
    xml_filename = "single_arm_heal_effort_actuation_rs_mj_2.xml"
    xml_path = os.path.join(repo_root, "robot_descriptions", xml_filename)

    if not os.path.exists(xml_path):
        # Fallback search if path is different
        alt_path = os.path.join(repo_root, "robot_descriptions", "single_arm_heal_effort_actuation_rs.xml")
        if os.path.exists(alt_path):
            xml_path = alt_path
        else:
            raise FileNotFoundError(f"HEAL XML file not found at: {xml_path}")

    print(f"\nLoading HEAL Robot XML: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    num_joints = model.njnt
    num_actuators = model.nu
    q_target = np.zeros(model.nv)

    print("\n==========================================")
    print("         TASK 2: HEAL ROBOT MODEL")
    print("==========================================")
    print(f"DOF       : {model.nv}")
    print(f"Joints    : {num_joints}")
    print(f"Actuators : {num_actuators}")

    print("\n---------- JOINTS ----------")
    for i in range(num_joints):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        print(f" Joint {i+1}: {name}")

    print("\n---------- ACTUATORS ----------")
    for i in range(num_actuators):
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
    print(" SPACE  -> Reset joints")
    print("------------------------------------------\n")

    # Gains for PD controller (one per actuated joint; HEAL has 6 motors on joints 1-6)
    n = num_actuators
    kp = np.array([100, 100, 100, 70, 50, 30], dtype=float)[:n]
    kd = np.array([ 10,  10,  10,  7,  5,  3], dtype=float)[:n]

    # End effector site lookup + our own FK for the same site
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "right_center")
    fk = PoEForwardKinematics(model, "right_center", ee_is_site=True)

    def control():
        # tau = Kp (q_ref - q) - Kd qdot + g(q)
        # Without the g(q) term the shoulder sagged ~20 deg below its target under gravity.
        tau = kp * (q_target[:n] - data.qpos[:n]) - kd * data.qvel[:n] + gravity_torque(model, data)[:n]
        lo, hi = model.actuator_ctrlrange[:n, 0], model.actuator_ctrlrange[:n, 1]
        limited = model.actuator_ctrllimited[:n].astype(bool)
        tau[limited] = np.clip(tau[limited], lo[limited], hi[limited])
        data.ctrl[:n] = tau

    data.qpos[:] = 0
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)

    try:
        with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
            last_print = time.time()

            while viewer.is_running():
                step_start = time.time()
                control()

                mujoco.mj_step(model, data)
                viewer.sync()

                now = time.time()
                if now - last_print > 0.5:
                    joint_deg = np.rad2deg(data.qpos[:num_joints])
                    print(f"Joints (deg): {np.round(joint_deg, 2)}")
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

"""
Task 1: 6-DOF Industrial Manipulator Simulation
MuJoCo Forward Kinematics, PD Torque Control with RNE Gravity Compensation,
and Interactive Keyboard Controls.
"""

import os
import sys
import time
import numpy as np
import mujoco

# Optional viewer import with headless fallback
try:
    import mujoco.viewer
    HAS_VIEWER = True
except ImportError:
    HAS_VIEWER = False


# ============================================================
# 6-DOF INDUSTRIAL MANIPULATOR XML MODEL
# ============================================================

XML = r"""
<mujoco model="HEAL_6DOF">

    <compiler angle="radian" coordinate="local"/>

    <option
        timestep="0.002"
        gravity="0 0 -9.81"
        integrator="implicitfast"
        iterations="100"
        tolerance="1e-8"/>

    <size
        njmax="1000"
        nconmax="500"/>

    <visual>
        <global offwidth="1920" offheight="1080"/>
        <quality shadowsize="2048"/>
        <headlight ambient="0.4 0.4 0.4" diffuse="0.7 0.7 0.7" specular="0.5 0.5 0.5"/>
    </visual>

    <asset>
        <material name="body" rgba="0.16 0.18 0.21 1" metallic="0.65" roughness="0.3"/>
        <material name="body_light" rgba="0.55 0.58 0.62 1" metallic="0.65" roughness="0.25"/>
        <material name="joint" rgba="0.035 0.04 0.05 1" metallic="0.85" roughness="0.2"/>
        <material name="ee" rgba="0.9 0.12 0.04 1" metallic="0.35" roughness="0.25"/>
        <material name="floor" rgba="0.2 0.21 0.23 1" roughness="0.75"/>
    </asset>

    <default>
        <joint limited="true" damping="12" armature="0.15"/>
        <geom density="350" friction="0.8 0.1 0.1" condim="3"/>
    </default>

    <worldbody>
        <geom name="floor" type="plane" size="5 5 0.1" material="floor"/>
        <light name="main_light" pos="2 -3 5" dir="-0.3 0.4 -1" diffuse="1 1 1" specular="0.8 0.8 0.8"/>
        <light name="fill_light" pos="-3 -2 3" dir="0.5 0.3 -0.8" diffuse="0.45 0.45 0.45"/>

        <body name="base" pos="0 0 0.18">
            <geom type="cylinder" size="0.32 0.18" material="body"/>
            <geom type="cylinder" size="0.23 0.20" pos="0 0 0.18" material="body_light"/>

            <body name="link1" pos="0 0 0.38">
                <joint name="joint1" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
                <geom type="cylinder" size="0.20 0.16" material="joint"/>
                <geom type="box" size="0.20 0.20 0.28" pos="0 0 0.28" material="body"/>

                <body name="link2" pos="0 0 0.56">
                    <joint name="joint2" type="hinge" axis="0 1 0" range="-1.4 1.4"/>
                    <geom type="cylinder" size="0.18 0.13" quat="0.7071 0 0.7071 0" material="joint"/>
                    <geom type="capsule" fromto="0 0 0 0 0 0.65" size="0.12" material="body_light"/>

                    <body name="link3" pos="0 0 0.65">
                        <joint name="joint3" type="hinge" axis="0 1 0" range="-2.0 2.0"/>
                        <geom type="cylinder" size="0.16 0.12" quat="0.7071 0 0.7071 0" material="joint"/>
                        <geom type="capsule" fromto="0 0 0 0 0 0.60" size="0.105" material="body_light"/>

                        <body name="link4" pos="0 0 0.60">
                            <joint name="joint4" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
                            <geom type="cylinder" size="0.13 0.10" material="joint"/>
                            <geom type="capsule" fromto="0 0 0 0 0 0.25" size="0.075" material="body_light"/>

                            <body name="link5" pos="0 0 0.25">
                                <joint name="joint5" type="hinge" axis="0 1 0" range="-1.5 1.5"/>
                                <geom type="cylinder" size="0.105 0.08" quat="0.7071 0 0.7071 0" material="joint"/>
                                <geom type="capsule" fromto="0 0 0 0 0 0.18" size="0.06" material="body_light"/>

                                <body name="link6" pos="0 0 0.18">
                                    <joint name="joint6" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
                                    <geom type="cylinder" size="0.09 0.07" material="joint"/>
                                    <geom type="cylinder" size="0.065 0.10" pos="0 0 0.10" material="ee"/>
                                    <geom type="sphere" size="0.085" pos="0 0 0.22" material="ee"/>
                                    <site name="EE" pos="0 0 0.28" size="0.04" rgba="1 0 0 1"/>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </body>
    </worldbody>

    <actuator>
        <position name="motor1" joint="joint1" kp="500" kv="60" forcerange="-500 500"/>
        <position name="motor2" joint="joint2" kp="700" kv="80" forcerange="-800 800"/>
        <position name="motor3" joint="joint3" kp="600" kv="70" forcerange="-700 700"/>
        <position name="motor4" joint="joint4" kp="400" kv="50" forcerange="-500 500"/>
        <position name="motor5" joint="joint5" kp="350" kv="45" forcerange="-400 400"/>
        <position name="motor6" joint="joint6" kp="250" kv="35" forcerange="-300 300"/>
    </actuator>
</mujoco>
"""

# Dynamic Global Target State
q_target = np.zeros(6)
STEP = np.deg2rad(2.0)

LOWER = np.array([-3.14, -1.4, -2.0, -3.14, -1.5, -3.14])
UPPER = np.array([ 3.14,  1.4,  2.0,  3.14,  1.5,  3.14])


def key_callback(key):
    global q_target

    if key == ord("Q"):
        q_target[0] += STEP
    elif key == ord("A"):
        q_target[0] -= STEP
    elif key == ord("W"):
        q_target[1] += STEP
    elif key == ord("S"):
        q_target[1] -= STEP
    elif key == ord("E"):
        q_target[2] += STEP
    elif key == ord("D"):
        q_target[2] -= STEP
    elif key == ord("R"):
        q_target[3] += STEP
    elif key == ord("F"):
        q_target[3] -= STEP
    elif key == ord("T"):
        q_target[4] += STEP
    elif key == ord("G"):
        q_target[4] -= STEP
    elif key == ord("Y"):
        q_target[5] += STEP
    elif key == ord("H"):
        q_target[5] -= STEP
    elif key == 32:  # SPACE bar resets joints
        q_target[:] = 0

    q_target[:] = np.clip(q_target, LOWER, UPPER)
    print("Target angles (deg):", np.round(np.rad2deg(q_target), 1))


def main():
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)

    print("\n==========================================")
    print("       TASK 1: 6-DOF MANIPULATOR SIMULATION")
    print("==========================================")
    print(f"Joints    : {model.njnt}")
    print(f"Actuators : {model.nu}")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        print(f"  Joint {i+1}: {name}")

    print("\n------------------------------------------")
    print("KEYBOARD CONTROLS (Passive Viewer)")
    print("------------------------------------------")
    print("  Q / A  -> Joint 1  + / -")
    print("  W / S  -> Joint 2  + / -")
    print("  E / D  -> Joint 3  + / -")
    print("  R / F  -> Joint 4  + / -")
    print("  T / G  -> Joint 5  + / -")
    print("  Y / H  -> Joint 6  + / -")
    print("  SPACE  -> Reset all joints")
    print("------------------------------------------\n")

    ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "EE")

    data.qpos[:6] = 0
    data.qvel[:6] = 0
    mujoco.mj_forward(model, data)

    kp = np.array([500, 700, 600, 400, 350, 250])
    kd = np.array([ 60,  80,  70,  50,  45,  35])
    torque_limits = np.array([500, 800, 700, 500, 400, 300])

    try:
        with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -18
            viewer.cam.distance = 3.2
            viewer.cam.lookat[:] = [0, 0, 1.0]

            last_print = time.time()
            while viewer.is_running():
                # RNE Gravity compensation
                mujoco.mj_rne(model, data, 0, data.qfrc_bias)
                gravity_torque = data.qfrc_bias[:6].copy()

                # PD Torque controller
                for i in range(6):
                    pos_err = q_target[i] - data.qpos[i]
                    vel_err = -data.qvel[i]
                    t_val = kp[i] * pos_err + kd[i] * vel_err + gravity_torque[i]
                    t_val = np.clip(t_val, -torque_limits[i], torque_limits[i])
                    data.ctrl[i] = t_val

                mujoco.mj_step(model, data)
                viewer.sync()

                now = time.time()
                if now - last_print > 0.5:
                    joint_deg = np.rad2deg(data.qpos[:6])
                    ee_pos = data.site_xpos[ee_id]
                    print(f"Joints (deg): {np.round(joint_deg, 1)}")
                    print(f"EE Position : X = {ee_pos[0]:.3f}, Y = {ee_pos[1]:.3f}, Z = {ee_pos[2]:.3f}\n")
                    last_print = now

                time.sleep(0.001)

    except Exception as e:
        print(f"Interactive viewer closed or not supported in this display environment: {e}")
        print("Executing headless simulation step verification...")
        for _ in range(500):
            mujoco.mj_rne(model, data, 0, data.qfrc_bias)
            data.ctrl[:6] = data.qfrc_bias[:6]
            mujoco.mj_step(model, data)
        ee_pos = data.site_xpos[ee_id]
        print(f"Final Headless EE Position: X = {ee_pos[0]:.3f}, Y = {ee_pos[1]:.3f}, Z = {ee_pos[2]:.3f}")


if __name__ == "__main__":
    main()

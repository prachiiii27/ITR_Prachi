"""
config.py - every number the project uses lives here, in one place.

Robot: Addverb HEAL, a 6-DoF arm (same model files as Challenge 2).
The HEAL model has NO gripper, so scene.py adds a simple two-finger gripper.

Units: metres, radians, seconds.  World frame: robot base at the origin,
+x points forward (away from the robot), +z points up.
"""
import os
import numpy as np

# ---------------------------------------------------------------- robot model
HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_XML = os.path.join(HERE, "assets", "heal", "heal.xml")   # HEAL from Challenge 2
ARM_JOINTS = [f"joint_{i}" for i in range(1, 7)]   # the 6 arm joints we control
N_ARM = len(ARM_JOINTS)                            # 6 (the Panda had 7)
EE_BODY = "end_effector"        # last link of the HEAL; the gripper is bolted on it
FINGER_JOINTS = ("finger_left", "finger_right")   # slide joints of the added gripper
# body names that belong to the robot (used for contact checks)
ROBOT_BODY_PREFIX = ("link_", "end_effector", "right_hand", "gripper", "finger")
# bodies near the tool, checked against table/tray with distance queries
WRIST_BODIES = ("link_4", "link_5", "end_effector", "gripper_palm",
                "finger_left", "finger_right")

# Gripper geometry (local frame of end_effector: +z points OUT of the flange)
FLANGE_Z = 0.007                # flange face (measured from the end_effector mesh)
PALM_H = 0.030                  # palm block height
FINGER_LEN = 0.050              # finger pad length
TCP_OFFSET = FLANGE_Z + PALM_H + FINGER_LEN - 0.010   # = 0.077 m: 1 cm above tips
FINGER_OPEN = 0.045             # each finger slides 0..4.5 cm  (gap up to 9 cm)
GRIPPER_OPEN, GRIPPER_CLOSED = FINGER_OPEN, 0.0      # position-actuator targets (m)

# Home pose: TCP at (0.33, 0, 0.50) pointing straight down, 20 cm above the table (found once with IK
# starting from q = 0 - run "python kinematics.py").  joint values in rad.
Q_HOME = np.array([0.0, -0.12982, 0.08431, 0.40052, -0.45106, -0.04425])
YAW_HOME = 1.0605               # tool yaw at home (60.8 deg, from link_5's twist)
# HEAL datasheet speeds are not public; we assume 2 rad/s (~115 deg/s) per joint,
# typical for a 5 kg-class cobot.
V_MAX = np.full(N_ARM, 2.0)

# Position servos that replace the HEAL's torque motors
KP_ARM = 1500.0                 # N m / rad
ARMATURE = 0.05                 # motor rotor inertia, keeps the simulation stable

# --------------------------------------------- table (chosen by workspace.py)
# These values come from the workspace analysis (see results/table_design.md).
TABLE_H = 0.30                  # table top height above the robot base (H)
TABLE_X = (0.20, 0.45)          # length L = 0.25 m
TABLE_Y = (-0.35, 0.15)         # width  W = 0.50 m (not centred: HEAL is asymmetric)
TABLE_THICK = 0.03
TABLE_X_MIN = 0.20                # table must start this far in front of the base

# ------------------------------------------------------------- cube and tray
CUBE_HALF = 0.02                # 4 cm cube
CUBE_MASS = 0.05
TRAY_POS = (0.325, 0.06)        # tray centre (x, y) on the table (+y end)
TRAY_INNER = 0.08               # inner half-size of the tray -> 16 x 16 cm
TRAY_WALL_H = 0.05
TRAY_WALL_T = 0.008

# cube spawn region (inside the table, away from the tray and table edges)
SPAWN_X = (0.24, 0.41)
SPAWN_Y = (-0.32, -0.11)
SPAWN_YAW = (-np.pi, np.pi)

# ------------------------------------------------------------ motion planning
PREGRASP_DZ = 0.12
SAFE_MARGIN = 0.05
STEP_POS = 0.01
STEP_ROT = 0.10
EE_SPEED = 0.20
ROT_SPEED = 1.2
POS_TOL, ROT_TOL = 1e-3, 1e-2
MAX_IK_ITERS = 150
PHYS_DT = 0.002

# ------------------------------------------------------- custom IK parameters
DLS_LAMBDA = 1e-2
IK_GAIN = 0.8
NULL_ALPHA = 0.05               # has NO effect on a 6-DoF arm (no null space)
QP_DT = 0.05
JOINT_MARGIN = 0.03

# ------------------------------------------------------ workspace analysis
SHOULDER = np.array([0.0, 0.0, 0.321])   # joint_2 position at q = 0
REACH = 0.85                              # generous upper bound of the reach
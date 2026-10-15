"""
kinematics.py - forward kinematics (FK), Jacobian and orientation maths.

FK  = "given joint angles q, where is the tool?"   -> tcp_pose(q)
J   = "if joints move a little (dq), how does the tool move (dx)?"  dx = J dq
IK  = the reverse of FK: "where must the joints be so the tool is HERE?"
      All IK solvers in ik_solvers.py call the functions in this file.

Quaternions: MuJoCo stores them SCALAR-FIRST  q = [w, x, y, z].
(The tutorial's derivation uses scalar-last [x, y, z, w] - same maths,
 just a different order.  Mixing the two up is the #1 bug, so be careful.)
"""
import os

import mujoco
import numpy as np
import config as C


class Kin:
    """Small wrapper: a model + its own MjData used ONLY for kinematics.
    (We never step physics here - the IK 'imagines' joint configurations.)"""

    def __init__(self, model):
        self.m = model
        self.d = mujoco.MjData(model)
        self.qadr = np.array([model.joint(j).qposadr[0] for j in C.ARM_JOINTS])
        self.dadr = np.array([model.joint(j).dofadr[0] for j in C.ARM_JOINTS])
        self.lo = np.array([model.joint(j).range[0] for j in C.ARM_JOINTS])
        self.hi = np.array([model.joint(j).range[1] for j in C.ARM_JOINTS])
        self.site = model.site("tcp").id
        self.d.qpos[self.qadr] = C.Q_HOME
        for f in C.FINGER_JOINTS:
            if f in [model.joint(i).name for i in range(model.njnt)]:
                self.d.qpos[model.joint(f).qposadr[0]] = C.FINGER_OPEN  # open
        self._jp = np.zeros((3, model.nv))
        self._jr = np.zeros((3, model.nv))

    def set_q(self, q):
        """Put the 6 arm joints at q and update all link poses (FK)."""
        self.d.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.m, self.d)   # link positions/orientations
        mujoco.mj_comPos(self.m, self.d)       # needed before mj_jacSite

    def tcp_pose(self):
        """Current TCP position (3,) and quaternion [w,x,y,z]."""
        pos = self.d.site_xpos[self.site].copy()
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, self.d.site_xmat[self.site])
        return pos, quat

    def fk(self, q):
        self.set_q(q)
        return self.tcp_pose()

    def jacobian(self):
        """6x6 geometric Jacobian of the TCP: rows 0-2 linear, 3-5 angular
        (both in the WORLD frame), columns = the 6 arm joints.
        6 rows = 6 columns -> J is SQUARE for the HEAL (Panda: 6x7)."""
        mujoco.mj_jacSite(self.m, self.d, self._jp, self._jr, self.site)
        return np.vstack([self._jp[:, self.dadr], self._jr[:, self.dadr]])

    def within_limits(self, q, tol=1e-6):
        return bool(np.all(q >= self.lo - tol) and np.all(q <= self.hi + tol))


# ------------------------------------------------------------ orientation
def quat_mul(a, b):
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, a, b)
    return out


def quat_conj(a):
    return np.array([a[0], -a[1], -a[2], -a[3]])   # inverse of a UNIT quaternion


def orientation_error(q_target, q_current):
    """Axis-angle error vector (3,) in the world frame, exactly as in the
    tutorial:  q_e = q_t (x) q_c^-1,  flip sign if w<0,  eps = theta * u."""
    qe = quat_mul(q_target, quat_conj(q_current))
    if qe[0] < 0:                       # hemisphere continuity (w is first!)
        qe = -qe
    w = np.clip(qe[0], -1.0, 1.0)
    theta = 2.0 * np.arccos(w)
    if theta < 1e-6:
        return np.zeros(3)
    return theta * qe[1:] / np.sin(theta / 2.0)


def pose_error(pos_t, quat_t, pos_c, quat_c):
    """6-vector [position error; orientation error] = desired 'twist' x_dot_d*dt."""
    return np.concatenate([pos_t - pos_c, orientation_error(quat_t, quat_c)])


def quat_down(yaw):
    """Gripper pointing straight down (TCP z-axis = -world z), rotated by yaw
    about the world vertical.  yaw = YAW_HOME (about 60.5 deg) is the HEAL's
    tool yaw at the home pose (the 0.51 rad twist built into link_5)."""
    cz, sz = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    R = Rz @ np.diag([1.0, -1.0, -1.0])
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.flatten())
    return q


def yaw_of(quat):
    """Yaw (rotation about world z) of a gripper-down quaternion."""
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, quat)
    R = R.reshape(3, 3)
    return np.arctan2(R[1, 0], R[0, 0])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def slerp(q0, q1, t):
    """Spherical interpolation between two quaternions (0<=t<=1)."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    dot = np.dot(q0, q1)
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th = np.arccos(dot)
    return (np.sin((1 - t) * th) * q0 + np.sin(t * th) * q1) / np.sin(th)



# ------------------------------------------------------------ IK seed library
class SeedLibrary:
    """Why this exists (HEAL-specific!):
    A 6-DoF arm has NO spare joint, so for one gripper pose there are only a
    few separate solutions ("branches": elbow up/down, wrist flipped...).
    A local IK solver started in the wrong branch gets stuck.  The Panda's
    7th joint hid this problem; the HEAL shows it clearly.

    Fix: sample many random joint vectors once, keep those where the gripper
    points down, and store them.  For a new target we take stored samples
    whose (distance from base, height) is close, then
        turn joint_1 so the sample faces the target   (yaw changes +1:1)
        turn joint_6 so the yaw matches exactly        (TCP does not move)
    -> starting guesses that are already in the right branch."""

    def __init__(self, kin, n=400000, seed=0, path=None):
        path = path or os.path.join(C.HERE, "results", "seed_library.npz")
        if os.path.exists(path):
            D = np.load(path)
            self.rz, self.q = D["rz"], D["q"]
            return
        rng = np.random.default_rng(seed)
        rz, qs = [], []
        for q in rng.uniform(kin.lo, kin.hi, size=(n, C.N_ARM)):
            kin.set_q(q)
            R = kin.d.site_xmat[kin.site].reshape(3, 3)
            if R[2, 2] > -0.95:            # tool z-axis must point down
                continue
            p = kin.d.site_xpos[kin.site]
            az = np.arctan2(p[1], p[0])
            q = q.copy()
            q[0] = wrap(q[0] - az)                       # face +x  (azimuth 0)
            q[-1] = wrap(q[-1] - np.arctan2(R[1, 0], R[0, 0]) + az)  # yaw 0
            rz.append([np.hypot(p[0], p[1]), p[2]])
            qs.append(q)
        self.rz, self.q = np.array(rz), np.array(qs)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        np.savez(path, rz=self.rz, q=self.q)

    def seeds(self, pos, yaw, k=6):
        """k starting guesses for 'TCP at pos, pointing down with this yaw'."""
        r, z = np.hypot(pos[0], pos[1]), pos[2]
        d = np.hypot(self.rz[:, 0] - r, self.rz[:, 1] - z)
        az = np.arctan2(pos[1], pos[0])
        out = []
        for i in np.argsort(d)[:k]:
            q = self.q[i].copy()
            q[0] = wrap(q[0] + az)
            q[-1] = wrap(q[-1] + yaw)
            out.append(q)
        return out


if __name__ == "__main__":
    # How Q_HOME in config.py was found: solve IK once for "TCP at
    # (0.33, 0, 0.50), pointing down", starting from q = 0 (which already
    # points the flange straight down on the HEAL).
    from scene import build_model
    from ik_solvers import QPIK
    m = build_model(cube=False, table=False)
    kin = Kin(m)
    p0, q0 = kin.fk(np.zeros(C.N_ARM))
    print("q = 0      -> TCP", p0.round(3), " yaw", round(np.degrees(yaw_of(q0)), 1), "deg")
    r = QPIK(m).solve(np.zeros(C.N_ARM), np.array([0.33, 0.0, 0.50]), quat_down(yaw_of(q0)))
    print("home IK    -> converged", r.converged, " q =", np.round(r.q, 5).tolist())
    p, q = kin.fk(C.Q_HOME)
    print("Q_HOME     -> TCP", p.round(3), " yaw", round(np.degrees(yaw_of(q)), 1), "deg")
    print("singular values of J at home:", np.linalg.svd(kin.jacobian())[1].round(3))
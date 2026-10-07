"""
Our own Forward Kinematics (Product of Exponentials) — used to check MuJoCo.

Idea (Lynch & Park, Modern Robotics, Ch. 4):
    T(q) = e^[S1]q1 · e^[S2]q2 · ... · e^[Sn]qn · M

  * M   = pose of the end-effector when every joint angle is 0 ("home").
  * S_i = screw axis of joint i, written in the space frame at home:
          S_i = [w_i ; v_i],  with  v_i = -w_i x p_i
          (w_i = unit rotation axis, p_i = any point on that axis)

We read w_i, p_i and M from MuJoCo ONCE at q = 0, then compute T(q) ourselves
for any q. If our position matches MuJoCo's site/body position, our FK is right.
"""

import numpy as np
import mujoco


def skew(w):
    """3-vector -> 3x3 skew-symmetric matrix [w], so that [w] @ x == w x x."""
    return np.array([[0.0, -w[2], w[1]],
                     [w[2], 0.0, -w[0]],
                     [-w[1], w[0], 0.0]])


def exp_twist(S, theta):
    """Matrix exponential e^[S]theta for a revolute screw axis S = [w; v] (|w| = 1).

    Rotation part   : Rodrigues' formula  R = I + sin(t)[w] + (1 - cos(t))[w]^2
    Translation part: p = (I t + (1 - cos t)[w] + (t - sin t)[w]^2) v
    """
    w, v = S[:3], S[3:]
    W = skew(w)
    I = np.eye(3)
    R = I + np.sin(theta) * W + (1 - np.cos(theta)) * (W @ W)
    p = (I * theta + (1 - np.cos(theta)) * W + (theta - np.sin(theta)) * (W @ W)) @ v
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


class PoEForwardKinematics:
    """Builds the PoE model of the serial chain that ends at an EE site or body."""

    def __init__(self, model, ee_name, ee_is_site=True):
        self.model = model
        self.ee_is_site = ee_is_site
        obj = mujoco.mjtObj.mjOBJ_SITE if ee_is_site else mujoco.mjtObj.mjOBJ_BODY
        self.ee_id = mujoco.mj_name2id(model, obj, ee_name)
        if self.ee_id < 0:
            raise ValueError(f"End-effector '{ee_name}' not found in model")

        # 1. Walk from the EE's body back to the world, collecting hinge joints
        body = model.site_bodyid[self.ee_id] if ee_is_site else self.ee_id
        joints = []
        while body > 0:
            for j in range(model.body_jntadr[body], model.body_jntadr[body] + model.body_jntnum[body]):
                if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
                    joints.append(j)
            body = model.body_parentid[body]
        self.joints = joints[::-1]                              # base -> tip order
        self.qadr = [model.jnt_qposadr[j] for j in self.joints]  # where each angle lives in qpos

        # 2. Put the robot at q = 0 (on a scratch copy) and read the home geometry
        d0 = mujoco.MjData(model)
        d0.qpos[:] = 0.0
        mujoco.mj_kinematics(model, d0)
        self.S = []
        for j in self.joints:
            w = d0.xaxis[j] / np.linalg.norm(d0.xaxis[j])   # joint axis in world frame
            p = d0.xanchor[j]                                # a point on the axis
            self.S.append(np.concatenate([w, -np.cross(w, p)]))
        self.M = np.eye(4)
        if ee_is_site:
            self.M[:3, :3] = d0.site_xmat[self.ee_id].reshape(3, 3)
            self.M[:3, 3] = d0.site_xpos[self.ee_id]
        else:
            self.M[:3, :3] = d0.xmat[self.ee_id].reshape(3, 3)
            self.M[:3, 3] = d0.xpos[self.ee_id]

    def compute(self, qpos):
        """Our FK: returns the 4x4 end-effector pose T(q)."""
        T = np.eye(4)
        for S, adr in zip(self.S, self.qadr):
            T = T @ exp_twist(S, qpos[adr])
        return T @ self.M

    def mujoco_pose(self, data):
        """MuJoCo's own answer for the same end-effector, for comparison."""
        T = np.eye(4)
        if self.ee_is_site:
            T[:3, :3] = data.site_xmat[self.ee_id].reshape(3, 3)
            T[:3, 3] = data.site_xpos[self.ee_id]
        else:
            T[:3, :3] = data.xmat[self.ee_id].reshape(3, 3)
            T[:3, 3] = data.xpos[self.ee_id]
        return T

    def report(self, data):
        """One-line text: our FK position vs MuJoCo's, and the difference in mm."""
        T_own = self.compute(data.qpos)
        T_mj = self.mujoco_pose(data)
        p, pm = T_own[:3, 3], T_mj[:3, 3]
        err_mm = 1000 * np.linalg.norm(p - pm)
        rot_err = np.linalg.norm(T_own[:3, :3] - T_mj[:3, :3])
        return (f"EE (our PoE FK): X = {p[0]:+.3f}, Y = {p[1]:+.3f}, Z = {p[2]:+.3f} m\n"
                f"EE (MuJoCo)    : X = {pm[0]:+.3f}, Y = {pm[1]:+.3f}, Z = {pm[2]:+.3f} m"
                f"   | diff = {err_mm:.3f} mm, ||R_own - R_mj|| = {rot_err:.1e}")


def gravity_torque(model, data):
    """Joint torques that exactly cancel gravity (+ Coriolis) at the current state.

    mj_rne with flg_acc = 0 runs Recursive Newton-Euler with zero acceleration,
    giving c(q, qdot) + g(q). We store it in our own array instead of
    overwriting MuJoCo's data.qfrc_bias.
    """
    tau = np.zeros(model.nv)
    mujoco.mj_rne(model, data, 0, tau)
    return tau

"""
02_verify_skew_properties.py -- HW1 Part 2, Task 2:
Verify skew-symmetric identities numerically in MuJoCo.
"""

import numpy as np
import mujoco

from utils import hat, get_body_orientation, is_close_to_identity

MODEL_PATH = "../model/asymmetric_body.xml"

N_CHECKS_PER_STEP = 5
N_LOGGED_STEPS = 5
STEPS_BETWEEN_LOGS = 200


def random_unit_angular_velocity(rng):
    """Generate a random angular velocity vector."""
    w = rng.normal(size=3)
    return 2.0 * w / np.linalg.norm(w)


def check_identities(R, rng):
    """
    Numerically check:

    1. R(v × w) = (Rv) × (Rw)
    2. R omega_hat R^T = (R omega)_hat

    Returns the maximum residual for each identity.
    """

    max_residual_cross = 0.0
    max_residual_skew = 0.0

    for _ in range(N_CHECKS_PER_STEP):

        # Generate random vectors
        v = rng.normal(size=3)
        w = rng.normal(size=3)
        omega = rng.normal(size=3)

        # -----------------------------------------
        # Identity 1:
        # R(v × w) = (Rv) × (Rw)
        # -----------------------------------------
        left_cross = R @ np.cross(v, w)
        right_cross = np.cross(R @ v, R @ w)

        residual_cross = np.linalg.norm(
            left_cross - right_cross
        )

        # -----------------------------------------
        # Identity 2:
        # R omega_hat R^T = (R omega)_hat
        # -----------------------------------------
        left_skew = R @ hat(omega) @ R.T
        right_skew = hat(R @ omega)

        residual_skew = np.linalg.norm(
            left_skew - right_skew
        )

        # Store worst-case residual
        max_residual_cross = max(
            max_residual_cross,
            residual_cross
        )

        max_residual_skew = max(
            max_residual_skew,
            residual_skew
        )

    return max_residual_cross, max_residual_skew


def main():

    # Load MuJoCo model
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    # Random number generator
    rng = np.random.default_rng(seed=0)

    # Give the body an angular velocity
    data.qvel[3:6] = random_unit_angular_velocity(rng)

    # Update simulation
    mujoco.mj_forward(model, data)

    # Print table header
    print(
        f"{'step':>5} "
        f"{'t (s)':>8} "
        f"{'max resid: R(vxw)=(Rv)x(Rw)':>28} "
        f"{'max resid: RwR^T=(Rw)^':>24}"
    )

    # Run simulation and check identities
    for log_i in range(N_LOGGED_STEPS):

        # Advance simulation
        for _ in range(STEPS_BETWEEN_LOGS):
            mujoco.mj_step(model, data)

        # Get current rotation matrix
        R = get_body_orientation(data)

        # Check that R is a valid rotation matrix
        assert is_close_to_identity(
            R @ R.T,
            tol=1e-6
        ), "R is not orthonormal!"

        # Check both identities
        resid_cross, resid_skew = check_identities(R, rng)

        # Print results
        print(
            f"{log_i:5d} "
            f"{data.time:8.3f} "
            f"{resid_cross:28.3e} "
            f"{resid_skew:24.3e}"
        )


if __name__ == "__main__":
    main()
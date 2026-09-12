"""
02_verify_skew_properties.py -- HW1 Part 2, Task 2

Live MuJoCo simulation + numerical verification of:

1. R(v x w) = (Rv) x (Rw)
2. R omega_hat R^T = (R omega)_hat

The script also generates a residual-vs-time plot.
"""

import time
import numpy as np
import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt

from utils import hat, get_body_orientation, is_close_to_identity


MODEL_PATH = "../model/asymmetric_body.xml"

N_CHECKS_PER_STEP = 5
N_LOGGED_STEPS = 5
STEPS_BETWEEN_LOGS = 200


def time_varying_angular_velocity(t):
    """
    Time-varying angular velocity vector.

    Returns angular velocity in rad/s.
    """
    return np.array([
        1.5 * np.sin(0.8 * t),
        1.5 * np.cos(0.6 * t),
        1.0
    ])


def check_identities(R, rng):
    """
    Numerically verify:

        R(v x w) = (Rv) x (Rw)

    and

        R omega_hat R^T = (R omega)_hat

    Returns:
        max_residual_cross
        max_residual_skew
    """

    max_residual_cross = 0.0
    max_residual_skew = 0.0

    for _ in range(N_CHECKS_PER_STEP):

        # Generate random vectors
        v = rng.normal(size=3)
        w = rng.normal(size=3)
        omega = rng.normal(size=3)

        # -------------------------------------------------
        # Identity 1:
        # R(v x w) = (Rv) x (Rw)
        # -------------------------------------------------

        left_cross = R @ np.cross(v, w)

        right_cross = np.cross(
            R @ v,
            R @ w
        )

        residual_cross = np.linalg.norm(
            left_cross - right_cross
        )

        # -------------------------------------------------
        # Identity 2:
        # R omega_hat R^T = (R omega)_hat
        # -------------------------------------------------

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

    # -------------------------------------------------
    # Load MuJoCo model
    # -------------------------------------------------

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    rng = np.random.default_rng(seed=0)

    # Lists for storing results
    logged_times = []
    cross_residuals = []
    skew_residuals = []

    print("\nStarting MuJoCo simulation...")
    print("The body will rotate with time-varying angular velocity.\n")

    print(
        f"{'step':>5} "
        f"{'t (s)':>8} "
        f"{'max resid: R(vxw)=(Rv)x(Rw)':>28} "
        f"{'max resid: RwR^T=(Rw)^':>24}"
    )

    # -------------------------------------------------
    # Open live MuJoCo viewer
    # -------------------------------------------------

    with mujoco.viewer.launch_passive(model, data) as viewer:

        total_steps = N_LOGGED_STEPS * STEPS_BETWEEN_LOGS

        log_counter = 0

        for step in range(total_steps):

            if not viewer.is_running():
                break

            # ---------------------------------------------
            # Apply time-varying angular velocity
            # ---------------------------------------------

            omega_sim = time_varying_angular_velocity(data.time)

            # Free joint angular velocity
            data.qvel[3:6] = omega_sim

            # Advance simulation
            mujoco.mj_step(model, data)

            # Update viewer
            viewer.sync()

            # Slow down visualization approximately
            # so the motion is visible
            time.sleep(0.002)

            # ---------------------------------------------
            # Log/check every STEPS_BETWEEN_LOGS steps
            # ---------------------------------------------

            if (step + 1) % STEPS_BETWEEN_LOGS == 0:

                # Get current rotation matrix R(t)
                R = get_body_orientation(data)

                # Check orthonormality
                assert is_close_to_identity(
                    R @ R.T,
                    tol=1e-6
                ), "R is not orthonormal!"

                # Check both identities
                resid_cross, resid_skew = check_identities(
                    R,
                    rng
                )

                # Save results
                logged_times.append(data.time)
                cross_residuals.append(resid_cross)
                skew_residuals.append(resid_skew)

                # Print results
                print(
                    f"{log_counter:5d} "
                    f"{data.time:8.3f} "
                    f"{resid_cross:28.3e} "
                    f"{resid_skew:24.3e}"
                )

                log_counter += 1

        print("\nSimulation complete.")
        print("Close the MuJoCo window to generate the plot.")

        # Keep final orientation visible
        while viewer.is_running():
            viewer.sync()
            time.sleep(1 / 60)

    # -------------------------------------------------
    # Generate residual plot
    # -------------------------------------------------

    if len(logged_times) > 0:

        plt.figure(figsize=(8, 5))

        plt.plot(
            logged_times,
            cross_residuals,
            marker="o",
            label=r"$R(v \times w) - (Rv) \times (Rw)$"
        )

        plt.plot(
            logged_times,
            skew_residuals,
            marker="s",
            label=r"$R\hat{\omega}R^T - \widehat{R\omega}$"
        )

        plt.yscale("log")

        plt.xlabel("Simulation Time (s)")
        plt.ylabel("Maximum Residual (log scale)")

        plt.title(
            "Numerical Verification of Rotation Matrix Identities"
        )

        plt.grid(True, which="both", linestyle="--", alpha=0.5)

        plt.legend()

        plt.tight_layout()

        # Save plot
        plt.savefig(
            "q8_residual_plot.png",
            dpi=300
        )

        print(
            "\nPlot saved successfully as:"
        )

        print(
            "q8_residual_plot.png"
        )

        # Display plot
        plt.show()


if __name__ == "__main__":
    main()
"""
01_rotation_sandbox.py -- HW1 Part 2, Task 1: does rotation order matter?

STARTER CODE. The model loading, viewer, and simulation loop below
are complete and working -- run this file as-is and you should see
the asymmetric dart sitting in the viewer. Your job is to fill in
the TODOs so that:

  1. The user can queue up a sequence of elemental rotations
     (about x, y, or z), each one EITHER about the current body
     frame OR about the fixed space frame (their choice).
  2. The dart's orientation updates to reflect that sequence.
  3. You can run the SAME sequence of angles twice -- once
     "current frame" and once "fixed frame" -- and see (and
     screen-record) that the final orientation is visibly
     different, exactly as you proved symbolically in HW1
     Problem 3 (Lynch & Park Ch.3 Ex.3.4-style reasoning) and
     the "Composition of Rotations" lecture derivation.

This is intentionally a plain script, not a GUI app -- editing the
`rotation_sequence` list below and re-running is a perfectly good
"sandbox." A slider UI is a nice-to-have, not a requirement. Use AI
freely here; document what you asked it for in your AI Use Note.
"""

import time
import numpy as np
import mujoco
import mujoco.viewer

from utils import Rx, Ry, Rz, ELEMENTARY_ROTATIONS, set_body_orientation

MODEL_PATH = "../model/asymmetric_body.xml"


rotation_sequence = [
    ("z", np.deg2rad(90), "current"),   # TODO: try "fixed" here instead
    ("x", np.deg2rad(90), "current"),   # TODO: try "fixed" here instead
]


def compose_sequence(sequence):
    R = np.eye(3)

    for axis, angle, frame in sequence:
        R_step = ELEMENTARY_ROTATIONS[axis](angle)

        if frame == "current":
            R = R @ R_step

        elif frame == "fixed":
            R = R_step @ R

        else:
            raise ValueError(f"Unknown frame: {frame}")

    return R

def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    # Start with identity orientation
    R = np.eye(3)
    set_body_orientation(data, R)
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:

        print("Viewer open.")
        print(f"Applied sequence: {rotation_sequence}")
        print("Animation starts in 3 seconds...")

        # Wait 3 seconds before starting animation
        start_time = time.time()
        while time.time() - start_time < 3.0 and viewer.is_running():
            viewer.sync()
            time.sleep(1 / 60)

        print("Starting rotation now!")

       
        # Perform each rotation one at a time
        for axis, angle, frame in rotation_sequence:

            # Orientation before this rotation
            R_start = R.copy()

            duration = 2.0                  
            frames = 120        
            for i in range(frames + 1):

                if not viewer.is_running():
                    return

                alpha = i / frames
                current_angle = alpha * angle

                R_step = ELEMENTARY_ROTATIONS[axis](current_angle)

                if frame == "current":
                    R = R_start @ R_step

                elif frame == "fixed":
                    R = R_step @ R_start

                set_body_orientation(data, R)
                mujoco.mj_forward(model, data)

                viewer.sync()
                time.sleep(1 / 60)

            # Pause after each completed rotation
            pause_start = time.time()
            while time.time() - pause_start < 0.7 and viewer.is_running():
                viewer.sync()
                time.sleep(1 / 60)

        print("Animation complete!")

        # Keep final orientation visible
        while viewer.is_running():
            viewer.sync()
            time.sleep(1 / 60)

if __name__ == "__main__":
    main()

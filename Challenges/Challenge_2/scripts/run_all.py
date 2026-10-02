"""
Main Launcher Script for Challenge 2
Select and run any of the 4 MuJoCo simulation scripts:
1. Task 1: 6-DOF Industrial Manipulator
2. Task 1: 7-DOF Franka-Style Manipulator
3. Task 2: HEAL Robot XML Model
4. Task 2: Franka Panda Robot XML Model
"""

import sys
import os
import subprocess

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))

    scripts = {
        "1": ("Task 1: 6-DOF Industrial Manipulator", os.path.join(script_dir, "task1_6dof_manipulator.py")),
        "2": ("Task 1: 7-DOF Franka-Style Manipulator", os.path.join(script_dir, "task1_7dof_manipulator.py")),
        "3": ("Task 2: HEAL Robot XML Model", os.path.join(script_dir, "task2_heal_robot.py")),
        "4": ("Task 2: Franka Panda Robot XML Model", os.path.join(script_dir, "task2_franka_robot.py")),
    }

    print("\n==========================================")
    print("      CHALLENGE 2: MUJOCO FK & ROBOTICS   ")
    print("==========================================")
    print("Select a simulation script to run:")
    for key, (name, _) in scripts.items():
        print(f" [{key}] {name}")
    print(" [Q] Quit")
    print("------------------------------------------")

    choice = input("Enter choice (1-4 or Q): ").strip().upper()
    if choice in scripts:
        name, path = scripts[choice]
        print(f"\nRunning {name}...\n")
        subprocess.run([sys.executable, path])
    elif choice == "Q":
        print("Exiting.")
    else:
        print("Invalid choice!")

if __name__ == "__main__":
    main()

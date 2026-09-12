import os
import sys

def main():
    print("""
====================================================================
  INTRODUCTION TO ROBOTICS — CHALLENGE 1 SIMULATION LAUNCHER
====================================================================
  Select the robot simulation to launch:

  [1] TurtleBot3 Waffle Pi (Ground Mobile Robot — Differential Drive)
  [2] Quadcopter Drone (Aerial Multi-rotor — 6-DOF Flight Dynamics)
  [Q] Exit

====================================================================
""")
    choice = input("Enter choice [1/2/Q]: ").strip().lower()
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    if choice == '1':
        os.system(f'python "{os.path.join(script_dir, "teleop_turtlebot.py")}"')
    elif choice == '2':
        os.system(f'python "{os.path.join(script_dir, "teleop_quadcopter.py")}"')
    else:
        print("Exiting.")

if __name__ == "__main__":
    main()

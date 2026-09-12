import numpy as np

def quat2mat(q):
    """Convert MuJoCo quaternion [w, x, y, z] to 3x3 rotation matrix R in SO(3)."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])

def mat2euler(R):
    """Extract Euler angles [roll, pitch, yaw] in radians from rotation matrix R."""
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2,1], R[2,2])
        pitch = np.arctan2(-R[2,0], sy)
        yaw = np.arctan2(R[1,0], R[0,0])
    else:
        roll = np.arctan2(-R[1,2], R[1,1])
        pitch = np.arctan2(-R[2,0], sy)
        yaw = 0.0
    return np.array([roll, pitch, yaw])

def format_rotation_matrix_hud(R, pos, rpy_deg, robot_name="Robot"):
    """Generate a formatted string displaying the rotation matrix and pose HUD."""
    lines = [
        f"\n=======================================================",
        f"  [ITR Challenge 1] {robot_name.upper()} POSE & ROTATION MATRIX",
        f"=======================================================",
        f"  Position (x, y, z): [{pos[0]:+7.3f}, {pos[1]:+7.3f}, {pos[2]:+7.3f}] m",
        f"  Euler (R, P, Y)   : [Roll={rpy_deg[0]:+6.1f}°, Pitch={rpy_deg[1]:+6.1f}°, Yaw={rpy_deg[2]:+6.1f}°]",
        f"-------------------------------------------------------",
        f"  Rotation Matrix R_sb in SO(3):",
        f"    ┌                                     ┐",
        f"    │  {R[0,0]:+7.4f}   {R[0,1]:+7.4f}   {R[0,2]:+7.4f}  │",
        f"    │  {R[1,0]:+7.4f}   {R[1,1]:+7.4f}   {R[1,2]:+7.4f}  │",
        f"    │  {R[2,0]:+7.4f}   {R[2,1]:+7.4f}   {R[2,2]:+7.4f}  │",
        f"    └                                     ┘",
        f"  Orthogonality check: ||R^T R - I|| = {np.linalg.norm(R.T @ R - np.eye(3)):.2e} (det={np.linalg.det(R):+.4f})",
        f"======================================================="
    ]
    return "\n".join(lines)

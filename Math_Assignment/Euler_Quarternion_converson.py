import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

def euler_to_quaternion(yaw, pitch, roll):
    """
    Convert Euler angles (in radians) in ZYX order (yaw, pitch, roll) to a quaternion.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy - sr * sp * sy
    x = sr * cp * cy + cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z])

def quaternion_to_euler(q):
    """
    Convert a quaternion to Euler angles (in radians) in ZYX order (yaw, pitch, roll).
    Handles gimbal lock cases where pitch is ±pi/2.
    """
    q = np.array(q, dtype=float)
    q /= np.linalg.norm(q)
    w, x, y, z = q

    sin_pitch = 2 * (w * y - x * z)
    sin_pitch = np.clip(sin_pitch, -1.0, 1.0)
    pitch = np.arcsin(sin_pitch)

    epsilon = 1e-7
    if np.abs(sin_pitch) >= 1 - epsilon:
        yaw = 0.0
        roll = np.arctan2(2 * (x * y + w * z), 1 - 2 * (y**2 + z**2))
    else:
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

    return np.array([yaw, pitch, roll])

def quaternion_to_rotation_matrix(q):
    """Convert a quaternion to a 3x3 rotation matrix"""
    w, x, y, z = q
    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z,     2*x*z + 2*w*y],
        [2*x*y + 2*w*z,       1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y,       2*y*z + 2*w*x,     1 - 2*x**2 - 2*y**2]
    ])

def visualize_orientation(rotation_matrix, euler_angles=None, quaternion=None):
    """Visualize the orientation in 3D space"""
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    
    # Original axes in dashed style
    ax.quiver(0, 0, 0, 1, 0, 0, color='r', linestyle='--', label='Original X')
    ax.quiver(0, 0, 0, 0, 1, 0, color='g', linestyle='--', label='Original Y')
    ax.quiver(0, 0, 0, 0, 0, 1, color='b', linestyle='--', label='Original Z')
    
    # Rotated axes (via the rotation matrix)
    rx = rotation_matrix @ np.array([1, 0, 0])
    ry = rotation_matrix @ np.array([0, 1, 0])
    rz = rotation_matrix @ np.array([0, 0, 1])
    
    ax.quiver(0, 0, 0, rx[0], rx[1], rx[2], color='r', label='Rotated X', linewidth=2)
    ax.quiver(0, 0, 0, ry[0], ry[1], ry[2], color='g', label='Rotated Y', linewidth=2)
    ax.quiver(0, 0, 0, rz[0], rz[1], rz[2], color='b', label='Rotated Z', linewidth=2)
    
    # Set plot limits and labels
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # Create title with angles
    title = "Orientation Visualization\n"
    if euler_angles is not None:
        title += f"Euler Angles: {np.degrees(euler_angles).round(2)}°\n"
    if quaternion is not None:
        title += f"Quaternion: {np.array(quaternion).round(4)}"
    plt.title(title)
    
    ax.legend()
    plt.show()

if __name__ == "__main__":
    print("Select conversion type:")
    print("1: Convert Euler angles (in degrees) to quaternion and visualize orientation")
    print("2: Convert a given quaternion to Euler angles and visualize orientation")
    
    choice = input("Enter 1 or 2: ").strip()
    
    if choice == "1":
        try:
            euler_input = input("Enter Euler angles in degrees as yaw, pitch, roll (comma separated, e.g., 90,0,0): ")
            user_angles_deg = [float(angle.strip()) for angle in euler_input.split(',')]
            if len(user_angles_deg) != 3:
                raise ValueError("Please enter exactly three values.")
        except Exception as e:
            print("Invalid input:", e)
            sys.exit(1)
        
        # Convert degrees to radians for internal computation
        euler_user = np.radians(user_angles_deg)
        q_user = euler_to_quaternion(*euler_user)
        rm_user = quaternion_to_rotation_matrix(q_user)
        
        print("\nConversion Result:")
        print("Quaternion:", np.array(q_user).round(4))
        print("Reconverted Euler angles (degrees):", np.degrees(quaternion_to_euler(q_user)).round(2))
        
        visualize_orientation(rm_user, euler_user, q_user)
    
    elif choice == "2":
        try:
            q_input = input("Enter quaternion as w, x, y, z (comma separated, e.g., 0.7071, 0, 0.7071, 0): ")
            user_q = [float(val.strip()) for val in q_input.split(',')]
            if len(user_q) != 4:
                raise ValueError("Please enter exactly four values.")
        except Exception as e:
            print("Invalid input:", e)
            sys.exit(1)
        
        euler_from_q = quaternion_to_euler(user_q)
        rm_from_q = quaternion_to_rotation_matrix(user_q)
        
        print("\nConversion Result:")
        print("Euler angles (radians):", np.array(euler_from_q).round(4))
        print("Euler angles (degrees):", np.degrees(euler_from_q).round(2))
        
        visualize_orientation(rm_from_q, euler_from_q, user_q)
    
    else:
        print("Invalid selection! Exiting.")


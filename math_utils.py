# math_utils.py
"""
Rotations and coordinate transformations for quadcopter simulation
"""

import math
import numpy as np


def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to rotation matrix
    Using ZYX convention (yaw-pitch-roll)
    """
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    # ZYX rotation matrix
    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr                 ]
    ])
    
    return R


def body_to_world_frame(body_vector, roll, pitch, yaw):
    """
    Transform a vector from body frame to world frame
    """
    R = euler_to_rotation_matrix(roll, pitch, yaw)
    world_vector = R @ body_vector
    return world_vector


def rotation_matrix_to_euler(R):
    """
    Convert rotation matrix to Euler angles
    """
    if R[2, 0] < -1.0:
        R[2, 0] = -1.0
    elif R[2, 0] > 1.0:
        R[2, 0] = 1.0
    
    pitch = -math.asin(R[2, 0])
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    
    return roll, pitch, yaw


def quaternion_normalize(q):
    """Normalize quaternion"""
    norm = math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
    return [q[i]/norm for i in range(4)]


def quaternion_multiply(q1, q2):
    """Multiply two quaternions"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    
    return [w, x, y, z]

#!/usr/bin/env python
"""
Kinematics module for x_omni4wd robot
4-wheel omni-directional robot with X-configuration
Thứ tự bánh: (1,2,3,4) = (FL,RL,RR,FR)
"""

import numpy as np

# Robot parameters
WHEEL_RADIUS = 0.05  # m
WHEEL_SEPARATION_X = 0.2  # m
WHEEL_SEPARATION_Y = 0.2  # m

# Wheel angles (radians) - Thứ tự: (1,2,3,4) = (FL,RL,RR,FR)
WHEEL_ANGLE_FL = np.pi / 4.0      # 45° - Bánh 1: front_left
WHEEL_ANGLE_RL = 3.0 * np.pi / 4.0  # 135° - Bánh 2: rear_left
WHEEL_ANGLE_RR = -3.0 * np.pi / 4.0 # -135° - Bánh 3: rear_right
WHEEL_ANGLE_FR = -np.pi / 4.0     # -45° - Bánh 4: front_right

# Separation sum for kinematics
SEPARATION_SUM = (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) / 2.0


class Position:
    """Robot position and orientation"""
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
    
    def magnitude(self):
        return np.sqrt(self.x**2 + self.y**2)


def inverse_kinematics(v_x, v_y, omega_z):
    """
    Inverse Kinematics: Tính vận tốc từng bánh từ vận tốc robot
    
    Args:
        v_x: Vận tốc tuyến tính theo trục X (m/s)
        v_y: Vận tốc tuyến tính theo trục Y (m/s)
        omega_z: Vận tốc góc quanh trục Z (rad/s)
    
    Returns:
        Tuple (v_fl, v_rl, v_rr, v_fr): Vận tốc góc của từng bánh (rad/s)
        Thứ tự: (1,2,3,4) = (FL,RL,RR,FR)
    """
    # Công thức Inverse Kinematics cho 4 bánh omni X-configuration
    # v1 = v_fl = v_x - v_y - ω_z * (wheel_separation_x + wheel_separation_y) / 2
    # v2 = v_rl = v_x - v_y + ω_z * (wheel_separation_x + wheel_separation_y) / 2
    # v3 = v_rr = v_x + v_y + ω_z * (wheel_separation_x + wheel_separation_y) / 2
    # v4 = v_fr = v_x + v_y - ω_z * (wheel_separation_x + wheel_separation_y) / 2
    
    v_fl = v_x - v_y - omega_z * SEPARATION_SUM  # Bánh 1: front_left
    v_rl = v_x - v_y + omega_z * SEPARATION_SUM  # Bánh 2: rear_left
    v_rr = v_x + v_y + omega_z * SEPARATION_SUM  # Bánh 3: rear_right
    v_fr = v_x + v_y - omega_z * SEPARATION_SUM  # Bánh 4: front_right
    
    return (v_fl, v_rl, v_rr, v_fr)


def forward_kinematics(v_fl, v_rl, v_rr, v_fr):
    """
    Forward Kinematics: Tính vận tốc robot từ vận tốc các bánh
    
    Args:
        v_fl, v_rl, v_rr, v_fr: Vận tốc góc của từng bánh (rad/s)
    
    Returns:
        Tuple (v_x, v_y, omega_z): Vận tốc robot
    """
    # Công thức Forward Kinematics
    # v_x = (v1 + v2 + v3 + v4) / 4
    # v_y = (-v1 - v2 + v3 + v4) / 4
    # ω_z = (-v1 + v2 + v3 - v4) / (2 * (wheel_separation_x + wheel_separation_y))
    
    v_x = (v_fl + v_rl + v_rr + v_fr) / 4.0
    v_y = (-v_fl - v_rl + v_rr + v_fr) / 4.0
    omega_z = (-v_fl + v_rl + v_rr - v_fr) / (2.0 * SEPARATION_SUM)
    
    return (v_x, v_y, omega_z)


def cmd_vel_to_wheel_velocities(cmd_vel):
    """
    Convert cmd_vel (Twist) to wheel velocities
    
    Args:
        cmd_vel: geometry_msgs/Twist message
    
    Returns:
        Tuple (v_fl, v_rl, v_rr, v_fr): Wheel angular velocities (rad/s)
    """
    v_x = cmd_vel.linear.x
    v_y = cmd_vel.linear.y
    omega_z = cmd_vel.angular.z
    
    return inverse_kinematics(v_x, v_y, omega_z)


def angle_between_vectors(a, b):
    """
    Tính góc có dấu giữa hai vector
    
    Args:
        a, b: Góc (radians)
    
    Returns:
        Góc có dấu (radians)
    """
    a_vec = Position(np.cos(a), np.sin(a))
    b_vec = Position(np.cos(b), np.sin(b))
    
    dot_product = a_vec.x * b_vec.x + a_vec.y * b_vec.y
    cross_product = a_vec.x * b_vec.y - a_vec.y * b_vec.x
    
    angle = np.arccos(np.clip(dot_product / (a_vec.magnitude() * b_vec.magnitude()), -1.0, 1.0))
    
    if cross_product > 0:
        return angle
    else:
        return -angle


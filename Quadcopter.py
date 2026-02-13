# Quadcopter.py
"""
Quadcopter Physical Model and Dynamics
"""

import math
import numpy as np


class Quadcopter:
    """
    Quadcopter 6-DOF dynamics model
    State: [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
    """
    
    def __init__(self, mass=1.0, length=0.3, Ixx=0.01, Iyy=0.01, Izz=0.02):
        """
        Initialize quadcopter with physical parameters
        
        Args:
            mass: Vehicle mass (kg)
            length: Arm length (m)
            Ixx, Iyy, Izz: Moments of inertia (kg*m^2)
        """
        # Physical parameters
        self.mass = mass
        self.length = length
        self.g = 9.81
        
        # Moments of inertia
        self.Ix = Ixx
        self.Iy = Iyy
        self.Iz = Izz
        
        # State: position
        self.x = 0.0
        self.y = 0.0
        self.z = 2.5    # Initial altitude
        
        # State: velocity
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        
        # State: Euler angles (roll, pitch, yaw)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # State: Angular velocities
        self.p = 0.0  # Roll rate
        self.q = 0.0  # Pitch rate
        self.r = 0.0  # Yaw rate
        
        # Control inputs (thrust başlangıçta hover için düzenlenmiş)
        self.torque_roll = 0.0
        self.torque_pitch = 0.0
        self.torque_yaw = 0.0
        self.thrust = 0.0  # PID tarafından ayarlanacak
        
        # Stabilization limits (high values - mainly for safety)
        self.MAX_ANGLE = 1.57     # radians (π/2 = 90 degrees!)
        self.MAX_VEL = 50.0
        self.MAX_POS = 500.0      # Increased - much higher, less likely to hit
        
        # History for logging
        self.history = []

    def set_control(self, torque_roll, torque_pitch, torque_yaw, thrust):
        """Set control inputs (torques and thrust)"""
        self.torque_roll = torque_roll
        self.torque_pitch = torque_pitch
        self.torque_yaw = torque_yaw
        self.thrust = max(0.0, thrust)

    def step(self, dt):
        """
        Integrate dynamics using Euler method for one time step
        """
        # Angular accelerations (from Newton's laws)
        pdot = self.torque_roll / self.Ix
        qdot = self.torque_pitch / self.Iy
        rdot = self.torque_yaw / self.Iz
        
        # Update angular velocities
        self.p += pdot * dt
        self.q += qdot * dt
        self.r += rdot * dt
        
        # Update Euler angles
        self.roll += self.p * dt
        self.pitch += self.q * dt
        self.yaw += self.r * dt
        
        # Limit angles to prevent simulation instability
        self.roll = max(-self.MAX_ANGLE, min(self.MAX_ANGLE, self.roll))
        self.pitch = max(-self.MAX_ANGLE, min(self.MAX_ANGLE, self.pitch))
        # yaw can rotate freely
        
        # Normalize yaw to [-pi, pi]
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi

        # Convert thrust to world frame using Euler angles
        # ZYX convention rotation matrix
        cr = math.cos(self.roll)
        sr = math.sin(self.roll)
        cp = math.cos(self.pitch)
        sp = math.sin(self.pitch)
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        
        # Rotation matrix elements (body frame to world frame)
        # For proper quadcopter dynamics:
        # Pitch (+) = forward tilt → +X direction
        # Roll (+) = right tilt → +Y direction
        
        # Thrust vector in body frame is [0, 0, T]
        # Transform to world frame
        r11 = cy * cp
        r12 = cy * sp * sr - sy * cr
        r13 = cy * sp * cr + sy * sr
        r21 = sy * cp
        r22 = sy * sp * sr + cy * cr
        r23 = sy * sp * cr - cy * sr
        r31 = -sp
        r32 = cp * sr
        r33 = cp * cr
        
        # Thrust vector in world frame (body thrust [0,0,T])
        fx = self.thrust * r13
        fy = self.thrust * r23
        fz = self.thrust * r33 - self.mass * self.g
        
        # Linear accelerations
        ax = fx / self.mass
        ay = fy / self.mass
        az = fz / self.mass
        
        # Update velocities
        self.vx += ax * dt
        self.vy += ay * dt
        self.vz += az * dt
        
        # Limit velocities
        self.vx = max(-self.MAX_VEL, min(self.MAX_VEL, self.vx))
        self.vy = max(-self.MAX_VEL, min(self.MAX_VEL, self.vy))
        self.vz = max(-self.MAX_VEL, min(self.MAX_VEL, self.vz))
        
        # Update position
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt
        
        # Prevent going underground
        if self.z < 0.0:
            self.z = 0.0
            self.vz = 0.0
        
        # Limit position
        self.x = max(-self.MAX_POS, min(self.MAX_POS, self.x))
        self.y = max(-self.MAX_POS, min(self.MAX_POS, self.y))
        self.z = max(0.0, min(self.MAX_POS, self.z))

    def get_state(self):
        """Return current state as dictionary"""
        return {
            "x": self.x, "y": self.y, "z": self.z,
            "vx": self.vx, "vy": self.vy, "vz": self.vz,
            "roll": self.roll, "pitch": self.pitch, "yaw": self.yaw,
            "p": self.p, "q": self.q, "r": self.r,
            "thrust": self.thrust,
            "torque_roll": self.torque_roll,
            "torque_pitch": self.torque_pitch,
            "torque_yaw": self.torque_yaw
        }
    
    def reset(self):
        """Reset quadcopter to initial state"""
        self.x = 0.0
        self.y = 0.0
        self.z = 2.5
        
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0
        
        self.history = []
# simulator.py
"""
Quadcopter Simulator - integrates physics, control, and state management
"""

from Quadcopter import Quadcopter
from pid_controller import CascadedController


class QuadcopterSimulator:
    """
    Main simulator that combines quadcopter dynamics with control algorithms
    """
    
    def __init__(self, mass=1.0, length=0.3, Ixx=0.01, Iyy=0.01, Izz=0.02):
        """
        Initialize simulator
        
        Args:
            mass: Vehicle mass (kg)
            length: Arm length (m)
            Ixx, Iyy, Izz: Moments of inertia
        """
        # Physical model
        self.quad = Quadcopter(mass=mass, length=length, 
                              Ixx=Ixx, Iyy=Iyy, Izz=Izz)
        
        # Control system
        self.controller = CascadedController()
        
        # Reference (setpoint) values
        self.z_ref = 2.5
        self.roll_ref = 0.0
        self.pitch_ref = 0.0
        self.yaw_ref = 0.0
        
        # Simulation statistics
        self.sim_time = 0.0
        self.step_count = 0
        
        # Data logging
        self.logged_states = []

    def set_references(self, roll_ref, pitch_ref, yaw_ref, z_ref):
        """Update reference values from user input or auto-pilot"""
        self.roll_ref = roll_ref
        self.pitch_ref = pitch_ref
        self.yaw_ref = yaw_ref
        self.z_ref = z_ref

    def step(self, dt):
        """
        Execute one simulation step
        
        Args:
            dt: Time step (seconds)
            
        Returns:
            Current state dictionary
        """
        # Get current state
        state = self.quad.get_state()
        
        # Altitude control loop
        thrust = self.controller.update_altitude(
            self.z_ref, state["z"], state["vz"],
            self.quad.mass, self.quad.g, dt
        )
        
        # Attitude control loops
        torque_roll, torque_pitch, torque_yaw = self.controller.update_attitude(
            self.roll_ref, self.pitch_ref, self.yaw_ref,
            state["roll"], state["pitch"], state["yaw"], dt
        )
        
        # Apply control inputs to quadcopter
        self.quad.set_control(
            torque_roll=torque_roll,
            torque_pitch=torque_pitch,
            torque_yaw=torque_yaw,
            thrust=thrust
        )
        
        # Advance physics simulation
        self.quad.step(dt)
        
        # Update simulation time
        self.sim_time += dt
        self.step_count += 1
        
        # Log state if needed
        if self.step_count % 4 == 0:  # Log every 4th step to save memory
            self.logged_states.append((self.sim_time, self.quad.get_state()))
            if len(self.logged_states) > 5000:  # Keep last 5000 states
                self.logged_states.pop(0)
        
        return self.quad.get_state()

    def reset(self):
        """Reset simulation to initial conditions"""
        self.quad.reset()
        self.controller.reset()
        self.sim_time = 0.0
        self.step_count = 0
        self.logged_states = []

    def get_extended_state(self):
        """
        Get extended state including references and errors
        """
        state = self.quad.get_state()
        state["z_ref"] = self.z_ref
        state["roll_ref"] = self.roll_ref
        state["pitch_ref"] = self.pitch_ref
        state["yaw_ref"] = self.yaw_ref
        
        # Calculate errors
        state["z_err"] = self.z_ref - state["z"]
        state["roll_err"] = self.roll_ref - state["roll"]
        state["pitch_err"] = self.pitch_ref - state["pitch"]
        state["yaw_err"] = self.yaw_ref - state["yaw"]
        
        # Simulation info
        state["sim_time"] = self.sim_time
        state["step_count"] = self.step_count
        
        return state

    def get_log_entries(self, num_entries=10):
        """Get last N logged states"""
        return self.logged_states[-num_entries:]

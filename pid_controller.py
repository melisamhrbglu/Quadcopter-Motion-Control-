# pid_controller.py
"""
PID Controller implementation for quadcopter control
"""


class PID:
    """
    Standard PID controller with integral saturation and derivative filtering
    """
    
    def __init__(self, kp, ki, kd, integrator_limit=None, derivative_filter_tau=0.0):
        """
        Initialize PID controller
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            integrator_limit: Limit for integral term (anti-windup)
            derivative_filter_tau: Low-pass filter time constant for derivative
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integrator_limit = integrator_limit
        self.derivative_filter_tau = derivative_filter_tau
        
        # State
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0
        self.first_call = True

    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0
        self.first_call = True

    def update(self, error, dt):
        """
        Update PID controller
        
        Args:
            error: Current error (setpoint - feedback)
            dt: Time step
            
        Returns:
            Control output
        """
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        if self.integrator_limit is not None:
            self.integral = max(-self.integrator_limit, 
                              min(self.integrator_limit, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term with filtering
        if self.first_call:
            d_term = 0.0
            self.first_call = False
        else:
            raw_derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
            
            # Low-pass filter on derivative
            if self.derivative_filter_tau > 0:
                alpha = dt / (dt + self.derivative_filter_tau)
                self.filtered_derivative = (alpha * raw_derivative + 
                                          (1 - alpha) * self.filtered_derivative)
                d_term = self.kd * self.filtered_derivative
            else:
                d_term = self.kd * raw_derivative
        
        self.prev_error = error
        
        return p_term + i_term + d_term


class CascadedController:
    """
    Cascaded controller for altitude and attitude control
    Altitude controller outputs thrust
    Attitude controllers output torques
    """
    
    def __init__(self):
        """Initialize cascaded control structure"""
        # Altitude control (z)
        self.alt_pid = PID(
            kp=10.0, ki=2.0, kd=5.0,
            integrator_limit=2.5,
            derivative_filter_tau=0.02
        )
        
        # Attitude control - VERY CONSERVATIVE
        self.roll_pid = PID(
            kp=2.0, ki=0.1, kd=1.0,  # Minimal gains
            integrator_limit=0.1,     # Very small!
            derivative_filter_tau=0.08
        )
        
        self.pitch_pid = PID(
            kp=2.0, ki=0.1, kd=1.0,  # Minimal gains
            integrator_limit=0.1,     # Very small!
            derivative_filter_tau=0.08
        )
        
        self.yaw_pid = PID(
            kp=1.0, ki=0.05, kd=0.5,  # Minimal gains
            integrator_limit=0.05,    # Very small!
            derivative_filter_tau=0.08
        )

    def reset(self):
        """Reset all controllers"""
        self.alt_pid.reset()
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()

    def update_altitude(self, z_ref, z_actual, vz_actual, mass, g, dt):
        """
        Altitude control with feedforward velocity damping
        
        Returns:
            thrust command
        """
        z_err = z_ref - z_actual
        # Velocity damping term
        alt_cmd = self.alt_pid.update(z_err - 0.2 * vz_actual, dt)
        thrust = mass * g + alt_cmd
        return thrust

    def update_attitude(self, roll_ref, pitch_ref, yaw_ref, 
                       roll_actual, pitch_actual, yaw_actual, dt):
        """
        Attitude control (roll, pitch, yaw)
        
        Returns:
            (torque_roll, torque_pitch, torque_yaw)
        """
        roll_err = roll_ref - roll_actual
        pitch_err = pitch_ref - pitch_actual
        yaw_err = yaw_ref - yaw_actual
        
        torque_roll = self.roll_pid.update(roll_err, dt)
        torque_pitch = self.pitch_pid.update(pitch_err, dt)
        torque_yaw = self.yaw_pid.update(yaw_err, dt)
        
        return torque_roll, torque_pitch, torque_yaw
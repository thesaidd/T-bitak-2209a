"""
PID Controller for drone control
Simple PID implementation for visual servoing
"""

import time
from typing import Optional


class PIDController:
    """Simple PID controller for drone control"""
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                 output_limits: Optional[tuple] = None):
        """
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limits: (min, max) output limits
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        # Internal state
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
    
    def update(self, error: float, dt: Optional[float] = None) -> float:
        """
        Calculate PID output
        
        Args:
            error: Current error (setpoint - measurement)
            dt: Time delta (optional, will calculate if None)
            
        Returns:
            Control output
        """
        # Calculate dt if not provided
        if dt is None:
            current_time = time.time()
            if self.last_time is not None:
                dt = current_time - self.last_time
            else:
                dt = 0.0
            self.last_time = current_time
        
        # Avoid division by zero
        if dt <= 0:
            dt = 0.01
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply output limits
        if self.output_limits is not None:
            output = max(self.output_limits[0], min(output, self.output_limits[1]))
        
        # Save for next iteration
        self.last_error = error
        
        return output
    
    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
    
    def set_gains(self, kp: Optional[float] = None, ki: Optional[float] = None, 
                  kd: Optional[float] = None):
        """Update PID gains"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd

"""
ROS Kalman Filtre Modülü
Nesne pozisyon ve hız tahmini için
Orijinal src/kalman_filter.py'den uyarlanmıştır
"""

import numpy as np
from filterpy.kalman import KalmanFilter
from typing import Tuple, Optional, List


class UAVKalmanFilter:
    """Kalman Filter implementation for UAV tracking"""
    
    def __init__(self, 
                 dt: float = 1.0,
                 process_noise: float = 1e-3,
                 measurement_noise: float = 1e-1):
        """
        Args:
            dt: Time step (time between frames)
            process_noise: Process noise (model uncertainty)
            measurement_noise: Measurement noise (detection uncertainty)
        """
        # Create Kalman filter
        # State: [x, y, vx, vy] - position and velocity
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        
        # State transition matrix (F)
        # x_new = x + vx*dt
        # y_new = y + vy*dt
        # vx_new = vx
        # vy_new = vy
        self.kf.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Measurement matrix (H)
        # We only measure position (x, y)
        self.kf.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Process noise covariance (Q)
        self.kf.Q = np.eye(4) * process_noise
        
        # Measurement noise covariance (R)
        self.kf.R = np.eye(2) * measurement_noise
        
        # Initial state covariance (P)
        self.kf.P = np.eye(4) * 1000
        
        self.dt = dt
        self.initialized = False
        self.prediction_history = []
    
    def init(self, x: float, y: float, vx: float = 0, vy: float = 0):
        """
        Initialize filter
        
        Args:
            x: Initial x position
            y: Initial y position
            vx: Initial x velocity (optional)
            vy: Initial y velocity (optional)
        """
        self.kf.x = np.array([x, y, vx, vy])
        self.initialized = True
        self.prediction_history.clear()
    
    def predict(self) -> Tuple[float, float]:
        """
        Predict next state
        
        Returns:
            (x, y) prediction
        """
        if not self.initialized:
            raise RuntimeError("Kalman filter not initialized!")
        
        self.kf.predict()
        
        x, y = self.kf.x[0], self.kf.x[1]
        return (x, y)
    
    def update(self, measurement: Tuple[float, float]) -> Tuple[float, float]:
        """
        Update with measurement
        
        Args:
            measurement: (x, y) measurement
            
        Returns:
            (x, y) updated prediction
        """
        if not self.initialized:
            # Initialize with first measurement
            self.init(measurement[0], measurement[1])
            return measurement
        
        # First predict
        self.predict()
        
        # Then update
        z = np.array(measurement)
        self.kf.update(z)
        
        x, y = self.kf.x[0], self.kf.x[1]
        return (x, y)
    
    def predict_ahead(self, steps: int = 5) -> List[Tuple[float, float]]:
        """
        Predict N steps ahead
        
        Args:
            steps: How many steps ahead
            
        Returns:
            [(x, y), ...] prediction list
        """
        if not self.initialized:
            return []
        
        # Save current state
        saved_x = self.kf.x.copy()
        saved_P = self.kf.P.copy()
        
        predictions = []
        
        for _ in range(steps):
            self.kf.predict()
            x, y = self.kf.x[0], self.kf.x[1]
            predictions.append((x, y))
        
        # Restore state
        self.kf.x = saved_x
        self.kf.P = saved_P
        
        return predictions
    
    def get_state(self) -> Tuple[float, float, float, float]:
        """
        Get current state
        
        Returns:
            (x, y, vx, vy)
        """
        if not self.initialized:
            return (0, 0, 0, 0)
        
        return tuple(self.kf.x)
    
    def get_position(self) -> Tuple[float, float]:
        """
        Get current position
        
        Returns:
            (x, y)
        """
        if not self.initialized:
            return (0, 0)
        
        return (self.kf.x[0], self.kf.x[1])
    
    def get_velocity(self) -> Tuple[float, float]:
        """
        Get current velocity
        
        Returns:
            (vx, vy)
        """
        if not self.initialized:
            return (0, 0)
        
        return (self.kf.x[2], self.kf.x[3])
    
    def get_speed(self) -> float:
        """
        Get speed magnitude
        
        Returns:
            Speed (pixels/frame)
        """
        vx, vy = self.get_velocity()
        return np.sqrt(vx**2 + vy**2)
    
    def reset(self):
        """Reset filter"""
        self.kf.P = np.eye(4) * 1000
        self.initialized = False
        self.prediction_history.clear()

"""
ROS Nesne Takip Modülü
OpenCV tabanlı nesne takibi
Orijinal src/tracker.py'den uyarlanmıştır
"""

import cv2
import numpy as np
from typing import Optional, Tuple, List, Dict
from collections import deque


class ObjectTracker:
    """OpenCV-based object tracking class"""
    
    # Available tracker types
    TRACKER_TYPES = {
        'CSRT': cv2.TrackerCSRT_create,      # High accuracy, slow
        'KCF': cv2.TrackerKCF_create,        # Balanced
        'MOSSE': cv2.legacy.TrackerMOSSE_create,  # Fast, lower accuracy
        'MIL': cv2.TrackerMIL_create,        # Medium
    }
    
    def __init__(self, tracker_type: str = 'CSRT', trajectory_length: int = 30):
        """
        Args:
            tracker_type: Tracker type (CSRT, KCF, MOSSE, MIL)
            trajectory_length: Trajectory length (frames)
        """
        if tracker_type not in self.TRACKER_TYPES:
            raise ValueError(f"Invalid tracker type: {tracker_type}")
        
        self.tracker_type = tracker_type
        self.tracker = None
        self.is_tracking = False
        self.trajectory_length = trajectory_length
        
        # Trajectory deque
        self.trajectory = deque(maxlen=trajectory_length)
        
        # Statistics
        self.frames_tracked = 0
        self.frames_lost = 0
        self.last_bbox = None
        self.last_center = None
    
    def init(self, frame: np.ndarray, bbox: Tuple[int, int, int, int]) -> bool:
        """
        Initialize tracker
        
        Args:
            frame: Initial frame
            bbox: Initial bbox (x1, y1, x2, y2)
            
        Returns:
            Success status
        """
        # Create new tracker
        self.tracker = self.TRACKER_TYPES[self.tracker_type]()
        
        # Convert bbox format: (x1,y1,x2,y2) -> (x,y,w,h)
        x1, y1, x2, y2 = bbox
        
        # Fix bbox (x1 < x2 and y1 < y2)
        if x1 > x2:
            x1, x2 = x2, x1
        if y1 > y2:
            y1, y2 = y2, y1
        
        x, y, w, h = x1, y1, x2 - x1, y2 - y1
        
        # Validation
        if w <= 0 or h <= 0:
            return False
        
        if w < 5 or h < 5:
            return False
        
        # Check frame boundaries
        frame_h, frame_w = frame.shape[:2]
        if x < 0 or y < 0 or x + w > frame_w or y + h > frame_h:
            # Clip bbox to frame
            x = max(0, min(x, frame_w - 1))
            y = max(0, min(y, frame_h - 1))
            w = min(w, frame_w - x)
            h = min(h, frame_h - y)
        
        # Initialize tracker
        try:
            success = self.tracker.init(frame, (x, y, w, h))
        except Exception:
            return False
        
        if success:
            self.is_tracking = True
            self.last_bbox = (x, y, x + w, y + h)
            self.last_center = (x + w // 2, y + h // 2)
            self.trajectory.clear()
            self.trajectory.append(self.last_center)
        
        return success
    
    def update(self, frame: np.ndarray) -> Tuple[bool, Optional[Tuple[int, int, int, int]]]:
        """
        Update tracker
        
        Args:
            frame: New frame
            
        Returns:
            (success, bbox) - bbox format (x1, y1, x2, y2)
        """
        if not self.is_tracking or self.tracker is None:
            return False, None
        
        # Update tracker
        success, bbox_xywh = self.tracker.update(frame)
        
        if success:
            # Convert bbox format: (x,y,w,h) -> (x1,y1,x2,y2)
            x, y, w, h = [int(v) for v in bbox_xywh]
            bbox = (x, y, x + w, y + h)
            
            # Center point
            center = ((x + x + w) // 2, (y + y + h) // 2)
            
            # Update
            self.last_bbox = bbox
            self.last_center = center
            self.trajectory.append(center)
            self.frames_tracked += 1
            
            return True, bbox
        else:
            self.frames_lost += 1
            self.is_tracking = False
            return False, None
    
    def reset(self):
        """Reset tracker"""
        self.tracker = None
        self.is_tracking = False
        self.trajectory.clear()
        self.last_bbox = None
        self.last_center = None
    
    def get_trajectory(self) -> List[Tuple[int, int]]:
        """Get trajectory points"""
        return list(self.trajectory)
    
    def get_velocity(self) -> Optional[Tuple[float, float]]:
        """
        Calculate velocity vector (pixels/frame)
        
        Returns:
            (vx, vy) or None
        """
        if len(self.trajectory) < 2:
            return None
        
        # Last two points
        p1 = self.trajectory[-2]
        p2 = self.trajectory[-1]
        
        vx = p2[0] - p1[0]
        vy = p2[1] - p1[1]
        
        return (vx, vy)
    
    def get_speed(self) -> Optional[float]:
        """
        Calculate speed magnitude (pixels/frame)
        
        Returns:
            Speed or None
        """
        velocity = self.get_velocity()
        if velocity is None:
            return None
        
        vx, vy = velocity
        speed = np.sqrt(vx**2 + vy**2)
        
        return speed

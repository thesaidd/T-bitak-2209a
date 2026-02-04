"""
Hybrid Tracker Node - Ana ROS 2 Node
Otonom drone takibi için state machine uygular
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum
import time

# Vision modüllerini içe aktar
from .vision.ros_detector import UAVDetector
from .vision.ros_robust_tracker import RobustTracker
from .vision.ros_kalman_filter import UAVKalmanFilter

# Controller'ları içe aktar
from .controllers.pid_controller import PIDController


class TrackingState(Enum):
    """State machine durumları"""
    VISUAL_TRACKING = 1      # Birincil: Görsel tespit + takip
    KALMAN_PREDICTION = 2    # Tampon: Kalman filtre tahmini
    GROUND_TRUTH_SEARCH = 3  # Yedek: GPS tabanlı arama


class HybridTrackerNode(Node):
    """
    Otonom drone takibi için hybrid tracking node
    
    State Machine:
    - VISUAL_TRACKING: Kamera + YOLOv8 + OpenCV tracker kullan
    - KALMAN_PREDICTION: 3 saniye boyunca Kalman filtre kullan
    - GROUND_TRUTH_SEARCH: Gazebo model states kullan
    """
    
    def __init__(self):
        super().__init__('hybrid_tracker_node')
        
        # Parametreleri tanımla
        self._declare_parameters()
        
        # Parametreleri yükle
        self._load_parameters()
        
        # CV bridge'i başlat
        self.bridge = CvBridge()
        
        # Vision modüllerini başlat
        self.get_logger().info('Vision modülleri başlatılıyor...')
        self._init_vision_modules()
        
        # Controller'ları başlat
        self.get_logger().info('PID controller'lar başlatılıyor...')
        self._init_controllers()
        
        # State machine
        self.current_state = TrackingState.VISUAL_TRACKING
        self.state_entry_time = time.time()
        self.visual_lost_time = None
        
        # Takip durumu
        self.last_detection = None
        self.last_bbox = None
        self.target_found = False
        
        # Frame işleme
        self.frame_count = 0
        self.last_frame = None
        self.image_width = 640
        self.image_height = 480
        
        # Kamera için QoS profili (gerçek zamanlı için best effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber'lar
        self.get_logger().info(f'{self.camera_topic} topic'ine abone oluyor...')
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile
        )
        
        self.get_logger().info(f'{self.model_states_topic} topic'ine abone oluyor...')
        self.model_states_sub = self.create_subscription(
            ModelStates,
            self.model_states_topic,
            self.model_states_callback,
            10
        )
        
        # Publisher'lar
        self.get_logger().info(f'{self.cmd_vel_topic} topic'ine yayınlanıyor...')
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            self.cmd_vel_topic,
            10
        )
        
        # Kontrol döngüsü için timer (30 Hz)
        self.control_timer = self.create_timer(1.0 / 30.0, self.control_loop)
        
        # Ground truth verisi
        self.target_position = None  # Dünya frame'inde (x, y, z)
        
        self.get_logger().info('Hybrid Tracker Node başarıyla başlatıldı!')
        self.get_logger().info(f'İlk durum: {self.current_state.name}')
    
    def _declare_parameters(self):
        """ROS 2 parametrelerini tanımla"""
        # Topics
        self.declare_parameter('camera_topic', '/takipIHA/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/takipIHA/mavros/setpoint_velocity/cmd_vel_unstamped')
        self.declare_parameter('model_states_topic', '/gazebo/model_states')
        self.declare_parameter('target_name', 'dusmanIHA')
        
        # Detection parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('target_classes', [4, 14])  # airplane, bird
        self.declare_parameter('tracker_type', 'CSRT')
        
        # Image processing
        self.declare_parameter('target_width', 640)
        self.declare_parameter('target_height', 480)
        
        # State machine
        self.declare_parameter('kalman_timeout', 3.0)  # seconds
        
        # PID gains - Yaw (horizontal centering)
        self.declare_parameter('yaw_kp', 0.005)
        self.declare_parameter('yaw_ki', 0.0)
        self.declare_parameter('yaw_kd', 0.001)
        
        # PID gains - Pitch/Speed (distance control)
        self.declare_parameter('pitch_kp', 0.003)
        self.declare_parameter('pitch_ki', 0.0)
        self.declare_parameter('pitch_kd', 0.001)
        
        # PID gains - Altitude (Z control)
        self.declare_parameter('alt_kp', 0.5)
        self.declare_parameter('alt_ki', 0.0)
        self.declare_parameter('alt_kd', 0.1)
        
        # Control limits
        self.declare_parameter('max_yaw_rate', 0.5)  # rad/s
        self.declare_parameter('max_linear_speed', 2.0)  # m/s
        self.declare_parameter('max_vertical_speed', 1.0)  # m/s
        
        # Target bbox size (for distance estimation)
        self.declare_parameter('target_bbox_size', 100)  # pixels
    
    def _load_parameters(self):
        """Load parameters from parameter server"""
        # Topics
        self.camera_topic = self.get_parameter('camera_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.model_states_topic = self.get_parameter('model_states_topic').value
        self.target_name = self.get_parameter('target_name').value
        
        # Detection
        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.target_classes = self.get_parameter('target_classes').value
        self.tracker_type = self.get_parameter('tracker_type').value
        
        # Image processing
        self.target_width = self.get_parameter('target_width').value
        self.target_height = self.get_parameter('target_height').value
        
        # State machine
        self.kalman_timeout = self.get_parameter('kalman_timeout').value
        
        # PID gains
        self.yaw_kp = self.get_parameter('yaw_kp').value
        self.yaw_ki = self.get_parameter('yaw_ki').value
        self.yaw_kd = self.get_parameter('yaw_kd').value
        
        self.pitch_kp = self.get_parameter('pitch_kp').value
        self.pitch_ki = self.get_parameter('pitch_ki').value
        self.pitch_kd = self.get_parameter('pitch_kd').value
        
        self.alt_kp = self.get_parameter('alt_kp').value
        self.alt_ki = self.get_parameter('alt_ki').value
        self.alt_kd = self.get_parameter('alt_kd').value
        
        # Control limits
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_vertical_speed = self.get_parameter('max_vertical_speed').value
        
        # Target size
        self.target_bbox_size = self.get_parameter('target_bbox_size').value
    
    def _init_vision_modules(self):
        """Initialize vision modules"""
        # YOLOv8 detector
        self.detector = UAVDetector(
            model_path=self.model_path,
            conf_threshold=self.conf_threshold,
            target_classes=self.target_classes,
            device='cpu',
            verbose=True
        )
        
        # Robust tracker
        self.tracker = RobustTracker(
            tracker_type=self.tracker_type,
            trajectory_length=30
        )
        
        # Kalman filter
        self.kalman = UAVKalmanFilter(
            dt=1.0/30.0,  # 30 Hz
            process_noise=1e-3,
            measurement_noise=1e-1
        )
    
    def _init_controllers(self):
        """Initialize PID controllers"""
        # Yaw controller (horizontal centering)
        self.yaw_controller = PIDController(
            kp=self.yaw_kp,
            ki=self.yaw_ki,
            kd=self.yaw_kd,
            output_limits=(-self.max_yaw_rate, self.max_yaw_rate)
        )
        
        # Pitch/Speed controller (distance maintenance)
        self.pitch_controller = PIDController(
            kp=self.pitch_kp,
            ki=self.pitch_ki,
            kd=self.pitch_kd,
            output_limits=(-self.max_linear_speed, self.max_linear_speed)
        )
        
        # Altitude controller (Z-axis)
        self.alt_controller = PIDController(
            kp=self.alt_kp,
            ki=self.alt_ki,
            kd=self.alt_kd,
            output_limits=(-self.max_vertical_speed, self.max_vertical_speed)
        )
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize to target resolution
            if cv_image.shape[1] != self.target_width or cv_image.shape[0] != self.target_height:
                cv_image = cv2.resize(cv_image, (self.target_width, self.target_height))
            
            self.last_frame = cv_image
            self.image_width = cv_image.shape[1]
            self.image_height = cv_image.shape[0]
            self.frame_count += 1
            
            # Process based on current state
            if self.current_state == TrackingState.VISUAL_TRACKING:
                self._process_visual_tracking(cv_image)
            elif self.current_state == TrackingState.KALMAN_PREDICTION:
                self._process_kalman_prediction(cv_image)
            
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')
    
    def model_states_callback(self, msg):
        """Process Gazebo model states"""
        try:
            # Find target drone in model states
            if self.target_name in msg.name:
                idx = msg.name.index(self.target_name)
                pose = msg.pose[idx]
                
                # Extract position
                self.target_position = (
                    pose.position.x,
                    pose.position.y,
                    pose.position.z
                )
        except Exception as e:
            self.get_logger().error(f'Model states callback error: {e}')
    
    def _process_visual_tracking(self, frame):
        """Process visual tracking (STATE A)"""
        # If tracker is active, update it
        if self.tracker.is_tracking:
            success, bbox = self.tracker.update(frame)
            
            if success:
                # Tracking successful
                self.last_bbox = bbox
                self.target_found = True
                self.visual_lost_time = None
                
                # Update Kalman filter with measurement
                center = ((bbox[0] + bbox[2]) // 2, (bbox[1] + bbox[3]) // 2)
                self.kalman.update(center)
                
                return
            else:
                # Tracking lost, try detection
                self.get_logger().warn('Tracker lost, attempting re-detection...')
                self.tracker.reset()
        
        # Run detection (every frame or when tracker is lost)
        detections = self.detector.detect(frame)
        
        if detections:
            # Get best detection
            best_det = self.detector.get_best_detection(detections)
            
            if best_det:
                bbox = best_det['bbox']
                
                # Initialize tracker
                success = self.tracker.init(frame, bbox)
                
                if success:
                    self.last_bbox = bbox
                    self.target_found = True
                    self.visual_lost_time = None
                    
                    # Initialize/update Kalman filter
                    center = best_det['center']
                    self.kalman.update(center)
                    
                    self.get_logger().info(f'Target acquired! Confidence: {best_det["conf"]:.2f}')
                else:
                    self.get_logger().warn('Failed to initialize tracker')
        else:
            # No detections
            if self.target_found:
                # Target was found before, now lost
                if self.visual_lost_time is None:
                    self.visual_lost_time = time.time()
                    self.get_logger().warn('Visual contact lost, switching to Kalman prediction...')
                
                # Check if we should switch to Kalman mode
                time_lost = time.time() - self.visual_lost_time
                if time_lost > 0.5:  # 0.5 second buffer before switching
                    self._switch_state(TrackingState.KALMAN_PREDICTION)
    
    def _process_kalman_prediction(self, frame):
        """Process Kalman prediction (STATE B)"""
        # Try to re-acquire visual contact
        detections = self.detector.detect(frame)
        
        if detections:
            best_det = self.detector.get_best_detection(detections)
            if best_det:
                # Visual contact re-established!
                bbox = best_det['bbox']
                success = self.tracker.init(frame, bbox)
                
                if success:
                    self.get_logger().info('Visual contact re-established!')
                    self._switch_state(TrackingState.VISUAL_TRACKING)
                    return
        
        # Check timeout
        time_in_state = time.time() - self.state_entry_time
        if time_in_state > self.kalman_timeout:
            self.get_logger().warn(f'Kalman timeout ({self.kalman_timeout}s), switching to ground truth search...')
            self._switch_state(TrackingState.GROUND_TRUTH_SEARCH)
    
    def _switch_state(self, new_state):
        """Switch to new state"""
        self.get_logger().info(f'State transition: {self.current_state.name} -> {new_state.name}')
        self.current_state = new_state
        self.state_entry_time = time.time()
    
    def control_loop(self):
        """Main control loop (30 Hz)"""
        # Create command message
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        
        # Generate control commands based on state
        if self.current_state == TrackingState.VISUAL_TRACKING:
            if self.last_bbox is not None:
                cmd = self._visual_control(cmd)
        
        elif self.current_state == TrackingState.KALMAN_PREDICTION:
            cmd = self._kalman_control(cmd)
        
        elif self.current_state == TrackingState.GROUND_TRUTH_SEARCH:
            cmd = self._ground_truth_control(cmd)
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
    
    def _visual_control(self, cmd):
        """Generate control commands from visual tracking"""
        x1, y1, x2, y2 = self.last_bbox
        
        # Calculate bbox center and size
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        bbox_width = x2 - x1
        bbox_height = y2 - y1
        bbox_area = bbox_width * bbox_height
        
        # Image center
        img_center_x = self.image_width / 2
        img_center_y = self.image_height / 2
        
        # --- YAW CONTROL (horizontal centering) ---
        # Error: positive = target is to the right
        yaw_error = center_x - img_center_x
        yaw_rate = self.yaw_controller.update(yaw_error)
        cmd.twist.angular.z = -yaw_rate  # Negative to turn towards target
        
        # --- PITCH/SPEED CONTROL (distance maintenance) ---
        # Error: positive = target is too small (too far)
        size_error = self.target_bbox_size - bbox_width
        forward_speed = self.pitch_controller.update(size_error)
        cmd.twist.linear.x = forward_speed
        
        # --- ALTITUDE CONTROL (vertical centering) ---
        # Error: positive = target is below center
        alt_error = center_y - img_center_y
        vertical_speed = self.alt_controller.update(alt_error)
        cmd.twist.linear.z = -vertical_speed  # Negative to go up when target is below
        
        return cmd
    
    def _kalman_control(self, cmd):
        """Generate control commands from Kalman prediction"""
        # Predict next position
        if self.kalman.initialized:
            pred_x, pred_y = self.kalman.predict()
            
            # Use predicted position for control
            img_center_x = self.image_width / 2
            img_center_y = self.image_height / 2
            
            # Yaw control
            yaw_error = pred_x - img_center_x
            yaw_rate = self.yaw_controller.update(yaw_error)
            cmd.twist.angular.z = -yaw_rate
            
            # Move forward slowly
            cmd.twist.linear.x = 0.5
            
            # Altitude control
            alt_error = pred_y - img_center_y
            vertical_speed = self.alt_controller.update(alt_error)
            cmd.twist.linear.z = -vertical_speed
        
        return cmd
    
    def _ground_truth_control(self, cmd):
        """Generate control commands from ground truth (Gazebo model states)"""
        if self.target_position is None:
            # No ground truth available, hover
            cmd.twist.linear.x = 0.0
            cmd.twist.linear.y = 0.0
            cmd.twist.linear.z = 0.0
            cmd.twist.angular.z = 0.0
            return cmd
        
        # For simplicity, move towards target in body frame
        # In a real system, you'd use TF2 to transform coordinates
        
        # Simple approach: move forward and adjust altitude
        cmd.twist.linear.x = 1.0  # Move forward
        
        # Altitude matching (assuming we can get our own altitude)
        # This is simplified - in practice you'd need odometry
        target_z = self.target_position[2]
        # cmd.twist.linear.z would be set based on altitude difference
        
        return cmd


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = HybridTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

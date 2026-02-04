# 🚁 Hybrid Tracker - ROS 2 Package

**Autonomous drone tracking system using hybrid visual-GPS approach for Gazebo Classic 11**

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.8+-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

## 📋 Overview

This package enables a follower drone (`takipIHA`) to autonomously track a target drone (`dusmanIHA`) in Gazebo using a sophisticated hybrid tracking approach that combines:

- **Visual Tracking** (Primary): YOLOv8 detection + OpenCV tracking
- **Kalman Prediction** (Buffer): 3-second prediction during signal loss
- **Ground Truth Search** (Fallback): Gazebo model states for recovery

## 🎯 Features

### State Machine Architecture

```
┌─────────────────────────┐
│  VISUAL_TRACKING        │ ◄─── Primary Mode
│  - Camera feed          │
│  - YOLOv8 detection     │
│  - OpenCV tracker       │
│  - PID control          │
└───────────┬─────────────┘
            │ Lost > 0.5s
            ▼
┌─────────────────────────┐
│  KALMAN_PREDICTION      │ ◄─── Safety Buffer
│  - 3-second prediction  │
│  - Continue tracking    │
│  - Attempt re-detection │
└───────────┬─────────────┘
            │ Timeout (3s)
            ▼
┌─────────────────────────┐
│  GROUND_TRUTH_SEARCH    │ ◄─── Recovery Mode
│  - Gazebo model states  │
│  - Direct navigation    │
│  - Re-acquire visual    │
└─────────────────────────┘
```

### Control System

- **Yaw Control**: PID-based horizontal centering
- **Pitch/Speed Control**: Distance maintenance via bbox size
- **Altitude Control**: 3D tracking with Z-axis matching

## 📁 Package Structure

```
hybrid_tracker/
├── hybrid_tracker/
│   ├── __init__.py
│   ├── hybrid_tracker_node.py    # Main ROS 2 node
│   ├── vision/                    # Computer vision modules
│   │   ├── __init__.py
│   │   ├── detector.py           # YOLOv8 detection
│   │   ├── tracker.py            # OpenCV tracking
│   │   ├── robust_tracker.py     # Enhanced tracker
│   │   └── kalman_filter.py      # Kalman filter
│   └── controllers/
│       ├── __init__.py
│       └── pid_controller.py     # PID control
├── launch/
│   └── tracking.launch.py
├── config/
│   └── tracker_params.yaml
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 🚀 Installation

### Prerequisites

- **ROS 2 Humble** on Ubuntu 22.04
- **Gazebo Classic 11**
- **MAVROS** (for drone control)
- **Python 3.8+**

### Step 1: Install System Dependencies

```bash
# ROS 2 dependencies
sudo apt update
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    python3-pip

# Python dependencies
pip3 install ultralytics filterpy opencv-python numpy
```

### Step 2: Build the Package

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws/src

# Copy the hybrid_tracker package here
# (or clone from your repository)

# Build
cd ~/ros2_ws
colcon build --packages-select hybrid_tracker

# Source the workspace
source install/setup.bash
```

## 🎮 Usage

### Quick Start

```bash
# Terminal 1: Launch Gazebo with drones
# (Use your existing Gazebo launch file)

# Terminal 2: Launch hybrid tracker
ros2 launch hybrid_tracker tracking.launch.py
```

### With Custom Parameters

```bash
ros2 launch hybrid_tracker tracking.launch.py \
    config_file:=/path/to/custom_params.yaml
```

### Run Node Directly

```bash
ros2 run hybrid_tracker hybrid_tracker_node \
    --ros-args --params-file config/tracker_params.yaml
```

## ⚙️ Configuration

Edit `config/tracker_params.yaml` to customize:

### Topics

```yaml
camera_topic: "/takipIHA/camera/image_raw"
cmd_vel_topic: "/takipIHA/mavros/setpoint_velocity/cmd_vel_unstamped"
model_states_topic: "/gazebo/model_states"
target_name: "dusmanIHA"
```

### Detection Parameters

```yaml
model_path: "yolov8n.pt"          # YOLOv8 model
conf_threshold: 0.5                # Detection confidence
target_classes: [4, 14]            # COCO: airplane, bird
tracker_type: "CSRT"               # CSRT, KCF, MOSSE, MIL
```

### PID Tuning

```yaml
# Yaw control (horizontal centering)
yaw_kp: 0.005
yaw_ki: 0.0
yaw_kd: 0.001

# Pitch/Speed control (distance)
pitch_kp: 0.003
pitch_ki: 0.0
pitch_kd: 0.001

# Altitude control (Z-axis)
alt_kp: 0.5
alt_ki: 0.0
alt_kd: 0.1
```

### Control Limits

```yaml
max_yaw_rate: 0.5       # rad/s
max_linear_speed: 2.0   # m/s
max_vertical_speed: 1.0 # m/s
```

## 🔍 Monitoring

### Check Topics

```bash
# List all topics
ros2 topic list

# Monitor camera feed
ros2 topic echo /takipIHA/camera/image_raw --no-arr

# Monitor control commands
ros2 topic echo /takipIHA/mavros/setpoint_velocity/cmd_vel_unstamped

# Monitor model states
ros2 topic echo /gazebo/model_states
```

### View Logs

```bash
# Real-time logs
ros2 run hybrid_tracker hybrid_tracker_node

# Check node info
ros2 node info /hybrid_tracker_node
```

### Visualize with RViz2

```bash
ros2 run rqt_image_view rqt_image_view
# Select: /takipIHA/camera/image_raw
```

## 🎯 State Machine Behavior

### Visual Tracking (Primary)

- **Input**: Camera images at 640x480
- **Processing**: YOLOv8 detection → OpenCV tracking
- **Control**: PID-based visual servoing
- **Transition**: Lost > 0.5s → Kalman Prediction

### Kalman Prediction (Buffer)

- **Duration**: 3 seconds maximum
- **Processing**: Kalman filter predicts target position
- **Control**: Follow predicted coordinates
- **Transition**: 
  - Re-detected → Visual Tracking
  - Timeout → Ground Truth Search

### Ground Truth Search (Recovery)

- **Input**: Gazebo `/model_states`
- **Processing**: Extract `dusmanIHA` position
- **Control**: Navigate directly to coordinates
- **Transition**: Visual contact → Visual Tracking

## 🛠️ Troubleshooting

### No Detections

**Problem**: YOLOv8 not detecting the target drone

**Solutions**:
1. Lower confidence threshold:
   ```yaml
   conf_threshold: 0.3
   ```

2. Train custom model on your drone:
   ```bash
   # Use your drone images
   yolo train data=drone_dataset.yaml model=yolov8n.pt epochs=100
   ```

3. Add more target classes or use all classes:
   ```yaml
   target_classes: []  # Empty = all classes
   ```

### Tracker Keeps Losing Target

**Problem**: OpenCV tracker fails frequently

**Solutions**:
1. Use more robust tracker:
   ```yaml
   tracker_type: "CSRT"  # Most accurate
   ```

2. Adjust PID gains for smoother motion

3. Increase Kalman timeout:
   ```yaml
   kalman_timeout: 5.0
   ```

### Control Commands Not Working

**Problem**: Drone not responding to commands

**Solutions**:
1. Verify MAVROS is running:
   ```bash
   ros2 topic list | grep mavros
   ```

2. Check topic names match your setup:
   ```bash
   ros2 topic info /takipIHA/mavros/setpoint_velocity/cmd_vel_unstamped
   ```

3. Ensure drone is in OFFBOARD mode (MAVROS requirement)

### High CPU Usage

**Problem**: System running slow

**Solutions**:
1. Use faster tracker:
   ```yaml
   tracker_type: "MOSSE"
   ```

2. Reduce image resolution:
   ```yaml
   target_width: 416
   target_height: 312
   ```

3. Use YOLOv8n (nano) model (already default)

## 📊 Performance

### Expected Performance

| Component | FPS | Latency |
|-----------|-----|---------|
| YOLOv8n Detection | 15-20 | ~50ms |
| OpenCV Tracking | 30+ | ~10ms |
| Control Loop | 30 | ~33ms |
| **Total System** | **30** | **~100ms** |

### Hardware Requirements

- **Minimum**: 4GB RAM, 2 CPU cores
- **Recommended**: 8GB RAM, 4 CPU cores
- **GPU**: Optional (improves detection speed)

## 🔬 Advanced Usage

### Custom YOLOv8 Model

```bash
# Train on your drone dataset
yolo train data=my_drone.yaml model=yolov8n.pt epochs=100

# Update config
model_path: "runs/detect/train/weights/best.pt"
```

### Multi-Drone Tracking

Modify the node to track multiple targets by:
1. Using `MultiObjectTracker` from vision modules
2. Publishing separate control commands
3. Managing multiple Kalman filters

### Integration with Path Planning

```python
# Subscribe to planned path
self.path_sub = self.create_subscription(
    Path,
    '/planned_path',
    self.path_callback,
    10
)
```

## 📝 Citation

If you use this package in your research, please cite:

```bibtex
@software{hybrid_tracker_2026,
  title={Hybrid Tracker: ROS 2 Package for Autonomous Drone Tracking},
  author={Abdullah Gül University},
  year={2026},
  url={https://github.com/yourusername/hybrid_tracker}
}
```

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Open a Pull Request

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/yourusername/hybrid_tracker/issues)
- **Email**: [email protected]
- **Documentation**: [Wiki](https://github.com/yourusername/hybrid_tracker/wiki)

## 🙏 Acknowledgments

- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
- [OpenCV](https://opencv.org/)
- [FilterPy](https://github.com/rlabbe/filterpy)
- [ROS 2](https://docs.ros.org/en/humble/)
- [MAVROS](https://github.com/mavlink/mavros)

---

**⭐ If you find this package useful, please star the repository!**

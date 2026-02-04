# Hybrid Tracker - Quick Reference

## 🚀 Quick Start

### Build Package
```bash
cd ~/ros2_ws
colcon build --packages-select hybrid_tracker
source install/setup.bash
```

### Launch Tracker
```bash
ros2 launch hybrid_tracker tracking.launch.py
```

## 📋 Common Commands

### Check Topics
```bash
# List all topics
ros2 topic list

# Monitor camera
ros2 topic echo /takipIHA/camera/image_raw --no-arr

# Monitor control
ros2 topic echo /takipIHA/mavros/setpoint_velocity/cmd_vel_unstamped
```

### View Images
```bash
ros2 run rqt_image_view rqt_image_view
```

### Check Node
```bash
ros2 node info /hybrid_tracker_node
```

## ⚙️ Configuration

Edit: `config/tracker_params.yaml`

### Key Parameters
- `conf_threshold`: Detection confidence (default: 0.5)
- `tracker_type`: CSRT, KCF, MOSSE, or MIL
- `kalman_timeout`: Seconds before GPS fallback (default: 3.0)
- `yaw_kp`, `pitch_kp`, `alt_kp`: PID gains

## 🔧 Troubleshooting

### No Detections
```yaml
conf_threshold: 0.3  # Lower threshold
target_classes: []   # All classes
```

### Tracker Lost
```yaml
tracker_type: "CSRT"  # Most robust
kalman_timeout: 5.0   # Longer buffer
```

### Slow Performance
```yaml
tracker_type: "MOSSE"  # Fastest
target_width: 416      # Lower resolution
target_height: 312
```

## 📊 State Machine

1. **VISUAL_TRACKING** (Primary)
   - Camera → YOLOv8 → Tracker → PID Control

2. **KALMAN_PREDICTION** (Buffer: 3s)
   - Kalman filter prediction → PID Control

3. **GROUND_TRUTH_SEARCH** (Fallback)
   - Gazebo model states → Direct navigation

## 🎯 PID Tuning

### Increase Responsiveness
```yaml
yaw_kp: 0.010   # Double the gain
pitch_kp: 0.006
alt_kp: 1.0
```

### Reduce Oscillation
```yaml
yaw_kd: 0.002   # Increase derivative
pitch_kd: 0.002
alt_kd: 0.2
```

## 📦 Dependencies

### System
- ros-humble-cv-bridge
- ros-humble-mavros
- ros-humble-gazebo-ros-pkgs

### Python
- ultralytics
- filterpy
- opencv-python
- numpy

## 🔗 Important Topics

### Subscriptions
- `/takipIHA/camera/image_raw`
- `/gazebo/model_states`

### Publications
- `/takipIHA/mavros/setpoint_velocity/cmd_vel_unstamped`

## 📝 Notes

- Requires MAVROS and drone in OFFBOARD mode
- Default resolution: 640x480
- Control loop: 30 Hz
- Detection: 15-20 FPS

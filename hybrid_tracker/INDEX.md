# 📚 Hybrid Tracker - Documentation Index

Welcome to the Hybrid Tracker ROS 2 package documentation!

## 🚀 Quick Navigation

### For First-Time Users
1. **Start Here**: [README.md](README.md) - Complete package overview
2. **Quick Setup**: [QUICKSTART.md](QUICKSTART.md) - Fast reference guide
3. **Installation**: [install.sh](install.sh) - Automated setup script

### For Developers
4. **Architecture**: [ARCHITECTURE.md](ARCHITECTURE.md) - System design & diagrams
5. **Package Summary**: [PACKAGE_SUMMARY.md](PACKAGE_SUMMARY.md) - Delivery details
6. **Source Code**: [hybrid_tracker/](hybrid_tracker/) - Implementation

---

## 📖 Documentation Files

### 1. README.md (10KB)
**Purpose**: Main documentation  
**Contents**:
- Package overview
- Installation instructions
- Usage examples
- Configuration guide
- Troubleshooting
- Performance benchmarks

**Read this if**: You're new to the package or need comprehensive information

---

### 2. QUICKSTART.md (2.4KB)
**Purpose**: Quick reference  
**Contents**:
- Common commands
- Configuration snippets
- PID tuning tips
- Troubleshooting shortcuts

**Read this if**: You need quick answers or command references

---

### 3. ARCHITECTURE.md (12KB)
**Purpose**: System design documentation  
**Contents**:
- System architecture diagrams
- State machine flow
- Data flow diagrams
- Module dependencies
- Performance characteristics

**Read this if**: You want to understand how the system works internally

---

### 4. PACKAGE_SUMMARY.md (8KB)
**Purpose**: Delivery summary  
**Contents**:
- Complete file listing
- Refactoring summary
- Feature checklist
- Deployment steps
- Success criteria

**Read this if**: You want to see what was delivered and verify completeness

---

### 5. install.sh (3.5KB)
**Purpose**: Automated installation  
**Contents**:
- Dependency installation
- Workspace setup
- Package building
- Environment configuration

**Use this if**: You want automated setup on Ubuntu 22.04

---

## 🗂️ Source Code Structure

### Main Node
- **[hybrid_tracker_node.py](hybrid_tracker/hybrid_tracker_node.py)** (19KB, 600+ lines)
  - Main ROS 2 node
  - State machine implementation
  - Control loop
  - ROS 2 integration

### Vision Modules
- **[vision/detector.py](hybrid_tracker/vision/detector.py)**
  - YOLOv8 detection
  - Refactored from `src/detector.py`

- **[vision/tracker.py](hybrid_tracker/vision/tracker.py)**
  - OpenCV tracking base class
  - Refactored from `src/tracker.py`

- **[vision/robust_tracker.py](hybrid_tracker/vision/robust_tracker.py)**
  - Enhanced tracker with stability features
  - Refactored from `src/robust_tracker.py`

- **[vision/kalman_filter.py](hybrid_tracker/vision/kalman_filter.py)**
  - Kalman filter for prediction
  - Refactored from `src/kalman_filter.py`

### Controllers
- **[controllers/pid_controller.py](hybrid_tracker/controllers/pid_controller.py)**
  - PID implementation
  - Used for Yaw, Pitch, Altitude control

### Configuration
- **[config/tracker_params.yaml](config/tracker_params.yaml)**
  - All tunable parameters
  - ROS 2 parameter format

### Launch Files
- **[launch/tracking.launch.py](launch/tracking.launch.py)**
  - ROS 2 launch file
  - Parameter loading

---

## 🎯 Common Tasks

### I want to...

#### Install the package
→ Read: [README.md § Installation](README.md#-installation)  
→ Run: `./install.sh`

#### Launch the system
→ Read: [QUICKSTART.md § Quick Start](QUICKSTART.md#-quick-start)  
→ Run: `ros2 launch hybrid_tracker tracking.launch.py`

#### Tune PID gains
→ Read: [QUICKSTART.md § PID Tuning](QUICKSTART.md#-pid-tuning)  
→ Edit: `config/tracker_params.yaml`

#### Understand the state machine
→ Read: [ARCHITECTURE.md § State Machine](ARCHITECTURE.md#state-transition-diagram)  
→ See: State transition diagrams

#### Debug detection issues
→ Read: [README.md § Troubleshooting](README.md#-troubleshooting)  
→ Check: Camera topic, confidence threshold

#### Modify the code
→ Read: [ARCHITECTURE.md § Module Dependencies](ARCHITECTURE.md#module-dependencies)  
→ Edit: Source files in `hybrid_tracker/`

---

## 📊 Package Statistics

| Metric | Value |
|--------|-------|
| **Total Files** | 20 |
| **Total Size** | 95 KB |
| **Python Modules** | 9 |
| **Documentation Files** | 6 |
| **Configuration Files** | 4 |
| **Lines of Code** | ~1,500+ |

---

## 🔗 External Resources

### ROS 2 Documentation
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [cv_bridge Tutorial](https://docs.ros.org/en/humble/p/cv_bridge/)
- [MAVROS Documentation](https://docs.px4.io/main/en/ros/mavros_installation.html)

### Computer Vision
- [YOLOv8 Documentation](https://docs.ultralytics.com/)
- [OpenCV Trackers](https://docs.opencv.org/4.x/d9/df8/group__tracking.html)
- [Kalman Filter Tutorial](https://filterpy.readthedocs.io/)

### Gazebo
- [Gazebo Classic 11](http://gazebosim.org/tutorials?tut=ros2_overview)
- [Model States API](http://docs.ros.org/en/api/gazebo_msgs/html/msg/ModelStates.html)

---

## 🆘 Getting Help

### Documentation Issues
If you find errors or unclear sections in the documentation:
1. Check other documentation files for clarification
2. Review the source code comments
3. Consult the architecture diagrams

### Technical Issues
If you encounter technical problems:
1. Check [README.md § Troubleshooting](README.md#-troubleshooting)
2. Review [QUICKSTART.md](QUICKSTART.md) for common solutions
3. Verify your ROS 2 and dependency versions

### Configuration Issues
If parameters aren't working as expected:
1. Check [config/tracker_params.yaml](config/tracker_params.yaml)
2. Review [README.md § Configuration](README.md#-configuration)
3. See [QUICKSTART.md § PID Tuning](QUICKSTART.md#-pid-tuning)

---

## 📝 Document Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2026-02-04 | Initial package creation |

---

## ✅ Pre-Flight Checklist

Before deploying the package, ensure:

- [ ] Read README.md
- [ ] Reviewed ARCHITECTURE.md
- [ ] Installed all dependencies
- [ ] Built package with colcon
- [ ] Configured tracker_params.yaml
- [ ] Tested camera topic
- [ ] Verified MAVROS connection
- [ ] Tuned PID gains for your setup

---

**Happy Tracking! 🚁**

For questions or issues, refer to the documentation files listed above.

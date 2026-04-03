# ROS2 Object Tracking Pipeline

A ROS2 Humble based object tracking system that performs:

- **YOLOv10m-based object detection** for designated object classes
- **Multiple object tracking** using **SORT** (Simple Online and Realtime Tracking)
- **Tracked result visualization** with bounding boxes and IDs
- **CSV export** of tracked trajectories
- **Docker / Docker Compose based execution**

This project was implemented as a ROS object tracking take-home assignment and satisfies the core requirements of:

- ROS2-based modular node design
- Dockerized setup and execution
- Configurable runtime
- Basic testing support
- Output visualization and CSV logging

---

## 1. Overview

The system consists of two ROS2 nodes:

1. **YOLO node**
   - Loads a TorchScript YOLOv10m model
   - Performs object detection
   - Publishes detection results as a custom ROS2 message

2. **Tracker node**
   - Subscribes to detections
   - Associates detections across frames using **SORT**
   - Draws tracking results
   - Saves tracked data to CSV
   - Optionally saves an output video

The current implementation supports two input modes:

- **ROS image topic / webcam mode**
- **Video-file mode** using a configured input video path

Because webcam access in Docker Desktop + WSL2 can be limited depending on host configuration, video-file mode is also supported for reproducible testing.

---

## 2. Technologies and environment

### ROS distribution
- **ROS2 Humble**

### Language
- **C++17**

### Main libraries
- ROS2 (`rclcpp`, `sensor_msgs`, `std_msgs`, custom messages)
- OpenCV
- PyTorch / LibTorch (TorchScript runtime)
- Docker / Docker Compose

---

## 3. Repository structure

```text
object_tracking_linux/
├── docker/
│   ├── Dockerfile
│   ├── docker-compose.yml
│   └── ros_entrypoint.sh
├── ros_ws/
│   ├── src/
│   │   └── tracker_pkg/
│   │       ├── CMakeLists.txt
│   │       ├── package.xml
│   │       ├── include/tracker_pkg/
│   │       ├── src/
│   │       ├── msg/
│   │       ├── config/
│   │       ├── tests/
│   │       └── video/
│   ├── build/
│   ├── install/
│   └── log/
├── make_torchscript/
│   └── makeTorchScript.ipynb
├── analysis/
├──.pre-commit-config.yaml
├── Makefile
└── README.md
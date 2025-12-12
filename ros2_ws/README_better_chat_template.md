# AutoCombat ROS2 Workspace

A ROS2-based autonomous combat robot with real-time vision processing, IMU fusion, motor control, and intelligent combat strategy.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation & Setup](#installation--setup)
- [Workspace Structure](#workspace-structure)
- [Building](#building)
- [Launch Files](#launch-files)
- [Nodes](#nodes)
- [Topics](#topics)
- [Services](#services)
- [Usage](#usage)
- [Development](#development)
- [Troubleshooting](#troubleshooting)

## Prerequisites

- **ROS2** (Jazzy) - Built on Ubuntu 24.04 LTS
- **Python 3.10+**
- **OpenCV** (via `python3-opencv`)
- **RealSense SDK 2.0** and ROS2 wrapper
- **PySerial** (for serial communication with Arduino)
- **NumPy, SciPy** (for computation)

## Installation & Setup

### 1. Setup ROS2 (Ubuntu/VM Recommended)

> **Note**: ROS2 is designed for Linux. While Windows support exists, building and developing on an Ubuntu VM is strongly recommended for better compatibility, easier dependency management, and native performance.

```bash
# Install ROS2 Jazzy
Follow these instructions: (https://docs.ros.org/en/jazzy/Installation.html)

sudo apt update
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop -y
```

### 2. Setup Colcon Build Tool

```bash
sudo apt install python3-colcon-common-extensions
```

### 3. Install Dependencies

```bash
# RealSense dependencies
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description -y

# Additional ROS2 packages
sudo apt install ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-cv-bridge -y

# Python dependencies
sudo apt install python3-pip
pip3 install opencv-python pyserial numpy scipy
```

### 4. Clone and Setup Workspace

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Workspace Structure

```
ros2_ws/
├── README.md                           # This file
├── launch/
│   └── realsense_launch.py            # Camera and sensor bringup
├── src/
│   └── andrews_realsense/             # Main package
│       ├── package.xml                # Package metadata
│       ├── setup.py                   # Python package setup
│       ├── setup.cfg
│       ├── andrews_realsense/         # Source code
│       │   ├── __init__.py
│       │   ├── camera_listener.py     # Realsense camera interface
│       │   ├── imu_listener.py        # IMU data processing
│       │   ├── tennis_ball_detector.py # Vision-based object detection
│       │   ├── orbit_controller.py    # Robot movement controller
│       │   ├── attack_controller.py   # Combat strategy logic
│       │   ├── battle_brain.py        # High-level decision making
│       │   ├── cmd_vel_serial_bridge.py # Serial communication to hardware
│       │   └── realsense_viewer.py    # Debug visualization
│       ├── resource/
│       └── test/
└── build/                              # Build artifacts (generated)
```

## Building

### Build the Workspace

```bash
# From ~/ros2_ws
colcon build

# Build with verbose output
colcon build --event-handlers console_direct+

# Build specific package
colcon build --packages-select andrews_realsense
```

### Clean Build

```bash
colcon clean workspace
colcon build
```

## Launch Files

### Realsense Camera Bringup

Launch the RealSense camera with IMU enabled:

```bash
ros2 launch andrews_realsense realsense_launch.py camera_name:=camera
```

**Parameters:**
- `camera_name` (default: `camera`) - Base namespace for camera topics

**What it does:**
- Initializes RealSense D455/D435 camera
- Enables RGB and Depth streams at 640x480@30fps
- Activates IMU (accelerometer and gyroscope)
- Publishes point clouds
- Aligns depth to RGB frame

## Nodes

All nodes are part of the `andrews_realsense` package. Run with:

```bash
ros2 run andrews_realsense <node_name>
```

### 1. **camera_listener**
**Type:** Subscriber/Publisher  
**Description:** Interface to RealSense camera, processes color and depth frames  
**Subscribes to:**
- `/camera/color/image_raw` - RGB camera feed
- `/camera/depth/image_rect_raw` - Depth map

**Publishes:**
- `/processed_image` - Processed RGB frames
- `/processed_depth` - Processed depth frames

### 2. **imu_listener**
**Type:** Subscriber/Publisher  
**Description:** Processes IMU data and fuses sensor inputs  
**Subscribes to:**
- `/camera/imu` - Raw IMU data

**Publishes:**
- `/imu/filtered` - Filtered IMU readings
- `/imu/orientation` - Estimated orientation

### 3. **tennis_ball_detector**
**Type:** Subscriber/Publisher  
**Description:** Detects tennis balls using color-based vision processing  
**Subscribes to:**
- `/camera/color/image_raw` - RGB feed for detection

**Publishes:**
- `/detection/balls` - Detected ball positions
- `/detection/visualization` - Annotated images

**Parameters:**
- `hsv_lower` - Lower HSV threshold for ball detection
- `hsv_upper` - Upper HSV threshold for ball detection

### 4. **orbit_controller**
**Type:** Subscriber/Publisher  
**Description:** Manages robot movement and circular orbiting behavior  
**Subscribes to:**
- `/target/position` - Target location
- `/robot/odometry` - Current position/velocity

**Publishes:**
- `/cmd_vel` - Velocity commands

### 5. **attack_controller**
**Type:** Subscriber/Publisher  
**Description:** Controls attacking behavior and launcher activation  
**Subscribes to:**
- `/detection/balls` - Ball positions
- `/robot/state` - Current robot state

**Publishes:**
- `/attack/command` - Attack instructions
- `/launcher/fire` - Launch trigger signals

### 6. **battle_brain**
**Type:** Subscriber/Publisher  
**Description:** High-level decision making and strategy coordination  
**Subscribes to:**
- `/detection/balls` - Visible targets
- `/robot/state` - Robot status
- `/opponent/detected` - Enemy detection

**Publishes:**
- `/strategy/action` - Action commands to controllers
- `/battle/status` - Current battle state

### 7. **cmd_vel_serial_bridge**
**Type:** Subscriber  
**Description:** Bridges ROS2 `/cmd_vel` commands to Arduino via serial  
**Subscribes to:**
- `/cmd_vel` - Velocity commands from controllers

**Publishes:**
- `/serial/status` - Connection and transmission status

## Topics

### Sensor Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB camera feed |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | Depth map (aligned) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration |
| `/camera/imu` | `sensor_msgs/Imu` | IMU data (accel + gyro) |

### Perception Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/detection/balls` | `geometry_msgs/PointStamped` | Detected ball positions |
| `/detection/visualization` | `sensor_msgs/Image` | Annotated detection image |

### Control Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Motor velocity commands |
| `/attack/command` | Custom | Attack action commands |
| `/launcher/fire` | `std_msgs/Bool` | Fire trigger signal |

### State Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/robot/state` | Custom | Current robot operational state |
| `/robot/odometry` | `nav_msgs/Odometry` | Position and velocity estimate |
| `/battle/status` | Custom | Current battle state and strategy |

### Diagnostic Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/serial/status` | `std_msgs/String` | Serial connection status |

### Echo Topics for Debugging

```bash
# View camera IMU stream
ros2 topic echo /camera/imu

# View detection results
ros2 topic echo /detection/balls

# View velocity commands
ros2 topic echo /cmd_vel

# View all available topics
ros2 topic list

# Get topic details
ros2 topic info /camera/imu
```

## Services

*To be defined based on your application needs. Common examples:*

- `/robot/enable` - Enable/disable robot
- `/strategy/mode` - Switch battle strategy
- `/calibration/camera` - Trigger camera calibration

## Usage

### Complete Startup Sequence

```bash
# Terminal 1: Launch RealSense camera
ros2 launch andrews_realsense realsense_launch.py

# Terminal 2: Start vision processing
ros2 run andrews_realsense camera_listener
ros2 run andrews_realsense tennis_ball_detector

# Terminal 3: Start IMU processing
ros2 run andrews_realsense imu_listener

# Terminal 4: Start control systems
ros2 run andrews_realsense orbit_controller
ros2 run andrews_realsense attack_controller
ros2 run andrews_realsense battle_brain

# Terminal 5: Start serial bridge to hardware
ros2 run andrews_realsense cmd_vel_serial_bridge

# Terminal 6 (Optional): Visualize camera feed
ros2 run andrews_realsense realsense_viewer
```

### Using a Launch File (Recommended)

Create an `andrews_realsense/launch/full_bringup.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        Node(package='andrews_realsense', executable='camera_listener', output='screen'),
        Node(package='andrews_realsense', executable='tennis_ball_detector', output='screen'),
        Node(package='andrews_realsense', executable='imu_listener', output='screen'),
        Node(package='andrews_realsense', executable='orbit_controller', output='screen'),
        Node(package='andrews_realsense', executable='attack_controller', output='screen'),
        Node(package='andrews_realsense', executable='battle_brain', output='screen'),
        Node(package='andrews_realsense', executable='cmd_vel_serial_bridge', output='screen'),
    ])
```

Then run:

```bash
ros2 launch andrews_realsense full_bringup.py
```

### Monitoring and Debugging

```bash
# List all active nodes
ros2 node list

# Get node information
ros2 node info /camera_listener

# View node graph
rqt_graph

# Monitor resource usage
ros2 topic hz /camera/imu    # Check publishing frequency

# Record data (rosbag)
ros2 bag record /camera/color/image_raw /detection/balls
```

## Development

### Adding a New Node

1. **Create Python file** in `andrews_realsense/`
2. **Implement main() function** with rclpy initialization
3. **Add entry point** in `setup.py`:
   ```python
   'my_node = andrews_realsense.my_node:main',
   ```
4. **Rebuild workspace**:
   ```bash
   colcon build --packages-select andrews_realsense
   source install/setup.bash
   ```

### Code Style

Follow PEP8 standards. Check with:

```bash
colcon test --packages-select andrews_realsense
```

## Troubleshooting

### RealSense Not Detected

```bash
# Check USB connection
lsusb | grep Intel

# Install udev rules
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
./scripts/setup_udev_rules.sh
```

### Import Errors

```bash
# Ensure workspace is sourced
source ~/ros2_ws/install/setup.bash

# Check Python path
python3 -c "import andrews_realsense; print(andrews_realsense.__file__)"
```

### Nodes Not Found

```bash
# Rebuild workspace
colcon build
source install/setup.bash

# Verify entry points
ros2 pkg executables andrews_realsense
```

### Camera Frame Issues

```bash
# Check camera topics are publishing
ros2 topic list | grep camera

# Verify frame rates
ros2 topic hz /camera/color/image_raw
```

### Serial Connection Issues

Ensure:
- Arduino is connected and recognized: `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`
- Correct permissions: `sudo usermod -a -G dialout $USER`
- Correct baud rate configured in `cmd_vel_serial_bridge.py`

## Additional Resources

- [ROS2 Official Documentation](https://docs.ros.org/en/humble/)
- [RealSense ROS2 Wrapper](https://github.com/IntelRealSense/realsense-ros)
- [ROS2 Python Development](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [rqt Tools Guide](http://wiki.ros.org/rqt)

## License

TODO: Add your license information

## Maintainers

- Cody Wang (codywang@berkeley.edu)
- EECS 106A Team

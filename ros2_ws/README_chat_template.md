# AutoCombat --- ROS 2 Workspace

Short description of what this package does.

## Package Structure

    AutoCombat/
    ├── launch/
    │   ├── bringup.launch.py
    │   ├── example_launch.py
    ├── src/
    │   ├── node_a.cpp / .py
    │   ├── node_b.cpp / .py
    ├── config/
    │   ├── params.yaml
    ├── urdf/
    │   ├── robot.urdf.xacro
    └── README.md

## Installation & Build

### 1. Clone

    cd ~/ros2_ws/src
    git clone <repo_url>

### 2. Install Dependencies

    rosdep install --from-paths src --ignore-src -r -y

### 3. Build

    cd ~/ros2_ws
    colcon build --packages-select PACKAGE_NAME

### 4. Source

    source install/setup.bash

## Running the Package

### Bringup Launch

    ros2 launch PACKAGE_NAME bringup.launch.py

### Other Launch Files

    ros2 launch PACKAGE_NAME example_launch.py

## Running Nodes Directly

    ros2 run PACKAGE_NAME node_a
    ros2 run PACKAGE_NAME node_b

## Topics

### Published

  Topic          Type                  Node     Description
  -------------- --------------------- -------- -------------
  /example/out   std_msgs/msg/String   node_a   Example

### Subscribed

  Topic         Type                    Node     Description
  ------------- ----------------------- -------- -------------
  /example/in   sensor_msgs/msg/Image   node_b   Example

## Parameters

  Parameter      Type     Default   Description
  -------------- -------- --------- --------------
  use_sim_time   bool     false     Use sim time
  foo_rate       double   10.0      Loop rate

## Services

  Service   Type                 Node     Description
  --------- -------------------- -------- -------------
  /reset    std_srvs/srv/Empty   node_b   Reset state

## Actions

  Action               Type                        Node     Description
  -------------------- --------------------------- -------- -------------
  /navigate_to_point   custom_action/action/GoTo   node_c   Example

## TF Frames

    base_link → imu_link
    base_link → camera_link

## Testing

    colcon test --packages-select PACKAGE_NAME
    colcon test-result

## To-Do

-   Add real topics
-   Add parameters
-   Add diagrams
-   Add RViz config

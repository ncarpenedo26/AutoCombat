https://docs.ros.org/en/jazzy/Installation.html

ROS2 Jazzy

Realsense Camera Startup Commands:
Command to start realsense camera camera:
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true  align_depth.enable:=true

Command to start realsense camera IMU:
ros2 launch realsense2_camera rs_launch.py enable_accel:=true enable_gyro:=true unite_imu_method:=1


EKF + VIO + NAV2 ==> Trajectory Planning!

1. Launch RealSense with Depth + IMU (Synced)

You must enable depth, IMU, and IMU unification, and TF publication:

Command to do all for EKF!: (Realsense w/Depth + IMU, Synced)
ros2 launch realsense2_camera rs_launch.py \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=1 \
    publish_odom_tf:=true \
    publish_tf:=true \
    enable_depth:=true

        This gives you:

        Depth:
        /camera/color/image_raw
        /camera/aligned_depth_to_color/image_raw
        /camera/color/camera_info

        IMU:
        /camera/imu       (merged accel+gyro)


2. Launch RTAB-Map RGB-D Odometry

RTAB-Map can produce a /odom topic that EKF can fuse.

ros2 launch rtabmap_launch rtabmap.launch.py \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    imu_topic:=/camera/imu \
    approx_sync:=false

    # NOTE: Why approx_sync:=false?
    # Because RealSense timestamps IMU & camera well enough after enabling IMU unification, so exact sync is ideal for VIO performance.

RTAB-Map will output:
/odom            (nav_msgs/Odometry)     <--- EKF input


3. Feed IMU + Odometry into EKF

Now your EKF gets:

Sensor 1 — RealSense IMU:
imu0: /camera/imu

Sensor 2 — RTAB-Map visual odometry:
odom0: /odom

https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html



⭐ 4. Diagram – What You Are Building
        RealSense
     (depth + imu)
            |
            | RGB + Depth + IMU
            v
     RTAB-Map RGBD Odometry
            |
            |   /odom
            v
     robot_localization EKF
            |
            |   /odom -> /tf (base_link)
            v
        Navigation stack (optional)



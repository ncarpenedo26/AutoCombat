from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # --- RealSense ---
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'enable_depth': 'true',
            'enable_accel': 'true',
            'enable_gyro': 'true',
            'unite_imu_method': '1',
            'publish_tf': 'true',
            'publish_odom_tf': 'true'
        }.items()
    )

    # --- RTAB-Map ---
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py'
            )
        ),
        launch_arguments={
            'rgb_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'imu_topic': '/camera/imu',
            'approx_sync': 'false'
        }.items()
    )

    # --- EKF ---
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('your_package'),
            'config',
            'ekf.yaml'
        )]
    )

    return LaunchDescription([realsense, rtabmap, ekf])

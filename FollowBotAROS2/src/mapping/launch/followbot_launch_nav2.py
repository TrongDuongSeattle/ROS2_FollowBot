from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # path to the SLAM toolbox launch file
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    # path to YAML config
    slam_config_file = os.path.join(
        get_package_share_directory('mapping'),
        'config',
        'mapper_params_online_async.yaml'
    )

    # path to Nav2 launch file
    nav2_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    # path to Nav2 config file
    nav2_config_file = os.path.join(
        get_package_share_directory('mapping'),
        'config',
        'nav2_params.yaml'
    )

    return LaunchDescription([
        // # Arduino IMU Serial Node
        Node(
            package='arduino_serial',
            executable='imu_serial_node',
            name='imu_serial_node',
            output='screen'
        ),

        # LIDAR Preprocessor Node
        Node(
            package='mapping',
            executable='mapping_node',
            name='lidar_preprocessor_node',
            output='screen'
        ),

        # custom Odometry Node
        Node(
            package='mapping',
            executable='odometry_node',
            name='odometry_node',
            output='screen'
        ),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_path),
            launch_arguments={'params_file': slam_config_file}.items()
        )

        # Navigation2 Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={'params_file': nav2_config_file}.items()
        )
    ])

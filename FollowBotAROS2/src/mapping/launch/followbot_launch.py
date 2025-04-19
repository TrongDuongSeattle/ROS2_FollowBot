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

    return LaunchDescription([
        # SLAM Toolbox Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_path),
            launch_arguments={'params_file': slam_config_file}.items()
        )
    ])

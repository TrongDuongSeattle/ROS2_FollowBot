# Copyright 2018 Open Source Robotics Foundation, Inc.
# Modified by FollowBot Team 2025

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory("mapping"),
            "config",
            "dual_ekf_navsat_params.yaml"
        ),
        description='Full path to the ROS2 parameters file to use'
    )

    rob_loc_params_file = LaunchConfiguration('params_file')
        
    return LaunchDescription([
        declare_params_file_cmd,

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[rob_loc_params_file, {"use_sim_time": False}],
            remappings=[("odometry/filtered", "odometry/local")],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            output="screen",
            parameters=[rob_loc_params_file, {"use_sim_time": False}],
            remappings=[("odometry/filtered", "odometry/global")],
        ),
        Node (
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_trasform",
            output="screen",
            parameters=[rob_loc_params_file, {"use_sim_time": False}],
            remappings=[
                ("imu/data", "imu/data"),
                ("gps/fix", "gps/fix"),
                ("gps/filtered", "gps/filtered"),
                ("odometry/gps", "odometry/gps"),
                ("odometry/filtered", "odometry/global"),
            ],
        ),
    ])

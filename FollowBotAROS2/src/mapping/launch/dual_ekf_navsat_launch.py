# Copyright 2018 Open Source Robotics Foundation, Inc.
# Modified by Joseph Hoang 2025

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    rob_loc_params_file = os.path.join(
        get_package_share_directory("mapping"),
        "config",
        "dual_ekf_navsat_parms.yaml"
    )

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen"
            parameters=[rob_loc_params_file, {"use_sim_time": True}]
            remappings=[()],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            output="screen",
            parameters=[params_file, {"use_sim_time": True}],
            remappings=[("odometry/filtered", "odometry/global")],
        ),
        Node (
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[params_file, {"use_sim_time": True}],
            remappings=[
                ("imu/data", "imu/data"),
                ("gps/fix", "gps/fix"),
                ("gps/filtered", "gps/filtered"),
                ("odometry/gps", "odometry/gps"),
                ("odometry/filtered", "odometry/global"),
            ],
        ),
    ])

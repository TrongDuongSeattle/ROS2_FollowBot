from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # LIDAR and RViz2 config launch path
    rviz2_config = os.path.join(
        get_package_share_directory('ldlidar_sl_ros2'),
        'rviz2',
        'ldlidar.rviz'
    )

    # SLAM Tolbox launch file path
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    # path to the YAML configuration file for SLAM
    slam_config_file = os.path.join(
        get_package_share_directory('mapping'),
        'config',
        'mapper_params_online_async.yaml'
    )

    # path to dual_ekf_navsat launch file
    dual_ekf_launch_path = os.path.join(
        get_package_share_directory('mapping'),
        'launch',
        'dual_ekf_navsat.launch.py'
    )

    # path to the EKF config YAML
    ekf_config_path = os.path.join(
        get_package_share_directory('mapping'),
        'config',
        'dual_ekf_navsat.yaml'
    )



    ld = LaunchDescription([
        # Odometry Node (robot_localization with IMU)
        #http://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html#publish-tf
        #Node(
        #    package='robot_localization',  # Using robot_localization for IMU-based odometry
        #    executable='ekf_node',  # Extended Kalman Filter node for sensor fusion
        #    name='ekf_filter_node',
        #    output='screen',
        #    parameters=[{
        #        'odom_frame': 'odom',  # Name of the odometry frame
        #        'base_frame': 'base_link',  # Robot's base frame
        #        'imu_topic': '/imu/data',  # IMU data topic
        #        'use_odometry': True,  
        #    }]
        #),

        # static transform from base_link to base_laser, arguments are based off physical mounting
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_laser",
            arguments=["0.1", "0.0", "0.2", "0.0", "0.0", "0.0", "base_link", "base_laser"],
            parameters=[{"use_sim_time": False}]
        ),

        # RViz2 node to visualize the LiDAR data
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_show_ld14',
            arguments=['-d', rviz2_config],
            output='screen'
        ),

        # include LDLidar launch file
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ldlidar_sl_ros2'),
                    'launch',
                    'ld14p.launch.py'  # Path to the specific launch file
                )
            )
        ),

        # launch EKF and navsat nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dual_ekf_launch_path),
            launch_arguments={'use_sim_time': 'false'}.items()
        ),

        # launch SLAM with custom config
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_path),
            launch_arguments={
                'params_file': slam_config_file,
                'slam_params_file': slam_config_file,
                'use_sim_time': 'false',
                'publish_map': 'true'
            }.items()  # Correctly pass dictionary items
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dual_ekf_launch_path),
            launch_arguments={'params_file': ekf_config_path}.items()
        ),

        # log that the launch is happening
        LogInfo(
            msg="SLAM Toolbox, LiDAR have been launched successfully!"
        ),
    ])

    return ld

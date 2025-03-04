from launch import LaunchDescription
from launch.ros.actions import Node

def generate_launch_description():
    return LaunchDescription[(
        Node(
            package='mapping',
            executable='mapping_node',
            name='lidar_preprocessor_node',
            output='screen'
        ),
        Node(
            package='mapping',
            executable='odometry_node',
            name='odemtry_node',
            output='screen'
        )
    )]

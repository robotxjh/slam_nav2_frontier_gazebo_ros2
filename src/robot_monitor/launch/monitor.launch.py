import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_monitor'), 'config', 'monitor.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true in simulation'),
        Node(
            package='robot_monitor',
            executable='monitor',
            name='robot_monitor',
            output='screen',
            parameters=[config,
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
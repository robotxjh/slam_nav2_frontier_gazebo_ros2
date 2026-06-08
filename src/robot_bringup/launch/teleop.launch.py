from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    """独立启动键盘控制节点，用于在仿真已运行时单独启动 teleop"""

    use_xterm = LaunchConfiguration('use_xterm', default='true')

    declare_use_xterm = DeclareLaunchArgument(
        'use_xterm',
        default_value='true',
        description='是否在新终端窗口中运行键盘控制 (需要 xterm)'
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        condition=IfCondition(use_xterm),
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    teleop_node_no_xterm = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        condition=UnlessCondition(use_xterm),
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    return LaunchDescription([
        declare_use_xterm,
        teleop_node,
        teleop_node_no_xterm,
    ])

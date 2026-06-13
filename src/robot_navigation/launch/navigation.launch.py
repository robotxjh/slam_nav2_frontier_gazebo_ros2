import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    nav2_params_path = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config', 'nav2_config.yaml'
    )

    map_yaml_path = os.path.join(
        get_package_share_directory('robot_navigation'),
        'maps', 'map.yaml'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_path,
        description='地图yaml文件路径'
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py'
            )
        ),
        launch_arguments=[
            ('map', LaunchConfiguration('map')),
            ('params_file', nav2_params_path),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('autostart', 'True')
        ]
    )

    # lifecycle_manager_navigation = Node(
    # package='nav2_lifecycle_manager',
    # executable='lifecycle_manager',
    # name='lifecycle_manager_navigation',
    # output='screen',
    # parameters=[{
    #     'use_sim_time': use_sim_time,
    #     'autostart': True,
    #     'node_names': [
    #         'controller_server',
    #         'smoother_server',
    #         'planner_server',
    #         'behavior_server',
    #         'bt_navigator',
    #         'waypoint_follower',
    #         'velocity_smoother'
    #         ]
    #     }]
    # )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        nav2_bringup,
        # lifecycle_manager_navigation
    
    ])
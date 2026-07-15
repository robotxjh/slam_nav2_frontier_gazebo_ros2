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
        default_value='False',
        description='使用仿真时间'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_path,
        description='地图yaml文件路径'
    )

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='是否启用SLAM建图模式（True时nav2走slam分支，不启动amcl）'
    )
    
    map = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map,
            'params_file': nav2_params_path,
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'slam': slam,
        }.items()
    )

    
    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_slam,
        nav2_bringup,
        # lifecycle_manager_navigation
        
    ])
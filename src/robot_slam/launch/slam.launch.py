from launch import LaunchDescription
from launch_ros.actions import Node
import os
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    slam_params = os.path.join(
        get_package_share_directory('robot_slam'),
        'config',
        'slam_params.yaml'
    )
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',
        output='screen',
        parameters=[slam_params,
                    {'use_sim_time': use_sim_time,
                     'odom_frame': 'odom'}]
        
        )
    
    # 延迟10秒自动激活SLAM
    activate_slam = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', 
                    '/async_slam_toolbox_node', 'configure'],
                output='screen'
            )
        ]
    )

    activate_slam2 = TimerAction(
        period=13.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set',
                    '/async_slam_toolbox_node', 'activate'],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([declare_sim_time,slam_toolbox_node, activate_slam, activate_slam2])
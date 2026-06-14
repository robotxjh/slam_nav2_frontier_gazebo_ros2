from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_path
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument,TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('robot_description'),
                             'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path('robot_description'),
                                    'rviz', 'rviz_config.rviz')
    worlds_path = os.path.join(get_package_share_path('robot_gazebo'),
                               'worlds', 'corridor.world')
    bridge_config_path = os.path.join(get_package_share_path('robot_gazebo'),
                                     'config', 'bridge_config.yaml')
    # ekf_path = os.path.join(get_package_share_path('robot_navigation'),
    #                         'config', 'ekf.yaml')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    declare_slam = DeclareLaunchArgument(
        'slam',
         default_value='True',
        description='是否启动 slam'
    )

    declare_navigation = DeclareLaunchArgument(
        'navigation',
        default_value='True',
        description='是否启动 navigation'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='是否use_sim_time'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    navigation = LaunchConfiguration('navigation')

    slam_launch_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path('robot_slam'),
                'launch',
                'slam.launch.py'
            )
        ),
        condition=IfCondition(slam)
    )

    navigation_launch_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path('robot_navigation'),
                'launch',
                'navigation.launch.py'
            )
        ),
        condition=IfCondition(navigation)
    )
    
    delayed_navigation = TimerAction(
        period=3.0,
        actions=[navigation_launch_node]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": use_sim_time,
                     "publish_frequency": 30.0}]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config_path],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    ros_gz_bridge_node = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    name="ros_gz_bridge_node",
    output="screen",
    parameters=[{
        "use_sim_time": use_sim_time,
        "config_file": bridge_config_path
    }]
)
    

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(get_package_share_path('ros_gz_sim'),
        'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', '-r -v 4 ' + worlds_path)]
    )

    # 在 Gazebo 中生成机器人模型
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[             #此处不能加name 参数，否则导致产生命名空间，tf树不完整，导致机器人不能转弯
            '-world', 'corridor',
            '-topic', 'robot_description',  # 从 robot_state_publisher 获取模型数据
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'  
        ],
        output='screen'
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{'use_sim_time': use_sim_time}]  
    )

    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_path,
    #                 {'use_sim_time': use_sim_time}]
    # )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_slam,
        declare_navigation,
    
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        ros_gz_bridge_node,
        gazebo,
        spawn_robot,
        teleop_node,
        slam_launch_node,
        delayed_navigation,
        # ekf_node
        
    ])


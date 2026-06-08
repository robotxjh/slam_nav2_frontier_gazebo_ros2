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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('robot_description'),
                             'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path('robot_description'),
                                    'rviz', 'urdf_config.rviz')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    slam_launch_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path('robot_slam'),
                         'launch', 'slam.launch.py')
                        ),
                         launch_arguments={'use_sim_time': 'true'}.items()
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": True}]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config_path]
    )

    ros_gz_bridge_node = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    name="ros_gz_bridge_node",
    output="screen",
    arguments=[
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',                    # ROS→Gazebo ✅
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',                      # Gazebo→ROS ✅
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',                 # Gazebo→ROS ✅
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',                         # Gazebo→ROS ✅
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',                      # Gazebo→ROS ✅
        '/world/empty/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',  # Gazebo→ROS ✅
    ]
)

    headless = LaunchConfiguration('headless', default='false')
    teleop = LaunchConfiguration('teleop', default='true')

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='是否通过headless模式运行gazebo (no GUI)'
    )

    declare_teleop = DeclareLaunchArgument(
        'teleop',
        default_value='true',
        description='是否启动键盘控制节点'
    )

    # 启动 Gazebo 仿真环境并加载空场景
    gazebo_server = IncludeLaunchDescription(   # 包含 Gazebo 启动文件
        PythonLaunchDescriptionSource(   # 指定 Gazebo 启动文件路径
            os.path.join(get_package_share_path(
                'ros_gz_sim'), 
                'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', '-s -r -v 4 empty.sdf')]
    )

    gazebo_client = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_path('ros_gz_sim'),
        'launch', 'gz_sim.launch.py')),
    launch_arguments=[('gz_args', '-g')],
    condition=UnlessCondition(headless)
)

    # 在 Gazebo 中生成机器人模型
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[             #此处不能加name 参数，否则导致产生命名空间，tf树不完整，导致机器人不能转弯
            '-topic', 'robot_description',  # 从 robot_state_publisher 获取模型数据
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'  
        ],
        output='screen'
    )

    # 键盘控制节点
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # 在新的终端窗口中运行 避免阻塞
        condition=IfCondition(teleop)
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        ros_gz_bridge_node,
        gazebo_server,
        gazebo_client,
        spawn_robot,
        teleop_node,
        slam_launch_node,
        declare_headless,
        declare_teleop

    ])


markdown
# 🤖 ROS2 移动机器人自主建图与导航系统

基于 **ROS2 Jazzy + Gazebo Harmonic** 开发的差速驱动移动机器人仿真项目，
实现了机器人建模、仿真环境搭建、多传感器集成、
SLAM 自主建图、Nav2 自主导航以及 Frontier 自主探索等完整功能。

---

## 📺 演示视频

> 待上传

---

## ✨ 功能特性

- ✅ 自定义四轮差速机器人 URDF 建模
- ✅ Gazebo Harmonic 仿真环境
- ✅ 激光雷达 / 相机 / IMU 多传感器集成
- ✅ SLAM Toolbox 实时2D建图
- ✅ Nav2 完整自主导航栈
- ✅ AMCL 粒子滤波定位
- ✅ robot_localization EKF 传感器融合
- ✅ 自实现 Frontier 自主探索算法
- ✅ 支持多种启动模式（建图/导航/探索/无头模式）

---

## 🏗️ 系统架构

### 工程结构

```
my_robot_ws/src/
├── robot_description    # 机器人模型(URDF/Xacro)
├── robot_gazebo         # 仿真环境和话题桥接
├── robot_slam           # SLAM建图
├── robot_navigation     # Nav2导航和Frontier探索
└── robot_bringup        # 系统总入口
```

### TF树结构

```
map
 └── odom
      └── base_footprint
           └── base_link
                ├── lidar_link      # 激光雷达
                ├── camera_link     # RGB相机
                ├── imu_link        # IMU
                ├── left_rear_wheel
                ├── right_rear_wheel
                ├── left_front_wheel
                └── right_front_wheel
```

### 数据流

**建图模式：**
```
Gazebo激光雷达 → /scan → SLAM Toolbox → /map
Gazebo差速驱动 → /odom/unfiltered → EKF(+/imu) → /odom → TF
键盘/Frontier → /cmd_vel → 机器人运动
```

**导航模式：**
```
map_server → /map
/scan + /map → AMCL → map→odom TF
2D Nav Goal → bt_navigator → planner → controller → /cmd_vel
```

---

## 🔧 环境依赖

- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic

### 安装依赖

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-smac-planner \
  ros-jazzy-robot-localization \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-teleop-twist-keyboard \
  xterm
```

---

## 🚀 快速开始

### 1. 克隆并编译

```bash
mkdir -p ~/my_robot_ws/src
cd ~/my_robot_ws/src
git clone git@github.com:robotxjh/my_robot.git
cd ~/my_robot_ws
colcon build
source install/setup.bash
```

建议加入 bashrc：

```bash
echo "source ~/my_robot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 2. 建图模式

**启动仿真+SLAM建图：**

```bash
ros2 launch robot_bringup robot.launch.py slam:=True navigation:=False explorer:=False
```


**键盘控制建图：**

xterm 窗口会自动弹出，使用以下按键控制：

| 按键 | 动作 |
|------|------|
| `i` | 前进 |
| `,` | 后退 |
| `j` | 左转 |
| `l` | 右转 |
| `k` | 停止 |
| `q/z` | 加速/减速 |

**保存地图：**

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/my_robot_ws/src/robot_navigation/maps/map
```

---

### 3. 自主探索建图模式

不需要手动控制，机器人自动探索未知区域完成建图：

```bash
ros2 launch robot_bringup robot.launch.py \
  slam:=True \
  navigation:=True \
  explore:=True
```

---

### 4. 导航模式

确保已有保存好的地图，然后：

```bash
# 终端1：启动机器人（关闭SLAM）
ros2 launch robot_bringup robot.launch.py \
  slam:=False \
  navigation:=True


在 RViz 里：
1. 点击 **2D Pose Estimate** 设置机器人初始位置
2. 点击 **2D Nav Goal** 设置导航目标点
3. 机器人自动规划路径并导航

---

### 5. 无头模式（节省性能）

```bash
ros2 launch robot_bringup robot.launch.py \
  slam:=True \
  headless:=True
```

---

## 🎛️ 启动参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `slam` | `True` | 是否启动SLAM建图 |
| `navigation` | `False` | 是否启动Nav2导航 |
| `explore` | `False` | 是否启动Frontier自主探索 |
| `headless` | `False` | 是否关闭Gazebo GUI |

---

## 📦 各功能包说明

### robot_description

机器人物理模型描述包：

```
robot_description/
└── urdf/
    ├── my_robot.urdf.xacro    # 主文件
    ├── base.xacro             # 底盘和轮子
    ├── lidar.xacro            # 激光雷达
    ├── camera.xacro           # RGB相机
    ├── imu.xacro              # IMU
    ├── gazebo_control.xacro   # Gazebo插件
    └── common_macro.xacro     # 通用宏定义
```

### robot_gazebo

仿真环境管理包：

```
robot_gazebo/
├── worlds/
│   ├── corridor.world         # 走廊场景
│   ├── maze.world             # 迷宫场景
└── config/
    └── bridge_config.yaml     # 话题桥接配置
```

### robot_slam

SLAM建图包：

- 使用 **SLAM Toolbox** 异步在线建图模式
- 坐标系：`map → odom → base_footprint`
- 支持地图保存和加载

### robot_navigation

导航包：

- **map_server**：加载已有地图
- **AMCL**：自适应蒙特卡洛定位
- **NavFn**：基于A*的全局路径规划
- **RegulatedPurePursuit**：局部路径跟踪控制器
- **bt_navigator**：行为树导航逻辑
- **Frontier Explorer**：自实现的自主探索算法

**Frontier探索算法原理：**

```
1. 订阅/map，检测已知空闲区域和未知区域的边界（frontier）
2. 通过TF获取机器人当前位置
3. 计算到每个frontier的距离，选择最近的
4. 调用Nav2 NavigateToPose Action导航过去
5. 记录失败的frontier，避免重复尝试
6. 到达后继续寻找下一个frontier，直到探索完成
```

### robot_bringup

系统总入口：

- 一个launch文件启动全部系统
- 支持参数灵活控制启动模式
- 包含RViz配置文件

---

## 🔍 常用调试命令

```bash
# 查看TF树
ros2 run tf2_tools view_frames

# 检查话题频率
ros2 topic hz /scan
ros2 topic hz /joint_states

# 查看节点列表
ros2 node list

# 查看话题列表
ros2 topic list

# 检查Nav2节点状态
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator

# 手动发送速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}}" --once
```

---

## 🛠️ 技术栈

| 类别 | 技术 |
|------|------|
| 操作系统 | Ubuntu 24.04 |
| 机器人框架 | ROS2 Jazzy |
| 仿真器 | Gazebo Harmonic |
| 建图 | SLAM Toolbox |
| 导航 | Nav2 |
| 传感器融合 | robot_localization (EKF) |
| 机器人描述 | URDF/Xacro |
| 编程语言 | Python / XML / YAML |

---

## 📝 项目总结

本项目在开发过程中解决了多个工程实际问题：

- Gazebo Harmonic 与旧版插件的兼容性迁移
- ros_gz_bridge QoS 不匹配导致地图无法显示
- lifecycle 节点重复导致 Nav2 无法激活
- EKF 与差速驱动插件 TF 冲突
- joint_states 话题名不一致导致频率异常
- Frontier 探索机器人卡死问题的算法改进
- 小车在导航时候抖动

---

## 📄 License

MIT License

---

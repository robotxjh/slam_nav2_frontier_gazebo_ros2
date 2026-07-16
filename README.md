# 🤖 ROS2 移动机器人自主建图与导航系统（真机部署分支）

基于 **ROS2 Humble** 的差速驱动移动机器人真机部署项目，将原本运行在 **ROS2 Jazzy + Gazebo Harmonic** 仿真环境下的机器人系统，完整迁移适配至真实 **FishBot** 硬件平台，实现了机器人建模、多传感器集成、SLAM 自主建图、Nav2 自主导航以及 Frontier 自主探索等完整功能在真实硬件上的落地运行。

> 本分支是 [main 分支](../../tree/main)（仿真版本）的真机部署延伸。核心算法与系统架构保持一致，但针对真实硬件环境、跨 ROS2 发行版差异，进行了大量底层适配与问题排查，详见下方「仿真到真机：问题与解决方案」章节。

---

## ✨ 功能特性

- ✅ Docker 三容器协同部署，环境隔离、可复现
- ✅ 建图/导航模式一键切换，各模块可独立开关
- ✅ 自定义差速机器人 URDF 建模，尺寸适配真实 FishBot 硬件
- ✅ 真实激光雷达 / IMU 多传感器集成
- ✅ AMCL 粒子滤波定位
- ✅ robot_localization EKF 传感器融合
- ✅ SLAM Toolbox 真实环境实时2D建图
- ✅ Nav2 完整自主导航栈（全局规划 + 局部避障 + 行为树调度）
- ✅ 自实现 Frontier 自主探索算法

---

## 🏗️ 工程结构

```
src/
├── robot_bringup          # 系统总入口
│   └── launch/
│       ├── robot.launch.py         # 仿真入口（遗留）
│       └── robot_real.launch.py    # 真机部署入口
│
├── robot_description       # 机器人模型(URDF/Xacro)，已适配真机尺寸
│   └── urdf/
│       ├── my_robot.urdf.xacro     # 主文件
│       ├── base.xacro              # 底盘和轮子
│       ├── lidar.xacro             # 激光雷达
│       ├── camera.xacro            # RGB相机
│       ├── imu.xacro               # IMU
│       └── common_macro.xacro      # 通用惯性/几何宏定义
│
├── robot_gazebo            # 仿真环境遗留，真机部署不使用
│
├── robot_navigation         # Nav2导航与Frontier探索
│   ├── config/
│   │   ├── nav2_config.yaml        # Nav2全部组件参数
│   │   └── ekf.yaml                # EKF传感器融合参数
│   ├── launch/
│   │   └── navigation.launch.py
│   ├── maps/                       # 已保存的地图文件
│   └── robot_navigation/
│       └── frontier_explorer.py    # 自主探索节点
│
└── robot_slam                # SLAM建图
    ├── config/
    │   └── slam_params.yaml
    └── launch/
        └── slam.launch.py
```

---

## 🔗 TF树结构

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

三段变换分别由三个互不感知彼此的节点独立发布，靠坐标系名称的精确匹配，由 TF2 自动拼接成完整的树：

| 变换 | 发布者 | 类型 |
|------|--------|------|
| `map → odom` | `async_slam_toolbox_node` | 动态（建图阶段），实时扫描匹配修正 |
| `odom → base_footprint` | `ekf_node` | 动态，融合轮式里程计+IMU |
| `base_footprint → base_link → 各传感器/轮子` | `robot_state_publisher` | 静态，来自URDF固定安装关系 |

---

## 🔄 数据流

**建图模式：**
```
真实雷达(TCP) → fishbot_laser驱动容器 → /scan → SLAM Toolbox → /map
底盘固件 → micro-ros-agent容器 → /odom → relay → /odom/unfiltered → EKF(+/imu) → /odom → TF
Frontier自主探索 → Nav2(controller/planner) → /cmd_vel → micro-ros-agent → 底盘电机
```

**导航模式：**
```
map_server(已存地图) → /map
/scan + /map → AMCL → map→odom TF
RViz 2D Nav Goal → bt_navigator → planner → controller → /cmd_vel
```

**Frontier 自主探索逻辑：**
```
1. 订阅 /map，识别已知空闲区域与未知区域的边界（frontier）
2. 通过 TF 获取机器人当前位置
3. 计算到各 frontier 的距离，选取最近点作为目标
4. 调用 Nav2 NavigateToPose Action 导航过去
5. 记录导航失败的 frontier，避免重复尝试
6. 循环上述过程，直至无可探索边界为止
```

---

## 🔧 部署架构

真机部署采用三容器协同，各自独立职责、通过 ROS2 DDS（`--net=host`）互相发现通信：

| 容器 | 作用 | 镜像来源 |
|------|------|----------|
| 主开发容器 | 编译运行 SLAM / Nav2 / Frontier 全部逻辑 | 本项目自建镜像 |
| micro-ros-agent | 底盘固件通讯（`/cmd_vel` 下发，`/odom`、`/imu` 上报） | fishros 官方镜像 |
| fishbot_laser | 激光雷达驱动，TCP协议解析原始点云 | fishros 官方镜像 |

### 拉取本项目预构建镜像

```bash
docker pull crpi-a5bkhicxgm0fmery.cn-shanghai.personal.cr.aliyuncs.com/aman-robot/robot_humble_deploy:latest
```

---

## 🚀 快速开始

### 1. 克隆代码

```bash
git clone -b fishbot-humble-deploy https://github.com/robotxjh/slam_nav2_frontier_gazebo_ros2.git my_robot_ws
```

### 2. 启动三个容器

**终端1：底盘通讯**
```bash
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host \
  registry.cn-hangzhou.aliyuncs.com/fishros/micro-ros-agent:humble udp4 --port 8888 -v4
```

**终端2：雷达驱动**
```bash
xhost + && sudo docker run -it --rm -p 8889:8889 -p 8889:8889/udp -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/snd -e DISPLAY=unix$DISPLAY \
  registry.cn-hangzhou.aliyuncs.com/fishros/fishbot_laser
```
选择1、驱动雷达  -----〉1、无线  -----〉默认端口

**终端3：主开发容器**
```bash
xhost +local:docker
# 请在执行过 git clone 的同一目录下运行以下命令
sudo docker run -it --net=host --name slam_nav2_dev \
  -v $(pwd)/my_robot_ws:/workspace \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  crpi-a5bkhicxgm0fmery.cn-shanghai.personal.cr.aliyuncs.com/aman-robot/robot_humble_deploy:latest bash
```

### 3. 容器内编译

```bash
source /opt/ros/humble/setup.bash
cd /workspace
colcon build
source install/setup.bash
```

---

### 4. 自主探索建图

```bash
ros2 launch robot_bringup robot_real.launch.py 
```

性能有限的设备建议关闭 RViz 减轻负载：追加 `rviz:=False`。

**手动点动控制（备用，用于探索卡滞时人工干预）：**

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

**保存地图：**

```bash
ros2 run nav2_map_server map_saver_cli -f /workspace/src/robot_navigation/maps/map
colcon build --packages-select robot_navigation
source install/setup.bash
```

---

### 5. 导航模式

确保已有保存好的地图：

```bash
ros2 launch robot_bringup robot_real.launch.py \
  slam:=False navigation:=True explorer:=False
```

在 RViz 里：
1. 点击 **2D Pose Estimate** 设置机器人初始位置
2. 点击 **2D Nav Goal** 设置导航目标点
3. 机器人自动规划路径并导航

---

## 🎛️ 启动参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `slam` | `True` | 是否启动SLAM建图 |
| `navigation` | `True` | 是否启动Nav2导航栈 |
| `explorer` | `True` | 是否启动Frontier自主探索 |
| `rviz` | `True` | 是否启动RViz，低性能设备建图时建议关闭 |

---

## 🔍 仿真到真机：踩坑与解决方案

这是本次部署投入精力最多、也最具参考价值的部分。跨越 ROS2 Jazzy 仿真到 ROS2 Humble 真机的迁移，暴露出的问题不止版本号不同这么简单，涉及 launch 参数传递机制、TF 系统设计原理、多模块协同架构等多个层面。

### 1. Nav2 插件命名格式跨版本不兼容

**现象**：`planner_server`、`behavior_server` 启动时报插件加载失败（`class does not exist`）。

**原因**：不同 ROS2 发行版下，同一插件包在 `pluginlib` 中注册的名称格式存在差异（如 `包名::类名` vs `包名/类名`），且各插件包之间格式并不统一，无法批量替换。

**解决**：逐个查询报错信息中列出的 "Declared types"，或直接读取对应插件包的 `plugins.xml`，确认真实注册名后逐一修正配置文件，不做批量假设性修改。

### 2. 里程计话题命名冲突

**现象**：`ekf_filter_node` 融合输出与底盘固件原始数据同时发布到 `/odom`，下游订阅者收到两路交替、不一致的数据。

**原因**：仿真环境下由 `ros_gz_bridge` 承担的话题重映射工作，在真机环境下不存在，固件直接把原始里程计发布到了 `/odom`，与 EKF 期望的输入命名约定冲突。

**解决**：新增 `topic_tools relay` 节点，将固件原始 `/odom` 转发为 `/odom/unfiltered` 供 EKF 订阅，EKF 融合结果继续发布至 `/odom`，避免命名冲突与潜在自循环。

### 3. TF 树因坐标系命名不一致而断裂

**现象**：SLAM 持续报 `Message Filter dropping message`，激光数据无法被使用，`map` 坐标系始终无法建立。

**原因**：雷达驱动发布的 `LaserScan` 消息 `frame_id` 为 `laser_frame`，URDF 中定义的雷达坐标系为 `lidar_link`。TF 系统依赖坐标系名称的精确字符串匹配来建立空间关系，两条独立正常工作的数据通路（TF树发布、激光数据发布）因命名不一致而无法关联，数据持续排队直至溢出丢弃。

**解决**：新增 `static_transform_publisher` 节点，桥接 `lidar_link → laser_frame`。（该工具在 Humble 下命令行参数格式由早期的位置参数改为具名参数 `--frame-id` / `--child-frame-id`，是另一处版本差异。）

### 4. SLAM 与 AMCL 的架构性互斥

**现象**：建图阶段机器人频繁触发原地旋转恢复行为、路径规划失败，TF 查询报时间戳越界错误。

**原因**：SLAM（假设地图未知，边探索边建图）与 AMCL（假设地图已知固定，仅做定位）都会尝试发布 `map → odom`，二者同时运行会导致 TF 数据交替覆盖、时间戳错乱，是架构层面的根本性冲突，而非配置错误。

**解决**：明确两阶段职责边界——建图阶段仅由 `async_slam_toolbox_node` 独立发布定位与建图数据，`nav2_bringup` 层不转发额外的 slam 参数，使其默认分支（AMCL）在建图阶段因未接收到初始位姿而保持静默、不产生实际干扰；导航阶段关闭 SLAM，由 AMCL 接管定位，需手动通过 RViz 设置初始位姿。

### 5. Nav2 参数文件缺失节点占位声明导致启动失败

**现象**：`map_server` 反复报 `parameter 'yaml_filename' is not initialized`，但 launch 参数传递链路经反复核实完全正确。

**原因**：`nav2_config.yaml` 中未显式声明 `map_server` 节点的参数命名空间。在当前 ROS2/Nav2 版本组合下，若 params_file 中完全不存在目标节点的参数命名空间，`launch_arguments` 传入的参数无法被正确合并进节点的参数服务，即便传递链路本身完全正确。这是 ROS2 launch_ros 库中一个已知的实现缺陷（详见 ros2/launch_ros#460）——在通过 Composable Node（组合节点）方式加载参数文件时，其内部负责将参数文件内容分发给目标节点的匹配逻辑存在缺陷，若 params_file 中完全不存在目标节点的命名空间作为匹配锚点，launch_arguments 传入的参数将无法被正确合并进最终发送给节点的请求中，即便传递链路本身完全正确。

**解决**：通过手动隔离测试逐层排除（独立测试 `localization_launch.py`、`bringup_launch.py`、封装层）定位到根因后，在 `nav2_config.yaml` 中显式添加空的 `map_server` 参数占位声明，确保参数合并机制正确生效。

### 6. URDF 坐标系命名体系意外混用

**现象**：一次仅针对机器人尺寸的修改后，TF 树整体断裂，`robot_state_publisher` 输出的坐标系名称与项目原有命名完全不符。

**原因**：调整过程中参考了第三方示例文件的写法，无意间引入了另一套坐标系命名习惯（如 `base_link`/`caster_link` 替代了原有的 `base_footprint`/`caster_wheel`），导致与下游所有配置文件（EKF、SLAM、Nav2 参数）中写死的坐标系名称全部失配。

**解决**：使用 Git 精确恢复至已验证可用的历史版本，此后严格遵循"仅修改数值参数，不变更坐标系命名"的原则进行迭代。

### 7. 真实硬件性能瓶颈定位

**现象**：自主建图过程中机器人频繁陷入原地旋转、卡顿，最初误判为 SLAM 图优化计算量随地图增大而超出负荷。

**排查过程**：通过 `htop` 逐步定位，系统负载（Load Average）长期超过 CPU 核心数，主要来自两方面叠加：雷达驱动（Python 实现，通过 TCP 持续解码原始点云数据流）本身固有的高 CPU 开销，以及低性能设备上 RViz 因 GPU 加速失效被迫软件渲染，进一步加重负载。

**结论**：确认瓶颈来自硬件算力限制及固定的软件设计成本，而非配置错误，据此调整为建图阶段关闭非必要图形渲染、必要时人工介入打断异常恢复循环的应对策略。

---

---

## 🛠️ 技术栈

| 类别 | 技术 |
|------|------|
| 机器人框架 | ROS2 Humble |
| 部署方式 | Docker 容器化，三容器协同 |
| 硬件平台 | FishBot 差速轮机器人 |
| 建图 | SLAM Toolbox |
| 导航 | Nav2 |
| 传感器融合 | robot_localization (EKF) |
| 机器人描述 | URDF/Xacro |
| 镜像仓库 | 阿里云容器镜像服务 |
| 编程语言 | Python / XML / YAML |

---

## 📝 项目总结

本分支在 main 分支仿真项目的基础上，完成了从 ROS2 Jazzy 仿真环境到 ROS2 Humble 真实硬件的完整迁移，核心收获包括：

- 深入理解 ROS2 launch 系统的参数声明、传递与作用域机制，排查跨多层 launch 文件的参数转发链路断裂问题
- 掌握 TF2 坐标变换系统的底层设计原理（分布式独立发布、字符串精确匹配自动拼接）
- 理清 SLAM 与 AMCL 两大定位方案的架构性互斥关系及其各自设计动机
- 具备跨 ROS2 发行版兼容性问题的系统性排查能力（插件命名、命令行参数格式差异）
- 完成从软件问题到硬件性能瓶颈的完整排查链路，掌握系统资源监控与瓶颈定位方法
- 掌握 Docker 容器化开发环境的构建、镜像管理与云端镜像仓库分发流程

---

## 📄 License

MIT License
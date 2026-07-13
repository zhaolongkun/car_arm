# Car Arm 小车机械臂联合仿真项目

本项目基于 Ubuntu 22.04、ROS2 Humble 和 Gazebo Classic，搭建“小车底盘 + MID360 雷达 + FAST-LIO/Nav2 + reBot B601-DM 机械臂”的联合仿真环境。

项目目标是让小车在 Gazebo 场景中完成建图、导航、局部避障和定点停车，并在靠近泄漏源后由车载机械臂控制喷头到达安全喷洒位置，完成泄漏点喷洒任务。

## 主要功能

- 小车、MID360 雷达、reBot B601-DM 机械臂一体化 Gazebo 仿真。
- RViz 可视化地图、点云、TF、Nav2 路径、代价地图、机器人状态和机械臂状态。
- 支持 FAST-LIO stub 仿真模式，默认不依赖真实 Livox 驱动即可运行。
- 支持切换真实 FAST-LIO / Livox 工作区。
- 支持 Nav2 导航和多中间点连续导航。
- 支持泄漏源目标点配置、小车停车距离配置和喷头安全高度约束。
- 支持安全强化学习策略控制机械臂喷洒动作。
- 支持重新训练整车 Gazebo 场景中的机械臂安全策略。
- 提供一键启动、停止和训练脚本。

## 项目结构

```text
.
├── readme.txt                    # 项目说明文档
├── start_car_arm.sh              # 启动小车 + 雷达 + Nav2 + 机械臂联合仿真
├── stop_car_arm.sh               # 清理 Gazebo、RViz、ROS2 launch 和任务节点
├── start_car.sh                  # 仅启动小车 + MID360 + FAST-LIO/Nav2 仿真
├── start_arm.sh                  # 仅启动 reBot B601-DM 机械臂仿真
├── train_car_arm_safe_rl.sh      # 训练车载机械臂安全强化学习策略
├── ros2_ws/                      # ROS2 工作区
│   └── src/
│       ├── version_car_sim/      # 小车、Gazebo、Nav2、任务流程、RL 控制代码
│       ├── FAST_LIO_ROS2/        # FAST-LIO ROS2 包
│       └── _external_rebot/      # reBot 机械臂外部依赖代码
└── 说明/                         # 补充说明文档
```

## 环境要求

推荐环境：

```text
Ubuntu 22.04
ROS2 Humble
Gazebo Classic
Python 3
```

常用依赖安装：

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher
sudo apt install ros-humble-moveit
sudo apt install python3-colcon-common-extensions python3-pip
```

默认 `FAST_LIO_MODE=stub`，只运行 Gazebo 仿真演示时不需要真实 Livox 驱动。

如需真实 FAST-LIO / Livox 驱动，请准备 `livox_ros_driver2` 工作区。脚本会自动查找：

```text
./third_party/ws_livox
../livox/ws_livox
```

也可以手动指定：

```bash
FAST_LIO_MODE=real LIVOX_WS=/path/to/ws_livox ./start_car_arm.sh
```

## 编译

进入项目根目录：

```bash
cd /home/kun/car_arm
```

编译 ROS2 工作区：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果只想使用一键脚本，通常可以直接运行启动脚本。脚本会自动构建需要的包：

```bash
cd /home/kun/car_arm
./start_car_arm.sh
```

跳过自动构建：

```bash
SKIP_BUILD=true ./start_car_arm.sh
```

## 一键启动完整联合仿真

```bash
cd /home/kun/car_arm
./start_car_arm.sh
```

默认会启动：

- Gazebo 图形界面；
- RViz；
- 小车模型；
- MID360 雷达模型；
- FAST-LIO stub；
- Nav2 导航；
- reBot B601-DM 机械臂；
- 泄漏源任务节点；
- 喷洒仿真节点；
- 安全强化学习机械臂控制器。

默认泄漏源位置：

```text
map: x=10.0, y=10.0, z=0.3
```

修改泄漏源位置：

```bash
LEAK_X=8.0 LEAK_Y=6.0 LEAK_Z=0.3 ./start_car_arm.sh
```

无 GUI / 无 RViz 启动：

```bash
GUI=false RVIZ=false ./start_car_arm.sh
```

启动前如果有旧进程，脚本默认会自动调用 `stop_car_arm.sh` 清理。也可以手动清理：

```bash
./stop_car_arm.sh
```

## 任务流程

完整联合任务流程：

```text
启动仿真
  -> 小车在起点生成
  -> MID360 / FAST-LIO stub 输出点云和里程计相关话题
  -> Nav2 根据泄漏源位置规划路线
  -> 小车按中间点连续导航到作业点附近
  -> 小车在泄漏源安全距离外停车
  -> 机械臂从 home 姿态运动到初始工作姿态
  -> 安全强化学习策略控制 spray_tip_link 接近喷洒位姿
  -> 喷头保持安全高度和喷洒距离
  -> 发布喷洒动作
```

默认机械臂控制模式：

```text
ARM_CONTROL_MODE=rl
```

默认会优先加载 Gazebo 训练策略：

```text
ros2_ws/src/version_car_sim/trained_policies/rebot_safe_ppo_lagrangian_gazebo/policy.pt
```

手动指定策略文件：

```bash
ARM_CONTROL_MODE=rl \
RL_POLICY_PATH=/path/to/policy.pt \
./start_car_arm.sh
```

## 常用启动参数

常用参数都可以通过环境变量覆盖：

```bash
LEAK_X=10.0 LEAK_Y=10.0 LEAK_Z=0.3 ./start_car_arm.sh
TARGET_CLEARANCE=0.8 ./start_car_arm.sh
GUI=true RVIZ=true ./start_car_arm.sh
FAST_LIO_MODE=stub ./start_car_arm.sh
```

重要参数说明：

```text
LEAK_X / LEAK_Y / LEAK_Z              泄漏源在 map 坐标系下的位置
TARGET_CLEARANCE                     小车外壳到泄漏源的净停车距离，默认 0.80 m
WORK_DISTANCE                        作业距离，默认等于 TARGET_CLEARANCE
SPRAY_STANDOFF                       喷头与泄漏点的喷洒前距，默认 0.15 m
MIN_SPRAY_TIP_Z                      喷头最低安全高度，默认 0.24 m
SPRAY_DURATION_SEC                   喷洒持续时间，默认 5.0 s
RETURN_HOME_AFTER_SPRAY              喷洒后是否回 home，默认 false
ARM_CONTROL_MODE                     机械臂控制模式，默认 rl
RL_POLICY_PATH                       强化学习策略路径
RL_ENABLE_TEACHER_FALLBACK           RL 失败时是否允许教师策略兜底，默认 true
RL_MAX_STEPS                         RL 最大步数，默认 360
RL_TIMEOUT_SEC                       RL 超时时间，默认 360.0 s
RL_MAX_ACTION_DELTA                  RL 单步关节动作上限，默认 0.04 rad
RL_CONTROL_RATE_HZ                   RL 控制频率，默认 8 Hz
NAVIGATION_WAYPOINT_ENABLED          是否启用中间点导航，默认 true
NAVIGATION_WAYPOINTS                 中间点列表，格式 x1,y1;x2,y2
NAVIGATION_GOAL_TOLERANCE            导航到达容差，默认 0.50 m
GUI                                  是否启动 Gazebo GUI，默认 true
RVIZ                                 是否启动 RViz，默认 true
SKIP_BUILD                           是否跳过构建，默认 false
FAST_LIO_MODE                        FAST-LIO 模式，默认 stub，可设为 real
LIVOX_WS                             Livox 工作区路径
```

自定义连续导航中间点：

```bash
NAVIGATION_WAYPOINTS="-7.0,-8.0;0.0,-7.0;5.0,-2.0" ./start_car_arm.sh
```

关闭中间点导航：

```bash
NAVIGATION_WAYPOINT_ENABLED=false ./start_car_arm.sh
```

## 只启动小车仿真

```bash
cd /home/kun/car_arm
./start_car.sh
```

默认启动 MID360 + FAST-LIO stub + Nav2。常用参数：

```bash
GOAL_X=10.0 GOAL_Y=10.0 ./start_car.sh
GUI=true RVIZ=true ./start_car.sh
FAST_LIO_MODE=stub ./start_car.sh
```

如果需要真实 FAST-LIO：

```bash
FAST_LIO_MODE=real LIVOX_WS=/path/to/ws_livox ./start_car.sh
```

## 只启动机械臂仿真

```bash
cd /home/kun/car_arm
./start_arm.sh
```

默认运行圆形点位运动演示：

```bash
DEMO_MODE=circle_points ./start_arm.sh
```

运行喷洒演示：

```bash
DEMO_MODE=spray LEAK_X=0.5 LEAK_Y=0.0 LEAK_Z=0.28 ./start_arm.sh
```

## 重新训练安全强化学习策略

训练命令：

```bash
cd /home/kun/car_arm
./train_car_arm_safe_rl.sh
```

快速测试训练：

```bash
TEACHER_EPISODES=3 BC_UPDATES=10 ./train_car_arm_safe_rl.sh
```

较正式训练示例：

```bash
TEACHER_EPISODES=200 BC_UPDATES=800 ./train_car_arm_safe_rl.sh
```

训练结果默认保存到：

```text
ros2_ws/src/version_car_sim/trained_policies/rebot_safe_ppo_lagrangian_gazebo/
```

训练完成后可直接指定新策略启动：

```bash
ARM_CONTROL_MODE=rl \
RL_POLICY_PATH=ros2_ws/src/version_car_sim/trained_policies/rebot_safe_ppo_lagrangian_gazebo/policy.pt \
./start_car_arm.sh
```

## 常用话题和节点

联合仿真中常见话题：

```text
/map
/odom
/tf
/mid360_points
/cmd_vel
/plan
/goal_pose
/local_costmap/costmap
/global_costmap/costmap
/spray_action
/mcu_frame_hex
```

常见节点和功能：

```text
fast_lio_stub                         FAST-LIO stub 仿真转发
pointcloud_to_costmap                 点云投影到 2D costmap
nav2_waypoint_commander               Nav2 中间点连续导航
gas_leak_mobile_manipulator_task      小车 + 机械臂任务流程
rebot_safe_rl_controller              安全强化学习机械臂控制
spray_simulator                       喷洒动作仿真
cmd_vel_monitor                       速度命令监控
trajectory_recorder                   轨迹记录
```

## 常见问题

### 1. 找不到 Nav2

安装 Nav2：

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### 2. 找不到 MoveIt 或 ros2_control

安装依赖：

```bash
sudo apt install ros-humble-moveit
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

### 3. FAST_LIO_MODE=real 时找不到 livox_ros_driver2

确认 Livox 工作区已经编译，并指定路径：

```bash
FAST_LIO_MODE=real LIVOX_WS=/path/to/ws_livox ./start_car_arm.sh
```

如果只是运行仿真演示，使用默认 stub 模式：

```bash
FAST_LIO_MODE=stub ./start_car_arm.sh
```

### 4. Gazebo / RViz / ROS2 节点残留

运行：

```bash
./stop_car_arm.sh
```

然后重新启动：

```bash
./start_car_arm.sh
```

### 5. 重新启动后代码没有更新

先清理旧进程，再重新构建：

```bash
./stop_car_arm.sh
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
cd ..
./start_car_arm.sh
```

## GitHub 分支

当前项目上传到 GitHub 仓库：

```text
https://github.com/zhaolongkun/car_arm/tree/branch1
```

推荐在 `branch1` 分支上继续开发和提交。

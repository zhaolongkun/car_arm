# Car Arm 仿真项目

本项目是一个基于 ROS2 Humble + Gazebo Classic 的“小车 + MID360 雷达 + reBot B601-DM 机械臂”联合仿真工程。

当前目标是让小车在仿真环境中完成建图、导航和避障，到达泄漏源附近后停止，再由车载机械臂控制喷头运动到泄漏点上方附近进行喷洒，同时避免机械臂与小车车体、雷达、车轮、地面和自身发生碰撞。

## 主要功能

- Gazebo 中显示小车、障碍物、MID360 雷达和 reBot B601-DM 机械臂。
- RViz 中显示地图、路径、点云、TF、局部/全局代价地图和机器人状态。
- 支持 FAST-LIO 仿真模式，默认使用 `stub` 模式，便于没有真实雷达时运行。
- 支持 Nav2 导航，小车从起点导航到泄漏源附近。
- 支持安全强化学习控制机械臂，默认使用已训练策略。
- 支持重新训练车载机械臂安全避碰策略。
- 使用相对路径组织工程，方便克隆后迁移和开源。

## 项目结构

```text
.
├── start_car_arm.sh              # 启动小车 + 雷达 + 机械臂 + Nav2 + RL 控制
├── stop_car_arm.sh               # 关闭仿真和相关 ROS2/Gazebo 进程
├── train_car_arm_safe_rl.sh      # 训练车载机械臂安全强化学习策略
├── start_car.sh                  # 仅启动小车相关仿真
├── start_arm.sh                  # 仅机械臂相关启动脚本
├── ros2_ws/                      # ROS2 工作区
│   └── src/
│       ├── version_car_sim/      # 小车、Gazebo、Nav2、RL、任务流程代码
│       ├── FAST_LIO_ROS2/        # FAST-LIO ROS2 包
│       └── _external_rebot/      # reBot 机械臂相关外部代码
└── readme.txt                    # 项目说明
```

## 环境要求

推荐系统：

```text
Ubuntu 22.04
ROS2 Humble
Gazebo Classic
Python 3
```

需要安装常用 ROS2 依赖：

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher
sudo apt install python3-colcon-common-extensions python3-pip
```

默认启动使用 `FAST_LIO_MODE=stub`，只做 Gazebo 仿真演示时不需要 `livox_ros_driver2`。

如果使用真实 FAST-LIO / Livox 驱动，需要准备 `livox_ros_driver2` 工作区。启动脚本会自动查找：

```text
./third_party/ws_livox
../livox/ws_livox
```

也可以手动指定，并切换到真实 FAST-LIO：

```bash
FAST_LIO_MODE=real LIVOX_WS=/path/to/ws_livox ./start_car_arm.sh
```

## 编译

进入项目根目录：

```bash
cd car_arm
```

编译 ROS2 工作区：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果提示找不到 `livox_ros_driver2`，先 source 你的 Livox 工作区：

```bash
source /path/to/ws_livox/install/setup.bash
colcon build --symlink-install
```

## 一键启动完整仿真

回到项目根目录：

```bash
cd ..
./start_car_arm.sh
```

默认会启动：

- Gazebo；
- RViz；
- 小车模型；
- MID360 雷达模型；
- reBot B601-DM 机械臂模型；
- FAST-LIO stub；
- Nav2；
- 泄漏源任务节点；
- 安全强化学习机械臂控制器。

默认泄漏源目标点为：

```text
map: x=10.0, y=10.0, z=0.3
```

可以通过环境变量修改：

```bash
LEAK_X=10.0 LEAK_Y=10.0 LEAK_Z=0.3 ./start_car_arm.sh
```

## 停止仿真

```bash
./stop_car_arm.sh
```

如果旧进程没有关干净，先执行这个脚本再重新启动。

## 小车与机械臂任务流程

完整流程如下：

```text
启动仿真
  -> 小车生成在起点
  -> Nav2 规划到泄漏源附近
  -> 小车到达目标附近并停止
  -> 任务节点触发机械臂
  -> 机械臂从 home 全 0 姿态开始
  -> 机械臂运动到 initial_pose
  -> 安全强化学习策略控制机械臂靠近喷洒目标
  -> 喷头停在泄漏点上方附近
  -> 发布喷洒动作
```

当前默认机械臂控制模式是：

```text
ARM_CONTROL_MODE=rl
```

也就是使用安全强化学习策略控制机械臂。

## 安全强化学习策略

机械臂控制使用 Safe PPO / PPO-Lagrangian 思路搭建安全强化学习框架。

训练目标：

- 让 `spray_tip_link` 靠近泄漏源附近的安全喷洒位姿；
- 避免机械臂碰撞小车车体、雷达、车轮、地面和自身；
- 使用 reward 鼓励到达目标；
- 使用 safety cost 约束碰撞风险；
- 使用 safety shield 在执行动作前做安全检查。

默认策略文件路径：

```text
ros2_ws/src/version_car_sim/trained_policies/rebot_safe_ppo_lagrangian_gazebo/policy.pt
```

如果想手动指定策略：

```bash
ARM_CONTROL_MODE=rl RL_POLICY_PATH=ros2_ws/src/version_car_sim/trained_policies/rebot_safe_ppo_lagrangian_gazebo/policy.pt ./start_car_arm.sh
```

## 重新训练机械臂策略

训练命令：

```bash
./train_car_arm_safe_rl.sh
```

常用快速测试训练：

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

训练完成后，`start_car_arm.sh` 会优先加载这个目录下的 `policy.pt`。

## 常用参数

启动时可以通过环境变量覆盖参数：

```bash
LEAK_X=10.0 LEAK_Y=10.0 LEAK_Z=0.3 ./start_car_arm.sh
```

```bash
TARGET_CLEARANCE=0.8 ./start_car_arm.sh
```

```bash
GUI=true RVIZ=true ./start_car_arm.sh
```

```bash
FAST_LIO_MODE=stub ./start_car_arm.sh
```

常见参数说明：

```text
LEAK_X / LEAK_Y / LEAK_Z       泄漏源在 map 坐标系下的位置
TARGET_CLEARANCE              小车外壳到泄漏源的净停车距离
ARM_CONTROL_MODE              机械臂控制模式，默认 rl
RL_POLICY_PATH                强化学习策略文件路径
GUI                           是否启动 Gazebo 图形界面
RVIZ                          是否启动 RViz
FAST_LIO_MODE                 FAST-LIO 模式，默认 stub
```

## 只编译指定包

开发时可以只编译主包：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select version_car_sim
```

## 调试命令

查看 ROS2 节点：

```bash
ros2 node list
```

查看控制器：

```bash
ros2 control list_controllers
```

查看机械臂关节状态：

```bash
ros2 topic echo /joint_states
```

查看 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map rebot_base_link
```

查看 Nav2 状态：

```bash
ros2 lifecycle nodes
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

查看速度话题：

```bash
ros2 topic echo /cmd_vel
```

手动测试小车底盘：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

机械臂最小动作测试：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run version_car_sim rebot_arm_minimal_test --target spray_demo --duration 2.0
```

## GitHub 开源注意事项

不要上传编译和运行生成目录：

```text
build/
install/
log/
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
ros2_ws/sim_results/
```

这些内容已经在 `.gitignore` 中忽略。

推荐上传的核心内容：

```text
start_car_arm.sh
stop_car_arm.sh
train_car_arm_safe_rl.sh
start_car.sh
start_arm.sh
ros2_ws/
readme.txt
.gitignore
```

## 常见问题

如果 Gazebo 没有启动，先执行：

```bash
./stop_car_arm.sh
./start_car_arm.sh
```

如果提示找不到 Livox 依赖，使用：

```bash
LIVOX_WS=/path/to/ws_livox ./start_car_arm.sh
```

如果机械臂不动，优先检查：

```bash
ros2 node list | grep rebot
ros2 control list_controllers
ros2 topic echo /joint_states
```

如果小车有路径但不动，检查：

```bash
ros2 topic echo /cmd_vel
ros2 lifecycle get /controller_server
ros2 control list_controllers
```

## 当前默认演示命令

```bash
cd car_arm
./stop_car_arm.sh
./start_car_arm.sh
```

运行后观察 Gazebo 和 RViz：小车会导航到泄漏源附近，随后车载 reBot B601-DM 机械臂会使用安全强化学习策略控制喷头靠近泄漏源上方附近并执行喷洒动作。

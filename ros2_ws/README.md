# version-car Gazebo 仿真

这个工作区先把“建图、路径规划、局部避障、阿克曼控制量转换、Gazebo 仿真、MCU 协议帧输出”跑通，不直接操作实物。

当前仿真阶段已经完成：

- Gazebo 中的模拟雷达建图；
- A* 全局路径规划；
- 基于 `/scan` 的局部避障保护；
- 将最终 `/cmd_vel` 转换成阿克曼车辆控制量；
- 将阿克曼控制量打包成未来可发给 STM32 的 `/mcu_frame_hex`。
- Gazebo 中使用四轮阿克曼外观模型和大尺寸测试场地。
- 支持 RViz 或 topic 手动设置 `/goal_pose`，再通过 `/start_navigation` 手动启动运动。
- 支持记录 `/actual_path` 实际轨迹、Gazebo 实时轨迹标记，以及到达终点后自动保存地图和轨迹结果。

当前尚未连接真实 MID360、真实 STM32 和真实小车，也不会打开真实串口。

## 组件

- `version_car.sdf`：四轮阿克曼小车模型，包含 `base_link`、两个前转向轮、两个后驱动轮、`front_steering_link/front_steering_joint` 和 `mid360_link`。当前 Gazebo 运动层使用稳定的 `libgazebo_ros_planar_move.so` 做 `/cmd_vel -> /odom` 近似运动；真实控制协议层仍由 `mcu_protocol_bridge` 按阿克曼结构换算。
- `version_car_mid360_fastlio.sdf`：MID360 3D 建图专用模型，车顶安装 MID360 风格 3D 激光雷达（`mid360_link`）和 IMU，独立输出 `/mid360_points` 和 `/imu`，用于 FAST-LIO 三维建图。
- `test_track.world`：约 120m x 100m 的大空旷测试场，四周有边界墙，默认不再放置中心固定障碍物，方便在 Gazebo 里手动摆放障碍物。
- `mid360_fast_lio_world.world`：3D 建图专用场景，包含箱体、圆柱、U 形墙等多高度障碍物，适合验证 FAST-LIO 三维建图效果。
- `occupancy_mapper`：订阅 `/scan` 和 `/odom`，发布 `/map`。这是里程计辅助栅格建图，没有回环优化。
- `astar_planner`：订阅 `/map`、`/odom`、`/goal_pose` 和 `/start_navigation`，默认没有写死目标点；只有收到目标点并且允许启动后才发布 `/planned_path` 和 `/cmd_vel_raw`。
- `local_obstacle_avoidance`：订阅 `/scan` 和 `/cmd_vel_raw`，发布带局部避障保护的 `/cmd_vel`。
- `pointcloud_to_costmap`：3D 点云 → 2D 栅格地图投影节点，供后续导航使用。
- `fast_lio_stub`：FAST-LIO 占位节点，转发 Gazebo `/odom` 和 `/mid360_points` 到 FAST-LIO 格式话题。
- `mapping_drive_node`：自动巡航建图驱动节点，使小车在场景中低速绕圈运动以采集数据。
- `mcu_protocol_bridge`：把最终 `/cmd_vel` 转成阿克曼车辆控制量，再打包成固件已有的 22 字节 `0x10` 跟随控制帧，dry-run 发布到 `/mcu_frame_hex`。
- `start_pose_setter`：订阅 `/start_pose`、`/start_pose_2d` 或 `/initialpose`，通过 Gazebo dry-run 服务把 `version_car` 移动到指定起点。
- `trajectory_recorder`：订阅 `/odom`、`/map`、`/planned_path` 和 `/goal_pose`，发布 `/actual_path`，可在 Gazebo 中实时生成黄色轨迹点，到达终点后保存 `sim_results/run_YYYYMMDD_HHMMSS/`。

## 车辆安全圆

当前不再使用固定小半径。A* 障碍物膨胀半径根据四轮模型尺寸计算，安全半径缩放系数固定为 1.6：

```text
wheelbase = 0.66 m
track_width = 0.62 m
front_wheel_center_x  =  0.34 m  (half_track = 0.31 m)
rear_wheel_center_x   = -0.32 m  (half_track = 0.31 m)
max_distance_from_center_to_wheel = max(
    sqrt(0.34^2 + 0.31^2),   # 前轮 = 0.460 m
    sqrt(0.32^2 + 0.31^2),   # 后轮 = 0.445 m
) = 0.460 m
vehicle_safety_radius = 1.6 * 0.460 = 0.736 m   # (之前 0.679 m 是 1.5 系数的旧值)
hard_collision_radius = vehicle_safety_radius = 0.736 m
soft_inflation_radius = vehicle_safety_radius + 0.30 m = 1.036 m
```

这个 `vehicle_safety_radius` 用于 A* 的 inflated grid 障碍物膨胀，也会作为 `local_obstacle_avoidance` 的安全距离参考。对应的轮心来自四轮模型坐标：左前轮中心约为 `(0.34, 0.31)`，右后轮中心约为 `(-0.32, -0.31)`。

真实小车底盘不是差速车，而是阿克曼式结构：后轮提供前进/后退动力，前轮由伺服电机负责左右转向。因此桥接器不会把 `/cmd_vel` 当成左右轮速度，而是按：

```text
v = cmd_vel.linear.x
w = cmd_vel.angular.z
delta = atan(wheel_base * w / v)
```

计算前轮目标转角 `delta`，同时把 `v` 转换为后轮驱动速度/油门/档位。`wheel_base` 是可配置参数。低速避障时如果出现 `v=0, w!=0`，仿真桥接器会输出后轮停止，并默认把前轮目标转角置为 0；如果后续确实需要低速预打小角度，可以通过 `low_speed_steering_limit_deg` 单独配置。

当前桥接器已经加入低速保护：当 `abs(v)` 小于 `low_speed_epsilon` 时，不再直接计算 `atan(wheel_base * w / v)`，而是输出安全后轮速度，并默认把前轮目标转角置为 0，避免除零和转角突变。前轮目标转角还会经过 `max_steering_angle_deg` 机械限幅和 `max_steering_rate_deg_s` 变化率限幅；后轮速度会经过 `max_rear_speed` 限幅。

当前 22 字节协议帧中的转向字段写入的是“前轮目标转角”。如果真实上车时前轮伺服电机只支持速度环控制，则不能直接把目标转角当速度命令发给伺服，还需要根据当前前轮转角反馈做闭环：

```text
delta_target = front_delta
delta_current = front_steering_feedback
steer_speed_cmd = Kp * (delta_target - delta_current)
```

代码中已经预留 `use_front_steering_feedback`、`front_steering_feedback_topic` 和内部 `delta_current` 接口说明。真实硬件闭环时，可以通过编码器、电位器或伺服驱动器反馈发布当前前轮转角；默认 topic 名为 `/front_steering_feedback`，建议使用弧度制 `Float32`。

## 运行

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch version_car_sim sim.launch.py
```

默认 `auto_start:=false`，小车 spawn 后保持静止，方便先在 Gazebo 里手动摆放障碍物。推荐可视化启动：

```bash
ros2 launch version_car_sim sim.launch.py gui:=true rviz:=true auto_start:=false
```

启动后，小车不会自动运动。推荐流程：

1. 在 Gazebo 里手动摆放障碍物；
2. 用命令设置小车起点；
3. 在 RViz 使用 `2D Goal Pose` 点选终点，或用 topic 发布 `/goal_pose`；
4. 发布 `/start_navigation`；
5. 小车开始规划、避障和运动；
6. 到达终点后自动保存地图、规划路径和实际轨迹。

推荐用 `Pose2D` 设置起点，`theta` 是弧度制车头朝向：

```bash
ros2 topic pub --once /start_pose_2d geometry_msgs/msg/Pose2D "{x: -8.0, y: -6.0, theta: 0.0}"
```

也可以用 `PoseStamped` 设置起点：

```bash
ros2 topic pub --once /start_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -8.0, y: -6.0, z: 0.0}, orientation: {w: 1.0}}}"
```

`start_pose_setter` 会先发布 0 速度，再把 Gazebo 里的 `version_car` 移动到指定起点。节点内部优先使用 Gazebo Classic 的 `gz model` 方式移动模型，并保留 `/gazebo/set_entity_state` 服务路径作为备用。你只需要发布 `/start_pose_2d` 或 `/start_pose`，不需要手动调用 Gazebo 服务。如果当前 world 里已经保存了小车，推荐使用这个 topic 方式设置起点；launch 参数 `start_x/start_y/start_yaw` 只在需要重新 spawn 小车时生效。

命令行设置终点示例：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 8.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}"
```

发布开始信号：

```bash
ros2 topic pub --once /start_navigation std_msgs/msg/Bool "{data: true}"
```

如果显式打开 `auto_start:=true`，规划器仍然需要先收到 `/goal_pose`，否则会保持停止，不会乱跑：

```bash
ros2 launch version_car_sim sim.launch.py gui:=true rviz:=true auto_start:=true
```

如果只想 headless 跑：

```bash
ros2 launch version_car_sim sim.launch.py gui:=false rviz:=false auto_start:=false
```

打开 RViz：

```bash
ros2 launch version_car_sim sim.launch.py rviz:=true
```

## D435i 深度相机视觉避障与路径规划仿真

新增 D435i 版本是 Gazebo dry-run：不接真实 D435i、不接真实 MID360、不接真实 STM32，也不会打开真实串口。它使用 `models/version_car_d435i.sdf` 中的 Gazebo depth camera 插件模拟前向 D435i，并通过 `d435i_vision_sim.launch.py` 独立启动，不影响原来的 `/scan -> /map -> /planned_path -> /cmd_vel_raw -> /cmd_vel -> /mcu_frame_hex` 激光雷达链路。

当前 D435i 版本是“前向视觉局部建图 + 局部路径规划/避障”的仿真，不等同于完整 SLAM。前向深度相机只看到车前方局部区域，因此 `d435i_astar_planner` 会在视觉局部 costmap 内规划到朝向全局目标的局部子目标，并持续重规划；目标是验证 D435i 深度视觉能发现前方障碍物，并影响路径、速度控制和最终 MCU dry-run 协议帧。

D435i 版本的碰撞边界使用以 `base_link` 为中心的车体安全圆，不使用相机中心半径。当前四个轮心在 `base_link` 下的 XY 坐标为：

```text
front_left_wheel  = ( 0.34,  0.31)
front_right_wheel = ( 0.34, -0.31)
rear_left_wheel   = (-0.32,  0.31)
rear_right_wheel  = (-0.32, -0.31)
```

计算公式：

```text
front_left_distance  = sqrt(0.34^2 + 0.31^2) = 0.460 m
front_right_distance = sqrt(0.34^2 + 0.31^2) = 0.460 m
rear_left_distance   = sqrt(0.32^2 + 0.31^2) = 0.446 m
rear_right_distance  = sqrt(0.32^2 + 0.31^2) = 0.446 m
max_distance_from_center_to_wheel = 0.460 m
vehicle_safety_radius = 1.6 * 0.460 = 0.737 m
```

`d435i_depth_obstacle_mapper` 使用这个半径对视觉障碍物做 inflation，`d435i_astar_planner` 只在膨胀后的视觉 costmap 中搜索安全路径，`d435i_visual_obstacle_avoidance` 会把阈值自动抬高到：

```text
slow_distance      >= vehicle_safety_radius + 0.80
stop_distance      >= vehicle_safety_radius + 0.40
emergency_distance >= vehicle_safety_radius + 0.15
```

启动：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch version_car_sim d435i_vision_sim.launch.py gui:=true rviz:=true auto_start:=false
```

D435i 专用场景和 RViz 配置位于：

```text
src/version_car_sim/config/worlds/d435i_vision_world.world
src/version_car_sim/config/rviz/d435i_vision_sim.rviz
```

启动后小车默认不运动。可以在 RViz 使用 `2D Goal Pose` 设置终点，也可以命令行发布：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 7.0, y: 2.5, z: 0.0}, orientation: {w: 1.0}}}"
```

然后手动启动导航：

```bash
ros2 topic pub --once /start_navigation std_msgs/msg/Bool "{data: true}"
```

可选地设置起点：

```bash
ros2 topic pub --once /start_pose_2d geometry_msgs/msg/Pose2D "{x: -5.0, y: -3.0, theta: 0.25}"
```

D435i 仿真相机话题：

- `/d435i/color/image_raw`
- `/d435i/depth/image_raw`
- `/d435i/depth/camera_info`
- `/d435i/depth/points`

视觉建图、规划、避障和 MCU dry-run 话题：

- `/vision_obstacle_grid`：D435i 高度过滤后的局部障碍栅格；
- `/vision_local_costmap`：带 inflation 的视觉局部 costmap，供 D435i A* 使用；
- `/vision_obstacle_points`：投影到 `map` 坐标系的视觉障碍点；
- `/planned_path`：D435i 局部 A* 输出的可见范围路径；
- `/cmd_vel_raw`：D435i 视觉局部规划器输出的原始速度；
- `/cmd_vel`：经过 D435i 视觉避障后的最终速度；
- `/d435i_avoidance_debug`：JSON 字符串，包含 `front_min`、`left_min`、`right_min`、`avoidance_state`、`raw_linear`、`raw_angular`、`final_linear`、`final_angular`；
- `/vehicle_safety_radius_marker`：RViz 中跟随 `base_link` 移动的车体安全圆；
- `/actual_path`：实际轨迹；
- `/mcu_frame_hex`：由最终 `/cmd_vel` 转换得到的 MCU 协议帧 dry-run 输出。

`/d435i_avoidance_debug` 还会输出 `vehicle_safety_radius`、`max_distance_from_center_to_wheel`、四个轮心距离、`front_min_obstacle_distance`、`distance_to_safety_boundary` 和 `using_vehicle_safety_radius`，用于确认避障使用的是车体安全圆而不是 D435i 相机中心距离。

GUI 启动后建议先检查这些命令，确认 RViz/Gazebo 看到的是同一条链路：

```bash
ros2 topic list | grep d435i
ros2 topic hz /d435i/color/image_raw
ros2 topic hz /d435i/depth/image_raw
ros2 topic hz /d435i/depth/points
ros2 topic echo --once /d435i/depth/camera_info
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /cmd_vel_raw
ros2 topic echo /cmd_vel
ros2 topic echo /odom
```

D435i launch 会发布静态 `map -> odom` TF，Gazebo 运动插件发布 `odom -> base_link`，`robot_state_publisher` 发布 `base_link -> d435i_link -> d435i_depth_optical_frame`。因此 RViz 的 Fixed Frame 保持为 `map`。

如果 Gazebo depth camera 的点云坐标看起来方向不对，可以切换点云坐标约定：

```bash
ros2 launch version_car_sim d435i_vision_sim.launch.py gui:=true rviz:=true camera_frame_convention:=x_forward
```

Gazebo camera 插件默认用 `point_cloud_qos:=reliable`；后续接真实 D435i 驱动时，如果点云是 sensor-data 风格 QoS，可以改用：

```bash
ros2 launch version_car_sim d435i_vision_sim.launch.py gui:=true rviz:=true point_cloud_qos:=best_effort
```

## MID360 3D 雷达 + IMU + FAST-LIO 三维建图仿真

独立的中距离 3D 建图仿真模式，通过 `mid360_fast_lio_mapping.launch.py` 独立启动，不影响原有的 2D LaserScan 导航链路（`sim.launch.py`）和 D435i 视觉模式（`d435i_vision_sim.launch.py`）。

### 模式定位

当前模式的**核心目标是 3D SLAM 建图**，不是 2D A* 导航。输入是 Gazebo 模拟的 MID360 3D 点云和 IMU，FAST-LIO 输出 3D 点云地图和位姿。后续如果要让小车在 3D 地图中导航，再单独把 3D 地图投影到 2D costmap。

### 数据链路

```text
Gazebo 仿真环境
├─ 小车型号: models/version_car_mid360_fastlio.sdf
│   ├─ base_link (车体 + 四轮 + 阿克曼转向)
│   ├─ mid360_link (车顶，含传感器)
│   │   ├─ Ray sensor → /mid360_points (PointCloud2, frame: mid360_link, 10 Hz)
│   │   └─ IMU sensor → /imu (Imu, frame: mid360_link, 200 Hz)
│   └─ planar_move plugin → /odom + odom→base_link TF
│
├─ World: config/worlds/mid360_fast_lio_world.world
│   (大地面 + 四面墙 + 箱体/圆柱/U 形结构等 3D 障碍物)
│
└─ FAST-LIO 模式 (由 fast_lio_mode 参数控制)
    ├─ fast_lio_mode:=stub (默认)
    │   └─ fast_lio_stub.py: 转发 /odom 为 Odometry，/mid360_points 为 /cloud_registered + /Laser_map
    └─ fast_lio_mode:=real (需要先安装 FAST-LIO2 ROS2)
        └─ fastlio_mapping: 真正的 FAST-LIO2 算法
            ├─ 订阅 /mid360_points + /imu
            ├─ 输出 /Odometry (frame: camera_init)
            ├─ 输出 /cloud_registered (当前帧注册点云)
            ├─ 输出 /Laser_map (累积 3D 点云地图)
            └─ 输出 /path (轨迹)
```

### TF 树

```text
mode=stub:                        mode=real (FAST-LIO2):
  map                                map
   └─ odom (静态 TF)                   └─ camera_init (FAST-LIO 世界系)
        └─ base_link (planar_move)          └─ base_link (FAST-LIO 估计)
             └─ mid360_link (固定关节)           └─ mid360_link (固定关节)
```

**注意**：`fast_lio_mode:=real` 时，FAST-LIO2 使用 `camera_init` 坐标系作为世界系。启动文件中已添加 `map→camera_init` 静态 TF，使 RViz 默认 `map` 固定帧下可见。

`fast_lio_mode:=stub` 时，`fast_lio_stub` 节点发布 `map→odom` 静态 TF，`planar_move` 发布 `odom→base_link`，小车运动依赖于 Gazebo ground truth。

### 模型与传感器

`models/version_car_mid360_fastlio.sdf`:

- **地点**: MID360 安装在车体顶部中央稍靠前，link 命名为 `mid360_link`
- **雷达**: Gazebo Ray 传感器，水平 360° (720 samples)，垂直 ±25° (32 lines)，10 Hz
- **输出**: `/mid360_points` (sensor_msgs/PointCloud2, frame_id=`mid360_link`)
- **IMU**: Gazebo IMU 传感器，200 Hz，frame_id=`mid360_link`（与雷达同 link，外参 Identity）
- **输出**: `/imu` (sensor_msgs/Imu, frame_id=`mid360_link`)
- **IMU 噪声**: 角速度高斯噪声 stddev=0.0005，加速度高斯噪声 stddev=0.01
- **运动插件**: `libgazebo_ros_planar_move.so`，接收 `/cmd_vel`，发布 `/odom` 和 `odom→base_link` TF
- **LiDAR 外观**: 黑色圆柱体 + 蓝色窗口，Gazebo 可视化

> **注**: Gazebo Ray 传感器是常规 3D LiDAR 扫描模式，不是 MID360 的非重复扫描 pattern。当前仿真近似 MID360 视场角和采样密度，足以验证 FAST-LIO 三维建图效果。

### 场景配置

`config/worlds/mid360_fast_lio_world.world`:

- 120 m × 100 m 大地面
- 四面边界墙
- 多样 3D 障碍物（验证 FAST-LIO 三维建图）：
  - 大型箱体 (1.8×1.4×1.5 m)
  - 中尺寸圆柱 (r=0.45, h=1.1 m)
  - 矮箱体 (1.6×1.0×0.7 m)
  - 高圆柱 (r=0.5, h=1.5 m)
  - 高箱体 (1.1×1.5×1.9 m)
  - U 形墙体组合（用于回环检测）
  - 小尺寸障碍物群

### FAST-LIO2 安装与配置

当前环境中 FAST-LIO2 ROS2 尚未安装（现有 `FAST_LIO/` 是 ROS1 版本）。当网络可用时，按以下步骤安装：

**步骤 1：克隆 FAST-LIO2 ROS2 分支**

```bash
cd ros2_ws/src
git clone https://github.com/hku-mars/FAST-LIO.git -b ros2
```

**步骤 2：安装依赖**

```bash
# livox_ros_driver2（如果 FAST-LIO2 CMakeLists.txt 要求）
cd ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# 其他系统依赖
sudo apt install -y libeigen3-dev libpcl-dev
```

**步骤 3：编译**

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select livox_ros_driver2 fast_lio
```

**步骤 4：修改 FAST-LIO2 配置**

使用本工程预置的配置文件 `config/fast_lio/mid360_gazebo.yaml`，已适配仿真话题名 `/mid360_points` 和 `/imu`。可通过 launch 参数传入：

```bash
ros2 launch version_car_sim mid360_fast_lio_mapping.launch.py \
    fast_lio_mode:=real \
    gui:=true rviz:=true
```

FAST-LIO2 输出话题（ros2 分支）：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/Odometry` | `nav_msgs/Odometry` | 里程计 (frame: `camera_init`) |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 当前帧注册点云 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | 车体系注册点云 |
| `/Laser_map` | `sensor_msgs/PointCloud2` | 累积全局地图 |
| `/path` | `nav_msgs/Path` | 轨迹 |

### fast_lio_stub 模式（默认）

在 FAST-LIO2 ROS2 安装之前，使用 `fast_lio_stub` 转发节点验证仿真链路。Stub 节点将 Gazebo ground truth `/odom` 转发为 FAST-LIO 风格的 `/Odometry`（frame 改为 `map`），并将 `/mid360_points` 转发为 `/cloud_registered` 和 `/Laser_map`。

**这不是真正的 FAST-LIO 建图**，仅用于：
1. 验证 Gazebo 仿真链路（MID360 点云 + IMU + 小车运动）正常；
2. 验证 3D 点云在 RViz 中显示正常；
3. 验证 TF 树完整；
4. 为 FAST-LIO2 安装后提供对比基准。

### 启动方式

#### Stub 模式（默认，无需 FAST-LIO2）

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch version_car_sim mid360_fast_lio_mapping.launch.py gui:=true rviz:=true
```

启动后小车自动低速绕圈运动（`enable_mapping_drive:=true`），用于采集建图数据。

#### 手动控制（关闭自动巡航）

```bash
ros2 launch version_car_sim mid360_fast_lio_mapping.launch.py gui:=true rviz:=true enable_mapping_drive:=false
```

启动后用 topic 控制小车运动：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.2}}"
```

停车：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist
```

#### Real FAST-LIO2 模式（需要先安装 FAST-LIO2）

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch version_car_sim mid360_fast_lio_mapping.launch.py fast_lio_mode:=real gui:=true rviz:=true
```

如果 FAST-LIO2 未安装，launch 会打印错误信息并提示安装步骤。

### 验证命令

启动后验证传感器和建图系统：

```bash
# 查看所有相关话题
ros2 topic list | grep -E "mid360|imu|cloud|Laser|Odom|path|cmd_vel"

# 检查 3D 点云频率
ros2 topic hz /mid360_points

# 检查 IMU 频率
ros2 topic hz /imu

# 查看 IMU 数据
ros2 topic echo /imu --once

# 查看点云数据概要
ros2 topic echo /mid360_points --once | head -5

# 查看 TF 树
ros2 run tf2_tools view_frames

# 检查具体 TF 链路
ros2 run tf2_ros tf2_echo base_link mid360_link
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

如果 FAST-LIO2 正常运行，还要检查：

```bash
# FAST-LIO 里程计
ros2 topic echo /Odometry --once

# FAST-LIO 累积地图（点云数）
ros2 topic echo /Laser_map --once | head -5

# FAST-LIO 轨迹
ros2 topic echo /path --once | head -10
```

### RViz 显示说明

专用 RViz 配置 `config/rviz/mid360_fast_lio_mapping.rviz` 默认显示：

- **Grid**: 地面网格
- **MID360 Raw PointCloud**: `/mid360_points` 原始 3D 点云（橙色）
- **Registered Cloud**: `/cloud_registered` FAST-LIO 当前帧注册点云（绿色）
- **Laser Map**: `/Laser_map` FAST-LIO 累积 3D 点云地图（蓝色，随建图进行逐步增长）
- **FAST-LIO Odometry**: `/Odometry` FAST-LIO 里程计可视化
- **FAST-LIO Path**: `/path` FAST-LIO 轨迹（青色）
- **Gazebo Odom**: `/odom` Gazebo ground truth（对比用）
- **Gazebo Odom Path**: `/actual_path` 实际行驶路径（绿色，对比 FAST-LIO 定位精度）
- **TF**: 完整 TF 树 (map → odom/camera_init → base_link → mid360_link)

### 参数说明

```bash
# 默认 stub 模式
ros2 launch version_car_sim mid360_fast_lio_mapping.launch.py

# Real FAST-LIO2 模式
ros2 launch version_car_sim mid360_fast_lio_mapping.launch.py fast_lio_mode:=real

# 关闭自动巡航（手动控制）
ros2 launch version_car_sim mid360_fast_lio_mapping.launch.py enable_mapping_drive:=false

# 自定义 world 和 RViz 配置
ros2 launch version_car_sim mid360_fast_lio_mapping.launch.py \
    world_file:=mid360_fast_lio_world.world \
    rviz_config_file:=mid360_fast_lio_mapping.rviz

# Headless 运行（无 GUI、无 RViz）
ros2 launch version_car_sim mid360_fast_lio_mapping.launch.py gui:=false rviz:=false \
    enable_mapping_drive:=true
```

### 已知限制

1. 当前 `fast_lio_stub` 模式使用 Gazebo ground-truth `/odom` 作为 FAST-LIO odometry，没有 SLAM 漂移和误差累积；
2. Gazebo Ray 传感器是常规 3D LiDAR 扫描，不能完全模拟 MID360 的非重复扫描 pattern（但视场角和采样密度已足够验证 FAST-LIO 算法）；
3. FAST-LIO2 ROS2 在当前环境中未安装（因 GitHub 无网络访问），需要手动克隆编译；
4. FAST-LIO2 的坐标系是 `camera_init`（不是 `map`），启动文件中已添加静态 TF 对齐；
5. 当 `fast_lio_mode:=real` 时，FAST-LIO2 和 Gazebo `planar_move` 各自发布独立的 `base_link` TF（FAST-LIO2 发布自己的定位，planar_move 发布 ground truth），通过不同父帧区分；
6. 当前仍然是 Gazebo dry-run 阶段：
   - 不接真实 MID360；
   - 不接真实 STM32；
   - 不打开真实串口；
   - 不控制真实小车。

## 使用自定义 Gazebo 和 RViz 地图配置运行仿真

你自己保存的 Gazebo world 文件放在：

```text
src/version_car_sim/config/worlds/
```

你自己保存的 RViz 配置文件放在：

```text
src/version_car_sim/config/rviz/
```

`colcon build` 后，这两个目录会被安装到 `install/version_car_sim/share/version_car_sim/config/`，因此 `source install/setup.bash` 后 launch 文件仍然能找到这些配置。

默认选择规则：

- Gazebo 优先使用 `config/worlds/default.world`；
- 如果没有 `default.world`，自动使用 `config/worlds/` 下按文件名排序的第一个 `.world` 文件；
- RViz 优先使用 `config/rviz/default.rviz`；
- 如果没有 `default.rviz`，自动使用 `config/rviz/` 下按文件名排序的第一个 `.rviz` 文件；
- 如果找不到可用文件，启动会在终端打印清楚错误。

当前工程里已经检测到：

```text
src/version_car_sim/config/worlds/car_01.world
src/version_car_sim/config/rviz/car_01.rviz
```

所以不传参数时会自动使用这两个文件。

指定文件名启动，不需要写绝对路径：

```bash
ros2 launch version_car_sim sim.launch.py gui:=true rviz:=true world_file:=car_01.world rviz_config_file:=car_01.rviz auto_start:=false
```

也支持从 `ros2_ws` 目录传包内相对路径：

```bash
ros2 launch version_car_sim sim.launch.py gui:=true rviz:=true world_file:=src/version_car_sim/config/worlds/car_01.world rviz_config_file:=src/version_car_sim/config/rviz/car_01.rviz auto_start:=false
```

启动后流程：

1. Gazebo 加载你保存的 world；
2. RViz 加载你保存的 rviz 配置；
3. 小车进入指定场景；
4. 你可以在 Gazebo 中检查障碍物；
5. 你可以用 `/start_pose_2d` 或 `/start_pose` 命令设置起点；
6. 你可以在 RViz 中用 `2D Goal Pose` 设置终点；
7. 发布 `/start_navigation` 后小车开始运动。

如果保存的 world 里已经包含 `version_car` 模型，`spawn_car:=auto` 会自动跳过重复 spawn，避免生成两辆同名小车；如果 world 里没有小车，则会自动 spawn `models/version_car.sdf`。也可以显式设置 `spawn_car:=true` 或 `spawn_car:=false`。

## 关键 topic

- `/scan`：Gazebo MID360 近似 360 度 LaserScan（2D 模式下使用）。
- `/mid360_points`：MID360 3D 点云（所有 MID360 模式均支持，frame=`mid360_link`）。
- `/imu`：Gazebo IMU 数据（200 Hz，frame=`mid360_link`，用于 FAST-LIO）。
- `/odom`：Gazebo 里程计（planar_move ground truth，frame=`odom`）。
- `/cloud_registered`：FAST-LIO 注册点云（stub 模式转发自 /mid360_points）。
- `/cloud_registered_body`：FAST-LIO 车体系注册点云。
- `/Laser_map`：FAST-LIO 累积 3D 点云地图（核心建图成果）。
- `/Odometry`：FAST-LIO 里程计（real 模式 frame=`camera_init`，stub 模式 frame=`map`）。
- `/path`：FAST-LIO 轨迹。
- `/fast_lio/odom`：FAST-LIO 里程计（旧版 stub 输出）。
- `/fast_lio/path`：FAST-LIO 路径（旧版 stub 输出）。
- `/map`：建图栅格（2D 模式或 3D→2D 投影）。
- `/costmap_2d`、`/obstacle_points_2d`：3D→2D 投影输出。
- `/planned_path`：A* 规划路径。
- `/actual_path`：由 `/odom` 实时记录的小车实际走过轨迹。
- `/start_pose_2d`：命令式设置起点，类型为 `geometry_msgs/Pose2D`，包含 `x`、`y`、`theta`。
- `/start_pose`：命令式设置起点，类型为 `geometry_msgs/PoseStamped`。
- `/start_pose_applied`：Gazebo 成功应用起点后发布的确认位姿。
- `/goal_pose`：手动终点，RViz 的 `2D Goal Pose` 默认发布到这里，也可以用命令行发布。
- `/cmd_vel_raw`：A* + Pure Pursuit 输出的原始速度命令。
- `/cmd_vel`：经过 `/scan` 或 `/obstacle_points_2d` 局部避障后的最终速度命令。
- `/mcu_frame_hex`：按最终 `/cmd_vel` 计算阿克曼后轮速度和前轮目标转角后，打包出的 MCU 串口协议十六进制帧。
- `/start_navigation`：`std_msgs/Bool`，默认 `auto_start:=false` 时发布 `{data: true}` 后小车才开始规划和运动。

## 可视化与结果保存

RViz 配置 `version_car_sim.rviz` 默认显示：

- `/map`
- `/scan`
- `/planned_path`
- `/actual_path`
- `/odom`
- TF

Gazebo 中开启 `show_gazebo_trail:=true` 时，小车每移动一段距离会生成黄色小球作为实际轨迹标记。该轨迹标记是可视化对象，不带碰撞体，不会作为新障碍物影响雷达。

到达 `/goal_pose` 且距离小于 `goal_tolerance` 后，`trajectory_recorder` 会向 `/cmd_vel_raw` 发布停止命令，并在下面目录按时间创建结果：

```bash
ls ros2_ws/sim_results/
```

每次运行保存：

- `map.pgm`
- `map.yaml`
- `actual_path.csv`
- `actual_path.json`
- `planned_path.csv`
- `map_with_path.png`
- `gazebo_actual_path.sdf`

`map_with_path.png` 会把栅格地图、蓝色规划路径、红色实际轨迹和绿色目标点叠加到同一张图片里，方便直接检查仿真结果。

## 后续接实物

真实 MID360 接入时，先把 Livox 驱动输出的点云转换成 `/scan` 或直接改 mapper 订阅点云；其余规划和局部避障可以继续沿用。当前 `mcu_protocol_bridge` 是 Gazebo dry-run 模式，只发布 `/mcu_frame_hex`，不会打开真实串口；后续接 STM32 时再单独恢复串口发送层。真实前轮闭环控制时，需要接入 `front_steering_feedback` 或等价的 `delta_current` 反馈，再把目标转角误差转换为伺服速度命令。

## 限制

当前 2D LaserScan 建图是轻量版 occupancy grid，适合先验证闭环；后面做正式导航时建议替换为 `slam_toolbox` 或 Livox 适配的 3D/2D SLAM，再把 A* 控制替换成 Nav2 或你自己的局部规划器。

当前 MID360 3D 建图模式使用 `fast_lio_stub` 占位（因为 FAST-LIO2 ROS2 在当前环境中因 GitHub 无网络访问无法安装）。Stub 模式转发 Gazebo ground truth，没有 SLAM 漂移，不能体现 FAST-LIO2 的真实建图能力。安装 FAST-LIO2 后使用 `fast_lio_mode:=real` 可获得真实建图效果。

当前 Gazebo 中保留四轮阿克曼外观，但运动插件采用平面运动近似来保证 `/cmd_vel`、`/odom`、建图和规划闭环稳定。真实上车时仍以 `mcu_protocol_bridge` 输出的 `rear_v` 和 `front_delta` 为协议层基准，不在仿真启动文件中打开真实串口。ROS 2 Humble 环境中虽然存在 `libgazebo_ros_ackermann_drive.so`，但它在当前轻量自建四轮模型上一运动就会导致 `/odom` 出现 NaN，因此本阶段先使用稳定近似运动层。

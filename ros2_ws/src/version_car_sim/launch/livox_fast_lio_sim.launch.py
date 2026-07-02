from glob import glob  # 导入启动文件需要的依赖。
import math  # 导入启动文件需要的依赖。
import os  # 导入启动文件需要的依赖。
import xml.etree.ElementTree as ET  # 导入启动文件需要的依赖。

from ament_index_python.packages import get_package_share_directory  # 导入启动文件需要的依赖。
from launch import LaunchDescription  # 导入启动文件需要的依赖。
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction  # 导入启动文件需要的依赖。
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 导入启动文件需要的依赖。
from launch.substitutions import LaunchConfiguration  # 导入启动文件需要的依赖。
from launch.conditions import IfCondition  # 导入启动文件需要的依赖。
from launch_ros.actions import Node  # 导入启动文件需要的依赖。
from launch_ros.parameter_descriptions import ParameterValue  # 导入启动文件需要的依赖。


def find_default_file(directory, preferred_name, pattern, description):  # 定义查找默认配置文件的函数。
    preferred = os.path.join(directory, preferred_name)  # 拼接首选文件路径。
    if os.path.isfile(preferred):  # 判断条件是否成立。
        return preferred  # 返回计算结果。

    matches = sorted(glob(os.path.join(directory, pattern)))  # 查找匹配的候选文件。
    if matches:  # 判断条件是否成立。
        return matches[0]  # 返回计算结果。

    raise RuntimeError(  # 抛出运行时错误提示。
        f'No usable {description} was found in {directory}. '  # 执行该行配置逻辑。
        f'Create {preferred_name} there, add a file matching {pattern}, '  # 执行该行配置逻辑。
        'or pass an absolute path through the launch argument.')  # 执行该行配置逻辑。


def resolve_config_file(raw_value, directory, preferred_name, pattern, description, arg_name):  # 定义解析配置文件路径的函数。
    value = raw_value.strip()  # 读取并规范化参数值。
    if value:  # 判断条件是否成立。
        expanded = os.path.expanduser(value)  # 展开用户目录路径。
        path = expanded if os.path.isabs(expanded) else os.path.join(directory, expanded)  # 计算最终文件路径。
    else:  # 处理其他情况。
        path = find_default_file(directory, preferred_name, pattern, description)  # 计算最终文件路径。

    if not os.path.isfile(path):  # 判断条件是否成立。
        raise RuntimeError(  # 抛出运行时错误提示。
            f'{description} file from {arg_name}: {path} does not exist. '  # 执行该行配置逻辑。
            f'Use a filename under {directory} or pass an absolute path.')  # 执行该行配置逻辑。
    return path  # 返回计算结果。


def world_has_direct_model(world_path, model_name):  # 定义检查 world 是否已包含车模的函数。
    try:  # 开始执行可能失败的解析逻辑。
        root = ET.parse(world_path).getroot()  # 解析 XML 根节点。
    except ET.ParseError:  # 处理解析失败的情况。
        return False  # 返回计算结果。

    for world in root.iter('world'):  # 遍历相关集合。
        for child in list(world):  # 遍历相关集合。
            if child.tag == 'model' and child.attrib.get('name') == model_name:  # 判断条件是否成立。
                return True  # 返回计算结果。
    return False  # 返回计算结果。


def should_spawn_car(context, world_path):  # 定义是否需要额外生成小车的判断函数。
    value = LaunchConfiguration('spawn_car').perform(context).strip().lower()  # 读取并规范化参数值。
    if value in ('true', '1', 'yes', 'on'):  # 判断条件是否成立。
        return True  # 返回计算结果。
    if value in ('false', '0', 'no', 'off'):  # 判断条件是否成立。
        return False  # 返回计算结果。
    if value != 'auto':  # 判断条件是否成立。
        raise RuntimeError(  # 抛出运行时错误提示。
            'spawn_car must be one of: auto, true, false. '  # 执行该行配置逻辑。
            f'Current value: {value}')  # 执行该行配置逻辑。
    return not world_has_direct_model(world_path, 'version_car')  # 返回计算结果。


def package_available(package_name):  # 判断某个 ROS2 包当前是否已经 source 到环境中。
    try:  # 开始执行可能失败的包查找逻辑。
        get_package_share_directory(package_name)  # 查询包共享目录。
        return True  # 包存在时返回 True。
    except Exception:  # 包不存在或环境未 source 时进入这里。
        return False  # 返回 False 供 launch 做清晰报错。


def launch_setup(context, *args, **kwargs):  # 定义启动时动态组装节点的主函数。
    package_share = get_package_share_directory('version_car_sim')  # 获取当前仿真包共享目录。
    gazebo_share = get_package_share_directory('gazebo_ros')  # 获取 Gazebo ROS 包共享目录。

    config_worlds_dir = os.path.join(package_share, 'config', 'worlds')  # 拼接 world 配置目录。
    config_rviz_dir = os.path.join(package_share, 'config', 'rviz')  # 拼接 RViz 配置目录。

    world_arg = LaunchConfiguration('world_file').perform(context)  # 读取 world_file 启动参数。
    legacy_world_arg = LaunchConfiguration('world').perform(context)  # 读取旧版 world 启动参数。
    if not world_arg.strip() and legacy_world_arg.strip():  # 判断条件是否成立。
        world_arg = legacy_world_arg  # 读取 world_file 启动参数。

    world_path = resolve_config_file(  # 解析最终 Gazebo world 路径。
        world_arg,  # 继续当前参数列表。
        config_worlds_dir,  # 继续当前参数列表。
        'livox_fast_lio_world.world',  # 继续当前参数列表。
        '*.world',  # 继续当前参数列表。
        'Gazebo world',  # 继续当前参数列表。
        'world_file',  # 继续当前参数列表。
    )  # 结束当前配置结构。
    rviz_config = resolve_config_file(  # 解析最终 RViz 配置路径。
        LaunchConfiguration('rviz_config_file').perform(context),  # 读取启动参数配置。
        config_rviz_dir,  # 继续当前参数列表。
        'livox_fast_lio.rviz',  # 继续当前参数列表。
        '*.rviz',  # 继续当前参数列表。
        'RViz config',  # 继续当前参数列表。
        'rviz_config_file',  # 继续当前参数列表。
    )  # 结束当前配置结构。

    # 使用 version_car_livox3d.sdf 模型，包含 MID360 3D 雷达和 IMU 传感器。
    car_model = os.path.join(package_share, 'models', 'version_car_livox3d.sdf')  # 拼接小车 SDF 模型路径。
    fast_lio_config = os.path.join(package_share, 'config', 'fast_lio', 'mid360_sim.yaml')  # 拼接 FAST-LIO 仿真参数文件。

    # ── 整车安全半径计算 ──────────────────────────────────────────────────
    # 始终保持 safety_diameter_scale = 1.6。
    # 当前四轮参数下 vehicle_safety_radius ≈ 0.736 m。
    model_wheel_base = 0.66  # 设置模型轴距。
    model_track_width = 0.62  # 设置模型轮距。
    safety_diameter_scale = 1.6  # 设置安全半径缩放系数（必须保持 1.6）。
    front_wheel_center_x = 0.34  # 设置前轮中心相对 base_link 的 x 坐标。
    rear_wheel_center_x = -0.32  # 设置后轮中心相对 base_link 的 x 坐标。
    half_track_width = model_track_width / 2.0  # 计算半轮距。
    max_distance_from_center_to_wheel = max(  # 计算 base_link 到四个轮心的最大距离。
        math.hypot(front_wheel_center_x, half_track_width),  # 计算前轮轮心距离。
        math.hypot(rear_wheel_center_x, half_track_width),  # 计算后轮轮心距离。
    )  # 结束最大轮心距离计算。
    vehicle_safety_radius = safety_diameter_scale * max_distance_from_center_to_wheel  # 计算整车安全半径。
    hard_collision_radius = vehicle_safety_radius  # 设置障碍物硬碰撞膨胀半径不小于整车安全半径。
    soft_inflation_radius = vehicle_safety_radius + 0.30  # 设置障碍物软代价膨胀半径。

    gui = LaunchConfiguration('gui')  # 读取是否打开 Gazebo GUI。
    rviz = LaunchConfiguration('rviz')  # 读取是否打开 RViz。
    use_sim_time = LaunchConfiguration('use_sim_time')  # 读取是否使用仿真时间。
    auto_start = LaunchConfiguration('auto_start')  # 读取是否自动开始导航。
    goal_tolerance = LaunchConfiguration('goal_tolerance')  # 读取目标到达容差。
    save_results = LaunchConfiguration('save_results')  # 读取是否保存仿真结果。
    show_gazebo_trail = LaunchConfiguration('show_gazebo_trail')  # 读取是否显示 Gazebo 轨迹。
    path_sample_distance = LaunchConfiguration('path_sample_distance')  # 读取轨迹采样距离。
    path_sample_period = LaunchConfiguration('path_sample_period')  # 读取轨迹采样周期。
    fast_lio_mode = LaunchConfiguration('fast_lio_mode')  # 读取 FAST-LIO 模式：stub | real。
    planner_backend = LaunchConfiguration('planner_backend')  # 读取规划后端：astar | ego。
    drone_id = LaunchConfiguration('drone_id')  # 读取 ego-planner 的无人机编号参数。
    ego_goal_z = LaunchConfiguration('ego_goal_z')  # 读取 ego-planner 目标高度参数。
    ego_map_size_x = LaunchConfiguration('ego_map_size_x')  # 读取 ego-planner 地图 x 尺寸。
    ego_map_size_y = LaunchConfiguration('ego_map_size_y')  # 读取 ego-planner 地图 y 尺寸。
    ego_map_size_z = LaunchConfiguration('ego_map_size_z')  # 读取 ego-planner 地图 z 尺寸。
    ego_max_vel = LaunchConfiguration('ego_max_vel')  # 读取 ego-planner 最大速度。
    ego_max_acc = LaunchConfiguration('ego_max_acc')  # 读取 ego-planner 最大加速度。
    ego_planning_horizon = LaunchConfiguration('ego_planning_horizon')  # 读取 ego-planner 规划视距。
    spawn_car = should_spawn_car(context, world_path)  # 判断是否需要生成车模。

    actions = [  # 开始定义启动动作列表。
        LogInfo(msg=f'LIVOX/FAST-LIO SIMULATION MODE'),  # 输出启动日志。
        LogInfo(msg=f'Using Gazebo world: {world_path}'),  # 输出启动日志。
        LogInfo(msg=f'Using RViz config: {rviz_config}'),  # 输出启动日志。
        LogInfo(msg=f'FAST-LIO mode: {fast_lio_mode.perform(context)}'),  # 输出启动日志。
        LogInfo(msg=f'Planner backend: {planner_backend.perform(context)}'),  # 输出启动日志。
        LogInfo(msg=f'FAST-LIO config: {fast_lio_config}'),  # 输出启动日志。
        LogInfo(msg=f'spawn_car resolved to: {spawn_car}'),  # 输出启动日志。
        LogInfo(msg=f'vehicle_safety_radius={vehicle_safety_radius:.3f} m, '  # 输出启动日志。
                f'hard_collision_radius={hard_collision_radius:.3f} m, '
                f'soft_inflation_radius={soft_inflation_radius:.3f} m'),  # 执行该行配置逻辑。
        IncludeLaunchDescription(  # 包含另一个 launch 文件。
            PythonLaunchDescriptionSource(  # 指定被包含的 Python launch 文件。
                os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')  # 执行该行配置逻辑。
            ),  # 结束当前配置结构。
            launch_arguments={  # 给变量赋值。
                'world': world_path,  # 传递 Gazebo 世界文件。
                'gui': gui,  # 设置是否打开 Gazebo 图形界面。
                'verbose': 'false',  # 设置 Gazebo 日志详细程度。
            }.items(),  # 继续当前参数列表。
        ),  # 结束当前配置结构。
    ]  # 结束当前配置结构。

    if spawn_car:  # 判断条件是否成立。
        actions.append(Node(  # 追加一个启动动作。
            package='gazebo_ros',  # 给变量赋值。
            executable='spawn_entity.py',  # 给变量赋值。
            arguments=[  # 给变量赋值。
                '-entity', 'version_car',  # 继续当前参数列表。
                '-file', car_model,  # 继续当前参数列表。
                '-x', LaunchConfiguration('start_x'),  # 继续当前参数列表。
                '-y', LaunchConfiguration('start_y'),  # 继续当前参数列表。
                '-Y', LaunchConfiguration('start_yaw'),  # 继续当前参数列表。
            ],  # 结束当前配置结构。
            output='screen',  # 给变量赋值。
        ))  # 执行该行配置逻辑。
    else:  # 处理其他情况。
        actions.append(LogInfo(  # 追加一个启动动作。
            msg='Skipping spawn_entity because the selected world already contains '  # 给变量赋值。
                'version_car, or spawn_car:=false was requested.'))  # 给变量赋值。

    # ── 基础 TF ───────────────────────────────────────────────────────────
    # Gazebo planar_move 发布 odom->base_link；这里补齐 map->odom 和
    # base_link->mid360_link，避免 FAST-LIO stub、点云投影和 RViz 依赖
    # Gazebo 是否发布固定关节 TF。
    actions.extend([  # 批量追加基础 TF 发布节点。
        Node(  # 发布 map 到 odom 的静态 TF。
            package='tf2_ros',  # 给变量赋值。
            executable='static_transform_publisher',  # 给变量赋值。
            name='map_to_odom_static_tf',  # 给变量赋值。
            arguments=[  # 给变量赋值。
                '--x', '0.0', '--y', '0.0', '--z', '0.0',  # 设置平移。
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',  # 设置旋转。
                '--frame-id', 'map', '--child-frame-id', 'odom',  # 设置父子坐标系。
            ],  # 结束当前配置结构。
            output='screen',  # 给变量赋值。
        ),  # 结束当前配置结构。
        Node(  # 发布 base_link 到 MID360 雷达坐标系的静态 TF。
            package='tf2_ros',  # 给变量赋值。
            executable='static_transform_publisher',  # 给变量赋值。
            name='base_to_mid360_static_tf',  # 给变量赋值。
            arguments=[  # 给变量赋值。
                '--x', '0.08', '--y', '0.0', '--z', '0.48',  # 与 version_car_livox3d.sdf 中 mid360_link 安装位姿一致。
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',  # 雷达与车体同向安装。
                '--frame-id', 'base_link', '--child-frame-id', 'mid360_link',  # 设置父子坐标系。
            ],  # 结束当前配置结构。
            output='screen',  # 给变量赋值。
        ),  # 结束当前配置结构。
    ])  # 结束当前配置结构。

    # ── FAST-LIO / FAST-LIO stub ─────────────────────────────────────────
    # stub 模式：使用 Gazebo /odom 作为 FAST-LIO 风格里程计，并将
    # /mid360_points 通过 TF 注册到 map 后发布为 /cloud_registered。
    # real 模式：启动已经 source 到环境中的 fast_lio/fastlio_mapping，
    # 输入仍然来自 Gazebo /mid360_points 和 /imu，不启动真实 Livox 驱动。
    fast_lio_mode_value = fast_lio_mode.perform(context).strip().lower()  # 读取 FAST-LIO 模式字符串。
    if fast_lio_mode_value == 'real':  # 判断是否启动真实 FAST-LIO 节点。
        if not package_available('fast_lio'):  # 检查 FAST-LIO 包是否可用。
            raise RuntimeError(  # 包不可用时直接给出明确错误。
                'fast_lio_mode:=real requires package fast_lio. '
                '请先 source Livox 工作区的 install/setup.bash，'
                '或改用 fast_lio_mode:=stub 做纯 Gazebo dry-run。')
        actions.append(LogInfo(msg='Launching real FAST-LIO node from package fast_lio.'))  # 输出启动日志。
        actions.append(Node(  # 启动真实 FAST-LIO 映射节点。
            package='fast_lio',  # 给变量赋值。
            executable='fastlio_mapping',  # 给变量赋值。
            name='laserMapping',  # 给变量赋值。
            output='screen',  # 给变量赋值。
            parameters=[  # 给变量赋值。
                fast_lio_config,  # 加载 FAST-LIO 仿真参数文件。
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},  # 启用仿真时间。
            ],  # 结束当前配置结构。
        ))  # 结束当前配置结构。
        actions.append(Node(  # FAST-LIO 常用 camera_init 世界系与 map 对齐。
            package='tf2_ros',  # 给变量赋值。
            executable='static_transform_publisher',  # 给变量赋值。
            name='map_to_camera_init_static_tf',  # 给变量赋值。
            arguments=[  # 给变量赋值。
                '--x', '0.0', '--y', '0.0', '--z', '0.0',  # 设置平移。
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',  # 设置旋转。
                '--frame-id', 'map', '--child-frame-id', 'camera_init',  # 设置父子坐标系。
            ],  # 结束当前配置结构。
            output='screen',  # 给变量赋值。
        ))  # 结束当前配置结构。
    elif fast_lio_mode_value == 'stub':  # 判断是否启动 dry-run stub。
        actions.append(LogInfo(  # 输出启动日志。
            msg='fast_lio_mode:=stub — dry-run only: Gazebo odom/points are '
                'republished as FAST-LIO-style /Odometry and /cloud_registered.'))  # 给变量赋值。
        actions.append(Node(  # 创建一个 ROS2 节点启动动作。
            package='version_car_sim',  # 给变量赋值。
            executable='fast_lio_stub',  # 给变量赋值。
            name='fast_lio_bridge',  # 给变量赋值。
            output='screen',  # 给变量赋值。
            parameters=[{  # 给变量赋值。
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),  # 启用仿真时间。
                'input_odom_topic': 'odom',  # 设置输入里程计话题（来自 Gazebo planar_move）。
                'input_points_topic': 'mid360_points',  # 设置输入点云话题（来自 Gazebo MID360 传感器）。
                'fast_lio_odom_topic': 'Odometry',  # 对齐 FAST-LIO 常见里程计输出话题。
                'fast_lio_path_topic': 'path',  # 对齐 FAST-LIO 常见路径输出话题。
                'registered_cloud_topic': 'cloud_registered',  # 设置注册点云输出话题。
                'laser_map_topic': 'Laser_map',  # 设置激光地图点云输出话题。
                'map_frame': 'map',  # 设置地图坐标系。
                'transform_cloud_to_map': True,  # 将仿真原始点云注册到 map 后再发布。
                'path_sample_distance': 0.05,  # 设置路径采样距离。
            }],  # 结束当前配置结构。
        ))  # 结束当前配置结构。
    else:  # 处理非法 FAST-LIO 模式。
        raise RuntimeError(  # 抛出运行时错误提示。
            f'Unknown fast_lio_mode: {fast_lio_mode_value}. Use stub or real.')  # 给变量赋值。

    # ── 3D 点云 → 2D 栅格地图投影 ─────────────────────────────────────────
    # pointcloud_to_costmap 接收 /mid360_points 3D 点云，
    # 过滤地面、投影到二维、根据安全半径膨胀后发布 /map 和 /costmap_2d。
    # 同时发布 /obstacle_points_2d 给 local_obstacle_avoidance 使用。
    actions.append(Node(  # 创建一个 ROS2 节点启动动作。
        package='version_car_sim',  # 给变量赋值。
        executable='pointcloud_to_costmap',  # 给变量赋值。
        name='pointcloud_to_costmap',  # 给变量赋值。
        output='screen',  # 给变量赋值。
        parameters=[{  # 给变量赋值。
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),  # 启用仿真时间。
            'point_cloud_topic': 'cloud_registered',  # 设置输入点云话题（优先使用 FAST-LIO 注册点云）。
            'point_cloud_qos': 'best_effort',  # 设置点云 QoS 为 best_effort。
            'map_topic': 'map',  # 设置输出地图话题。
            'costmap_topic': 'costmap_2d',  # 设置代价地图话题。
            'obstacle_points_topic': 'obstacle_points_2d',  # 设置二维障碍物点云话题。
            'map_frame': 'map',  # 设置地图坐标系。
            'resolution': 0.12,  # 设置栅格地图分辨率。
            'width_m': 120.0,  # 设置地图宽度。
            'height_m': 100.0,  # 设置地图高度。
            'origin_x': -60.0,  # 设置地图原点 x。
            'origin_y': -50.0,  # 设置地图原点 y。
            'publish_map_to_odom_tf': False,  # map->odom 已由 launch 中静态 TF 统一发布。
            'point_stride': 2,  # 设置点云抽样步长。
            'publish_rate': 2.0,  # 设置发布频率。
            'min_obstacle_height': 0.08,  # 设置障碍物最小高度（过滤地面）。
            'max_obstacle_height': 1.60,  # 设置障碍物最大高度。
            'max_range': 40.0,  # 设置最大有效测距。
            'hard_collision_radius': hard_collision_radius,  # 设置硬碰撞膨胀半径。
            'soft_inflation_radius': soft_inflation_radius,  # 设置软代价膨胀半径。
            'boundary_inflation_radius': vehicle_safety_radius + 0.25,  # 设置地图边界膨胀半径。
            'vehicle_safety_scale': safety_diameter_scale,  # 设置安全半径缩放系数。
        }],  # 结束当前配置结构。
    ))  # 结束当前配置结构。

    # ── A* 全局路径规划 ────────────────────────────────────────────────────
    # astar_planner 直接使用 /map 做规划，输出 /planned_path 和 /cmd_vel_raw。
    actions.append(Node(  # 创建一个 ROS2 节点启动动作。
        package='version_car_sim',  # 给变量赋值。
        executable='astar_planner',  # 给变量赋值。
        name='astar_planner',  # 给变量赋值。
        output='screen',  # 给变量赋值。
        parameters=[{  # 给变量赋值。
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),  # 启用仿真时间。
            'map_topic': 'map',  # 设置地图话题。
            'goal_topic': 'goal_pose',  # 设置目标点话题。
            'auto_start': ParameterValue(auto_start, value_type=bool),  # 设置是否自动开始导航。
            'start_topic': 'start_navigation',  # 设置启动导航话题。
            'replan_request_topic': 'replan_requested',  # 设置重新规划请求话题。
            'wheel_base': model_wheel_base,  # 设置车辆轴距参数。
            'track_width': model_track_width,  # 设置车辆轮距参数。
            'safety_diameter_scale': safety_diameter_scale,  # 设置安全直径缩放系数。
            'max_speed': 0.35,  # 设置最大线速度。
            'max_steer': 0.35,  # 设置最大转向角。
            'max_yaw_rate': 0.35,  # 设置最大角速度。
            'lookahead': 1.50,  # 设置路径跟踪前视距离。
            'goal_tolerance': ParameterValue(goal_tolerance, value_type=float),  # 设置到达目标的容差。
            'cmd_topic': 'cmd_vel_raw',  # 设置原始速度命令话题。
            'debug_topic': 'planner_debug',  # 设置调试信息话题。
            'direct_path_topic': 'direct_path',  # 设置直线路径可视化话题。
            'planning_corridor_topic': 'planning_corridor',  # 设置规划走廊可视化话题。
            'inflated_map_topic': 'inflated_map',  # 设置硬膨胀地图话题。
            'costmap_debug_topic': 'costmap_debug',  # 设置软代价地图话题。
            'robot_radius': vehicle_safety_radius,  # 设置规划使用的车体半径。
            'safety_margin': 0.0,  # 设置额外安全余量。
            'inflation_radius': hard_collision_radius,  # 设置障碍物硬膨胀半径。
            'allow_unknown': True,  # 设置未知区域是否允许通行。
            'boundary_margin': 0.25,  # 设置地图边界保护余量。
            'enforce_map_boundaries': True,  # 启用地图边界硬约束。
            'pass_margin': 0.25,  # 设置通道通过余量。
            'planning_corridor_half_width': 3.0,  # 设置默认规划走廊半宽。
            'max_planning_corridor_half_width': 8.0,  # 设置最大规划走廊半宽。
            'corridor_expansion_step': 1.0,  # 设置规划走廊扩展步长。
            'soft_inflation_radius': soft_inflation_radius,  # 设置障碍物软代价膨胀半径。
        }],  # 结束当前配置结构。
    ))  # 结束当前配置结构。

    # ── 局部避障 ──────────────────────────────────────────────────────────
    # 在 Livox 模式下，local_obstacle_avoidance 使用 /obstacle_points_2d
    # （由 pointcloud_to_costmap 发布的二维投影点云）代替 /scan 进行
    # 反应式避障。边界检查仍然基于 /map。
    # scan_topic 设置为 /scan_livox（不会收到数据），
    # 节点自动回退到使用 obstacle_points_2d。
    actions.append(Node(  # 创建一个 ROS2 节点启动动作。
        package='version_car_sim',  # 给变量赋值。
        executable='local_obstacle_avoidance',  # 给变量赋值。
        name='local_obstacle_avoidance',  # 给变量赋值。
        output='screen',  # 给变量赋值。
        parameters=[{  # 给变量赋值。
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),  # 启用仿真时间。
            'scan_topic': 'scan_livox',  # 设置激光扫描话题（Livox 模式下不使用 /scan，避免冲突）。
            'obstacle_points_topic': 'obstacle_points_2d',  # 设置二维障碍物点云话题（作为避障数据源）。
            'map_topic': 'map',  # 设置地图话题。
            'odom_topic': 'odom',  # 设置里程计话题（来自 Gazebo planar_move）。
            'input_cmd_topic': 'cmd_vel_raw',  # 设置局部避障输入速度话题。
            'output_cmd_topic': 'cmd_vel',  # 设置局部避障输出速度话题。
            'debug_topic': 'local_avoidance_debug',  # 设置调试信息话题。
            'safety_marker_topic': 'vehicle_safety_radius_marker',  # 设置安全圆 marker 话题。
            'goal_topic': 'goal_pose',  # 设置目标点话题。
            'replan_request_topic': 'replan_requested',  # 设置重新规划请求话题。
            'slow_distance': 0.90,  # 设置减速距离。
            'stop_distance': 0.55,  # 设置停车距离。
            'emergency_stop_distance': 0.30,  # 设置急停距离。
            'front_angle_deg': 25.0,  # 设置前方检测角度。
            'side_angle_deg': 90.0,  # 设置侧向检测角度。
            'avoid_turn_speed': 0.30,  # 设置避障转向速度。
            'max_linear_speed': 0.30,  # 设置局部避障最大线速度。
            'max_angular_speed': 0.35,  # 设置局部避障最大角速度。
            'vehicle_safety_radius': vehicle_safety_radius,  # 设置车体安全半径。
            'slow_margin': 0.35,  # 设置减速安全余量。
            'self_filter_distance': 0.42,  # 设置雷达自车点过滤距离。
            'boundary_margin': 0.25,  # 设置地图边界保护余量。
            'enforce_map_boundaries': True,  # 启用地图边界硬约束。
            'max_continuous_avoidance_sec': 3.0,  # 设置连续避障触发重规划时间。
        }],  # 结束当前配置结构。
    ))  # 结束当前配置结构。

    # ── MCU 协议桥（仍只发布 /mcu_frame_hex，不打开真实串口）───────────────
    actions.append(Node(  # 创建一个 ROS2 节点启动动作。
        package='version_car_sim',  # 给变量赋值。
        executable='mcu_protocol_bridge',  # 给变量赋值。
        name='mcu_protocol_bridge',  # 给变量赋值。
        output='screen',  # 给变量赋值。
        parameters=[{  # 给变量赋值。
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),  # 启用仿真时间。
            'tcp_enabled': False,  # 仿真默认只输出 MickX4 十六进制速度帧。
            'tcp_host': '192.168.0.7',  # MickX4 默认底盘地址。
            'tcp_port': 8234,  # MickX4 默认底盘端口。
            'max_linear_speed': 0.35,  # 设置差速底盘最大线速度。
            'max_angular_speed': 0.80,  # 设置差速底盘最大角速度。
            'allow_lateral_speed': False,  # 四轮差速底盘不使用横向速度。
        }],  # 结束当前配置结构。
    ))  # 结束当前配置结构。

    # ── 起点设置 ──────────────────────────────────────────────────────────
    actions.append(Node(  # 创建一个 ROS2 节点启动动作。
        package='version_car_sim',  # 给变量赋值。
        executable='start_pose_setter',  # 给变量赋值。
        name='start_pose_setter',  # 给变量赋值。
        output='screen',  # 给变量赋值。
        parameters=[{  # 给变量赋值。
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),  # 启用仿真时间。
            'entity_name': 'version_car',  # 设置 Gazebo 模型名称。
            'start_pose_topic': 'start_pose',  # 设置三维起点话题。
            'start_pose_2d_topic': 'start_pose_2d',  # 设置二维起点话题。
            'initialpose_topic': 'initialpose',  # 设置 RViz 初始位姿话题。
            'set_entity_state_service': '/gazebo/set_entity_state',  # 设置 Gazebo 位姿服务。
            'reference_frame': 'world',  # 设置 Gazebo 参考坐标系。
            'default_z': 0.0,  # 设置默认高度。
            'prefer_gazebo_cli': True,  # 优先使用 Gazebo 命令行设置位姿。
            'gazebo_cli_timeout': 3.0,  # 设置 Gazebo 命令行超时。
        }],  # 结束当前配置结构。
    ))  # 结束当前配置结构。

    # ── 轨迹记录 ──────────────────────────────────────────────────────────
    actions.append(Node(  # 创建一个 ROS2 节点启动动作。
        package='version_car_sim',  # 给变量赋值。
        executable='trajectory_recorder',  # 给变量赋值。
        name='trajectory_recorder',  # 给变量赋值。
        output='screen',  # 给变量赋值。
        parameters=[{  # 给变量赋值。
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),  # 启用仿真时间。
            'goal_tolerance': ParameterValue(goal_tolerance, value_type=float),  # 设置到达目标的容差。
            'save_results': ParameterValue(save_results, value_type=bool),  # 设置是否保存结果。
            'show_gazebo_trail': ParameterValue(show_gazebo_trail, value_type=bool),  # 设置是否显示 Gazebo 轨迹。
            'path_sample_distance': ParameterValue(  # 设置轨迹采样距离。
                path_sample_distance, value_type=float),  # 给变量赋值。
            'path_sample_period': ParameterValue(path_sample_period, value_type=float),  # 设置轨迹采样周期。
            'results_root': 'sim_results',  # 设置仿真结果保存目录。
        }],  # 结束当前配置结构。
    ))  # 结束当前配置结构。

    # ── RViz ──────────────────────────────────────────────────────────────
    actions.append(Node(  # 创建一个 ROS2 节点启动动作。
        package='rviz2',  # 给变量赋值。
        executable='rviz2',  # 给变量赋值。
        name='rviz2',  # 给变量赋值。
        arguments=['-d', rviz_config],  # 给变量赋值。
        parameters=[{  # 给变量赋值。
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),  # 启用仿真时间。
        }],  # 结束当前配置结构。
        condition=IfCondition(rviz),  # 给变量赋值。
        output='screen',  # 给变量赋值。
    ))  # 结束当前配置结构。

    return actions  # 返回计算结果。


def generate_launch_description():  # 定义 ROS2 launch 的入口函数。
    return LaunchDescription([  # 返回计算结果。
        DeclareLaunchArgument('gui', default_value='true'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument('rviz', default_value='false'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument('use_sim_time', default_value='true'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument('auto_start', default_value='false'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument(  # 声明可由命令行传入的启动参数。
            'fast_lio_mode',  # 继续当前参数列表。
            default_value='stub',  # 给变量赋值。
            description=(  # 给变量赋值。
                'FAST-LIO operational mode: stub (republish Gazebo odom) or '  # 执行该行配置逻辑。
                'real (use actual FAST-LIO2 binary; requires FAST-LIO2 compiled).'  # 执行该行配置逻辑。
            ),  # 继续当前参数列表。
        ),  # 结束当前配置结构。
        DeclareLaunchArgument(  # 声明可由命令行传入的启动参数。
            'world_file',  # 继续当前参数列表。
            default_value='',  # 给变量赋值。
            description=(  # 给变量赋值。
                'Gazebo world filename under config/worlds or an absolute path. '  # 执行该行配置逻辑。
                'Empty means livox_fast_lio_world.world, then the first *.world file.')),  # 继续当前参数列表。
        DeclareLaunchArgument(  # 声明可由命令行传入的启动参数。
            'rviz_config_file',  # 继续当前参数列表。
            default_value='',  # 给变量赋值。
            description=(  # 给变量赋值。
                'RViz config filename under config/rviz or an absolute path. '  # 执行该行配置逻辑。
                'Empty means livox_fast_lio.rviz, then the first *.rviz file.')),  # 继续当前参数列表。
        DeclareLaunchArgument(  # 声明可由命令行传入的启动参数。
            'world',  # 继续当前参数列表。
            default_value='',  # 给变量赋值。
            description='Legacy alias for world_file. Prefer world_file.'),  # 给变量赋值。
        DeclareLaunchArgument(  # 声明可由命令行传入的启动参数。
            'spawn_car',  # 继续当前参数列表。
            default_value='auto',  # 给变量赋值。
            description=(  # 给变量赋值。
                'auto skips spawn if the selected world already contains version_car; '  # 执行该行配置逻辑。
                'true always spawns; false never spawns.')),  # 继续当前参数列表。
        DeclareLaunchArgument('start_x', default_value='-10.0'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument('start_y', default_value='-10.0'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument('start_yaw', default_value='0.0'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument('goal_tolerance', default_value='0.3'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument('save_results', default_value='true'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument('show_gazebo_trail', default_value='true'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument('path_sample_distance', default_value='0.05'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument('path_sample_period', default_value='0.2'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument('serial_enabled', default_value='false'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument('serial_port', default_value=''),  # 声明可由命令行传入的启动参数。
        OpaqueFunction(function=launch_setup),  # 注册动态启动配置函数。
    ])  # 结束当前配置结构。

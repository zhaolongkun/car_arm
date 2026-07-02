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
        'default.world',  # 继续当前参数列表。
        '*.world',  # 继续当前参数列表。
        'Gazebo world',  # 继续当前参数列表。
        'world_file',  # 继续当前参数列表。
    )  # 结束当前配置结构。
    rviz_config = resolve_config_file(  # 解析最终 RViz 配置路径。
        LaunchConfiguration('rviz_config_file').perform(context),  # 读取启动参数配置。
        config_rviz_dir,  # 继续当前参数列表。
        'default.rviz',  # 继续当前参数列表。
        '*.rviz',  # 继续当前参数列表。
        'RViz config',  # 继续当前参数列表。
        'rviz_config_file',  # 继续当前参数列表。
    )  # 结束当前配置结构。

    car_model = os.path.join(package_share, 'models', 'version_car.sdf')  # 拼接小车 SDF 模型路径。

    model_wheel_base = 0.66  # 设置模型轴距。
    model_track_width = 0.62  # 设置模型轮距。
    model_body_width = 0.46  # 设置车身碰撞盒宽度。
    wheel_collision_radius = 0.11  # 设置轮子碰撞半径。
    front_wheel_center_x = 0.34  # 设置前轮中心相对 base_link 的 x 坐标。
    rear_wheel_center_x = -0.32  # 设置后轮中心相对 base_link 的 x 坐标。
    half_track_width = model_track_width / 2.0  # 计算半轮距。
    max_distance_from_center_to_wheel = max(  # 计算 base_link 到四个轮心的最大距离。
        math.hypot(front_wheel_center_x, half_track_width),  # 计算前轮轮心距离。
        math.hypot(rear_wheel_center_x, half_track_width),  # 计算后轮轮心距离。
    )  # 结束最大轮心距离计算。
    footprint_half_width = max(  # 计算小车横向实际半宽。
        0.5 * model_body_width,
        half_track_width + wheel_collision_radius,
    )  # 结束横向实际半宽计算。
    passage_clearance_margin = 0.12  # 设置柱间通行的额外安全余量。
    vehicle_safety_radius = footprint_half_width + passage_clearance_margin  # 按车宽计算通道安全半径。
    safety_diameter_scale = 2.0 * vehicle_safety_radius / max(
        1e-6, math.hypot(model_wheel_base, model_track_width))  # 换算为调试用缩放系数。
    hard_collision_radius = vehicle_safety_radius  # 设置障碍物硬碰撞膨胀半径不小于整车安全半径。
    soft_inflation_radius = vehicle_safety_radius + 0.20  # 设置障碍物软代价膨胀半径。

    gui = LaunchConfiguration('gui')  # 读取是否打开 Gazebo GUI。
    rviz = LaunchConfiguration('rviz')  # 读取是否打开 RViz。
    use_sim_time = LaunchConfiguration('use_sim_time')  # 读取是否使用仿真时间。
    auto_start = LaunchConfiguration('auto_start')  # 读取是否自动开始导航。
    goal_tolerance = LaunchConfiguration('goal_tolerance')  # 读取目标到达容差。
    save_results = LaunchConfiguration('save_results')  # 读取是否保存仿真结果。
    show_gazebo_trail = LaunchConfiguration('show_gazebo_trail')  # 读取是否显示 Gazebo 轨迹。
    path_sample_distance = LaunchConfiguration('path_sample_distance')  # 读取轨迹采样距离。
    path_sample_period = LaunchConfiguration('path_sample_period')  # 读取轨迹采样周期。
    spawn_car = should_spawn_car(context, world_path)  # 判断是否需要生成车模。

    actions = [  # 开始定义启动动作列表。
        LogInfo(msg=f'Using Gazebo world: {world_path}'),  # 输出启动日志。
        LogInfo(msg=f'Using RViz config: {rviz_config}'),  # 输出启动日志。
        LogInfo(msg=f'spawn_car resolved to: {spawn_car}'),  # 输出启动日志。
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

    actions.extend([  # 追加多个启动动作。
        Node(  # 创建一个 ROS2 节点启动动作。
            package='version_car_sim',  # 给变量赋值。
            executable='occupancy_mapper',  # 给变量赋值。
            name='occupancy_mapper',  # 给变量赋值。
            output='screen',  # 给变量赋值。
            parameters=[{  # 给变量赋值。
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),  # 启用仿真时间。
                'resolution': 0.12,  # 设置栅格地图分辨率。
                'width_m': 120.0,  # 设置地图宽度。
                'height_m': 100.0,  # 设置地图高度。
                'ray_stride': 2,  # 设置激光束抽样步长。
                'publish_rate': 2.0,  # 设置发布频率。
                'boundary_inflation_radius': vehicle_safety_radius + 0.20,  # 设置地图边界膨胀半径。
            }],  # 结束当前配置结构。
        ),  # 结束当前配置结构。

        Node(  # 创建一个 ROS2 节点启动动作。
            package='version_car_sim',  # 给变量赋值。
            executable='astar_planner',  # 给变量赋值。
            name='astar_planner',  # 给变量赋值。
            output='screen',  # 给变量赋值。
            parameters=[{  # 给变量赋值。
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),  # 启用仿真时间。
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
                'lookahead': 0.90,  # 设置路径跟踪前视距离。
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
                'boundary_margin': 0.20,  # 设置地图边界保护余量。
                'enforce_map_boundaries': True,  # 启用地图边界硬约束。
                'pass_margin': 0.08,  # 设置通道通过余量。
                'planning_corridor_half_width': 3.0,  # 设置默认规划走廊半宽。
                'max_planning_corridor_half_width': 8.0,  # 设置最大规划走廊半宽。
                'corridor_expansion_step': 1.0,  # 设置规划走廊扩展步长。
                'soft_inflation_radius': soft_inflation_radius,  # 设置障碍物软代价膨胀半径。
            }],  # 结束当前配置结构。
        ),  # 结束当前配置结构。

        Node(  # 创建一个 ROS2 节点启动动作。
            package='version_car_sim',  # 给变量赋值。
            executable='local_obstacle_avoidance',  # 给变量赋值。
            name='local_obstacle_avoidance',  # 给变量赋值。
            output='screen',  # 给变量赋值。
            parameters=[{  # 给变量赋值。
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),  # 启用仿真时间。
                'scan_topic': 'scan',  # 设置激光扫描话题。
                'map_topic': 'map',  # 设置地图话题。
                'odom_topic': 'odom',  # 设置里程计话题。
                'input_cmd_topic': 'cmd_vel_raw',  # 设置局部避障输入速度话题。
                'output_cmd_topic': 'cmd_vel',  # 设置局部避障输出速度话题。
                'debug_topic': 'local_avoidance_debug',  # 设置调试信息话题。
                'safety_marker_topic': 'vehicle_safety_radius_marker',  # 设置安全圆 marker 话题。
                'goal_topic': 'goal_pose',  # 设置目标点话题。
                'replan_request_topic': 'replan_requested',  # 设置重新规划请求话题。
                'slow_distance': 0.75,  # 设置减速距离。
                'stop_distance': 0.45,  # 设置停车距离。
                'emergency_stop_distance': 0.25,  # 设置急停距离。
                'front_angle_deg': 25.0,  # 设置前方检测角度。
                'center_clear_angle_deg': 9.0,  # 设置窄门中心通道检测角度。
                'side_angle_deg': 90.0,  # 设置侧向检测角度。
                'avoid_turn_speed': 0.30,  # 设置避障转向速度。
                'max_linear_speed': 0.30,  # 设置局部避障最大线速度。
                'max_angular_speed': 0.35,  # 设置局部避障最大角速度。
                'vehicle_safety_radius': vehicle_safety_radius,  # 设置车体安全半径。
                'vehicle_half_width': footprint_half_width,  # 设置真实车体半宽。
                'footprint_clearance_margin': passage_clearance_margin,  # 设置车体矩形走廊余量。
                'slow_margin': 0.20,  # 设置减速安全余量。
                'self_filter_distance': 0.42,  # 设置雷达自车点过滤距离。
                'boundary_margin': 0.20,  # 设置地图边界保护余量。
                'enforce_map_boundaries': True,  # 启用地图边界硬约束。
                'narrow_passage_extra_clearance': 0.22,  # 设置窄门中心额外净空。
                'narrow_passage_speed': 0.16,  # 设置窄门通行限速。
                'narrow_passage_max_angular_speed': 0.12,  # 设置窄门通行角速度限制。
                'narrow_passage_centering_gain': 0.60,  # 设置窄门居中控制增益。
                'narrow_passage_lateral_window': 1.60,  # 设置窄门左右测距窗口。
                'max_continuous_avoidance_sec': 3.0,  # 设置连续避障触发重规划时间。
            }],  # 结束当前配置结构。
        ),  # 结束当前配置结构。

        Node(  # 创建一个 ROS2 节点启动动作。
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
        ),  # 结束当前配置结构。

        Node(  # 创建一个 ROS2 节点启动动作。
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
        ),  # 结束当前配置结构。

        Node(  # 创建一个 ROS2 节点启动动作。
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
        ),  # 结束当前配置结构。

        Node(  # 创建一个 ROS2 节点启动动作。
            package='rviz2',  # 给变量赋值。
            executable='rviz2',  # 给变量赋值。
            name='rviz2',  # 给变量赋值。
            arguments=['-d', rviz_config],  # 给变量赋值。
            parameters=[{  # 给变量赋值。
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),  # 启用仿真时间。
            }],  # 结束当前配置结构。
            condition=IfCondition(rviz),  # 给变量赋值。
            output='screen',  # 给变量赋值。
        ),  # 结束当前配置结构。
    ])  # 结束当前配置结构。

    return actions  # 返回计算结果。


def generate_launch_description():  # 定义 ROS2 launch 的入口函数。
    return LaunchDescription([  # 返回计算结果。
        DeclareLaunchArgument('gui', default_value='true'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument('rviz', default_value='false'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument('use_sim_time', default_value='true'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument('auto_start', default_value='false'),  # 声明可由命令行传入的启动参数。
        DeclareLaunchArgument(  # 声明可由命令行传入的启动参数。
            'world_file',  # 继续当前参数列表。
            default_value='',  # 给变量赋值。
            description=(  # 给变量赋值。
                'Gazebo world filename under config/worlds or an absolute path. '  # 执行该行配置逻辑。
                'Empty means default.world, then the first *.world file.')),  # 继续当前参数列表。
        DeclareLaunchArgument(  # 声明可由命令行传入的启动参数。
            'rviz_config_file',  # 继续当前参数列表。
            default_value='',  # 给变量赋值。
            description=(  # 给变量赋值。
                'RViz config filename under config/rviz or an absolute path. '  # 执行该行配置逻辑。
                'Empty means default.rviz, then the first *.rviz file.')),  # 继续当前参数列表。
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

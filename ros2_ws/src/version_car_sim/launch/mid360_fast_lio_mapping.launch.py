from glob import glob
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def find_default_file(directory, preferred_name, pattern, description):
    preferred = os.path.join(directory, preferred_name)
    if os.path.isfile(preferred):
        return preferred

    matches = sorted(glob(os.path.join(directory, pattern)))
    if matches:
        return matches[0]

    raise RuntimeError(
        f'No usable {description} was found in {directory}. '
        f'Create {preferred_name} there, add a file matching {pattern}, '
        'or pass an absolute path through the launch argument.')


def resolve_config_file(raw_value, directory, preferred_name, pattern, description, arg_name):
    value = raw_value.strip()
    if value:
        expanded = os.path.expanduser(value)
        path = expanded if os.path.isabs(expanded) else os.path.join(directory, expanded)
    else:
        path = find_default_file(directory, preferred_name, pattern, description)

    if not os.path.isfile(path):
        raise RuntimeError(
            f'{description} file from {arg_name}: {path} does not exist. '
            f'Use a filename under {directory} or pass an absolute path.')
    return path


def world_has_direct_model(world_path, model_name):
    try:
        root = ET.parse(world_path).getroot()
    except ET.ParseError:
        return False

    for world in root.iter('world'):
        for child in list(world):
            if child.tag == 'model' and child.attrib.get('name') == model_name:
                return True
    return False


def should_spawn_car(context, world_path):
    value = LaunchConfiguration('spawn_car').perform(context).strip().lower()
    if value in ('true', '1', 'yes', 'on'):
        return True
    if value in ('false', '0', 'no', 'off'):
        return False
    if value != 'auto':
        raise RuntimeError(
            'spawn_car must be one of: auto, true, false. '
            f'Current value: {value}')
    return not world_has_direct_model(world_path, 'version_car')


def check_fast_lio_available():
    """Check if FAST-LIO2 package is installed in the workspace."""
    try:
        get_package_share_directory('fast_lio')
        return True
    except Exception:
        return False


def check_nav2_available():
    required_packages = [
        'nav2_bringup',
        'nav2_bt_navigator',
        'nav2_controller',
        'nav2_planner',
        'nav2_lifecycle_manager',
        'nav2_msgs',
    ]
    for package in required_packages:
        try:
            get_package_share_directory(package)
        except Exception:
            return False
    return True


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory('version_car_sim')
    config_worlds_dir = os.path.join(package_share, 'config', 'worlds')
    config_rviz_dir = os.path.join(package_share, 'config', 'rviz')

    world_arg = LaunchConfiguration('world_file').perform(context)
    legacy_world_arg = LaunchConfiguration('world').perform(context)
    if not world_arg.strip() and legacy_world_arg.strip():
        world_arg = legacy_world_arg

    world_path = resolve_config_file(
        world_arg,
        config_worlds_dir,
        'mid360_fast_lio_world.world',
        '*.world',
        'Gazebo world',
        'world_file',
    )
    rviz_config = resolve_config_file(
        LaunchConfiguration('rviz_config_file').perform(context),
        config_rviz_dir,
        'mid360_fast_lio_mapping.rviz',
        '*.rviz',
        'RViz config',
        'rviz_config_file',
    )

    models_dir = os.path.join(package_share, 'models')
    car_model = resolve_config_file(
        LaunchConfiguration('car_model_file').perform(context),
        models_dir,
        'version_car_mid360_fastlio.sdf',
        '*.sdf',
        'Gazebo car model',
        'car_model_file',
    )
    fast_lio_config = os.path.join(
        package_share, 'config', 'fast_lio', 'mid360_gazebo.yaml')
    nav2_params = os.path.join(
        package_share, 'config', 'nav2', 'mid360_nav2.yaml')

    model_wheel_base = 0.40
    model_track_width = 0.40
    safety_diameter_scale = 1.6
    front_wheel_center_x = 0.20
    rear_wheel_center_x = -0.20
    half_track_width = model_track_width / 2.0
    max_distance_from_center_to_wheel = max(
        math.hypot(front_wheel_center_x, half_track_width),
        math.hypot(rear_wheel_center_x, half_track_width),
    )
    vehicle_safety_radius = safety_diameter_scale * max_distance_from_center_to_wheel
    hard_collision_radius = 0.0
    soft_inflation_radius = 0.0

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    fast_lio_mode = LaunchConfiguration('fast_lio_mode')
    enable_mapping_drive = LaunchConfiguration('enable_mapping_drive')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_custom_soft_inflation = LaunchConfiguration('enable_custom_soft_inflation')
    navigation_backend = LaunchConfiguration('navigation_backend')
    auto_start = LaunchConfiguration('auto_start')
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    waypoints = LaunchConfiguration('waypoints')
    spawn_car = should_spawn_car(context, world_path)
    navigation_enabled = (
        enable_navigation.perform(context).strip().lower()
        in ('true', '1', 'yes', 'on')
    )
    navigation_backend_value = navigation_backend.perform(context).strip().lower()
    if navigation_backend_value == 'astar':
        navigation_backend_value = 'custom'
    if navigation_backend_value not in ('nav2', 'custom'):
        raise RuntimeError(
            'navigation_backend must be one of: nav2, custom. '
            f'Current value: {navigation_backend_value}')

    actions = [
        LogInfo(msg='=' * 60),
        LogInfo(msg='MID360 + FAST-LIO 3D Mapping Simulation'),
        LogInfo(msg='=' * 60),
        LogInfo(msg=f'Using Gazebo world: {world_path}'),
        LogInfo(msg=f'Using RViz config: {rviz_config}'),
        LogInfo(msg=f'Using car model: {car_model}'),
        LogInfo(msg=f'FAST-LIO mode: {fast_lio_mode.perform(context)}'),
        LogInfo(msg=f'FAST-LIO config: {fast_lio_config}'),
        LogInfo(msg=f'Mapping drive enabled: {enable_mapping_drive.perform(context)}'),
        LogInfo(msg=f'Navigation enabled: {enable_navigation.perform(context)}'),
        LogInfo(msg=f'Navigation backend: {navigation_backend_value}'),
        LogInfo(msg=f'Default goal: x={goal_x.perform(context)}, y={goal_y.perform(context)}'),
        LogInfo(msg=f'spawn_car resolved to: {spawn_car}'),
        LogInfo(msg=f'vehicle_safety_radius={vehicle_safety_radius:.3f} m'),
        LogInfo(msg='=' * 60),

        ExecuteProcess(
            cmd=[
                'gzserver',
                world_path,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                '-s', 'libgazebo_ros_force_system.so',
            ],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['gzclient'],
            condition=IfCondition(gui),
            output='screen',
        ),
    ]

    if spawn_car:
        actions.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'version_car',
                '-file', car_model,
                '-x', LaunchConfiguration('start_x'),
                '-y', LaunchConfiguration('start_y'),
                '-Y', LaunchConfiguration('start_yaw'),
            ],
            output='screen',
        ))
    else:
        actions.append(LogInfo(
            msg='Skipping spawn_entity because the selected world already contains '
                'version_car, or spawn_car:=false was requested.'))

    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_mid360_static_tf',
        arguments=[
            '--x', '0.24', '--y', '0.0', '--z', '0.115',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'mid360_link',
        ],
        output='screen',
    ))

    # ── FAST-LIO2 (real mode) ──────────────────────────────────────────
    fast_lio_mode_value = fast_lio_mode.perform(context).strip().lower()
    fast_lio_available = check_fast_lio_available()

    if fast_lio_mode_value == 'real':
        if not fast_lio_available:
            actions.append(LogInfo(
                msg='ERROR: FAST-LIO2 package not found! '
                    'fast_lio_mode:=real requires FAST-LIO2 compiled in the workspace.\n'
                    '  To install:\n'
                    '    cd <project-root>/ros2_ws/src\n'
                    '    git clone https://github.com/hku-mars/FAST-LIO.git -b ros2\n'
                    '    cd <project-root>/ros2_ws\n'
                    '    colcon build --symlink-install --packages-select fast_lio\n'
                    '  See README.md "MID360 + FAST-LIO 3D Mapping Simulation" for details.'))
        else:
            actions.append(LogInfo(msg='Launching real FAST-LIO2 node...'))
            actions.append(Node(
                package='fast_lio',
                executable='fastlio_mapping',
                name='laserMapping',
                output='screen',
                parameters=[fast_lio_config],
                remappings=[],
            ))
            # FAST-LIO2 publishes in "camera_init" frame; add static TF map->camera_init
            actions.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_camera_init_static_tf',
                arguments=[
                    '--x', LaunchConfiguration('start_x'),
                    '--y', LaunchConfiguration('start_y'),
                    '--z', '0.275',
                    '--roll', '0.0',
                    '--pitch', '0.0',
                    '--yaw', LaunchConfiguration('start_yaw'),
                    '--frame-id', 'map', '--child-frame-id', 'camera_init',
                ],
                output='screen',
            ))
            # Keep Gazebo frames visible in RViz while FAST-LIO publishes map data in camera_init.
            actions.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom_static_tf',
                arguments=[
                    '--x', '0.0', '--y', '0.0', '--z', '0.0',
                    '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                    '--frame-id', 'map', '--child-frame-id', 'odom',
                ],
                output='screen',
            ))
    elif fast_lio_mode_value == 'stub':
        actions.append(LogInfo(
            msg='WARNING: fast_lio_mode:=stub — using Gazebo /odom as FAST-LIO odometry. '
                'This is NOT real FAST-LIO mapping. For real FAST-LIO, use fast_lio_mode:=real.'))
        # Publish static map->odom TF (since FAST-LIO is not providing localization)
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            output='screen',
        ))
        actions.append(Node(
            package='version_car_sim',
            executable='fast_lio_stub',
            name='fast_lio_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'input_odom_topic': 'odom',
                'input_points_topic': 'mid360_points',
                'fast_lio_odom_topic': 'Odometry',
                'fast_lio_path_topic': 'path',
                'registered_cloud_topic': 'cloud_registered',
                'laser_map_topic': 'Laser_map',
                'map_frame': 'map',
            }],
        ))
    else:
        raise RuntimeError(
            f'Unknown fast_lio_mode: {fast_lio_mode_value}. '
            'Use "real" for real FAST-LIO2 or "stub" for simulation stub.')

    # ── 3D point cloud -> 2D planning map ──────────────────────────────
    actions.append(Node(
        package='version_car_sim',
        executable='pointcloud_to_costmap',
        name='pointcloud_to_costmap',
        output='screen',
        condition=IfCondition(enable_navigation),
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'point_cloud_topic': 'mid360_points',
            'point_cloud_qos': 'best_effort',
            'map_topic': 'map',
            'costmap_topic': 'costmap_2d',
            'obstacle_points_topic': 'obstacle_points_2d',
            'map_frame': 'map',
            'resolution': 0.08,
            'width_m': 40.0,
            'height_m': 40.0,
            'origin_x': -20.0,
            'origin_y': -20.0,
            'publish_map_to_odom_tf': False,
            'point_stride': 2,
            'publish_rate': 5.0,
            'min_range': 0.60,
            'self_filter_x_min': -0.70,
            'self_filter_x_max': 0.85,
            'self_filter_y_min': -0.50,
            'self_filter_y_max': 0.50,
            'self_filter_z_min': -0.20,
            'self_filter_z_max': 1.25,
            'self_filtered_points_topic': 'self_filtered_points_debug',
            'front_debug_x_min': 0.05,
            'front_debug_x_max': 2.00,
            'front_debug_abs_y': 0.80,
            'front_debug_warn_distance': 1.20,
            'min_obstacle_height': 0.12,
            'max_obstacle_height': 1.60,
            'max_range': 14.0,
            'persistent_map': False,
            'hard_collision_radius': hard_collision_radius,
            'soft_inflation_radius': soft_inflation_radius,
            'enable_custom_soft_inflation': ParameterValue(
                enable_custom_soft_inflation, value_type=bool),
            'use_vehicle_radius_as_hard_collision': False,
            'boundary_inflation_radius': vehicle_safety_radius + 0.10,
            'vehicle_safety_scale': safety_diameter_scale,
        }],
    ))

    if navigation_enabled and navigation_backend_value == 'nav2':
        if not check_nav2_available():
            raise RuntimeError(
                'navigation_backend:=nav2 was requested, but Nav2 is not installed.\n'
                'Install it first:\n'
                '  sudo apt update\n'
                '  sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup\n'
                'Then restart ./start_3d_fastlio.sh')
        nav2_bringup_share = get_package_share_directory('nav2_bringup')
        actions.append(LogInfo(msg=f'Launching Nav2 with params: {nav2_params}'))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params,
                'autostart': 'true',
            }.items(),
        ))
        actions.append(Node(
            package='version_car_sim',
            executable='nav2_waypoint_commander',
            name='nav2_waypoint_commander',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'action_name': 'navigate_to_pose',
                'map_topic': 'map',
                'odom_topic': 'odom',
                'goal_frame': 'map',
                'goal_x': ParameterValue(goal_x, value_type=float),
                'goal_y': ParameterValue(goal_y, value_type=float),
                'goal_yaw': ParameterValue(goal_yaw, value_type=float),
                'waypoints': ParameterValue(waypoints, value_type=str),
                'use_through_poses': False,
                'wait_for_map': True,
                'wait_for_odom': True,
                'auto_start_navigation': ParameterValue(auto_start, value_type=bool),
                'initial_delay_sec': 10.0,
                'retry_failed_goals': False,
                'goal_result_timeout_sec': 160.0,
            }],
        ))
        actions.append(Node(
            package='version_car_sim',
            executable='cmd_vel_monitor',
            name='cmd_vel_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'nav_cmd_topic': 'cmd_vel_nav',
                'final_cmd_topic': 'cmd_vel',
                'nav_status_topic': 'navigate_to_pose/_action/status',
                'log_period_sec': 1.0,
                'stale_timeout_sec': 1.0,
                'warn_only_during_active_goal': True,
            }],
        ))
    elif navigation_enabled and navigation_backend_value == 'custom':
        actions.append(LogInfo(
            msg='WARNING: navigation_backend:=custom uses the legacy local A* stack. '
                'Use navigation_backend:=nav2 for Nav2.'))
        # ── Legacy custom A* global planner ─────────────────────────────
        actions.append(Node(
            package='version_car_sim',
            executable='astar_planner',
            name='astar_planner',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'map_topic': 'map',
                'goal_topic': 'goal_pose',
                'auto_start': False,
                'start_topic': 'start_navigation',
                'replan_request_topic': 'replan_requested',
                'wheel_base': model_wheel_base,
                'track_width': model_track_width,
                'safety_diameter_scale': safety_diameter_scale,
                'max_speed': 0.36,
                'max_steer': 0.0,
                'max_yaw_rate': 0.95,
                'lookahead': 0.90,
                'min_lookahead': 0.75,
                'max_lookahead': 2.00,
                'lookahead_clearance_distance': 3.00,
                'steering_smoothing_alpha': 1.0,
                'goal_tolerance': ParameterValue(goal_tolerance, value_type=float),
                'cmd_topic': 'cmd_vel_raw',
                'debug_topic': 'planner_debug',
                'direct_path_topic': 'direct_path',
                'planning_corridor_topic': 'planning_corridor',
                'inflated_map_topic': 'inflated_map',
                'costmap_debug_topic': 'costmap_debug',
                'robot_radius': vehicle_safety_radius,
                'safety_margin': 0.0,
                'inflation_radius': hard_collision_radius,
                'allow_unknown': True,
                'boundary_margin': 0.25,
                'enforce_map_boundaries': True,
                'pass_margin': 0.25,
                'planning_corridor_half_width': 3.0,
                'max_planning_corridor_half_width': 8.0,
                'corridor_expansion_step': 1.0,
                'soft_inflation_radius': soft_inflation_radius,
            }],
        ))

        actions.append(Node(
            package='version_car_sim',
            executable='local_obstacle_avoidance',
            name='local_obstacle_avoidance',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'scan_topic': 'scan_livox',
                'obstacle_points_topic': 'obstacle_points_2d',
                'map_topic': 'map',
                'odom_topic': 'odom',
                'input_cmd_topic': 'cmd_vel_raw',
                'output_cmd_topic': 'cmd_vel',
                'debug_topic': 'local_avoidance_debug',
                'safety_marker_topic': 'vehicle_safety_radius_marker',
                'goal_topic': 'goal_pose',
                'replan_request_topic': 'replan_requested',
                'slow_distance': 1.35,
                'stop_distance': 0.78,
                'emergency_stop_distance': 0.38,
                'front_angle_deg': 35.0,
                'center_clear_angle_deg': 12.0,
                'side_angle_deg': 90.0,
                'avoid_turn_speed': 0.55,
                'recovery_reverse_speed': 0.12,
                'reverse_recovery_delay_sec': 0.8,
                'edge_clearance_speed': 0.12,
                'edge_clearance_front_margin': 0.20,
                'max_linear_speed': 0.40,
                'max_angular_speed': 0.70,
                'narrow_passage_speed': 0.20,
                'vehicle_safety_radius': vehicle_safety_radius,
                'slow_margin': 0.35,
                'self_filter_distance': 0.45,
                'self_filter_front': 0.42,
                'self_filter_back': 0.40,
                'self_filter_half_width': 0.34,
                'obstacle_points_timeout': 0.8,
                'prefer_obstacle_points': True,
                'boundary_margin': 0.25,
                'enforce_map_boundaries': True,
                'max_continuous_avoidance_sec': 1.5,
            }],
        ))

        actions.append(Node(
            package='version_car_sim',
            executable='auto_goal_publisher',
            name='auto_goal_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'goal_topic': 'goal_pose',
                'start_topic': 'start_navigation',
                'map_topic': 'map',
                'odom_topic': 'odom',
                'goal_frame': 'map',
                'goal_x': ParameterValue(goal_x, value_type=float),
                'goal_y': ParameterValue(goal_y, value_type=float),
                'goal_yaw': ParameterValue(goal_yaw, value_type=float),
                'waypoints': ParameterValue(waypoints, value_type=str),
                'waypoint_tolerance': 1.0,
                'initial_delay_sec': 8.0,
                'repeat_period_sec': 0.5,
                'publish_repeats': 4,
                'wait_for_map': True,
                'wait_for_odom': True,
                'auto_start_navigation': ParameterValue(auto_start, value_type=bool),
            }],
        ))

    # ── Mapping Drive Node (自动巡航建图) ──────────────────────────────
    actions.append(Node(
        package='version_car_sim',
        executable='mapping_drive_node',
        name='mapping_drive',
        output='screen',
        condition=IfCondition(enable_mapping_drive),
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'mode': 'circle',
            'linear_speed': 0.30,
            'angular_speed': 0.15,
            'cmd_topic': 'cmd_vel',
            'auto_enable': ParameterValue(enable_mapping_drive, value_type=bool),
        }],
    ))

    # ── Gazebo Odom 路径记录（用于对比 FAST-LIO 定位）───────────────────
    actions.append(Node(
        package='version_car_sim',
        executable='trajectory_recorder',
        name='trajectory_recorder',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'path_sample_distance': 0.05,
            'path_sample_period': 0.2,
            'save_results': False,
            'goal_tolerance': ParameterValue(goal_tolerance, value_type=float),
            'results_root': 'sim_results',
        }],
    ))

    # ── RViz ────────────────────────────────────────────────────────────
    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=IfCondition(rviz),
        output='screen',
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'fast_lio_mode',
            default_value='stub',
            description=(
                'FAST-LIO mode: "real" (requires FAST-LIO2 compiled in workspace) '
                'or "stub" (uses Gazebo /odom as placeholder). '
                'Default is stub since FAST-LIO2 may not be installed yet.')),
        DeclareLaunchArgument(
            'enable_mapping_drive',
            default_value='false',
            description=(
                'Auto-drive the car in a circle pattern for mapping data collection. '
                'Set to false for manual /cmd_vel control.')),
        DeclareLaunchArgument(
            'enable_navigation',
            default_value='true',
            description='Enable pointcloud costmap and navigation backend.'),
        DeclareLaunchArgument(
            'enable_custom_soft_inflation',
            default_value='false',
            description=(
                'Enable the debug /costmap_2d soft inflation inside '
                'pointcloud_to_costmap. Keep false when Nav2 InflationLayer '
                'is the main inflation source.')),
        DeclareLaunchArgument(
            'navigation_backend',
            default_value='nav2',
            description='Navigation backend: nav2 or custom. custom is the legacy local A* stack.'),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Publish /start_navigation automatically after the default goal.'),
        DeclareLaunchArgument(
            'world_file',
            default_value='',
            description=(
                'Gazebo world filename under config/worlds or absolute path. '
                'Empty defaults to mid360_fast_lio_world.world.')),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value='',
            description=(
                'RViz config filename under config/rviz or absolute path. '
                'Empty defaults to mid360_fast_lio_mapping.rviz.')),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Legacy alias for world_file.'),
        DeclareLaunchArgument(
            'car_model_file',
            default_value='',
            description=(
                'Gazebo car model filename under models or absolute path. '
                'Empty defaults to version_car_mid360_fastlio.sdf.')),
        DeclareLaunchArgument(
            'spawn_car',
            default_value='auto',
            description=(
                'auto skips spawn if world already contains version_car; '
                'true always spawns; false never spawns.')),
        DeclareLaunchArgument('start_x', default_value='-10.0'),
        DeclareLaunchArgument('start_y', default_value='-10.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.785398'),
        DeclareLaunchArgument('goal_x', default_value='10.0'),
        DeclareLaunchArgument('goal_y', default_value='10.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'waypoints',
            default_value='[]'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.20'),
        OpaqueFunction(function=launch_setup),
    ])

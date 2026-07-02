from glob import glob
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def default_source_config_root(package_share):
    workspace_config = os.path.abspath(
        os.path.join(package_share, '..', '..', '..', '..', 'src',
                     'version_car_sim', 'config'))
    if os.path.isdir(workspace_config):
        return workspace_config
    return os.path.join(package_share, 'config')


def resolve_config_root(raw_value, package_share):
    value = raw_value.strip()
    if value:
        path = os.path.abspath(os.path.expanduser(value))
    else:
        path = default_source_config_root(package_share)

    if not os.path.isdir(path):
        raise RuntimeError(f'Config root does not exist: {path}')
    return path


def resolve_config_dir(config_root, names, description):
    for name in names:
        path = os.path.join(config_root, name)
        if os.path.isdir(path):
            return path
    joined = ', '.join(os.path.join(config_root, name) for name in names)
    raise RuntimeError(f'No usable {description} directory found. Tried: {joined}')


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


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory('version_car_sim')
    gazebo_share = get_package_share_directory('gazebo_ros')

    config_root = resolve_config_root(
        LaunchConfiguration('config_root').perform(context), package_share)
    config_worlds_dir = resolve_config_dir(
        config_root, ('gazebo', 'worlds'), 'Gazebo world')
    config_rviz_dir = resolve_config_dir(config_root, ('rviz',), 'RViz')
    models_dir = os.path.join(package_share, 'models')

    world_path = resolve_config_file(
        LaunchConfiguration('world_file').perform(context),
        config_worlds_dir,
        'car_01.world',
        '*.world',
        'D435i Gazebo world',
        'world_file',
    )
    rviz_config = resolve_config_file(
        LaunchConfiguration('rviz_config_file').perform(context),
        config_rviz_dir,
        'car_01.rviz',
        '*.rviz',
        'D435i RViz config',
        'rviz_config_file',
    )

    car_model = os.path.join(models_dir, 'version_car_d435i.sdf')
    car_urdf = os.path.join(models_dir, 'version_car_d435i.urdf')
    with open(car_urdf, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    model_wheel_base = 0.66
    model_track_width = 0.62
    vehicle_safety_scale = 1.6
    wheel_centers = {
        'front_left': (0.34, 0.31),
        'front_right': (0.34, -0.31),
        'rear_left': (-0.32, 0.31),
        'rear_right': (-0.32, -0.31),
    }
    wheel_distances = {
        name: math.hypot(x, y)
        for name, (x, y) in wheel_centers.items()
    }
    max_wheel_distance = max(wheel_distances.values())
    vehicle_safety_radius = vehicle_safety_scale * max_wheel_distance
    vehicle_safety_parameters = {
        'vehicle_safety_scale': vehicle_safety_scale,
        'front_left_wheel_x': wheel_centers['front_left'][0],
        'front_left_wheel_y': wheel_centers['front_left'][1],
        'front_right_wheel_x': wheel_centers['front_right'][0],
        'front_right_wheel_y': wheel_centers['front_right'][1],
        'rear_left_wheel_x': wheel_centers['rear_left'][0],
        'rear_left_wheel_y': wheel_centers['rear_left'][1],
        'rear_right_wheel_x': wheel_centers['rear_right'][0],
        'rear_right_wheel_y': wheel_centers['rear_right'][1],
    }

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_start = LaunchConfiguration('auto_start')
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    save_results = LaunchConfiguration('save_results')
    show_gazebo_trail = LaunchConfiguration('show_gazebo_trail')
    path_sample_distance = LaunchConfiguration('path_sample_distance')
    path_sample_period = LaunchConfiguration('path_sample_period')
    spawn_car = should_spawn_car(context, world_path)

    actions = [
        LogInfo(msg=f'Using D435i Gazebo world: {world_path}'),
        LogInfo(msg=f'Using D435i RViz config: {rviz_config}'),
        LogInfo(msg=f'Using D435i config root: {config_root}'),
        LogInfo(msg=f'D435i spawn_car resolved to: {spawn_car}'),
        LogInfo(
            msg=(
                f'D435i vehicle safety radius: {vehicle_safety_radius:.3f} m '
                f'= {vehicle_safety_scale:.2f} * max wheel-center distance '
                f'{max_wheel_distance:.3f} m')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world_path,
                'gui': gui,
                'verbose': 'false',
            }.items(),
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
            msg='Skipping D435i spawn_entity because the selected world already '
                'contains version_car, or spawn_car:=false was requested.'))

    actions.extend([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'robot_description': robot_description,
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom',
            ],
            output='screen',
        ),

        Node(
            package='version_car_sim',
            executable='d435i_depth_obstacle_mapper',
            name='d435i_depth_obstacle_mapper',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'point_cloud_topic': '/d435i/depth/points',
                'point_cloud_qos': LaunchConfiguration('point_cloud_qos'),
                'odom_topic': 'odom',
                'obstacle_grid_topic': 'vision_obstacle_grid',
                'local_costmap_topic': 'vision_local_costmap',
                'obstacle_points_topic': 'vision_obstacle_points',
                'debug_topic': 'vehicle_safety_debug',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'tf_timeout_sec': 0.10,
                'min_depth': 0.20,
                'max_depth': 6.0,
                'min_obstacle_height': 0.06,
                'max_obstacle_height': 1.35,
                'grid_size_m': 12.0,
                'resolution': 0.12,
                'inflation_radius': vehicle_safety_radius,
                'inflation_cost': 70,
                'point_stride': 4,
                **vehicle_safety_parameters,
            }],
        ),

        Node(
            package='version_car_sim',
            executable='d435i_astar_planner',
            name='d435i_astar_planner',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'map_topic': 'vision_local_costmap',
                'odom_topic': 'odom',
                'goal_topic': 'goal_pose',
                'auto_start': ParameterValue(auto_start, value_type=bool),
                'start_topic': 'start_navigation',
                'path_topic': 'planned_path',
                'cmd_topic': 'cmd_vel_raw',
                'debug_topic': 'planner_debug',
                'wheel_base': model_wheel_base,
                'max_speed': 0.32,
                'max_steer': 0.35,
                'max_yaw_rate': 0.35,
                'lookahead': 0.85,
                'goal_tolerance': ParameterValue(goal_tolerance, value_type=float),
                'local_goal_distance': 4.5,
                'min_local_goal_distance': 0.8,
                'obstacle_threshold': 55,
                'replan_period': 0.35,
                'start_clearance_margin': 0.50,
                **vehicle_safety_parameters,
            }],
        ),

        Node(
            package='version_car_sim',
            executable='d435i_visual_obstacle_avoidance',
            name='d435i_visual_obstacle_avoidance',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'obstacle_points_topic': 'vision_obstacle_points',
                'odom_topic': 'odom',
                'input_cmd_topic': 'cmd_vel_raw',
                'output_cmd_topic': 'cmd_vel',
                'debug_topic': 'd435i_avoidance_debug',
                'safety_marker_topic': 'vehicle_safety_radius_marker',
                'target_frame': 'base_link',
                'inflation_radius': vehicle_safety_radius,
                'slow_distance': vehicle_safety_radius + 0.80,
                'stop_distance': vehicle_safety_radius + 0.40,
                'emergency_stop_distance': vehicle_safety_radius + 0.15,
                'slow_margin': 0.80,
                'stop_margin': 0.40,
                'emergency_margin': 0.15,
                'turn_clearance_margin': 0.50,
                'turn_distance_margin': 0.35,
                'turn_release_margin': 0.20,
                'front_angle_deg': 25.0,
                'side_angle_deg': 90.0,
                'avoid_turn_speed': 0.30,
                'max_linear_speed': 0.30,
                'max_angular_speed': 0.35,
                **vehicle_safety_parameters,
            }],
        ),

        Node(
            package='version_car_sim',
            executable='mcu_protocol_bridge',
            name='mcu_protocol_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'cmd_topic': 'cmd_vel',
                'tcp_enabled': False,
                'tcp_host': '192.168.0.7',
                'tcp_port': 8234,
                'max_linear_speed': 0.35,
                'max_angular_speed': 0.80,
                'allow_lateral_speed': False,
            }],
        ),

        Node(
            package='version_car_sim',
            executable='start_pose_setter',
            name='start_pose_setter',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'entity_name': 'version_car',
                'start_pose_topic': 'start_pose',
                'start_pose_2d_topic': 'start_pose_2d',
                'initialpose_topic': 'initialpose',
                'set_entity_state_service': '/gazebo/set_entity_state',
                'reference_frame': 'world',
                'default_z': 0.0,
                'prefer_gazebo_cli': True,
                'gazebo_cli_timeout': 3.0,
            }],
        ),

        Node(
            package='version_car_sim',
            executable='trajectory_recorder',
            name='trajectory_recorder',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'map_topic': 'vision_obstacle_grid',
                'planned_path_topic': 'planned_path',
                'goal_tolerance': ParameterValue(goal_tolerance, value_type=float),
                'save_results': ParameterValue(save_results, value_type=bool),
                'show_gazebo_trail': ParameterValue(show_gazebo_trail, value_type=bool),
                'path_sample_distance': ParameterValue(
                    path_sample_distance, value_type=float),
                'path_sample_period': ParameterValue(path_sample_period, value_type=float),
                'results_root': 'sim_results',
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            condition=IfCondition(rviz),
            output='screen',
        ),

        LogInfo(msg='D435i GUI debug: check topics with `ros2 topic list | grep d435i`.'),
        LogInfo(msg='D435i GUI debug: check TF with `ros2 run tf2_ros tf2_echo map odom` and `ros2 run tf2_ros tf2_echo odom base_link`.'),
        LogInfo(msg='D435i navigation waits because auto_start:=false. Set /goal_pose, then publish /start_navigation true.'),
    ])

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument(
            'config_root',
            default_value='',
            description=(
                'Config root containing rviz and gazebo/worlds folders. '
                'Defaults to the source tree config directory when available.')),
        DeclareLaunchArgument(
            'world_file',
            default_value='',
            description='D435i world filename under config/gazebo or config/worlds, or an absolute path.'),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value='',
            description='D435i RViz config filename under config/rviz or an absolute path.'),
        DeclareLaunchArgument(
            'spawn_car',
            default_value='auto',
            description='auto skips spawning when the selected world already contains version_car.'),
        DeclareLaunchArgument('start_x', default_value='-5.0'),
        DeclareLaunchArgument('start_y', default_value='-3.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.25'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.3'),
        DeclareLaunchArgument('save_results', default_value='true'),
        DeclareLaunchArgument('show_gazebo_trail', default_value='true'),
        DeclareLaunchArgument('path_sample_distance', default_value='0.05'),
        DeclareLaunchArgument('path_sample_period', default_value='0.2'),
        DeclareLaunchArgument('serial_enabled', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value=''),
        DeclareLaunchArgument(
            'point_cloud_qos',
            default_value='reliable',
            description='Use reliable for Gazebo camera points or best_effort for sensor-data style drivers.'),
        OpaqueFunction(function=launch_setup),
    ])

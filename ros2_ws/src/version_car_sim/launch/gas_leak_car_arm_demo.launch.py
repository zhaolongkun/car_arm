import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def moveit_parameters(moveit_config):
    parameters = moveit_config.to_dict()
    ompl = parameters.setdefault('ompl', {})
    ompl['planning_plugin'] = 'ompl_interface/OMPLPlanner'
    if os.environ.get('ROS_DISTRO') == 'humble':
        ompl['request_adapters'] = ' '.join([
            'default_planner_request_adapters/AddTimeOptimalParameterization',
            'default_planner_request_adapters/ResolveConstraintFrames',
            'default_planner_request_adapters/FixWorkspaceBounds',
            'default_planner_request_adapters/FixStartStateBounds',
            'default_planner_request_adapters/FixStartStateCollision',
        ])
        ompl.pop('response_adapters', None)
    return parameters


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    model = LaunchConfiguration('model').perform(context).strip().lower()
    if model != 'dm':
        raise RuntimeError(
            'This car + arm demo only supports Seeed reBot Arm B601-DM. '
            'Use model:=dm; RS files are intentionally not loaded.')

    package_share = get_package_share_directory('version_car_sim')
    mid360_launch = os.path.join(package_share, 'launch', 'mid360_fast_lio_mapping.launch.py')
    ros2_controllers = os.path.join(package_share, 'config', 'rebot', 'ros2_controllers.yaml')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    fast_lio_mode = LaunchConfiguration('fast_lio_mode')

    arm_mount_xyz = ' '.join([
        LaunchConfiguration('arm_mount_x').perform(context),
        LaunchConfiguration('arm_mount_y').perform(context),
        LaunchConfiguration('arm_mount_z').perform(context),
    ])
    arm_mount_rpy = ' '.join([
        LaunchConfiguration('arm_mount_roll').perform(context),
        LaunchConfiguration('arm_mount_pitch').perform(context),
        LaunchConfiguration('arm_mount_yaw').perform(context),
    ])

    moveit_config = (
        MoveItConfigsBuilder('mobile_rebot_b601_dm', package_name='version_car_sim')
        .robot_description(
            file_path='models/mobile_rebot_b601_dm.urdf.xacro',
            mappings={
                'arm_mount_xyz': arm_mount_xyz,
                'arm_mount_rpy': arm_mount_rpy,
            },
        )
        .robot_description_semantic(file_path='config/rebot/mobile_rebot_b601_dm.srdf')
        .robot_description_kinematics(file_path='config/rebot/kinematics.yaml')
        .joint_limits(file_path='config/rebot/joint_limits.yaml')
        .trajectory_execution(file_path='config/rebot/moveit_controllers.yaml')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=['ompl'], load_all=False)
        .to_moveit_configs()
    )
    moveit_params = moveit_parameters(moveit_config)

    return [
        LogInfo(msg='Starting gas leak car + reBot B601-DM arm demo.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mid360_launch),
            launch_arguments={
                'gui': gui,
                'rviz': rviz,
                'use_sim_time': use_sim_time,
                'fast_lio_mode': fast_lio_mode,
                'enable_navigation': 'true',
                'navigation_backend': 'nav2',
                'enable_mapping_drive': 'false',
                'enable_custom_soft_inflation': 'false',
                'auto_start': 'false',
                'goal_x': '10.0',
                'goal_y': '10.0',
                'goal_yaw': '0.0',
                'waypoints': '[]',
                'start_x': LaunchConfiguration('start_x'),
                'start_y': LaunchConfiguration('start_y'),
                'start_yaw': LaunchConfiguration('start_yaw'),
                'spawn_car': LaunchConfiguration('spawn_car'),
                'world_file': LaunchConfiguration('world_file'),
                'car_model_file': LaunchConfiguration('car_model_file'),
                'rviz_config_file': LaunchConfiguration('rviz_config_file'),
            }.items(),
        ),
        TimerAction(
            period=4.0,
            actions=[ExecuteProcess(
                cmd=[
                    'bash',
                    '-lc',
                    'pgrep -x gzclient >/dev/null || '
                    'exec gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so',
                ],
                condition=IfCondition(gui),
                output='screen',
            )],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='mobile_rebot_robot_state_publisher',
            output='both',
            parameters=[
                moveit_config.robot_description,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                ros2_controllers,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
        TimerAction(
            period=2.0,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen',
            )],
        ),
        TimerAction(
            period=4.0,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=['rebotarm_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )],
        ),
        TimerAction(
            period=6.0,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )],
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_params,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
        Node(
            package='version_car_sim',
            executable='gas_field_simulator',
            name='gas_field_simulator',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'leak_x': ParameterValue(LaunchConfiguration('leak_x'), value_type=float),
                'leak_y': ParameterValue(LaunchConfiguration('leak_y'), value_type=float),
                'leak_z': ParameterValue(LaunchConfiguration('leak_z'), value_type=float),
                'source_strength': 1.0,
            }],
        ),
        Node(
            package='version_car_sim',
            executable='spray_simulator',
            name='spray_simulator',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'spray_duration_sec': ParameterValue(
                    LaunchConfiguration('spray_duration_sec'), value_type=float),
                'neutralize_rate': 0.18,
                'nozzle_frame': 'spray_tip_link',
                'map_frame': 'map',
            }],
        ),
        Node(
            package='version_car_sim',
            executable='rebot_safe_rl_controller',
            name='rebot_safe_rl_controller',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'policy_path': LaunchConfiguration('rl_policy_path'),
                'auto_start': False,
                'control_rate_hz': ParameterValue(
                    LaunchConfiguration('rl_control_rate_hz'), value_type=float),
                'target_x': ParameterValue(LaunchConfiguration('rl_target_x'), value_type=float),
                'target_y': ParameterValue(LaunchConfiguration('rl_target_y'), value_type=float),
                'target_z': ParameterValue(LaunchConfiguration('rl_target_z'), value_type=float),
                'safe_distance': ParameterValue(LaunchConfiguration('rl_safe_distance'), value_type=float),
                'max_action_delta': ParameterValue(
                    LaunchConfiguration('rl_max_action_delta'), value_type=float),
                'success_tolerance': ParameterValue(
                    LaunchConfiguration('rl_success_tolerance'), value_type=float),
                'tf_success_tolerance': ParameterValue(
                    LaunchConfiguration('rl_tf_success_tolerance'), value_type=float),
                'max_steps': ParameterValue(LaunchConfiguration('rl_max_steps'), value_type=int),
                'trajectory_duration_sec': ParameterValue(
                    LaunchConfiguration('rl_trajectory_duration_sec'), value_type=float),
                'trajectory_min_duration_sec': ParameterValue(
                    LaunchConfiguration('rl_trajectory_min_duration_sec'), value_type=float),
                'trajectory_max_duration_sec': ParameterValue(
                    LaunchConfiguration('rl_trajectory_max_duration_sec'), value_type=float),
                'trajectory_nominal_joint_speed': ParameterValue(
                    LaunchConfiguration('rl_trajectory_nominal_joint_speed'), value_type=float),
                'trajectory_waypoints': ParameterValue(
                    LaunchConfiguration('rl_trajectory_waypoints'), value_type=int),
                'action_low_pass_alpha': ParameterValue(
                    LaunchConfiguration('rl_action_low_pass_alpha'), value_type=float),
                'min_spray_tip_z': ParameterValue(
                    LaunchConfiguration('min_spray_tip_z'), value_type=float),
                'ground_clearance': ParameterValue(
                    LaunchConfiguration('ground_clearance'), value_type=float),
                'spray_tip_work_height': ParameterValue(
                    LaunchConfiguration('spray_tip_work_height'), value_type=float),
                'enable_teacher_fallback': ParameterValue(
                    LaunchConfiguration('rl_enable_teacher_fallback'), value_type=bool),
                'spray_on_success': False,
            }],
        ),
        Node(
            package='version_car_sim',
            executable='gas_leak_car_arm_task',
            name='gas_leak_car_arm_task',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'map_frame': 'map',
                'base_frame': 'base_link',
                'arm_base_frame': 'rebot_base_link',
                'map_topic': 'map',
                'odom_topic': 'odom',
                'work_distance': ParameterValue(LaunchConfiguration('work_distance'), value_type=float),
                'vehicle_safety_radius': ParameterValue(
                    LaunchConfiguration('vehicle_safety_radius'), value_type=float),
                'nav_ready_timeout_sec': ParameterValue(
                    LaunchConfiguration('nav_ready_timeout_sec'), value_type=float),
                'nav_ready_settle_sec': 1.0,
                'nav_goal_retry_count': 3,
                'nav_goal_retry_delay_sec': 1.0,
                'nav_result_retry_count': 2,
                'nav_result_retry_delay_sec': 2.0,
                'nav_goal_timeout_sec': ParameterValue(
                    LaunchConfiguration('nav_goal_timeout_sec'), value_type=float),
                'navigation_goal_tolerance': ParameterValue(
                    LaunchConfiguration('navigation_goal_tolerance'), value_type=float),
                'navigation_goal_hold_sec': ParameterValue(
                    LaunchConfiguration('navigation_goal_hold_sec'), value_type=float),
                'navigation_waypoint_enabled': ParameterValue(
                    LaunchConfiguration('navigation_waypoint_enabled'), value_type=bool),
                'navigation_waypoints': ParameterValue(
                    LaunchConfiguration('navigation_waypoints'), value_type=str),
                'navigation_waypoint_x': ParameterValue(
                    LaunchConfiguration('navigation_waypoint_x'), value_type=float),
                'navigation_waypoint_y': ParameterValue(
                    LaunchConfiguration('navigation_waypoint_y'), value_type=float),
                'nav_debug_log_period_sec': 3.0,
                'nav_cmd_topic': 'cmd_vel_nav',
                'final_cmd_topic': 'cmd_vel',
                'spray_duration_sec': ParameterValue(
                    LaunchConfiguration('spray_duration_sec'), value_type=float),
                'return_home_after_spray': ParameterValue(
                    LaunchConfiguration('return_home_after_spray'), value_type=bool),
                'enable_gazebo_arm_mirror': ParameterValue(
                    LaunchConfiguration('enable_gazebo_arm_mirror'), value_type=bool),
                'gazebo_arm_mirror_duration_sec': ParameterValue(
                    LaunchConfiguration('gazebo_arm_mirror_duration_sec'), value_type=float),
                'auto_start': ParameterValue(LaunchConfiguration('start_task'), value_type=bool),
                'tip_link': 'spray_tip_link',
                'spray_standoff': ParameterValue(
                    LaunchConfiguration('spray_standoff'), value_type=float),
                'min_spray_tip_z': ParameterValue(
                    LaunchConfiguration('min_spray_tip_z'), value_type=float),
                'ground_clearance': ParameterValue(
                    LaunchConfiguration('ground_clearance'), value_type=float),
                'spray_tip_work_height': ParameterValue(
                    LaunchConfiguration('spray_tip_work_height'), value_type=float),
                'spray_aim_height': ParameterValue(
                    LaunchConfiguration('spray_aim_height'), value_type=float),
                'spray_max_downward_z': ParameterValue(
                    LaunchConfiguration('spray_max_downward_z'), value_type=float),
                'spray_min_range': ParameterValue(
                    LaunchConfiguration('spray_min_range'), value_type=float),
                'spray_max_range': ParameterValue(
                    LaunchConfiguration('spray_max_range'), value_type=float),
                'base_link_ground_z': ParameterValue(
                    LaunchConfiguration('base_link_ground_z'), value_type=float),
                'allow_position_only_spray_fallback': ParameterValue(
                    LaunchConfiguration('allow_position_only_spray_fallback'), value_type=bool),
                'allow_direct_spray_joint_fallback': ParameterValue(
                    LaunchConfiguration('allow_direct_spray_joint_fallback'), value_type=bool),
                'arm_speed_multiplier': ParameterValue(
                    LaunchConfiguration('arm_speed_multiplier'), value_type=float),
                'min_arm_trajectory_duration': ParameterValue(
                    LaunchConfiguration('min_arm_trajectory_duration'), value_type=float),
                'max_arm_trajectory_duration': ParameterValue(
                    LaunchConfiguration('max_arm_trajectory_duration'), value_type=float),
                'home_to_initial_duration': ParameterValue(
                    LaunchConfiguration('home_to_initial_duration'), value_type=float),
                'initial_to_target_duration': ParameterValue(
                    LaunchConfiguration('initial_to_target_duration'), value_type=float),
                'simulation_fast_mode': ParameterValue(
                    LaunchConfiguration('simulation_fast_mode'), value_type=bool),
                'real_hardware_mode': ParameterValue(
                    LaunchConfiguration('real_hardware_mode'), value_type=bool),
                'max_velocity_scaling_factor': ParameterValue(
                    LaunchConfiguration('arm_max_velocity_scaling_factor'), value_type=float),
                'max_acceleration_scaling_factor': ParameterValue(
                    LaunchConfiguration('arm_max_acceleration_scaling_factor'), value_type=float),
                'arm_control_mode': LaunchConfiguration('arm_control_mode'),
                'rl_timeout_sec': ParameterValue(LaunchConfiguration('rl_timeout_sec'), value_type=float),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='dm'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fast_lio_mode', default_value='real'),
        DeclareLaunchArgument('spawn_car', default_value='auto'),
        DeclareLaunchArgument('world_file', default_value='mid360_fast_lio_world.world'),
        DeclareLaunchArgument('car_model_file', default_value='version_car_mid360_rebot_b601_dm.sdf'),
        DeclareLaunchArgument('rviz_config_file', default_value='mid360_fast_lio_mapping.rviz'),
        DeclareLaunchArgument('start_x', default_value='-10.0'),
        DeclareLaunchArgument('start_y', default_value='-10.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.785398'),
        DeclareLaunchArgument('arm_mount_x', default_value='0.0'),
        DeclareLaunchArgument('arm_mount_y', default_value='0.0'),
        DeclareLaunchArgument('arm_mount_z', default_value='0.08'),
        DeclareLaunchArgument('arm_mount_roll', default_value='0.0'),
        DeclareLaunchArgument('arm_mount_pitch', default_value='0.0'),
        DeclareLaunchArgument('arm_mount_yaw', default_value='0.0'),
        DeclareLaunchArgument('leak_x', default_value='10.0'),
        DeclareLaunchArgument('leak_y', default_value='10.0'),
        DeclareLaunchArgument('leak_z', default_value='0.3'),
        DeclareLaunchArgument('work_distance', default_value='0.80'),
        DeclareLaunchArgument('vehicle_safety_radius', default_value='0.452548'),
        DeclareLaunchArgument('spray_standoff', default_value='0.15'),
        DeclareLaunchArgument('min_spray_tip_z', default_value='0.24'),
        DeclareLaunchArgument('ground_clearance', default_value='0.10'),
        DeclareLaunchArgument('spray_tip_work_height', default_value='0.30'),
        DeclareLaunchArgument('spray_aim_height', default_value='0.24'),
        DeclareLaunchArgument('spray_max_downward_z', default_value='-0.20'),
        DeclareLaunchArgument('spray_min_range', default_value='0.25'),
        DeclareLaunchArgument('spray_max_range', default_value='0.78'),
        DeclareLaunchArgument('base_link_ground_z', default_value='0.0'),
        DeclareLaunchArgument('allow_position_only_spray_fallback', default_value='false'),
        DeclareLaunchArgument('allow_direct_spray_joint_fallback', default_value='false'),
        DeclareLaunchArgument('arm_speed_multiplier', default_value='20.0'),
        DeclareLaunchArgument('simulation_fast_mode', default_value='true'),
        DeclareLaunchArgument('real_hardware_mode', default_value='false'),
        DeclareLaunchArgument('arm_max_velocity_scaling_factor', default_value='1.0'),
        DeclareLaunchArgument('arm_max_acceleration_scaling_factor', default_value='1.0'),
        DeclareLaunchArgument('spray_duration_sec', default_value='5.0'),
        DeclareLaunchArgument('return_home_after_spray', default_value='false'),
        DeclareLaunchArgument('enable_gazebo_arm_mirror', default_value='true'),
        DeclareLaunchArgument('gazebo_arm_mirror_duration_sec', default_value='3.0'),
        DeclareLaunchArgument('nav_ready_timeout_sec', default_value='45.0'),
        DeclareLaunchArgument('nav_goal_timeout_sec', default_value='0.0'),
        DeclareLaunchArgument('navigation_goal_tolerance', default_value='0.50'),
        DeclareLaunchArgument('navigation_goal_hold_sec', default_value='2.0'),
        DeclareLaunchArgument('navigation_waypoint_enabled', default_value='true'),
        DeclareLaunchArgument('navigation_waypoints', default_value=''),
        DeclareLaunchArgument('navigation_waypoint_x', default_value='5.0'),
        DeclareLaunchArgument('navigation_waypoint_y', default_value='-9.0'),
        DeclareLaunchArgument('start_task', default_value='true'),
        DeclareLaunchArgument('arm_control_mode', default_value='moveit'),
        DeclareLaunchArgument('rl_policy_path', default_value=''),
        DeclareLaunchArgument('rl_target_x', default_value='0.45'),
        DeclareLaunchArgument('rl_target_y', default_value='0.0'),
        DeclareLaunchArgument('rl_target_z', default_value='0.22'),
        DeclareLaunchArgument('rl_safe_distance', default_value='0.10'),
        DeclareLaunchArgument('rl_max_action_delta', default_value='0.04'),
        DeclareLaunchArgument('rl_control_rate_hz', default_value='8.0'),
        DeclareLaunchArgument('rl_success_tolerance', default_value='0.06'),
        DeclareLaunchArgument('rl_tf_success_tolerance', default_value='0.07'),
        DeclareLaunchArgument('rl_max_steps', default_value='360'),
        DeclareLaunchArgument('rl_timeout_sec', default_value='360.0'),
        DeclareLaunchArgument('rl_trajectory_duration_sec', default_value='0.32'),
        DeclareLaunchArgument('rl_trajectory_min_duration_sec', default_value='0.22'),
        DeclareLaunchArgument('rl_trajectory_max_duration_sec', default_value='0.55'),
        DeclareLaunchArgument('rl_trajectory_nominal_joint_speed', default_value='0.18'),
        DeclareLaunchArgument('rl_trajectory_waypoints', default_value='6'),
        DeclareLaunchArgument('rl_action_low_pass_alpha', default_value='0.55'),
        DeclareLaunchArgument('rl_enable_teacher_fallback', default_value='true'),
        DeclareLaunchArgument('min_arm_trajectory_duration', default_value='0.5'),
        DeclareLaunchArgument('max_arm_trajectory_duration', default_value='5.0'),
        DeclareLaunchArgument('home_to_initial_duration', default_value='2.0'),
        DeclareLaunchArgument('initial_to_target_duration', default_value='3.0'),
        OpaqueFunction(function=launch_setup),
    ])

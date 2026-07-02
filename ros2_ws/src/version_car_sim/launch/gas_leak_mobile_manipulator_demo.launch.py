import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
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
            'This demo only supports Seeed reBot Arm B601-DM. '
            'Use model:=dm; RS model files are intentionally not loaded.')

    package_share = get_package_share_directory('version_car_sim')
    mid360_launch = os.path.join(
        package_share, 'launch', 'mid360_fast_lio_mapping.launch.py')
    ros2_controllers = os.path.join(
        package_share, 'config', 'rebot', 'ros2_controllers.yaml')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    fast_lio_mode = LaunchConfiguration('fast_lio_mode')

    moveit_config = (
        MoveItConfigsBuilder('mobile_rebot_b601_dm', package_name='version_car_sim')
        .robot_description(
            file_path='models/mobile_rebot_b601_dm.urdf.xacro',
            mappings={
                'arm_mount_xyz': LaunchConfiguration('arm_mount_xyz'),
                'arm_mount_rpy': LaunchConfiguration('arm_mount_rpy'),
            },
        )
        .robot_description_semantic(
            file_path='config/rebot/mobile_rebot_b601_dm.srdf')
        .robot_description_kinematics(
            file_path='config/rebot/kinematics.yaml')
        .joint_limits(file_path='config/rebot/joint_limits.yaml')
        .trajectory_execution(
            file_path='config/rebot/moveit_controllers.yaml')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=['ompl'], load_all=False)
        .to_moveit_configs()
    )
    moveit_params = moveit_parameters(moveit_config)

    actions = [
        LogInfo(msg='Starting gas leak mobile manipulator demo with reBot Arm B601-DM.'),
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
            }.items(),
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
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['rebotarm_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
            output='screen',
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
                'nozzle_frame': 'spray_nozzle_link',
            }],
        ),
        Node(
            package='version_car_sim',
            executable='gas_leak_mobile_manipulator_task',
            name='gas_leak_mobile_manipulator_task',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'work_distance': ParameterValue(LaunchConfiguration('work_distance'), value_type=float),
                'spray_duration_sec': ParameterValue(
                    LaunchConfiguration('spray_duration_sec'), value_type=float),
                'auto_start': ParameterValue(LaunchConfiguration('start_task'), value_type=bool),
            }],
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='dm'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fast_lio_mode', default_value='real'),
        DeclareLaunchArgument('spawn_car', default_value='auto'),
        DeclareLaunchArgument('start_x', default_value='-10.0'),
        DeclareLaunchArgument('start_y', default_value='-10.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.785398'),
        DeclareLaunchArgument('arm_mount_xyz', default_value='0.0 0.0 0.08'),
        DeclareLaunchArgument('arm_mount_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('leak_x', default_value='9.2'),
        DeclareLaunchArgument('leak_y', default_value='9.2'),
        DeclareLaunchArgument('leak_z', default_value='0.55'),
        DeclareLaunchArgument('work_distance', default_value='0.8'),
        DeclareLaunchArgument('spray_duration_sec', default_value='5.0'),
        DeclareLaunchArgument('start_task', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])

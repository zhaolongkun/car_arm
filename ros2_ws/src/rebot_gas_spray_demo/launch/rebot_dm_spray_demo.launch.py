import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
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
            'This standalone spray demo only supports Seeed reBot Arm B601-DM. '
            'Use model:=dm; RS files are intentionally not loaded.')

    package_share = get_package_share_directory('rebot_gas_spray_demo')
    ros2_controllers = os.path.join(package_share, 'config', 'ros2_controllers.yaml')
    rviz_config = os.path.join(package_share, 'rviz', 'rebot_dm_spray_demo.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    moveit_config = (
        MoveItConfigsBuilder('rebot_dm_spray', package_name='rebot_gas_spray_demo')
        .robot_description(file_path='config/rebot_dm_spray.urdf.xacro')
        .robot_description_semantic(file_path='config/rebot_dm_spray.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=['ompl'], load_all=False)
        .to_moveit_configs()
    )
    moveit_params = moveit_parameters(moveit_config)

    return [
        LogInfo(msg='Starting standalone reBot B601-DM spray simulation demo.'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_rebot_base_static_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'world', '--child-frame-id', 'base_link',
            ],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rebot_robot_state_publisher',
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
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[
                moveit_params,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
            output='screen',
        ),
        Node(
            package='rebot_gas_spray_demo',
            executable='gas_field_simulator',
            name='gas_field_simulator',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'leak_x': ParameterValue(LaunchConfiguration('leak_x'), value_type=float),
                'leak_y': ParameterValue(LaunchConfiguration('leak_y'), value_type=float),
                'leak_z': ParameterValue(LaunchConfiguration('leak_z'), value_type=float),
                'initial_concentration': 1.0,
            }],
        ),
        Node(
            package='rebot_gas_spray_demo',
            executable='spray_simulator',
            name='spray_simulator',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'spray_duration_sec': ParameterValue(
                    LaunchConfiguration('spray_duration_sec'), value_type=float),
                'nozzle_frame': 'spray_tip_link',
                'world_frame': 'world',
            }],
        ),
        Node(
            package='rebot_gas_spray_demo',
            executable='rebot_spray_task',
            name='rebot_spray_task',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'auto_start': ParameterValue(LaunchConfiguration('start_task'), value_type=bool),
                'spray_duration_sec': ParameterValue(
                    LaunchConfiguration('spray_duration_sec'), value_type=float),
                'leak_standoff': ParameterValue(
                    LaunchConfiguration('leak_standoff'), value_type=float),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='dm'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('leak_x', default_value='0.50'),
        DeclareLaunchArgument('leak_y', default_value='0.0'),
        DeclareLaunchArgument('leak_z', default_value='0.28'),
        DeclareLaunchArgument('leak_standoff', default_value='0.12'),
        DeclareLaunchArgument('spray_duration_sec', default_value='4.0'),
        DeclareLaunchArgument('start_task', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])

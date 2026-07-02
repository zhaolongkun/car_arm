import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
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
            'This circle point demo only supports Seeed reBot Arm B601-DM. '
            'Use model:=dm; RS files are intentionally not loaded.')

    package_share = get_package_share_directory('rebot_gas_spray_demo')
    ros2_controllers = os.path.join(package_share, 'config', 'ros2_controllers.yaml')
    rviz_config = os.path.join(package_share, 'rviz', 'rebot_dm_circle_points_demo.rviz')
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
        LogInfo(msg='Starting standalone reBot B601-DM circle point motion demo.'),
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
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_rebot_base_link_static_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'rebot_base_link',
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
            executable='rebot_circle_point_task',
            name='rebot_circle_point_task',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'auto_start': ParameterValue(LaunchConfiguration('start_task'), value_type=bool),
                'start_delay_sec': ParameterValue(
                    LaunchConfiguration('start_delay_sec'), value_type=float),
                'frame_id': LaunchConfiguration('frame_id'),
                'center_x': ParameterValue(LaunchConfiguration('center_x'), value_type=float),
                'center_y': ParameterValue(LaunchConfiguration('center_y'), value_type=float),
                'center_z': ParameterValue(LaunchConfiguration('center_z'), value_type=float),
                'radius': ParameterValue(LaunchConfiguration('radius'), value_type=float),
                'num_points': ParameterValue(LaunchConfiguration('num_points'), value_type=int),
                'loop_count': ParameterValue(LaunchConfiguration('loop_count'), value_type=int),
                'hold_sec': ParameterValue(LaunchConfiguration('hold_sec'), value_type=float),
                'tip_link': LaunchConfiguration('tip_link'),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='dm'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_task', default_value='true'),
        DeclareLaunchArgument('start_delay_sec', default_value='4.0'),
        DeclareLaunchArgument('frame_id', default_value='rebot_base_link'),
        DeclareLaunchArgument('center_x', default_value='0.35'),
        DeclareLaunchArgument('center_y', default_value='0.0'),
        DeclareLaunchArgument('center_z', default_value='0.05'),
        DeclareLaunchArgument('radius', default_value='0.25'),
        DeclareLaunchArgument('num_points', default_value='6'),
        DeclareLaunchArgument('loop_count', default_value='0'),
        DeclareLaunchArgument('hold_sec', default_value='1.0'),
        DeclareLaunchArgument('tip_link', default_value='spray_tip_link'),
        OpaqueFunction(function=launch_setup),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('version_car_sim')
    mid360_launch = os.path.join(package_share, 'launch', 'mid360_fast_lio_mapping.launch.py')
    ros2_controllers = os.path.join(package_share, 'config', 'rebot', 'ros2_controllers.yaml')

    robot_xacro = os.path.join(package_share, 'models', 'mobile_rebot_b601_dm.urdf.xacro')
    robot_description = {
        'robot_description': Command([
            'xacro ',
            robot_xacro,
            ' arm_mount_xyz:="0.0 0.0 0.08"',
            ' arm_mount_rpy:="0.0 0.0 0.0"',
        ])
    }

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world_file', default_value='mid360_fast_lio_world.world'),
        DeclareLaunchArgument('car_model_file', default_value='version_car_mid360_rebot_b601_dm.sdf'),
        LogInfo(
            msg=(
                'Starting reBot arm speed debug: Gazebo car+arm only, '
                'Nav2 disabled, direct FollowJointTrajectory timing test.')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mid360_launch),
            launch_arguments={
                'gui': gui,
                'rviz': 'false',
                'use_sim_time': use_sim_time,
                'fast_lio_mode': 'stub',
                'enable_navigation': 'false',
                'navigation_backend': 'nav2',
                'enable_mapping_drive': 'false',
                'auto_start': 'false',
                'start_x': '-3.0',
                'start_y': '0.0',
                'start_yaw': '0.0',
                'spawn_car': 'auto',
                'world_file': LaunchConfiguration('world_file'),
                'car_model_file': LaunchConfiguration('car_model_file'),
                'rviz_config_file': 'mid360_fast_lio_mapping.rviz',
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='mobile_rebot_robot_state_publisher',
            output='screen',
            parameters=[
                robot_description,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                robot_description,
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
        TimerAction(
            period=9.0,
            actions=[Node(
                package='version_car_sim',
                executable='rebot_arm_minimal_test',
                name='rebot_arm_speed_home_to_initial',
                output='screen',
                arguments=[
                    '--target', 'initial_pose',
                    '--duration', '2.0',
                    '--joint-state-timeout', '20.0',
                ],
                parameters=[{
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                }],
            )],
        ),
        TimerAction(
            period=13.0,
            actions=[Node(
                package='version_car_sim',
                executable='rebot_arm_minimal_test',
                name='rebot_arm_speed_initial_to_target',
                output='screen',
                arguments=[
                    '--target', 'target_test_pose',
                    '--duration', '3.0',
                    '--joint-state-timeout', '20.0',
                ],
                parameters=[{
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                }],
            )],
        ),
    ])

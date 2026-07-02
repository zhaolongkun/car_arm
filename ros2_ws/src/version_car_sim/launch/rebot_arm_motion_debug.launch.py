from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('target', default_value='initial_pose'),
        DeclareLaunchArgument('duration', default_value='1.5'),
        DeclareLaunchArgument('start_delay', default_value='3.0'),
        DeclareLaunchArgument('joint_state_timeout', default_value='20.0'),
        LogInfo(
            msg=(
                'Starting reBot arm motion debug: no Nav2 goal is sent; '
                'the node only sends FollowJointTrajectory to rebotarm_controller.')),
        Node(
            package='version_car_sim',
            executable='rebot_arm_minimal_test',
            name='rebot_arm_motion_debug',
            output='screen',
            arguments=[
                '--target',
                LaunchConfiguration('target'),
                '--duration',
                LaunchConfiguration('duration'),
                '--start-delay',
                LaunchConfiguration('start_delay'),
                '--joint-state-timeout',
                LaunchConfiguration('joint_state_timeout'),
            ],
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool),
            }],
            additional_env={
                'RCUTILS_LOGGING_BUFFERED_STREAM': '1',
            },
        ),
    ])

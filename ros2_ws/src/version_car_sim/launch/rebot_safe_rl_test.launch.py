import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def moveit_parameters(moveit_config):
    parameters = moveit_config.to_dict()
    ompl = parameters.setdefault("ompl", {})
    ompl["planning_plugin"] = "ompl_interface/OMPLPlanner"
    if os.environ.get("ROS_DISTRO") == "humble":
        ompl["request_adapters"] = " ".join([
            "default_planner_request_adapters/AddTimeOptimalParameterization",
            "default_planner_request_adapters/ResolveConstraintFrames",
            "default_planner_request_adapters/FixWorkspaceBounds",
            "default_planner_request_adapters/FixStartStateBounds",
            "default_planner_request_adapters/FixStartStateCollision",
        ])
        ompl.pop("response_adapters", None)
    return parameters


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    package_share = get_package_share_directory("version_car_sim")
    mid360_launch = os.path.join(package_share, "launch", "mid360_fast_lio_mapping.launch.py")
    ros2_controllers = os.path.join(package_share, "config", "rebot", "ros2_controllers.yaml")

    arm_mount_xyz = " ".join([
        LaunchConfiguration("arm_mount_x").perform(context),
        LaunchConfiguration("arm_mount_y").perform(context),
        LaunchConfiguration("arm_mount_z").perform(context),
    ])
    arm_mount_rpy = " ".join([
        LaunchConfiguration("arm_mount_roll").perform(context),
        LaunchConfiguration("arm_mount_pitch").perform(context),
        LaunchConfiguration("arm_mount_yaw").perform(context),
    ])

    moveit_config = (
        MoveItConfigsBuilder("mobile_rebot_b601_dm", package_name="version_car_sim")
        .robot_description(
            file_path="models/mobile_rebot_b601_dm.urdf.xacro",
            mappings={
                "arm_mount_xyz": arm_mount_xyz,
                "arm_mount_rpy": arm_mount_rpy,
            },
        )
        .robot_description_semantic(file_path="config/rebot/mobile_rebot_b601_dm.srdf")
        .robot_description_kinematics(file_path="config/rebot/kinematics.yaml")
        .joint_limits(file_path="config/rebot/joint_limits.yaml")
        .trajectory_execution(file_path="config/rebot/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"], load_all=False)
        .to_moveit_configs()
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    return [
        LogInfo(msg="Starting reBot Safe RL test without full navigation task."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mid360_launch),
            launch_arguments={
                "gui": LaunchConfiguration("gui"),
                "rviz": LaunchConfiguration("rviz"),
                "use_sim_time": use_sim_time,
                "fast_lio_mode": "stub",
                "enable_navigation": "false",
                "navigation_backend": "nav2",
                "enable_mapping_drive": "false",
                "enable_custom_soft_inflation": "false",
                "auto_start": "false",
                "spawn_car": LaunchConfiguration("spawn_car"),
                "world_file": LaunchConfiguration("world_file"),
                "car_model_file": "version_car_mid360_rebot_b601_dm.sdf",
                "rviz_config_file": LaunchConfiguration("rviz_config_file"),
                "start_x": LaunchConfiguration("start_x"),
                "start_y": LaunchConfiguration("start_y"),
                "start_yaw": LaunchConfiguration("start_yaw"),
            }.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="mobile_rebot_robot_state_publisher",
            output="both",
            parameters=[
                moveit_config.robot_description,
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                ros2_controllers,
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
        TimerAction(
            period=2.0,
            actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            )],
        ),
        TimerAction(
            period=4.0,
            actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=["rebotarm_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            )],
        ),
        TimerAction(
            period=6.0,
            actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            )],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_parameters(moveit_config),
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
        TimerAction(
            period=7.0,
            actions=[Node(
                package="version_car_sim",
                executable="rebot_safe_rl_controller",
                name="rebot_safe_rl_controller",
                output="screen",
                parameters=[{
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "policy_path": LaunchConfiguration("policy_path"),
                    "auto_start": ParameterValue(LaunchConfiguration("auto_start_rl"), value_type=bool),
                    "target_x": ParameterValue(LaunchConfiguration("target_x"), value_type=float),
                    "target_y": ParameterValue(LaunchConfiguration("target_y"), value_type=float),
                    "target_z": ParameterValue(LaunchConfiguration("target_z"), value_type=float),
                    "safe_distance": ParameterValue(LaunchConfiguration("safe_distance"), value_type=float),
                    "max_action_delta": ParameterValue(LaunchConfiguration("max_action_delta"), value_type=float),
                    "success_tolerance": ParameterValue(LaunchConfiguration("success_tolerance"), value_type=float),
                    "max_steps": ParameterValue(LaunchConfiguration("max_steps"), value_type=int),
                    "enable_teacher_fallback": ParameterValue(
                        LaunchConfiguration("enable_teacher_fallback"), value_type=bool),
                    "spray_on_success": ParameterValue(LaunchConfiguration("spray_on_success"), value_type=bool),
                }],
            )],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("spawn_car", default_value="true"),
        DeclareLaunchArgument("world_file", default_value="mid360_fast_lio_world.world"),
        DeclareLaunchArgument("rviz_config_file", default_value="mid360_fast_lio_mapping.rviz"),
        DeclareLaunchArgument("start_x", default_value="-3.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("arm_mount_x", default_value="0.0"),
        DeclareLaunchArgument("arm_mount_y", default_value="0.0"),
        DeclareLaunchArgument("arm_mount_z", default_value="0.08"),
        DeclareLaunchArgument("arm_mount_roll", default_value="0.0"),
        DeclareLaunchArgument("arm_mount_pitch", default_value="0.0"),
        DeclareLaunchArgument("arm_mount_yaw", default_value="0.0"),
        DeclareLaunchArgument("target_x", default_value="0.952548"),
        DeclareLaunchArgument("target_y", default_value="0.0"),
        DeclareLaunchArgument("target_z", default_value="0.16"),
        DeclareLaunchArgument("safe_distance", default_value="0.10"),
        DeclareLaunchArgument("max_action_delta", default_value="0.025"),
        DeclareLaunchArgument("success_tolerance", default_value="0.035"),
        DeclareLaunchArgument("max_steps", default_value="360"),
        DeclareLaunchArgument("enable_teacher_fallback", default_value="true"),
        DeclareLaunchArgument("policy_path", default_value=""),
        DeclareLaunchArgument("auto_start_rl", default_value="true"),
        DeclareLaunchArgument("spray_on_success", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])

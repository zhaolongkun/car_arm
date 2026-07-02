import copy
import math
import os
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import PerformanceMetrics
from geometry_msgs.msg import Pose, PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from version_car_sim.rl.safety_geometry import (
    JOINT_LIMITS,
    SimplifiedArmSafetyModel,
    dict_to_joint_array,
)


class GasLeakMobileManipulatorTask(Node):
    def __init__(self):
        super().__init__('gas_leak_mobile_manipulator_task')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('move_group_action_name', 'move_action')
        self.declare_parameter('arm_controller_action_name', 'rebotarm_controller/follow_joint_trajectory')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('arm_base_frame', 'rebot_base_link')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('work_distance', 0.5)
        self.declare_parameter('vehicle_safety_radius', 0.452548)
        self.declare_parameter('nav_ready_timeout_sec', 45.0)
        self.declare_parameter('nav_ready_settle_sec', 1.0)
        self.declare_parameter('nav_goal_retry_count', 3)
        self.declare_parameter('nav_goal_retry_delay_sec', 1.0)
        self.declare_parameter('nav_result_retry_count', 2)
        self.declare_parameter('nav_result_retry_delay_sec', 2.0)
        self.declare_parameter('nav_goal_timeout_sec', 180.0)
        self.declare_parameter('nav_goal_reached_tolerance', 0.50)
        self.declare_parameter('navigation_goal_tolerance', 0.50)
        self.declare_parameter('navigation_goal_hold_sec', 2.0)
        self.declare_parameter('nav_debug_log_period_sec', 3.0)
        self.declare_parameter('nav_cmd_topic', 'cmd_vel_nav')
        self.declare_parameter('final_cmd_topic', 'cmd_vel')
        self.declare_parameter('spray_duration_sec', 5.0)
        self.declare_parameter('return_home_after_spray', False)
        self.declare_parameter('enable_gazebo_arm_mirror', True)
        self.declare_parameter('gazebo_arm_trajectory_topic', '/gazebo_arm/set_joint_trajectory')
        self.declare_parameter('gazebo_arm_mirror_duration_sec', 3.0)
        self.declare_parameter('done_source_strength', 0.25)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('tip_link', 'spray_tip_link')
        self.declare_parameter('fallback_tip_links', [])
        self.declare_parameter('spray_standoff', 0.15)
        self.declare_parameter('min_spray_tip_z', 0.24)
        self.declare_parameter('ground_clearance', 0.10)
        self.declare_parameter('spray_tip_work_height', 0.30)
        self.declare_parameter('spray_aim_height', 0.24)
        self.declare_parameter('spray_max_downward_z', -0.20)
        self.declare_parameter('spray_min_range', 0.25)
        self.declare_parameter('spray_max_range', 0.78)
        self.declare_parameter('spray_full_pose_attempt_limit', 8)
        self.declare_parameter('spray_position_only_attempt_limit', 1)
        self.declare_parameter('base_link_ground_z', 0.0)
        self.declare_parameter('ground_collision_size', 30.0)
        self.declare_parameter('ground_collision_thickness', 0.02)
        self.declare_parameter('trajectory_safety_margin', 0.005)
        self.declare_parameter('runtime_safety_check_period_sec', 0.05)
        self.declare_parameter('arm_speed_multiplier', 20.0)
        self.declare_parameter('min_arm_trajectory_duration', 0.5)
        self.declare_parameter('max_arm_trajectory_duration', 5.0)
        self.declare_parameter('home_to_initial_duration', 2.0)
        self.declare_parameter('initial_to_target_duration', 3.0)
        self.declare_parameter('min_trajectory_point_dt', 0.01)
        self.declare_parameter('arm_planning_timeout', 10.0)
        self.declare_parameter('arm_execution_timeout', 10.0)
        self.declare_parameter('simulation_fast_mode', True)
        self.declare_parameter('real_hardware_mode', False)
        self.declare_parameter('enable_elbow_up_constraints', True)
        self.declare_parameter('move_spray_ready_before_pose', True)
        self.declare_parameter('position_tolerance', 0.040)
        self.declare_parameter('position_only_tolerance', 0.060)
        self.declare_parameter('orientation_tolerance', 0.55)
        self.declare_parameter('allow_position_only_spray_fallback', False)
        self.declare_parameter('position_only_use_posture_constraints', False)
        self.declare_parameter('allow_direct_spray_joint_fallback', False)
        self.declare_parameter('max_velocity_scaling_factor', 1.0)
        self.declare_parameter('max_acceleration_scaling_factor', 1.0)
        self.declare_parameter('fallback_trajectory_min_duration_sec', 1.5)
        self.declare_parameter('fallback_trajectory_max_duration_sec', 5.0)
        self.declare_parameter('fallback_trajectory_nominal_joint_speed', 0.60)
        self.declare_parameter('fallback_trajectory_waypoints', 18)
        self.declare_parameter('joint_velocity_limit', 1.6)
        self.declare_parameter('joint_acceleration_limit', 2.5)
        self.declare_parameter('arm_control_mode', 'moveit')
        self.declare_parameter('arm_runtime_ready_timeout_sec', 12.0)
        self.declare_parameter('rl_start_topic', 'rebot_safe_rl/start')
        self.declare_parameter('rl_done_topic', 'rebot_safe_rl/done')
        self.declare_parameter('rl_timeout_sec', 120.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_leak_pose = None
        self.latest_map_time = None
        self.latest_odom_time = None
        self.latest_odom_msg = None
        self.latest_nav_cmd = None
        self.latest_nav_cmd_time = None
        self.latest_final_cmd = None
        self.latest_final_cmd_time = None
        self.latest_nav_feedback = None
        self.latest_real_time_factor = None
        self.source_strength = 1.0
        self.spray_done = False
        self.rl_done = False
        self.task_thread = None
        self.task_done = False
        self.task_state = 'IDLE'
        self.shutdown_event = threading.Event()
        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.latest_joint_positions = {}
        self.safety_model = SimplifiedArmSafetyModel(
            safe_distance=max(0.0, float(self.get_parameter('trajectory_safety_margin').value)),
            hard_clearance=0.0,
            max_action_delta=0.04)

        self.create_subscription(PoseStamped, 'leak_pose_map', self.on_leak_pose, 10)
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('map_topic').value), self.on_map, 2)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self.on_odom, 10)
        self.create_subscription(
            Twist, str(self.get_parameter('nav_cmd_topic').value), self.on_nav_cmd, 10)
        self.create_subscription(
            Twist, str(self.get_parameter('final_cmd_topic').value), self.on_final_cmd, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.create_subscription(Float32, 'gas/source_strength', self.on_source_strength, 10)
        self.create_subscription(Bool, 'spray/done', self.on_spray_done, 10)
        self.create_subscription(
            Bool,
            str(self.get_parameter('rl_done_topic').value),
            self.on_rl_done,
            10)
        self.create_subscription(
            PerformanceMetrics,
            '/gazebo/performance_metrics',
            self.on_performance_metrics,
            10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.spray_start_pub = self.create_publisher(Bool, 'spray/start', 10)
        self.rl_start_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('rl_start_topic').value),
            10)
        self.spray_pose_pub = self.create_publisher(PoseStamped, 'spray/target_pose', 10)
        self.gazebo_arm_trajectory_pub = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter('gazebo_arm_trajectory_topic').value),
            10)

        self.nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter('nav_action_name').value))
        self.move_group_client = ActionClient(
            self, MoveGroup, str(self.get_parameter('move_group_action_name').value))
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory,
            str(self.get_parameter('arm_controller_action_name').value))
        self.controller_state_client = self.create_client(
            GetState, '/controller_server/get_state')
        self.bt_state_client = self.create_client(
            GetState, '/bt_navigator/get_state')
        self.velocity_smoother_state_client = self.create_client(
            GetState, '/velocity_smoother/get_state')

        self.start_timer = None
        if self.parameter_as_bool('auto_start'):
            self.start_timer = self.create_timer(1.0, self.maybe_start_task)
        else:
            self.get_logger().info('auto_start=false，联合任务节点已启动但不会自动发送 Nav2/MoveIt 目标。')

        self.log_loaded_task_code()

    def on_leak_pose(self, msg):
        self.latest_leak_pose = msg

    def on_map(self, _msg):
        self.latest_map_time = time.monotonic()

    def on_odom(self, msg):
        self.latest_odom_time = time.monotonic()
        self.latest_odom_msg = msg

    def on_nav_cmd(self, msg):
        self.latest_nav_cmd = msg
        self.latest_nav_cmd_time = time.monotonic()

    def on_final_cmd(self, msg):
        self.latest_final_cmd = msg
        self.latest_final_cmd_time = time.monotonic()

    def on_joint_states(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.arm_joint_names:
                self.latest_joint_positions[name] = float(position)

    def on_source_strength(self, msg):
        self.source_strength = max(0.0, float(msg.data))

    def on_spray_done(self, msg):
        if msg.data:
            self.spray_done = True

    def on_rl_done(self, msg):
        if msg.data:
            self.rl_done = True

    def on_performance_metrics(self, msg):
        self.latest_real_time_factor = float(msg.real_time_factor)

    def maybe_start_task(self):
        if self.task_thread is not None:
            return
        if self.shutdown_event.is_set():
            return
        if not self.parameter_as_bool('auto_start'):
            return
        self.task_thread = threading.Thread(target=self.run_task, daemon=True)
        self.task_thread.start()

    def request_shutdown(self):
        self.shutdown_event.set()
        if (
            self.task_thread is not None
            and self.task_thread.is_alive()
            and self.task_thread is not threading.current_thread()
        ):
            try:
                self.task_thread.join(timeout=1.0)
            except KeyboardInterrupt:
                pass

    def parameter_as_bool(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def set_task_state(self, state):
        if self.task_state == state:
            return
        self.task_state = state
        self.get_logger().info(f'TASK_STATE: {state}')

    def log_loaded_task_code(self):
        use_sim_time = (
            self.get_parameter('use_sim_time').value
            if self.has_parameter('use_sim_time')
            else False)
        self.get_logger().info(
            '机械臂任务节点已加载: '
            f'script_path={os.path.abspath(__file__)}, '
            f'real_script_path={os.path.realpath(__file__)}, '
            f'arm_speed_multiplier={float(self.get_parameter("arm_speed_multiplier").value):.2f}, '
            f'simulation_fast_mode={self.parameter_as_bool("simulation_fast_mode")}, '
            f'real_hardware_mode={self.parameter_as_bool("real_hardware_mode")}, '
            f'home_to_initial_duration={float(self.get_parameter("home_to_initial_duration").value):.3f}s, '
            f'initial_to_target_duration={float(self.get_parameter("initial_to_target_duration").value):.3f}s, '
            f'min_arm_trajectory_duration={float(self.get_parameter("min_arm_trajectory_duration").value):.3f}s, '
            f'max_arm_trajectory_duration={float(self.get_parameter("max_arm_trajectory_duration").value):.3f}s, '
            f'use_sim_time={use_sim_time}')

    def run_task(self):
        self.set_task_state('WAIT_LEAK')
        self.get_logger().info('等待 /leak_pose_map 泄漏源位置...')
        leak_pose = self.wait_for_leak_pose()
        if self.shutdown_event.is_set():
            return
        if leak_pose is None:
            self.get_logger().error('一直没有收到 leak_pose_map，任务停止。')
            return

        self.set_task_state('WAIT_NAVIGATION')
        if not self.wait_for_navigation_ready():
            self.get_logger().error('导航系统没有准备好，任务停止。')
            return

        nav_ok = False
        work_pose = None
        nav_attempts = max(1, int(self.get_parameter('nav_result_retry_count').value))
        nav_retry_delay = max(
            0.0, float(self.get_parameter('nav_result_retry_delay_sec').value))
        for attempt in range(1, nav_attempts + 1):
            work_pose = self.compute_work_pose(leak_pose)
            self.set_task_state('NAVIGATING')
            self.get_logger().info(
                f'导航到泄漏源作业位姿({attempt}/{nav_attempts}): '
                f'x={work_pose.pose.position.x:.2f}, y={work_pose.pose.position.y:.2f}')
            nav_ok = self.send_nav_goal(work_pose)
            if not nav_ok and self.is_robot_near_goal(work_pose):
                nav_ok = True
                self.get_logger().warn(
                    'Nav2 没有返回 SUCCEEDED，但机器人已经进入到达阈值，继续触发机械臂。')
            if nav_ok or self.shutdown_event.is_set():
                break
            if attempt < nav_attempts:
                self.get_logger().warn('Nav2 本次未到达，重新计算作业位姿后再试一次。')
                self.sleep_for(nav_retry_delay)
        if self.shutdown_event.is_set():
            return
        if nav_ok:
            self.set_task_state('NAVIGATION_REACHED')
        self.stop_car()
        self.get_logger().info('小车到达作业位姿，已连续发布 /cmd_vel=0，准备触发机械臂。')
        if not nav_ok:
            self.get_logger().error('Nav2 未报告成功到达，任务停止；不会让机械臂去够远处泄漏点。')
            return

        self.get_logger().info(
            '泄漏点 map/world 调试: '
            f'frame={leak_pose.header.frame_id}, '
            f'x={leak_pose.pose.position.x:.3f}, '
            f'y={leak_pose.pose.position.y:.3f}, '
            f'z={leak_pose.pose.position.z:.3f}')
        leak_in_arm = self.transform_point_to_arm_base(leak_pose)
        if leak_in_arm is None:
            self.get_logger().warn('无法把泄漏点转换到 rebot_base_link，使用默认正前方喷洒姿态。')
            leak_in_arm = (0.8, 0.0, 0.0)

        self.get_logger().info(
            '泄漏点已转换到 rebot_base_link: '
            f'x={leak_in_arm[0]:.3f}, y={leak_in_arm[1]:.3f}, z={leak_in_arm[2]:.3f}')
        arm_mode = str(self.get_parameter('arm_control_mode').value).strip().lower()
        if not self.check_arm_runtime_ready(require_moveit=arm_mode not in ('rl', 'safe_rl', 'safe-rl')):
            self.set_task_state('ARM_FAILED')
            self.get_logger().error('机械臂运行环境未准备好，任务停止。')
            return

        if arm_mode in ('rl', 'safe_rl', 'safe-rl'):
            self.publish_spray_target(leak_in_arm)
            self.set_task_state('ARM_EXECUTION')
            arm_ok = self.run_safe_rl_arm_controller()
            if not arm_ok:
                self.set_task_state('ARM_FAILED')
                self.get_logger().error('安全强化学习机械臂控制未完成，任务停止。')
                return
        else:
            arm_ok = self.run_two_stage_moveit_task(leak_pose, leak_in_arm)
            if not arm_ok:
                self.set_task_state('ARM_FAILED')
                self.get_logger().error(
                    'spray_tip_link 位姿 IK/规划失败，任务停止；'
                    '不会降级执行可能扎地或碰车的旧关节姿态。')
                return

        self.mirror_arm_pose_to_gazebo()
        if not self.check_current_tip_ground_clearance(leak_in_arm):
            self.set_task_state('ARM_FAILED')
            self.get_logger().error('喷药前最终喷头高度检查失败，拒绝发布 /spray/start。')
            return
        self.set_task_state('SPRAYING')
        self.start_spray()
        self.get_logger().info('机械臂已到喷药姿态，开始喷药。')
        self.wait_for_neutralization()
        self.stop_spray()
        self.get_logger().info('喷药动作结束。')

        if not self.parameter_as_bool('return_home_after_spray'):
            self.task_done = True
            self.set_task_state('DONE')
            self.get_logger().info('泄漏源处置完成，机械臂保持在喷药姿态。')
            return

        home = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0,
            'joint4': 0.0,
            'joint5': 0.0,
            'joint6': 0.0,
        }
        if not self.move_arm_with_moveit(home):
            self.send_joint_trajectory(home, duration_sec=2.0)
        self.mirror_arm_pose_to_gazebo()
        self.task_done = True
        self.set_task_state('DONE')
        self.get_logger().info('泄漏源处置完成，机械臂已回 home。')

    def is_robot_near_goal(self, goal_pose):
        info = self.navigation_goal_distance_info(goal_pose)
        if info is None:
            self.get_logger().warn('无法读取当前小车位置，不能用距离阈值判断是否到达。')
            return False
        base_xy, distance, tolerance, source = info
        self.get_logger().info(
            f'到达阈值检查: robot=({base_xy[0]:.3f}, {base_xy[1]:.3f}) from {source}, '
            f'goal=({goal_pose.pose.position.x:.3f}, {goal_pose.pose.position.y:.3f}), '
            f'distance={distance:.3f}, tolerance={tolerance:.3f}')
        return distance <= tolerance

    def navigation_goal_tolerance(self):
        if self.has_parameter('navigation_goal_tolerance'):
            tolerance = float(self.get_parameter('navigation_goal_tolerance').value)
            if tolerance > 0.0:
                return max(0.05, tolerance)
        return max(0.05, float(self.get_parameter('nav_goal_reached_tolerance').value))

    def navigation_goal_distance_info(self, goal_pose):
        tolerance = self.navigation_goal_tolerance()
        base_xy = self.lookup_base_xy()
        source = (
            f'tf({self.get_parameter("map_frame").value}'
            f'->{self.get_parameter("base_frame").value})')
        if base_xy is None and self.latest_odom_msg is not None:
            odom_pose = self.latest_odom_msg.pose.pose.position
            base_xy = (float(odom_pose.x), float(odom_pose.y))
            source = 'odom fallback'
        if base_xy is None:
            return None

        dx = float(base_xy[0]) - float(goal_pose.pose.position.x)
        dy = float(base_xy[1]) - float(goal_pose.pose.position.y)
        distance = math.hypot(dx, dy)
        return base_xy, distance, tolerance, source

    def log_navigation_reach_status(
        self,
        label,
        goal_pose,
        reached_by_action=False,
        reached_by_distance=False,
    ):
        info = self.navigation_goal_distance_info(goal_pose)
        if info is None:
            self.get_logger().info(
                f'{label}: current_robot_pose=unavailable, '
                f'target_pose=({goal_pose.pose.position.x:.3f}, '
                f'{goal_pose.pose.position.y:.3f}), '
                f'distance_to_goal=unavailable, '
                f'navigation_goal_tolerance={self.navigation_goal_tolerance():.3f}, '
                f'navigation_reached_by_action={reached_by_action}, '
                f'navigation_reached_by_distance={reached_by_distance}')
            return
        base_xy, distance, tolerance, source = info
        self.get_logger().info(
            f'{label}: current_robot_pose=({base_xy[0]:.3f}, {base_xy[1]:.3f}) '
            f'from {source}, target_pose=({goal_pose.pose.position.x:.3f}, '
            f'{goal_pose.pose.position.y:.3f}), '
            f'distance_to_goal={distance:.3f}, '
            f'navigation_goal_tolerance={tolerance:.3f}, '
            f'navigation_reached_by_action={reached_by_action}, '
            f'navigation_reached_by_distance={reached_by_distance}')

    def check_arm_runtime_ready(self, require_moveit=True):
        timeout = max(1.0, float(self.get_parameter('arm_runtime_ready_timeout_sec').value))
        start = time.monotonic()
        next_log = start
        move_group_ready = False
        trajectory_ready = False
        joints_ready = False
        base_to_arm = None
        arm_to_tip = None

        while rclpy.ok() and not self.shutdown_event.is_set():
            move_group_ready = (
                True if not require_moveit
                else self.move_group_client.wait_for_server(timeout_sec=0.1)
            )
            trajectory_ready = self.trajectory_client.wait_for_server(timeout_sec=0.1)
            joints_ready = all(name in self.latest_joint_positions for name in self.arm_joint_names)
            base_to_arm = self.lookup_link_xyz(
                str(self.get_parameter('base_frame').value),
                str(self.get_parameter('arm_base_frame').value),
                timeout_sec=0.05)
            arm_to_tip = self.lookup_link_xyz(
                str(self.get_parameter('arm_base_frame').value),
                str(self.get_parameter('tip_link').value),
                timeout_sec=0.05)

            if move_group_ready and trajectory_ready and joints_ready and base_to_arm and arm_to_tip:
                self.get_logger().info(
                    '机械臂运行环境检查通过: '
                    f'move_group={move_group_ready}, '
                    f'{self.get_parameter("arm_controller_action_name").value}=ready, '
                    f'joint_states={self.arm_joint_text()}, '
                    f'base_link->rebot_base_link=({base_to_arm[0]:.3f}, '
                    f'{base_to_arm[1]:.3f}, {base_to_arm[2]:.3f}), '
                    f'rebot_base_link->{self.get_parameter("tip_link").value}=('
                    f'{arm_to_tip[0]:.3f}, {arm_to_tip[1]:.3f}, {arm_to_tip[2]:.3f})')
                return True

            now = time.monotonic()
            if now - start > timeout:
                self.get_logger().error(
                    '机械臂运行环境检查超时: '
                    f'move_group={move_group_ready}, '
                    f'controller_action={trajectory_ready}, '
                    f'joint_states_ready={joints_ready}, '
                    f'base_to_arm_tf={base_to_arm is not None}, '
                    f'arm_to_tip_tf={arm_to_tip is not None}')
                return False
            if now >= next_log:
                self.get_logger().info(
                    '等待机械臂运行环境: '
                    f'move_group={move_group_ready}, '
                    f'controller_action={trajectory_ready}, '
                    f'joint_states_ready={joints_ready}, '
                    f'base_to_arm_tf={base_to_arm is not None}, '
                    f'arm_to_tip_tf={arm_to_tip is not None}')
                next_log = now + 1.0
            self.sleep_for(0.05)
        return False

    def arm_joint_text(self):
        if not all(name in self.latest_joint_positions for name in self.arm_joint_names):
            return 'incomplete'
        return '[' + ', '.join(
            f'{name}={self.latest_joint_positions[name]:+.3f}'
            for name in self.arm_joint_names
        ) + ']'

    def wait_for_leak_pose(self):
        start = time.monotonic()
        while rclpy.ok() and not self.shutdown_event.is_set():
            if self.latest_leak_pose is not None:
                return self.latest_leak_pose
            if time.monotonic() - start > 60.0:
                return None
            self.sleep_for(0.1)
        return None

    def wait_for_navigation_ready(self):
        timeout = float(self.get_parameter('nav_ready_timeout_sec').value)
        settle_sec = max(0.0, float(self.get_parameter('nav_ready_settle_sec').value))
        start = time.monotonic()
        next_log = start

        while rclpy.ok() and not self.shutdown_event.is_set():
            elapsed = time.monotonic() - start
            if elapsed > timeout:
                return False

            map_ready = self.latest_map_time is not None
            odom_ready = self.latest_odom_time is not None
            tf_ready = self.lookup_base_xy() is not None
            nav_ready = self.nav_client.wait_for_server(timeout_sec=0.1)

            if map_ready and odom_ready and tf_ready and nav_ready:
                if settle_sec > 0.0:
                    self.get_logger().info(
                        f'导航输入已就绪，等待 Nav2 完成激活稳定 {settle_sec:.1f}s...')
                    self.sleep_for(settle_sec)
                return True

            now = time.monotonic()
            if now >= next_log:
                self.get_logger().info(
                    '等待导航准备: '
                    f'map={map_ready}, odom={odom_ready}, '
                    f'tf(map->{self.get_parameter("base_frame").value})={tf_ready}, '
                    f'nav2_action={nav_ready}')
                next_log = now + 2.0
            self.sleep_for(0.1)

        return False

    def compute_work_pose(self, leak_pose):
        leak_x = leak_pose.pose.position.x
        leak_y = leak_pose.pose.position.y
        base_xy = self.lookup_base_xy()
        if base_xy is None:
            base_xy = (0.0, 0.0)

        dx = base_xy[0] - leak_x
        dy = base_xy[1] - leak_y
        length = math.hypot(dx, dy)
        if length < 1e-3:
            dx, dy, length = -1.0, -1.0, math.sqrt(2.0)

        work_clearance = max(0.0, float(self.get_parameter('work_distance').value))
        vehicle_radius = max(0.0, float(self.get_parameter('vehicle_safety_radius').value))
        center_to_leak_distance = vehicle_radius + work_clearance
        work_x = leak_x + dx / length * center_to_leak_distance
        work_y = leak_y + dy / length * center_to_leak_distance
        yaw = math.atan2(leak_y - work_y, leak_x - work_x)
        self.get_logger().info(
            f'作业距离按车体半径计算: vehicle_radius={vehicle_radius:.3f} m, '
            f'clearance={work_clearance:.3f} m, '
            f'base_center_to_leak={center_to_leak_distance:.3f} m')

        goal = PoseStamped()
        goal.header.frame_id = str(self.get_parameter('map_frame').value)
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = work_x
        goal.pose.position.y = work_y
        goal.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.orientation.w = math.cos(yaw * 0.5)
        return goal

    def lookup_base_xy(self):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                str(self.get_parameter('map_frame').value),
                str(self.get_parameter('base_frame').value),
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
        except TransformException:
            return None
        return (tf_msg.transform.translation.x, tf_msg.transform.translation.y)

    def lookup_link_xyz(self, target_frame, source_frame, timeout_sec=0.2):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=timeout_sec))
        except TransformException:
            return None
        t = tf_msg.transform.translation
        return (float(t.x), float(t.y), float(t.z))

    def send_nav_goal(self, pose):
        if not self.nav_client.wait_for_server(timeout_sec=25.0):
            self.get_logger().error('找不到 Nav2 navigate_to_pose action server。')
            return False

        retry_count = max(1, int(self.get_parameter('nav_goal_retry_count').value))
        retry_delay = max(0.0, float(self.get_parameter('nav_goal_retry_delay_sec').value))

        handle = None
        for attempt in range(1, retry_count + 1):
            pose.header.stamp = self.get_clock().now().to_msg()
            goal = NavigateToPose.Goal()
            goal.pose = pose
            self.latest_nav_feedback = None
            self.log_nav_execution_snapshot(
                f'准备发送 Nav2 目标 {attempt}/{retry_count}', pose)
            send_future = self.nav_client.send_goal_async(
                goal,
                feedback_callback=self.on_nav_feedback)
            if not self.wait_future(send_future, 10.0):
                self.get_logger().warn(
                    f'发送 Nav2 目标超时，第 {attempt}/{retry_count} 次。')
            else:
                handle = send_future.result()
                if handle.accepted:
                    self.get_logger().info(
                        f'Nav2 action accepted: goal=('
                        f'{pose.pose.position.x:.3f}, '
                        f'{pose.pose.position.y:.3f}), '
                        f'attempt={attempt}/{retry_count}')
                    self.log_nav_execution_snapshot('Nav2 goal accepted', pose)
                    break
                self.get_logger().warn(
                    f'Nav2 拒绝了作业位姿目标，第 {attempt}/{retry_count} 次。')

            handle = None
            if attempt < retry_count:
                self.sleep_for(retry_delay)

        if handle is None or not handle.accepted:
            self.get_logger().error('Nav2 多次拒绝作业位姿目标。')
            return False

        result_future = handle.get_result_async()
        timeout = float(self.get_parameter('nav_goal_timeout_sec').value)
        hold_sec = max(0.0, float(self.get_parameter('navigation_goal_hold_sec').value))
        start = time.monotonic()
        next_debug = start
        distance_reached_since = None
        while rclpy.ok() and not self.shutdown_event.is_set() and not result_future.done():
            now = time.monotonic()
            if now - start > timeout:
                self.log_nav_execution_snapshot('Nav2 导航等待超时前诊断', pose)
                self.log_navigation_reach_status(
                    'Nav2 timeout reach status',
                    pose,
                    reached_by_action=False,
                    reached_by_distance=False)
                self.get_logger().error('Nav2 导航等待超时。')
                return False
            distance_info = self.navigation_goal_distance_info(pose)
            distance_reached = False
            if distance_info is not None:
                _, distance, tolerance, _ = distance_info
                distance_reached = distance <= tolerance
                if distance_reached:
                    if distance_reached_since is None:
                        distance_reached_since = now
                    elif now - distance_reached_since >= hold_sec:
                        self.log_navigation_reach_status(
                            'Nav2 distance fallback reached',
                            pose,
                            reached_by_action=False,
                            reached_by_distance=True)
                        self.get_logger().warn(
                            f'Nav2 action result 尚未返回，但距离目标小于阈值已持续 '
                            f'{hold_sec:.2f}s，触发机械臂流程并取消当前导航 goal。')
                        cancel_future = handle.cancel_goal_async()
                        self.wait_future(cancel_future, 1.0)
                        return True
                else:
                    distance_reached_since = None
            if now >= next_debug:
                self.log_nav_execution_snapshot('Nav2 正在执行', pose)
                self.log_navigation_reach_status(
                    'Nav2 reach check',
                    pose,
                    reached_by_action=False,
                    reached_by_distance=distance_reached)
                next_debug = now + max(
                    0.5, float(self.get_parameter('nav_debug_log_period_sec').value))
            self.sleep_for(0.05)
        if self.shutdown_event.is_set():
            return False

        result = result_future.result()
        status_name = self.goal_status_name(result.status)
        self.get_logger().info(f'Nav2 action result: {status_name} ({result.status})')
        self.log_nav_execution_snapshot(f'Nav2 result {status_name}', pose)
        reached_by_action = result.status == GoalStatus.STATUS_SUCCEEDED
        distance_info = self.navigation_goal_distance_info(pose)
        reached_by_distance = (
            False if distance_info is None
            else distance_info[1] <= distance_info[2])
        self.log_navigation_reach_status(
            'Nav2 final reach status',
            pose,
            reached_by_action=reached_by_action,
            reached_by_distance=reached_by_distance)
        return reached_by_action

    def on_nav_feedback(self, msg):
        self.latest_nav_feedback = msg.feedback

    def log_nav_execution_snapshot(self, label, goal_pose=None):
        controller_state = self.lifecycle_state_label(self.controller_state_client)
        bt_state = self.lifecycle_state_label(self.bt_state_client)
        smoother_state = self.lifecycle_state_label(self.velocity_smoother_state_client)
        robot_pose = self.odom_pose_text()
        nav_cmd = self.twist_text(self.latest_nav_cmd, self.latest_nav_cmd_time)
        final_cmd = self.twist_text(self.latest_final_cmd, self.latest_final_cmd_time)
        feedback = self.nav_feedback_text()
        goal_text = 'none'
        if goal_pose is not None:
            goal_text = (
                f'({goal_pose.pose.position.x:.3f}, '
                f'{goal_pose.pose.position.y:.3f}, '
                f'yaw_qz={goal_pose.pose.orientation.z:.3f}, '
                f'qw={goal_pose.pose.orientation.w:.3f})')
        endpoint_text = self.cmd_vel_endpoint_text()
        self.get_logger().info(
            f'{label}: goal={goal_text}, robot={robot_pose}, '
            f'controller_server={controller_state}, bt_navigator={bt_state}, '
            f'velocity_smoother={smoother_state}, '
            f'cmd_chain={self.get_parameter("nav_cmd_topic").value}'
            f' -> {self.get_parameter("final_cmd_topic").value}'
            ' -> Gazebo skid_steer_drive, '
            f'nav_cmd={nav_cmd}, final_cmd={final_cmd}, '
            f'feedback={feedback}, endpoints={endpoint_text}')

    def lifecycle_state_label(self, client):
        if client is None or not client.wait_for_service(timeout_sec=0.02):
            return 'unavailable'
        future = client.call_async(GetState.Request())
        if not self.wait_future(future, 0.2):
            return 'timeout'
        state = future.result().current_state
        return f'{state.label}[{state.id}]'

    def odom_pose_text(self):
        if self.latest_odom_msg is None:
            return 'odom=none'
        pose = self.latest_odom_msg.pose.pose
        return (
            f'odom=({pose.position.x:.3f}, {pose.position.y:.3f}, '
            f'qz={pose.orientation.z:.3f}, qw={pose.orientation.w:.3f})')

    def nav_feedback_text(self):
        feedback = self.latest_nav_feedback
        if feedback is None:
            return 'none'
        current = feedback.current_pose.pose.position
        return (
            f'distance_remaining={feedback.distance_remaining:.3f}, '
            f'current=({current.x:.3f}, {current.y:.3f}), '
            f'recoveries={feedback.number_of_recoveries}')

    def cmd_vel_endpoint_text(self):
        final_topic = str(self.get_parameter('final_cmd_topic').value)
        nav_topic = str(self.get_parameter('nav_cmd_topic').value)
        nav_pubs = self.node_names_for_endpoints(
            self.get_publishers_info_by_topic(nav_topic))
        nav_subs = self.node_names_for_endpoints(
            self.get_subscriptions_info_by_topic(nav_topic))
        final_pubs = self.node_names_for_endpoints(
            self.get_publishers_info_by_topic(final_topic))
        final_subs = self.node_names_for_endpoints(
            self.get_subscriptions_info_by_topic(final_topic))
        return (
            f'{nav_topic}: pubs={nav_pubs}, subs={nav_subs}; '
            f'{final_topic}: pubs={final_pubs}, subs={final_subs}')

    @staticmethod
    def node_names_for_endpoints(endpoints):
        names = sorted({
            f'/{endpoint.node_name}' if not endpoint.node_namespace.strip('/') else
            f'{endpoint.node_namespace.rstrip("/")}/{endpoint.node_name}'
            for endpoint in endpoints
        })
        return '[' + ','.join(names) + ']'

    @staticmethod
    def twist_text(msg, stamp):
        if msg is None or stamp is None:
            return 'none'
        age = max(0.0, time.monotonic() - stamp)
        return f'vx={msg.linear.x:+.3f}, wz={msg.angular.z:+.3f}, age={age:.2f}s'

    @staticmethod
    def goal_status_name(status):
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, f'STATUS_{status}')

    def stop_car(self):
        zero = Twist()
        for _ in range(10):
            if self.shutdown_event.is_set():
                return
            self.cmd_pub.publish(zero)
            self.sleep_for(0.05)

    def mirror_arm_pose_to_gazebo(self):
        if not self.parameter_as_bool('enable_gazebo_arm_mirror'):
            return
        if not self.wait_for_arm_joint_positions(2.0):
            self.get_logger().warn('没有拿到完整 /joint_states，Gazebo 机械臂外观无法同步。')
            return

        duration_sec = max(
            0.1, float(self.get_parameter('gazebo_arm_mirror_duration_sec').value))
        traj = JointTrajectory()
        # The Gazebo joint trajectory plugin executes zero-stamped trajectories
        # immediately; wall-time stamps can look like future sim-time commands.
        traj.header.frame_id = 'world'
        traj.joint_names = list(self.arm_joint_names)

        point = JointTrajectoryPoint()
        point.positions = [self.latest_joint_positions[name] for name in self.arm_joint_names]
        point.time_from_start = self.duration_msg(duration_sec)
        traj.points.append(point)

        for _ in range(5):
            if self.shutdown_event.is_set():
                return
            self.gazebo_arm_trajectory_pub.publish(traj)
            self.sleep_for(0.05)
        self.get_logger().info(
            '已同步 Gazebo 机械臂外观到当前关节角: '
            + ', '.join(
                f'{name}={self.latest_joint_positions[name]:.2f}'
                for name in self.arm_joint_names))
        self.sleep_for(duration_sec)

    def wait_for_arm_joint_positions(self, timeout_sec):
        start = time.monotonic()
        while rclpy.ok() and not self.shutdown_event.is_set():
            if all(name in self.latest_joint_positions for name in self.arm_joint_names):
                return True
            if time.monotonic() - start > timeout_sec:
                return False
            self.sleep_for(0.05)
        return False

    @staticmethod
    def duration_msg(duration_sec):
        sec = int(duration_sec)
        nanosec = int(round((duration_sec - sec) * 1e9))
        if nanosec >= 1000000000:
            sec += 1
            nanosec -= 1000000000
        return DurationMsg(sec=sec, nanosec=nanosec)

    def transform_point_to_arm_base(self, pose):
        target_frame = str(self.get_parameter('arm_base_frame').value)
        source_frame = pose.header.frame_id
        tf_msg = None
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame, source_frame,
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
        except TransformException as exc:
            self.get_logger().warn(f'TF 转换失败: {source_frame}->{target_frame}: {exc}')
            if source_frame == str(self.get_parameter('map_frame').value):
                fallback_frame = 'odom'
                try:
                    tf_msg = self.tf_buffer.lookup_transform(
                        target_frame, fallback_frame,
                        rclpy.time.Time(), timeout=Duration(seconds=0.5))
                    self.get_logger().warn(
                        'map->rebot_base_link 不可用，按当前仿真 map==odom 的假设 '
                        '临时使用 odom->rebot_base_link 转换泄漏点。')
                except TransformException as fallback_exc:
                    self.get_logger().warn(
                        f'备用 TF 转换也失败: {fallback_frame}->{target_frame}: '
                        f'{fallback_exc}')
                    return None
            else:
                return None

        q = tf_msg.transform.rotation
        p = pose.pose.position
        rotated = self.rotate_vector([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        x = rotated[0] + tf_msg.transform.translation.x
        y = rotated[1] + tf_msg.transform.translation.y
        z = rotated[2] + tf_msg.transform.translation.z
        return (x, y, z)

    def publish_spray_target(self, leak_in_arm):
        self.wait_for_publisher_subscribers(
            self.spray_pose_pub,
            'spray/target_pose',
            timeout_sec=10.0)
        standoff = max(0.0, float(self.get_parameter('spray_standoff').value))
        pose = self.compute_spray_tip_pose(leak_in_arm, standoff, 0.0)
        pose = self.validate_spray_pose(pose, leak_in_arm, roll=0.0)
        current_tip = self.lookup_link_xyz(
            str(self.get_parameter('arm_base_frame').value),
            str(self.get_parameter('tip_link').value))
        current_tip_text = (
            'unavailable'
            if current_tip is None
            else f'({current_tip[0]:.3f}, {current_tip[1]:.3f}, {current_tip[2]:.3f})'
        )
        ground_z = float(leak_in_arm[2])
        min_tip_height = max(
            float(self.get_parameter('min_spray_tip_z').value),
            float(self.get_parameter('ground_clearance').value),
            float(self.get_parameter('spray_tip_work_height').value))
        aim_axis = self.pose_x_axis(pose)
        self.get_logger().info(
            '发布 RL 安全喷头工作点: '
            f'leak=({leak_in_arm[0]:.3f}, {leak_in_arm[1]:.3f}, {leak_in_arm[2]:.3f}), '
            f'standoff={standoff:.3f}, '
            f'ground_z_in_rebot={ground_z:.3f}, '
            f'min_tip_height={min_tip_height:.3f}, '
            f'tip_z_above_leak={pose.pose.position.z - ground_z:.3f}, '
            f'current_{self.get_parameter("tip_link").value}={current_tip_text}, '
            f'target=({pose.pose.position.x:.3f}, '
            f'{pose.pose.position.y:.3f}, '
            f'{pose.pose.position.z:.3f}), '
            f'aim_axis=({aim_axis[0]:.3f}, {aim_axis[1]:.3f}, {aim_axis[2]:.3f})')
        for _ in range(5):
            if self.shutdown_event.is_set():
                return
            pose.header.stamp = self.get_clock().now().to_msg()
            self.spray_pose_pub.publish(pose)
            self.sleep_for(0.05)

    def run_safe_rl_arm_controller(self):
        self.rl_done = False
        self.wait_for_publisher_subscribers(
            self.rl_start_pub,
            str(self.get_parameter('rl_start_topic').value),
            timeout_sec=10.0)
        msg = Bool()
        msg.data = True
        for _ in range(8):
            if self.shutdown_event.is_set():
                return False
            self.rl_start_pub.publish(msg)
            self.sleep_for(0.05)

        timeout = max(1.0, float(self.get_parameter('rl_timeout_sec').value))
        start = time.monotonic()
        next_log = start
        self.get_logger().info('已启动安全强化学习机械臂控制，等待 rebot_safe_rl/done...')
        while rclpy.ok() and not self.shutdown_event.is_set():
            if self.rl_done:
                self.get_logger().info('安全强化学习机械臂控制完成。')
                return True
            if time.monotonic() - start > timeout:
                self.get_logger().error(f'安全强化学习机械臂控制超时: {timeout:.1f}s')
                return False
            now = time.monotonic()
            if now >= next_log:
                self.get_logger().info('等待安全强化学习机械臂控制完成...')
                next_log = now + 3.0
            self.sleep_for(0.1)
        return False

    def wait_for_publisher_subscribers(self, publisher, topic_name, timeout_sec):
        start = time.monotonic()
        next_log = start
        while rclpy.ok() and not self.shutdown_event.is_set():
            if publisher.get_subscription_count() > 0:
                return True
            elapsed = time.monotonic() - start
            if elapsed > timeout_sec:
                self.get_logger().warn(
                    f'{topic_name} 暂时没有订阅者，继续发布；如果机械臂不动，请检查 RL 控制器。')
                return False
            now = time.monotonic()
            if now >= next_log:
                self.get_logger().info(f'等待 {topic_name} 订阅者就绪...')
                next_log = now + 1.0
            self.sleep_for(0.05)
        return False

    def run_two_stage_moveit_task(self, leak_pose_map, leak_in_arm):
        home_zero_pose = self.home_zero_pose()
        initial_pose = self.compute_initial_pose(leak_in_arm)
        target_pose_text = (
            f'({leak_pose_map.pose.position.x:.3f}, '
            f'{leak_pose_map.pose.position.y:.3f}, '
            f'{leak_pose_map.pose.position.z:.3f})')
        self.get_logger().info(
            '机械臂两段式任务配置: '
            f'arm_speed_multiplier={self.arm_speed_multiplier():.2f}, '
            f'simulation_fast_mode={self.parameter_as_bool("simulation_fast_mode")}, '
            f'real_hardware_mode={self.parameter_as_bool("real_hardware_mode")}, '
            f'home_zero_pose={self.joint_target_text(home_zero_pose)}, '
            f'initial_pose={self.joint_target_text(initial_pose)}, '
            f'target_pose={target_pose_text}')
        self.log_planning_scene_debug()

        if not self.wait_for_arm_joint_positions(3.0):
            self.get_logger().error('没有收到完整 joint1~joint6，不能开始机械臂两段式任务。')
            return False

        if not self.current_arm_pose_near(home_zero_pose, tolerance=0.08):
            self.get_logger().warn(
                '当前机械臂不在 home_zero_pose，先安全回到 home_zero_pose 后再开始两段式流程: '
                f'current={self.arm_joint_text()}')
            if not self.move_arm_with_moveit(
                home_zero_pose,
                segment_label='current_to_home_zero_pose',
                leak_in_arm=leak_in_arm):
                return False

        home_initial_start_wall = time.monotonic()
        self.get_logger().info(
            f'wall time: start home->initial at {home_initial_start_wall:.6f}; '
            f'{self.time_status_text()}')
        self.get_logger().info('Move arm from home zero pose to initial pose')
        if not self.move_arm_with_moveit(
            initial_pose,
            segment_label='home_to_initial',
            leak_in_arm=leak_in_arm):
            return False
        home_initial_done_wall = time.monotonic()
        self.get_logger().info(
            f'wall time: initial reached at {home_initial_done_wall:.6f}; '
            f'elapsed={home_initial_done_wall - home_initial_start_wall:.3f}s; '
            f'{self.time_status_text()}')
        self.get_logger().info('Initial pose reached')

        initial_target_start_wall = time.monotonic()
        self.get_logger().info(
            f'wall time: start initial->target at {initial_target_start_wall:.6f}; '
            f'{self.time_status_text()}')
        self.get_logger().info(
            'Move arm from initial pose to target pose '
            f'{target_pose_text}')
        if not self.move_spray_tip_to_leak(leak_in_arm, leak_pose_map):
            return False
        initial_target_done_wall = time.monotonic()
        self.get_logger().info(
            f'wall time: target reached at {initial_target_done_wall:.6f}; '
            f'elapsed={initial_target_done_wall - initial_target_start_wall:.3f}s; '
            f'{self.time_status_text()}')
        self.get_logger().info('Target pose reached')
        return True

    def home_zero_pose(self):
        return {name: 0.0 for name in self.arm_joint_names}

    def compute_initial_pose(self, leak_in_arm):
        return self.compute_spray_joints(leak_in_arm)

    def current_arm_pose_near(self, target, tolerance):
        if not all(name in self.latest_joint_positions for name in self.arm_joint_names):
            return False
        return all(
            abs(float(self.latest_joint_positions[name]) - float(target[name])) <= tolerance
            for name in self.arm_joint_names)

    @staticmethod
    def joint_target_text(target):
        return '[' + ', '.join(
            f'{name}={float(target[name]):+.3f}'
            for name in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        ) + ']'

    def log_planning_scene_debug(self):
        map_frame = str(self.get_parameter('map_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        base_in_map = self.lookup_link_xyz(map_frame, base_frame, timeout_sec=0.2)
        base_height_text = (
            'unavailable'
            if base_in_map is None
            else f'{base_in_map[2]:.3f}'
        )
        ground_thickness = float(self.get_parameter('ground_collision_thickness').value)
        ground_top_z = float(self.get_parameter('base_link_ground_z').value)
        ground_center_z = ground_top_z - 0.5 * ground_thickness
        objects = self.planning_scene_collision_objects()
        object_names = [obj.id for obj in objects]
        object_details = '; '.join(self.collision_object_debug_text(obj) for obj in objects)
        self.get_logger().info(
            'planning scene debug: '
            f'base_link height in {map_frame}={base_height_text}, '
            f'Gazebo ground assumed z=0.000 in {map_frame}/world, '
            f'ground collision object frame={base_frame}, '
            f'center_z={ground_center_z:.3f}, top_z={ground_top_z:.3f}, '
            f'objects={object_names}, '
            f'collision_objects_detail=[{object_details}], '
            f'planning_scene_update=diff_attached_to_each_goal')

    @staticmethod
    def collision_object_debug_text(obj):
        if not obj.primitives or not obj.primitive_poses:
            return f'{obj.id}(frame={obj.header.frame_id}, empty)'
        primitive = obj.primitives[0]
        pose = obj.primitive_poses[0]
        shape = 'box' if primitive.type == SolidPrimitive.BOX else (
            'cylinder' if primitive.type == SolidPrimitive.CYLINDER else
            f'type_{primitive.type}')
        dims = ','.join(f'{float(value):.3f}' for value in primitive.dimensions)
        return (
            f'{obj.id}(frame={obj.header.frame_id}, shape={shape}, '
            f'dims=[{dims}], xyz=({pose.position.x:.3f}, '
            f'{pose.position.y:.3f}, {pose.position.z:.3f}))')

    def move_spray_tip_to_leak(self, leak_in_arm, leak_pose_map=None):
        primary_link = str(self.get_parameter('tip_link').value)
        fallback_links = list(self.get_parameter('fallback_tip_links').value)
        link_candidates = self.unique_links([primary_link] + fallback_links)
        configured_standoff = float(self.get_parameter('spray_standoff').value)
        standoff_candidates = self.unique_values([configured_standoff, 0.10, 0.12, 0.15, 0.20])
        roll_candidates = [0.0, math.pi / 2.0, -math.pi / 2.0, math.pi]
        full_pose_attempt_limit = max(
            1, int(self.get_parameter('spray_full_pose_attempt_limit').value))
        full_pose_attempts = 0
        full_pose_limit_reached = False

        if leak_pose_map is not None:
            self.get_logger().info(
                'target_pose = '
                f'({leak_pose_map.pose.position.x:.3f}, '
                f'{leak_pose_map.pose.position.y:.3f}, '
                f'{leak_pose_map.pose.position.z:.3f}) in '
                f'{leak_pose_map.header.frame_id}; 将从该目标派生安全 spray_tip_link 位姿。')

        for link_name in link_candidates:
            if link_name != primary_link:
                self.get_logger().warn(f'尝试备用末端 link: {link_name}')
            for standoff in standoff_candidates:
                for roll in roll_candidates:
                    if full_pose_attempts >= full_pose_attempt_limit:
                        full_pose_limit_reached = True
                        break
                    full_pose_attempts += 1
                    target_pose = self.compute_spray_tip_pose(leak_in_arm, standoff, roll)
                    target_pose = self.validate_spray_pose(
                        target_pose, leak_in_arm, roll=roll)
                    self.publish_spray_target_pose(target_pose)
                    self.log_spray_pose_debug(target_pose, leak_in_arm, link_name)
                    self.get_logger().info(
                        f'尝试 {link_name} 位姿 IK: standoff={standoff:.3f}, '
                        f'roll={roll:.2f}, pose=('
                        f'{target_pose.pose.position.x:.3f}, '
                        f'{target_pose.pose.position.y:.3f}, '
                        f'{target_pose.pose.position.z:.3f})')
                    ok, code = self.move_tip_pose_with_moveit(
                        target_pose,
                        link_name,
                        leak_in_arm,
                        segment_label='initial_to_target')
                    if ok:
                        self.get_logger().info('Arm trajectory execution done: MoveIt 喷头位姿轨迹已完成。')
                        self.mirror_arm_pose_to_gazebo()
                        if not self.check_current_tip_ground_clearance(leak_in_arm):
                            self.get_logger().error(
                                'MoveIt 执行后喷头高度安全检查失败，立即回到 spray_ready，'
                                '本次不喷药。')
                            self.move_arm_with_moveit(self.compute_spray_joints(leak_in_arm))
                            self.mirror_arm_pose_to_gazebo()
                            return False
                        self.get_logger().info(f'{link_name} 已对准泄漏点。')
                        return True
                    self.get_logger().warn(
                        f'{link_name} 位姿 IK/规划失败: '
                        f'standoff={standoff:.3f}, roll={roll:.2f}, MoveItErrorCode={code}')
                if full_pose_limit_reached:
                    break
            if full_pose_limit_reached:
                self.get_logger().warn(
                    f'完整喷头位姿规划已尝试 {full_pose_attempts} 次仍失败，'
                    '转入位置约束/直接关节安全兜底，避免任务长时间卡住。')
                break
        if (
            self.parameter_as_bool('allow_position_only_spray_fallback')
            and self.try_position_only_spray_move(primary_link, leak_in_arm, standoff_candidates)
        ):
            return self.check_current_tip_ground_clearance(leak_in_arm)
        if self.parameter_as_bool('allow_direct_spray_joint_fallback'):
            fallback_joints = self.compute_direct_spray_fallback_joints(leak_in_arm)
            self.get_logger().warn(
                '喷头完整位姿 IK 和位置约束规划都失败，执行安全关节兜底喷洒姿态: '
                + ', '.join(f'{name}={value:.2f}' for name, value in fallback_joints.items()))
            if self.send_joint_trajectory(fallback_joints):
                self.set_task_state('ARM_EXECUTION')
                self.mirror_arm_pose_to_gazebo()
                return self.check_current_tip_ground_clearance(leak_in_arm)
        return False

    def try_position_only_spray_move(self, link_name, leak_in_arm, standoff_candidates):
        self.get_logger().warn(
            'spray_tip_link 完整 position+orientation IK 全部失败，'
            '开始尝试只约束喷头位置的 MoveIt 规划。')
        attempt_limit = max(
            1, int(self.get_parameter('spray_position_only_attempt_limit').value))
        attempts = 0
        for standoff in standoff_candidates:
            if attempts >= attempt_limit:
                self.get_logger().warn(
                    f'position-only 已尝试 {attempts} 次仍失败，'
                    '转入直接安全关节兜底。')
                break
            attempts += 1
            target_pose = self.compute_spray_tip_pose(leak_in_arm, standoff, roll=0.0)
            target_pose = self.validate_spray_pose(target_pose, leak_in_arm, roll=0.0)
            self.publish_spray_target_pose(target_pose)
            self.log_spray_pose_debug(target_pose, leak_in_arm, link_name)
            ok, code = self.move_tip_position_with_moveit(
                target_pose,
                link_name,
                leak_in_arm,
                segment_label='initial_to_target')
            if ok:
                self.get_logger().warn(
                    f'{link_name} 已到达喷洒位置；本次使用 position-only 兜底，'
                    '喷头方向由当前安全姿态保持。')
                self.mirror_arm_pose_to_gazebo()
                return True
            self.get_logger().warn(
                f'{link_name} position-only 规划失败: '
                f'standoff={standoff:.3f}, MoveItErrorCode={code}')
        return False

    def compute_spray_tip_pose(self, leak_in_arm, standoff, roll):
        leak = [float(leak_in_arm[0]), float(leak_in_arm[1]), float(leak_in_arm[2])]
        horizontal_axis = [leak[0], leak[1], 0.0]
        if self.norm(horizontal_axis) < 1e-6:
            horizontal_axis = [1.0, 0.0, 0.0]
        approach_axis = self.normalize(horizontal_axis)
        min_tip_height = max(
            float(self.get_parameter('min_spray_tip_z').value),
            float(self.get_parameter('ground_clearance').value),
            float(self.get_parameter('spray_tip_work_height').value))
        standoff = max(0.05, float(standoff))
        tip = [
            leak[0] - approach_axis[0] * standoff,
            leak[1] - approach_axis[1] * standoff,
            leak[2] + min_tip_height,
        ]
        tip = self.clamp_spray_tip_range(tip, approach_axis)
        aim_axis = self.compute_spray_aim_axis(tip, leak, approach_axis)
        quat = self.quaternion_with_x_axis(aim_axis, roll)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = str(self.get_parameter('arm_base_frame').value)
        pose.pose.position.x = tip[0]
        pose.pose.position.y = tip[1]
        pose.pose.position.z = tip[2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def validate_spray_target_pose(self, pose, leak_in_arm, roll=0.0):
        return self.validate_spray_pose(pose, leak_in_arm, roll)

    def validate_spray_pose(self, pose, leak_in_arm, roll=0.0):
        ground_z = float(leak_in_arm[2])
        min_tip_height = max(
            float(self.get_parameter('min_spray_tip_z').value),
            float(self.get_parameter('ground_clearance').value),
            float(self.get_parameter('spray_tip_work_height').value))
        min_target_z = ground_z + min_tip_height
        adjusted = False
        if pose.pose.position.z < min_target_z:
            old_z = pose.pose.position.z
            pose.pose.position.z = min_target_z
            adjusted = True
            self.get_logger().warn(
                '喷头目标 z 太低，已自动抬高: '
                f'old_z={old_z:.3f}, new_z={pose.pose.position.z:.3f}, '
                f'ground_z_in_rebot={ground_z:.3f}, '
                f'min_tip_height={min_tip_height:.3f}')

        leak = [float(leak_in_arm[0]), float(leak_in_arm[1]), float(leak_in_arm[2])]
        tip = [
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            float(pose.pose.position.z),
        ]
        horizontal_axis = [leak[0], leak[1], 0.0]
        if self.norm(horizontal_axis) < 1e-6:
            horizontal_axis = [1.0, 0.0, 0.0]
        approach_axis = self.normalize(horizontal_axis)
        clamped_tip = self.clamp_spray_tip_range(tip, approach_axis)
        if any(abs(clamped_tip[idx] - tip[idx]) > 1e-6 for idx in range(3)):
            self.get_logger().warn(
                '喷头目标超出自然可达半径，已调整到安全工作环: '
                f'old=({tip[0]:.3f}, {tip[1]:.3f}, {tip[2]:.3f}), '
                f'new=({clamped_tip[0]:.3f}, {clamped_tip[1]:.3f}, {clamped_tip[2]:.3f})')
            pose.pose.position.x = clamped_tip[0]
            pose.pose.position.y = clamped_tip[1]
            pose.pose.position.z = clamped_tip[2]
            tip = clamped_tip
            adjusted = True

        aim_axis = self.compute_spray_aim_axis(tip, leak, approach_axis)
        if pose.pose.position.z < ground_z + float(self.get_parameter('min_spray_tip_z').value):
            self.get_logger().error(
                '喷头目标低于地面安全下限，拒绝执行该目标: '
                f'target_z={pose.pose.position.z:.3f}, '
                f'ground_z_in_rebot={ground_z:.3f}, '
                f'min_spray_tip_z={float(self.get_parameter("min_spray_tip_z").value):.3f}')
        quat = self.quaternion_with_x_axis(aim_axis, roll)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        if not adjusted:
            self.get_logger().info(
                '喷头目标合法性检查通过: '
                f'z={pose.pose.position.z:.3f}, '
                f'ground_z_in_rebot={ground_z:.3f}, '
                f'min_spray_tip_z={float(self.get_parameter("min_spray_tip_z").value):.3f}, '
                f'ground_clearance={float(self.get_parameter("ground_clearance").value):.3f}, '
                f'aim_axis=({aim_axis[0]:.3f}, {aim_axis[1]:.3f}, {aim_axis[2]:.3f})')
        return pose

    def clamp_spray_tip_range(self, tip, approach_axis):
        min_range = max(0.0, float(self.get_parameter('spray_min_range').value))
        max_range = max(min_range + 1e-3, float(self.get_parameter('spray_max_range').value))
        xy_range = math.hypot(float(tip[0]), float(tip[1]))
        if xy_range < 1e-6:
            axis = self.normalize([approach_axis[0], approach_axis[1], 0.0])
            return [axis[0] * min_range, axis[1] * min_range, float(tip[2])]
        if xy_range < min_range or xy_range > max_range:
            target_range = min(max(xy_range, min_range), max_range)
            scale = target_range / xy_range
            return [float(tip[0]) * scale, float(tip[1]) * scale, float(tip[2])]
        return [float(tip[0]), float(tip[1]), float(tip[2])]

    def compute_spray_aim_axis(self, tip, leak, approach_axis):
        aim_height = max(0.0, float(self.get_parameter('spray_aim_height').value))
        aim_point = [float(leak[0]), float(leak[1]), float(leak[2]) + aim_height]
        raw = [
            aim_point[0] - float(tip[0]),
            aim_point[1] - float(tip[1]),
            aim_point[2] - float(tip[2]),
        ]
        if self.norm(raw) < 1e-6:
            raw = [approach_axis[0], approach_axis[1], 0.0]
        aim_axis = self.normalize(raw)
        max_downward_z = float(self.get_parameter('spray_max_downward_z').value)
        max_downward_z = min(0.0, max(-0.9, max_downward_z))
        if aim_axis[2] < max_downward_z:
            horizontal = math.sqrt(max(0.0, 1.0 - max_downward_z * max_downward_z))
            aim_axis = self.normalize([
                approach_axis[0] * horizontal,
                approach_axis[1] * horizontal,
                max_downward_z,
            ])
            self.get_logger().warn(
                '喷头朝向过度向下，已限制俯角: '
                f'limited_aim_axis=({aim_axis[0]:.3f}, {aim_axis[1]:.3f}, {aim_axis[2]:.3f})')
        return aim_axis

    def publish_spray_target_pose(self, pose):
        self.spray_pose_pub.publish(pose)

    def log_spray_pose_debug(self, pose, leak_in_arm, link_name):
        current_tip = self.lookup_link_xyz(
            str(self.get_parameter('arm_base_frame').value),
            str(self.get_parameter('tip_link').value))
        current_tip_text = (
            'unavailable'
            if current_tip is None
            else f'({current_tip[0]:.3f}, {current_tip[1]:.3f}, {current_tip[2]:.3f})'
        )
        aim_axis = self.pose_x_axis(pose)
        object_count = len(self.planning_scene_collision_objects())
        spray_pose_map = self.transform_arm_pose_to_map_text(pose)
        ground_size = float(self.get_parameter('ground_collision_size').value)
        ground_thickness = float(self.get_parameter('ground_collision_thickness').value)
        ground_z = (
            float(self.get_parameter('base_link_ground_z').value)
            - 0.5 * ground_thickness)
        self.get_logger().info(
            '喷洒位姿调试: '
            f'tip_link={link_name}, '
            f'leak_rebot=({leak_in_arm[0]:.3f}, {leak_in_arm[1]:.3f}, {leak_in_arm[2]:.3f}), '
            f'current_spray_tip={current_tip_text}, '
            f'spray_pose_rebot_base=({pose.pose.position.x:.3f}, '
            f'{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}), '
            f'spray_pose_map={spray_pose_map}, '
            f'target_z={pose.pose.position.z:.3f}, '
            f'min_spray_tip_z={float(self.get_parameter("min_spray_tip_z").value):.3f}, '
            f'ground_clearance={float(self.get_parameter("ground_clearance").value):.3f}, '
            f'aim_axis=({aim_axis[0]:.3f}, {aim_axis[1]:.3f}, {aim_axis[2]:.3f}), '
            f'planning_scene_collision_objects={object_count}, '
            f'ground_collision_object=base_link box '
            f'{ground_size:.1f}x{ground_size:.1f}x{ground_thickness:.3f} '
            f'at z={ground_z:.3f}, '
            f'elbow_up_constraints={self.parameter_as_bool("enable_elbow_up_constraints")}')

    def transform_arm_pose_to_map_text(self, pose):
        target_frame = str(self.get_parameter('map_frame').value)
        source_frame = str(self.get_parameter('arm_base_frame').value)
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except TransformException:
            return 'unavailable'
        q = tf_msg.transform.rotation
        p = pose.pose.position
        rotated = self.rotate_vector([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        return (
            f'({rotated[0] + tf_msg.transform.translation.x:.3f}, '
            f'{rotated[1] + tf_msg.transform.translation.y:.3f}, '
            f'{rotated[2] + tf_msg.transform.translation.z:.3f})')

    def check_current_tip_ground_clearance(self, leak_in_arm):
        tip = self.lookup_link_xyz(
            str(self.get_parameter('arm_base_frame').value),
            str(self.get_parameter('tip_link').value),
            timeout_sec=0.5)
        if tip is None:
            self.get_logger().error('无法读取当前 spray_tip_link TF，拒绝继续喷药。')
            return False
        ground_z = float(leak_in_arm[2])
        min_height = max(
            float(self.get_parameter('min_spray_tip_z').value),
            float(self.get_parameter('ground_clearance').value),
            float(self.get_parameter('spray_tip_work_height').value))
        safe_z = ground_z + min_height
        ok = tip[2] >= safe_z
        self.get_logger().info(
            '喷头执行后地面安全检查: '
            f'current_spray_tip_rebot=({tip[0]:.3f}, {tip[1]:.3f}, {tip[2]:.3f}), '
            f'ground_z_in_rebot={ground_z:.3f}, '
            f'actual_tip_z_above_leak={tip[2] - ground_z:.3f}, '
            f'min_spray_tip_z={float(self.get_parameter("min_spray_tip_z").value):.3f}, '
            f'spray_tip_work_height={float(self.get_parameter("spray_tip_work_height").value):.3f}, '
            f'required_tip_z>={safe_z:.3f}, '
            f'result={"passed" if ok else "FAILED"}')
        return ok

    def pose_x_axis(self, pose):
        q = pose.pose.orientation
        return self.rotate_vector([1.0, 0.0, 0.0], [q.x, q.y, q.z, q.w])

    def move_tip_pose_with_moveit(
        self,
        target_pose,
        link_name,
        leak_in_arm,
        segment_label='initial_to_target',
    ):
        if not self.move_group_client.wait_for_server(timeout_sec=12.0):
            self.get_logger().warn('MoveGroup action server 不可用。')
            self.set_task_state('ARM_PLANNING_FAILED')
            return False, 'NO_MOVE_GROUP_SERVER'

        self.set_segment_state(segment_label, 'PLANNING')
        self.get_logger().info(
            'Start arm planning: '
            f'link={link_name}, target=('
            f'{target_pose.pose.position.x:.3f}, '
            f'{target_pose.pose.position.y:.3f}, '
            f'{target_pose.pose.position.z:.3f}), '
            f'orientation_tolerance={float(self.get_parameter("orientation_tolerance").value):.3f}')
        goal = self.base_move_group_goal()
        goal.request.goal_constraints.append(self.pose_constraints(target_pose, link_name))

        send_future = self.move_group_client.send_goal_async(goal)
        if not self.wait_future(send_future, self.arm_planning_timeout()):
            self.set_task_state('ARM_PLANNING_FAILED')
            return False, 'SEND_TIMEOUT'
        handle = send_future.result()
        if not handle.accepted:
            self.set_task_state('ARM_PLANNING_FAILED')
            return False, 'GOAL_REJECTED'
        self.get_logger().info('MoveIt planning accepted: plan_only=true，等待轨迹规划结果。')

        result_future = handle.get_result_async()
        if not self.wait_future(result_future, self.arm_planning_timeout()):
            self.set_task_state('ARM_PLANNING_FAILED')
            return False, 'RESULT_TIMEOUT'
        result = result_future.result().result
        trajectory = result.planned_trajectory.joint_trajectory
        self.log_moveit_trajectory_debug(
            f'MoveIt spray-tip trajectory tip_link={link_name}',
            trajectory,
            result.error_code.val)
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.set_task_state('ARM_PLANNING_FAILED')
            return False, result.error_code.val
        ok = self.execute_checked_trajectory(
            trajectory,
            segment_label=segment_label,
            leak_in_arm=leak_in_arm)
        return ok, result.error_code.val if ok else 'EXECUTION_REJECTED'

    def move_tip_position_with_moveit(
        self,
        target_pose,
        link_name,
        leak_in_arm,
        segment_label='initial_to_target',
    ):
        if not self.move_group_client.wait_for_server(timeout_sec=12.0):
            self.get_logger().warn('MoveGroup action server 不可用。')
            self.set_task_state('ARM_PLANNING_FAILED')
            return False, 'NO_MOVE_GROUP_SERVER'

        self.set_segment_state(segment_label, 'PLANNING')
        self.get_logger().info(
            'Start arm planning: '
            f'position-only link={link_name}, target=('
            f'{target_pose.pose.position.x:.3f}, '
            f'{target_pose.pose.position.y:.3f}, '
            f'{target_pose.pose.position.z:.3f}), '
            f'tolerance={float(self.get_parameter("position_only_tolerance").value):.3f}')
        goal = self.base_move_group_goal()
        goal.request.goal_constraints.append(
            self.position_only_constraints(target_pose, link_name))

        send_future = self.move_group_client.send_goal_async(goal)
        if not self.wait_future(send_future, self.arm_planning_timeout()):
            self.set_task_state('ARM_PLANNING_FAILED')
            return False, 'SEND_TIMEOUT'
        handle = send_future.result()
        if not handle.accepted:
            self.set_task_state('ARM_PLANNING_FAILED')
            return False, 'GOAL_REJECTED'
        self.get_logger().info('MoveIt position-only planning accepted: plan_only=true。')

        result_future = handle.get_result_async()
        if not self.wait_future(result_future, self.arm_planning_timeout()):
            self.set_task_state('ARM_PLANNING_FAILED')
            return False, 'RESULT_TIMEOUT'
        result = result_future.result().result
        trajectory = result.planned_trajectory.joint_trajectory
        self.log_moveit_trajectory_debug(
            f'MoveIt spray-tip position-only trajectory tip_link={link_name}',
            trajectory,
            result.error_code.val)
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.set_task_state('ARM_PLANNING_FAILED')
            return False, result.error_code.val
        ok = self.execute_checked_trajectory(
            trajectory,
            segment_label=segment_label,
            leak_in_arm=leak_in_arm)
        return ok, result.error_code.val if ok else 'EXECUTION_REJECTED'

    def pose_constraints(self, target_pose, link_name):
        position_tolerance = float(self.get_parameter('position_tolerance').value)
        orientation_tolerance = float(self.get_parameter('orientation_tolerance').value)

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [position_tolerance]

        sphere_pose = Pose()
        sphere_pose.position = target_pose.pose.position
        sphere_pose.orientation.w = 1.0

        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(sphere_pose)

        position_constraint = PositionConstraint()
        position_constraint.header = target_pose.header
        position_constraint.link_name = link_name
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = target_pose.header
        orientation_constraint.link_name = link_name
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = orientation_tolerance
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = f'{link_name}_spray_pose_goal'
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        if self.parameter_as_bool('enable_elbow_up_constraints'):
            self.add_elbow_up_goal_constraints(constraints)
        return constraints

    def position_only_constraints(self, target_pose, link_name):
        position_tolerance = float(self.get_parameter('position_only_tolerance').value)

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [position_tolerance]

        sphere_pose = Pose()
        sphere_pose.position = target_pose.pose.position
        sphere_pose.orientation.w = 1.0

        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(sphere_pose)

        position_constraint = PositionConstraint()
        position_constraint.header = target_pose.header
        position_constraint.link_name = link_name
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = f'{link_name}_spray_position_goal'
        constraints.position_constraints.append(position_constraint)
        if self.parameter_as_bool('position_only_use_posture_constraints'):
            self.add_relaxed_spray_goal_constraints(constraints)
        return constraints

    def add_elbow_up_goal_constraints(self, constraints):
        # These are intentionally broad final-pose constraints. They do not
        # force an exact joint pose, but they steer IK away from wrist-down
        # folded solutions that put the nozzle near the ground.
        posture = {
            'joint2': (-0.75, 0.70, 0.80),
            'joint3': (-1.25, 0.85, 0.95),
            'joint4': (0.20, 1.15, 1.15),
            'joint5': (0.35, 0.95, 0.80),
        }
        for joint_name, (position, tolerance_above, tolerance_below) in posture.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(position)
            jc.tolerance_above = float(tolerance_above)
            jc.tolerance_below = float(tolerance_below)
            jc.weight = 0.5
            constraints.joint_constraints.append(jc)

    def add_relaxed_spray_goal_constraints(self, constraints):
        # Position-only fallback is allowed to relax nozzle orientation, but it
        # should still avoid extreme folded IK solutions.
        posture = {
            'joint2': (-0.75, 1.25, 1.10),
            'joint3': (-1.25, 1.55, 1.00),
            'joint4': (0.00, 1.80, 1.60),
            'joint5': (0.25, 1.60, 1.60),
        }
        for joint_name, (position, tolerance_above, tolerance_below) in posture.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(position)
            jc.tolerance_above = float(tolerance_above)
            jc.tolerance_below = float(tolerance_below)
            jc.weight = 0.35
            constraints.joint_constraints.append(jc)

    def compute_spray_joints(self, leak_in_arm):
        yaw = math.atan2(leak_in_arm[1], leak_in_arm[0])
        yaw = max(-2.6, min(2.6, yaw))
        return {
            'joint1': yaw,
            'joint2': -0.65,
            'joint3': -1.45,
            'joint4': 0.15,
            'joint5': 0.75,
            'joint6': 0.0,
        }

    def compute_direct_spray_fallback_joints(self, leak_in_arm):
        yaw = math.atan2(leak_in_arm[1], leak_in_arm[0])
        yaw = max(-2.6, min(2.6, yaw))
        reach = min(1.0, max(0.0, (math.hypot(leak_in_arm[0], leak_in_arm[1]) - 0.35) / 0.55))
        return {
            'joint1': yaw,
            'joint2': -0.55 + 0.15 * reach,
            'joint3': -1.35 + 0.25 * reach,
            'joint4': 0.35,
            'joint5': 0.55,
            'joint6': 0.0,
        }

    def segment_state_prefix(self, segment_label):
        normalized = str(segment_label).strip().lower().replace(' ', '_').replace('->', '_to_')
        normalized = normalized.replace('__', '_')
        if 'home_to_initial' in normalized:
            return 'ARM_HOME_TO_INITIAL'
        if 'initial_to_target' in normalized or 'position-only' in normalized:
            return 'ARM_INITIAL_TO_TARGET'
        return None

    def set_segment_state(self, segment_label, phase):
        prefix = self.segment_state_prefix(segment_label)
        if prefix is None:
            return
        self.set_task_state(f'{prefix}_{phase}')

    def arm_planning_timeout(self):
        return max(1.0, float(self.get_parameter('arm_planning_timeout').value))

    def arm_execution_timeout(self, planned_duration):
        configured = max(1.0, float(self.get_parameter('arm_execution_timeout').value))
        return max(configured, max(0.0, float(planned_duration)) + 5.0)

    def demo_target_duration(self, segment_label, original_duration):
        if not self.parameter_as_bool('simulation_fast_mode') or self.parameter_as_bool('real_hardware_mode'):
            return None
        prefix = self.segment_state_prefix(segment_label)
        if prefix == 'ARM_HOME_TO_INITIAL':
            return max(0.1, float(self.get_parameter('home_to_initial_duration').value))
        if prefix == 'ARM_INITIAL_TO_TARGET':
            return max(0.1, float(self.get_parameter('initial_to_target_duration').value))
        min_duration = max(
            0.05,
            float(self.get_parameter('min_arm_trajectory_duration').value))
        max_duration = max(
            min_duration,
            float(self.get_parameter('max_arm_trajectory_duration').value))
        multiplier = self.arm_speed_multiplier()
        if original_duration <= 1e-9:
            return min_duration
        return min(max(original_duration / multiplier, min_duration), max_duration)

    def arm_speed_multiplier(self):
        if self.parameter_as_bool('real_hardware_mode'):
            return 1.0
        if not self.parameter_as_bool('simulation_fast_mode'):
            return 1.0
        return max(1.0, float(self.get_parameter('arm_speed_multiplier').value))

    def moveit_velocity_scaling(self):
        if self.parameter_as_bool('simulation_fast_mode') and not self.parameter_as_bool('real_hardware_mode'):
            return 1.0
        return min(1.0, max(0.01, float(self.get_parameter('max_velocity_scaling_factor').value)))

    def moveit_acceleration_scaling(self):
        if self.parameter_as_bool('simulation_fast_mode') and not self.parameter_as_bool('real_hardware_mode'):
            return 1.0
        return min(1.0, max(0.01, float(self.get_parameter('max_acceleration_scaling_factor').value)))

    def execute_checked_trajectory(self, trajectory, segment_label, leak_in_arm=None):
        if not trajectory.points:
            self.get_logger().error(f'{segment_label}: MoveIt 返回空轨迹，拒绝执行。')
            self.set_task_state('ARM_PLANNING_FAILED')
            return False
        original_duration = self.trajectory_duration(trajectory)
        wall_start = time.monotonic()
        self.get_logger().info(
            f'wall time: start {segment_label} safety/planning-execution phase '
            f'at {wall_start:.6f}; {self.time_status_text()}')
        self.set_segment_state(segment_label, 'SAFETY_CHECK')
        safety_start = time.monotonic()
        safety_ok, safety_reason = self.trajectory_level_safety_check(
            trajectory,
            segment_label,
            leak_in_arm)
        safety_elapsed = time.monotonic() - safety_start
        self.get_logger().info(
            f'safety check耗时: segment={segment_label}, elapsed_wall={safety_elapsed:.3f}s')
        if not safety_ok:
            self.get_logger().error(
                f'Trajectory rejected before execution: segment={segment_label}, '
                f'Reason: {safety_reason}')
            self.set_task_state('ARM_TRAJECTORY_REJECTED')
            return False

        scaled_trajectory = self.retime_trajectory_for_demo(
            trajectory,
            segment_label=segment_label,
            speed_multiplier=self.arm_speed_multiplier(),
            min_duration=float(self.get_parameter('min_arm_trajectory_duration').value),
            max_duration=float(self.get_parameter('max_arm_trajectory_duration').value))
        scaled_duration = self.trajectory_duration(scaled_trajectory)
        self.get_logger().info(
            f'{segment_label} 原始轨迹时间={original_duration:.3f}s, '
            f'加速后轨迹时间={scaled_duration:.3f}s, '
            f'arm_speed_multiplier={self.arm_speed_multiplier():.2f}, '
            f'min_arm_trajectory_duration={float(self.get_parameter("min_arm_trajectory_duration").value):.3f}s, '
            f'points={len(scaled_trajectory.points)}, '
            f'first_time_from_start={self.point_time_sec(scaled_trajectory.points[0]):.3f}s, '
            f'final_time_from_start={scaled_duration:.3f}s')
        return self.send_checked_joint_trajectory(
            scaled_trajectory,
            segment_label,
            leak_in_arm,
            scaled_duration)

    def retime_trajectory_for_demo(
        self,
        trajectory,
        segment_label,
        speed_multiplier=20.0,
        min_duration=1.0,
        max_duration=5.0,
    ):
        scaled = copy.deepcopy(trajectory)
        if not scaled.points:
            return scaled
        original_duration = self.trajectory_duration(trajectory)
        min_duration = max(0.05, float(min_duration))
        max_duration = max(min_duration, float(max_duration))
        min_dt = max(
            0.001,
            float(self.get_parameter('min_trajectory_point_dt').value))
        fixed_duration = self.demo_target_duration(segment_label, original_duration)
        if fixed_duration is None:
            multiplier = max(1.0, float(speed_multiplier))
            if original_duration <= 1e-9:
                desired_duration = min_duration
            else:
                desired_duration = original_duration / multiplier
        else:
            desired_duration = fixed_duration
        desired_duration = min(max(desired_duration, min_duration), max_duration)
        effective_multiplier = (
            original_duration / desired_duration
            if original_duration > 1e-9 and desired_duration > 1e-9
            else 1.0)

        previous_time = 0.0
        point_count = len(scaled.points)
        for idx, point in enumerate(scaled.points):
            original_time = self.point_time_sec(point)
            if original_duration > 1e-9:
                scaled_time = original_time / effective_multiplier
            else:
                scaled_time = desired_duration * float(idx + 1) / float(point_count)
            min_allowed = min_dt if idx == 0 else previous_time + min_dt
            if idx == point_count - 1:
                scaled_time = max(scaled_time, desired_duration)
            scaled_time = max(scaled_time, min_allowed)
            point.time_from_start = self.duration_msg(scaled_time)
            # Position-only trajectories avoid stale MoveIt velocity fields
            # fighting the compressed demo timing in joint_trajectory_controller.
            point.velocities = []
            point.accelerations = []
            previous_time = scaled_time
        self.get_logger().info(
            'retime_trajectory_for_demo: '
            f'segment={segment_label}, original_duration={original_duration:.3f}s, '
            f'target_duration={desired_duration:.3f}s, '
            f'effective_multiplier={effective_multiplier:.2f}, '
            f'point_count={point_count}, '
            f'first_time={self.point_time_sec(scaled.points[0]):.3f}s, '
            f'last_time={self.trajectory_duration(scaled):.3f}s, '
            'velocity_acceleration_fields=cleared')
        return scaled

    def scale_trajectory_timing(self, trajectory):
        return self.retime_trajectory_for_demo(
            trajectory,
            segment_label='generic',
            speed_multiplier=self.arm_speed_multiplier(),
            min_duration=float(self.get_parameter('min_arm_trajectory_duration').value),
            max_duration=float(self.get_parameter('max_arm_trajectory_duration').value))

    @staticmethod
    def point_time_sec(point):
        return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9

    def trajectory_level_safety_check(self, trajectory, segment_label, leak_in_arm):
        if leak_in_arm is None:
            ground_z = None
        else:
            ground_z = float(leak_in_arm[2])
        self.safety_model.set_ground_min_z(ground_z)
        default = dict_to_joint_array(self.latest_joint_positions)
        margin = max(0.0, float(self.get_parameter('trajectory_safety_margin').value))
        min_tip_z = max(
            float(self.get_parameter('min_spray_tip_z').value),
            float(self.get_parameter('ground_clearance').value))
        joint_index = {name: idx for idx, name in enumerate(trajectory.joint_names)}
        missing = [name for name in self.arm_joint_names if name not in joint_index]
        if missing:
            self.get_logger().error(
                f'Trajectory safety check started: checking segment={segment_label}, '
                f'points={len(trajectory.points)}, missing joints={missing}')
            return False, f'missing joints in trajectory: {missing}'

        self.get_logger().info(
            f'Trajectory safety check started: checking segment={segment_label}, '
            f'points={len(trajectory.points)}, ground_z_in_rebot={ground_z}, '
            f'margin={margin:.3f}, min_tip_z_above_ground={min_tip_z:.3f}')

        for idx, point in enumerate(trajectory.points):
            joints = default.copy()
            for joint_idx, name in enumerate(self.arm_joint_names):
                value = float(point.positions[joint_index[name]])
                lower = float(JOINT_LIMITS[joint_idx, 0])
                upper = float(JOINT_LIMITS[joint_idx, 1])
                if value < lower - 1e-6 or value > upper + 1e-6:
                    reason = (
                        f'point {idx}: joint limit violation {name}={value:.3f}, '
                        f'limit=[{lower:.3f}, {upper:.3f}]')
                    self.log_trajectory_safety_failure(segment_label, idx, joints, reason)
                    return False, reason
                joints[joint_idx] = value

            report = self.safety_model.distance_report(joints)
            if report.collision:
                reason = (
                    f'point {idx}: collision pairs={report.collision_pairs}, '
                    f'min_obstacle={report.min_obstacle_distance:.3f}, '
                    f'min_self={report.min_self_distance:.3f}')
                self.log_trajectory_safety_failure(
                    segment_label,
                    idx,
                    joints,
                    reason,
                    report=report)
                return False, reason
            if report.min_distance < margin:
                reason = (
                    f'point {idx}: safety margin violation, '
                    f'min_distance={report.min_distance:.3f}, margin={margin:.3f}')
                self.log_trajectory_safety_failure(
                    segment_label,
                    idx,
                    joints,
                    reason,
                    report=report)
                return False, reason

            positions = self.safety_model.forward_kinematics(joints)
            if ground_z is not None:
                required_tip_z = ground_z + min_tip_z
                tip_z = float(positions['spray_tip_link'][2])
                nozzle_z = float(positions['spray_nozzle_link'][2])
                if tip_z < required_tip_z:
                    reason = (
                        f'point {idx}: spray_tip_link below min height, '
                        f'tip_z={tip_z:.3f}, required={required_tip_z:.3f}')
                    self.log_trajectory_safety_failure(
                        segment_label,
                        idx,
                        joints,
                        reason,
                        positions=positions,
                        report=report)
                    return False, reason
                if nozzle_z < ground_z + float(self.get_parameter('ground_clearance').value):
                    reason = (
                        f'point {idx}: spray_nozzle_link too close to ground, '
                        f'nozzle_z={nozzle_z:.3f}, ground_z={ground_z:.3f}')
                    self.log_trajectory_safety_failure(
                        segment_label,
                        idx,
                        joints,
                        reason,
                        positions=positions,
                        report=report)
                    return False, reason

        self.get_logger().info(
            f'trajectory-level safety check passed: segment={segment_label}, '
            f'points={len(trajectory.points)}, ground_z_in_rebot={ground_z}, '
            f'margin={margin:.3f}')
        return True, 'passed'

    def log_trajectory_safety_failure(
        self,
        segment_label,
        point_index,
        joints,
        reason,
        positions=None,
        report=None,
    ):
        if positions is None:
            try:
                positions = self.safety_model.forward_kinematics(joints)
            except Exception:
                positions = {}
        tip = positions.get('spray_tip_link')
        nozzle = positions.get('spray_nozzle_link')
        tip_z = 'unavailable' if tip is None else f'{float(tip[2]):.3f}'
        nozzle_z = 'unavailable' if nozzle is None else f'{float(nozzle[2]):.3f}'
        collision_pairs = 'none' if report is None else report.collision_pairs
        min_obstacle = 'unavailable' if report is None else f'{report.min_obstacle_distance:.3f}'
        min_self = 'unavailable' if report is None else f'{report.min_self_distance:.3f}'
        self.get_logger().error(
            'Trajectory rejected before execution detail: '
            f'segment={segment_label}, failed_point_index={point_index}, '
            f'joint positions={self.format_joint_values(joints)}, '
            f'spray_tip_link z={tip_z}, spray_nozzle_link z={nozzle_z}, '
            f'collision object name/pairs={collision_pairs}, '
            f'min_obstacle_distance={min_obstacle}, min_self_distance={min_self}, '
            f'failure reason={reason}')

    def send_checked_joint_trajectory(self, trajectory, segment_label, leak_in_arm, duration_sec):
        if not self.trajectory_client.wait_for_server(timeout_sec=8.0):
            self.get_logger().error('找不到 rebotarm_controller/follow_joint_trajectory。')
            return False

        goal = FollowJointTrajectory.Goal()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        goal.trajectory = trajectory
        self.set_segment_state(segment_label, 'EXECUTING')
        self.log_controller_trajectory_send_details(trajectory, segment_label)
        sent_wall = time.monotonic()
        self.get_logger().info(
            f'wall time: trajectory sent for {segment_label} at {sent_wall:.6f}; '
            f'{self.time_status_text()}')
        self.get_logger().info(f'trajectory execution started: segment={segment_label}')
        send_future = self.trajectory_client.send_goal_async(goal)
        if not self.wait_future(send_future, 8.0):
            self.get_logger().error(f'{segment_label}: 发送 FollowJointTrajectory goal 超时。')
            self.set_task_state('ARM_EXECUTION_FAILED')
            return False
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error(f'{segment_label}: FollowJointTrajectory goal 被拒绝。')
            self.set_task_state('ARM_EXECUTION_FAILED')
            return False
        accepted_wall = time.monotonic()
        self.get_logger().info(
            f'FollowJointTrajectory goal accepted: segment={segment_label}; '
            f'wall time: controller accepted at {accepted_wall:.6f}; '
            f'send_to_accept_elapsed={accepted_wall - sent_wall:.3f}s; '
            f'{self.time_status_text()}')

        result_future = handle.get_result_async()
        start = time.monotonic()
        check_period = max(0.01, float(self.get_parameter('runtime_safety_check_period_sec').value))
        next_check = start
        timeout = self.arm_execution_timeout(duration_sec)
        while rclpy.ok() and not self.shutdown_event.is_set() and not result_future.done():
            now = time.monotonic()
            if now - start > timeout:
                self.get_logger().error(f'{segment_label}: FollowJointTrajectory 执行等待超时。')
                self.set_task_state('ARM_EXECUTION_FAILED')
                return False
            if now >= next_check:
                ok, reason = self.runtime_arm_safety_check(leak_in_arm)
                if not ok:
                    self.get_logger().error(
                        'Emergency stop arm trajectory due to collision risk: '
                        f'segment={segment_label}, reason={reason}')
                    cancel_future = handle.cancel_goal_async()
                    self.wait_future(cancel_future, 1.0)
                    self.set_task_state('ARM_EXECUTION_FAILED')
                    return False
                next_check = now + check_period
            self.sleep_for(0.01)

        if self.shutdown_event.is_set():
            return False
        result = result_future.result().result
        ok = int(result.error_code) == 0
        self.get_logger().info(
            f'trajectory execution done: segment={segment_label}, '
            f'success={ok}, error_code={result.error_code}, '
            f'error_string="{result.error_string}", '
            f'wall_elapsed_since_sent={time.monotonic() - sent_wall:.3f}s, '
            f'{self.time_status_text()}')
        if ok:
            self.set_segment_state(segment_label, 'DONE')
        else:
            self.set_task_state('ARM_EXECUTION_FAILED')
        return ok

    def log_controller_trajectory_send_details(self, trajectory, segment_label):
        first = trajectory.points[0] if trajectory.points else None
        last = trajectory.points[-1] if trajectory.points else None
        first_time = 0.0 if first is None else self.point_time_sec(first)
        last_time = 0.0 if last is None else self.point_time_sec(last)
        first_positions = [] if first is None else first.positions
        last_positions = [] if last is None else last.positions
        self.get_logger().info(
            'Sending trajectory to controller: '
            f'segment={segment_label}, '
            f'controller action name={self.get_parameter("arm_controller_action_name").value}, '
            f'trajectory joint_names={list(trajectory.joint_names)}, '
            f'trajectory point count={len(trajectory.points)}, '
            f'trajectory first time_from_start={first_time:.3f}s, '
            f'trajectory last time_from_start={last_time:.3f}s, '
            f'trajectory first positions={self.format_joint_values(first_positions)}, '
            f'trajectory last positions={self.format_joint_values(last_positions)}')

    def runtime_arm_safety_check(self, leak_in_arm):
        if not all(name in self.latest_joint_positions for name in self.arm_joint_names):
            return False, 'missing joint states during execution'
        joints = dict_to_joint_array(self.latest_joint_positions)
        if leak_in_arm is None:
            ground_z = None
        else:
            ground_z = float(leak_in_arm[2])
        self.safety_model.set_ground_min_z(ground_z)
        report = self.safety_model.distance_report(joints)
        if report.collision:
            return False, f'collision pairs={report.collision_pairs}'
        margin = max(0.0, float(self.get_parameter('trajectory_safety_margin').value))
        if report.min_distance < margin:
            return False, f'min_distance={report.min_distance:.3f} < margin={margin:.3f}'
        if ground_z is not None:
            positions = self.safety_model.forward_kinematics(joints)
            min_tip_z = max(
                float(self.get_parameter('min_spray_tip_z').value),
                float(self.get_parameter('ground_clearance').value))
            required_tip_z = ground_z + min_tip_z
            if float(positions['spray_tip_link'][2]) < required_tip_z:
                return False, (
                    f'spray_tip_link below min height: '
                    f'{positions["spray_tip_link"][2]:.3f} < {required_tip_z:.3f}')
        return True, 'safe'

    def move_arm_with_moveit(self, joint_targets, segment_label='joint target', leak_in_arm=None):
        if not self.move_group_client.wait_for_server(timeout_sec=12.0):
            self.get_logger().warn('MoveGroup action server 不可用。')
            self.set_task_state('ARM_PLANNING_FAILED')
            return False

        self.set_segment_state(segment_label, 'PLANNING')
        self.get_logger().info(
            'Start arm planning: joint target '
            + ', '.join(f'{name}={value:.3f}' for name, value in joint_targets.items()))
        goal = self.base_move_group_goal()

        constraints = Constraints()
        constraints.name = 'spray_or_home_joint_goal'
        for joint_name, position in joint_targets.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(position)
            jc.tolerance_above = 0.03
            jc.tolerance_below = 0.03
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)

        send_future = self.move_group_client.send_goal_async(goal)
        if not self.wait_future(send_future, self.arm_planning_timeout()):
            self.set_task_state('ARM_PLANNING_FAILED')
            return False
        handle = send_future.result()
        if not handle.accepted:
            self.set_task_state('ARM_PLANNING_FAILED')
            return False
        self.get_logger().info('MoveIt joint planning accepted: plan_only=true。')

        result_future = handle.get_result_async()
        if not self.wait_future(result_future, self.arm_planning_timeout()):
            self.set_task_state('ARM_PLANNING_FAILED')
            return False
        result = result_future.result().result
        self.log_moveit_trajectory_debug(
            'MoveIt joint trajectory',
            result.planned_trajectory.joint_trajectory,
            result.error_code.val)
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.set_task_state('ARM_PLANNING_FAILED')
            return False
        return self.execute_checked_trajectory(
            result.planned_trajectory.joint_trajectory,
            segment_label=segment_label,
            leak_in_arm=leak_in_arm)

    def base_move_group_goal(self):
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.pipeline_id = 'ompl'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 8.0
        goal.request.max_velocity_scaling_factor = self.moveit_velocity_scaling()
        goal.request.max_acceleration_scaling_factor = self.moveit_acceleration_scaling()
        goal.request.start_state.is_diff = True
        goal.request.workspace_parameters.header.frame_id = 'base_link'
        goal.request.workspace_parameters.min_corner.x = -15.0
        goal.request.workspace_parameters.min_corner.y = -15.0
        goal.request.workspace_parameters.min_corner.z = -0.25
        goal.request.workspace_parameters.max_corner.x = 15.0
        goal.request.workspace_parameters.max_corner.y = 15.0
        goal.request.workspace_parameters.max_corner.z = 3.0
        goal.planning_options.plan_only = True
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.planning_scene_diff.world.collision_objects = (
            self.planning_scene_collision_objects())
        return goal

    def planning_scene_collision_objects(self):
        ground_size = max(1.0, float(self.get_parameter('ground_collision_size').value))
        ground_thickness = max(
            0.001,
            float(self.get_parameter('ground_collision_thickness').value))
        ground_top_z = float(self.get_parameter('base_link_ground_z').value)
        ground_z = ground_top_z - 0.5 * ground_thickness
        objects = [
            self.box_collision_object(
                'ground',
                'base_link',
                (ground_size, ground_size, ground_thickness),
                (0.0, 0.0, ground_z)),
            self.box_collision_object(
                'car_body',
                'base_link',
                (0.62, 0.38, 0.08),
                (0.0, 0.0, ground_top_z + 0.04)),
            self.box_collision_object(
                'front_left_wheel',
                'base_link',
                (0.14, 0.12, 0.22),
                (0.20, 0.20, ground_top_z + 0.10)),
            self.box_collision_object(
                'front_right_wheel',
                'base_link',
                (0.14, 0.12, 0.22),
                (0.20, -0.20, ground_top_z + 0.10)),
            self.box_collision_object(
                'rear_left_wheel',
                'base_link',
                (0.14, 0.12, 0.22),
                (-0.20, 0.20, ground_top_z + 0.10)),
            self.box_collision_object(
                'rear_right_wheel',
                'base_link',
                (0.14, 0.12, 0.22),
                (-0.20, -0.20, ground_top_z + 0.10)),
            self.box_collision_object(
                'arm_mount_base',
                'base_link',
                (0.24, 0.22, 0.05),
                (0.0, 0.0, ground_top_z + 0.025)),
            self.cylinder_collision_object(
                'mid360_lidar',
                'base_link',
                height=0.12,
                radius=0.07,
                xyz=(0.24, 0.0, ground_top_z + 0.115)),
        ]
        return objects

    @staticmethod
    def box_collision_object(object_id, frame_id, size, xyz):
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]
        return GasLeakMobileManipulatorTask.collision_object(
            object_id, frame_id, primitive, xyz)

    @staticmethod
    def cylinder_collision_object(object_id, frame_id, height, radius, xyz):
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [float(height), float(radius)]
        return GasLeakMobileManipulatorTask.collision_object(
            object_id, frame_id, primitive, xyz)

    @staticmethod
    def collision_object(object_id, frame_id, primitive, xyz):
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = object_id
        obj.operation = CollisionObject.ADD
        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        pose.orientation.w = 1.0
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        return obj

    def send_joint_trajectory(self, joint_targets, duration_sec=None):
        if not self.trajectory_client.wait_for_server(timeout_sec=8.0):
            self.get_logger().error('找不到 rebotarm_controller/follow_joint_trajectory。')
            return False

        waypoint_count = max(2, int(self.get_parameter('fallback_trajectory_waypoints').value))
        joint_names = list(joint_targets.keys())
        target_positions = [float(joint_targets[name]) for name in joint_names]
        start_positions = [
            float(self.latest_joint_positions.get(name, target_positions[idx]))
            for idx, name in enumerate(joint_names)
        ]
        joint_delta = [
            target - start
            for start, target in zip(start_positions, target_positions)
        ]
        if duration_sec is None:
            duration_sec = self.compute_fallback_trajectory_duration(joint_delta)
        else:
            duration_sec = max(
                float(duration_sec),
                float(self.get_parameter('fallback_trajectory_min_duration_sec').value))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        for idx in range(1, waypoint_count + 1):
            ratio = float(idx) / float(waypoint_count)
            smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
            point = JointTrajectoryPoint()
            point.positions = [
                start + (target - start) * smooth_ratio
                for start, target in zip(start_positions, target_positions)
            ]
            point.time_from_start = self.duration_msg(duration_sec * ratio)
            goal.trajectory.points.append(point)

        self.log_direct_trajectory_debug(
            'Direct FollowJointTrajectory fallback',
            joint_names,
            start_positions,
            target_positions,
            joint_delta,
            goal.trajectory,
            duration_sec)

        send_future = self.trajectory_client.send_goal_async(goal)
        if not self.wait_future(send_future, 8.0):
            return False
        handle = send_future.result()
        if not handle.accepted:
            return False
        self.set_task_state('ARM_EXECUTION')
        self.get_logger().info('Executing arm trajectory: direct FollowJointTrajectory goal accepted。')
        result_future = handle.get_result_async()
        if not self.wait_future(result_future, duration_sec + 8.0):
            self.get_logger().error('Direct FollowJointTrajectory 执行等待超时。')
            return False
        result = result_future.result().result
        ok = int(result.error_code) == 0
        self.get_logger().info(
            f'Arm trajectory execution done: direct FollowJointTrajectory '
            f'error_code={result.error_code}, error_string="{result.error_string}"')
        return ok

    def compute_fallback_trajectory_duration(self, joint_delta):
        max_delta = max((abs(float(delta)) for delta in joint_delta), default=0.0)
        nominal_speed = max(
            1e-3,
            float(self.get_parameter('fallback_trajectory_nominal_joint_speed').value))
        min_duration = max(
            0.05,
            float(self.get_parameter('fallback_trajectory_min_duration_sec').value))
        max_duration = max(
            min_duration,
            float(self.get_parameter('fallback_trajectory_max_duration_sec').value))
        adaptive = max_delta / nominal_speed if max_delta > 1e-6 else min_duration
        return min(max(adaptive, min_duration), max_duration)

    def log_moveit_trajectory_debug(self, label, trajectory, error_code):
        point_count = len(trajectory.points)
        duration = self.trajectory_duration(trajectory)
        start = []
        target = []
        delta = []
        if trajectory.points:
            start = list(trajectory.points[0].positions)
            target = list(trajectory.points[-1].positions)
            delta = [
                float(target_pos) - float(start_pos)
                for start_pos, target_pos in zip(start, target)
            ]
        collision_status = 'passed' if error_code == MoveItErrorCodes.SUCCESS else 'failed_or_no_solution'
        self.get_logger().info(
            f'{label}: current={self.format_joint_values(start)}, '
            f'ik_solution_joint1_to_joint6={self.format_joint_values(target)}, '
            f'delta={self.format_joint_values(delta)}, '
            f'points={point_count}, duration={duration:.2f}s, '
            f'vel_scale={self.moveit_velocity_scaling():.2f}, '
            f'acc_scale={self.moveit_acceleration_scaling():.2f}, '
            f'arm_speed_multiplier={self.arm_speed_multiplier():.2f}, '
            f'joint_limits={self.joint_limit_text()}, '
            f'gazebo_real_time_factor={self.real_time_factor_text()}, '
            f'moveit_collision_check={collision_status}, '
            f'error_code={error_code}')

    def log_direct_trajectory_debug(
        self,
        label,
        joint_names,
        start_positions,
        target_positions,
        joint_delta,
        trajectory,
        duration_sec,
    ):
        self.get_logger().info(
            f'{label}: joints={joint_names}, '
            f'current={self.format_joint_values(start_positions)}, '
            f'target={self.format_joint_values(target_positions)}, '
            f'delta={self.format_joint_values(joint_delta)}, '
            f'points={len(trajectory.points)}, duration={duration_sec:.2f}s, '
            f'joint_limits={self.joint_limit_text()}, '
            f'gazebo_real_time_factor={self.real_time_factor_text()}')

    @staticmethod
    def trajectory_duration(trajectory):
        if not trajectory.points:
            return 0.0
        last_time = trajectory.points[-1].time_from_start
        return float(last_time.sec) + float(last_time.nanosec) * 1e-9

    @staticmethod
    def format_joint_values(values):
        if not values:
            return '[]'
        return '[' + ', '.join(f'{float(value):+.3f}' for value in values) + ']'

    def joint_limit_text(self):
        velocity = float(self.get_parameter('joint_velocity_limit').value)
        acceleration = float(self.get_parameter('joint_acceleration_limit').value)
        return (
            '{joint1..joint6: '
            f'max_velocity={velocity:.2f}rad/s, '
            f'max_acceleration={acceleration:.2f}rad/s^2'
            '}')

    def real_time_factor_text(self):
        if self.latest_real_time_factor is None:
            return 'unavailable'
        return f'{self.latest_real_time_factor:.2f}'

    def time_status_text(self):
        use_sim_time = (
            self.get_parameter('use_sim_time').value
            if self.has_parameter('use_sim_time')
            else False)
        now_msg = self.get_clock().now().to_msg()
        return (
            f'use_sim_time={use_sim_time}, '
            f'ros_time={now_msg.sec}.{now_msg.nanosec:09d}, '
            f'gazebo_real_time_factor={self.real_time_factor_text()}')

    def start_spray(self):
        self.spray_done = False
        msg = Bool()
        msg.data = True
        for _ in range(5):
            if self.shutdown_event.is_set():
                return
            self.spray_start_pub.publish(msg)
            self.sleep_for(0.05)

    def stop_spray(self):
        msg = Bool()
        msg.data = False
        self.spray_start_pub.publish(msg)

    def wait_for_neutralization(self):
        duration_sec = float(self.get_parameter('spray_duration_sec').value)
        done_strength = float(self.get_parameter('done_source_strength').value)
        start = time.monotonic()
        while rclpy.ok():
            if self.shutdown_event.is_set():
                return
            if self.source_strength <= done_strength or self.spray_done:
                return
            if time.monotonic() - start > duration_sec + 3.0:
                return
            self.sleep_for(0.1)

    def sleep_for(self, duration_sec):
        end_time = time.monotonic() + max(0.0, float(duration_sec))
        while rclpy.ok() and not self.shutdown_event.is_set():
            remaining = end_time - time.monotonic()
            if remaining <= 0.0:
                return True
            time.sleep(min(0.05, remaining))
        return False

    @staticmethod
    def rotate_vector(vector, quat):
        qx, qy, qz, qw = quat
        vx, vy, vz = vector
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return [
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        ]

    @staticmethod
    def cross(a, b):
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]

    @staticmethod
    def norm(v):
        return math.sqrt(sum(float(value) * float(value) for value in v))

    def normalize(self, v):
        length = self.norm(v)
        if length < 1e-9:
            return [1.0, 0.0, 0.0]
        return [float(value) / length for value in v]

    def quaternion_with_x_axis(self, x_axis, roll):
        x_axis = self.normalize(x_axis)
        up = [0.0, 0.0, 1.0]
        if self.norm(self.cross(up, x_axis)) < 1e-6:
            up = [0.0, 1.0, 0.0]
        y_axis = self.normalize(self.cross(up, x_axis))
        z_axis = self.normalize(self.cross(x_axis, y_axis))

        cos_r = math.cos(roll)
        sin_r = math.sin(roll)
        y_roll = [y_axis[i] * cos_r + z_axis[i] * sin_r for i in range(3)]
        z_roll = [-y_axis[i] * sin_r + z_axis[i] * cos_r for i in range(3)]

        matrix = [
            [x_axis[0], y_roll[0], z_roll[0]],
            [x_axis[1], y_roll[1], z_roll[1]],
            [x_axis[2], y_roll[2], z_roll[2]],
        ]
        return self.quaternion_from_basis_matrix(matrix)

    @staticmethod
    def quaternion_from_basis_matrix(matrix):
        trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            quat = [
                (matrix[2][1] - matrix[1][2]) / s,
                (matrix[0][2] - matrix[2][0]) / s,
                (matrix[1][0] - matrix[0][1]) / s,
                0.25 * s,
            ]
        elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
            s = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
            quat = [
                0.25 * s,
                (matrix[0][1] + matrix[1][0]) / s,
                (matrix[0][2] + matrix[2][0]) / s,
                (matrix[2][1] - matrix[1][2]) / s,
            ]
        elif matrix[1][1] > matrix[2][2]:
            s = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
            quat = [
                (matrix[0][1] + matrix[1][0]) / s,
                0.25 * s,
                (matrix[1][2] + matrix[2][1]) / s,
                (matrix[0][2] - matrix[2][0]) / s,
            ]
        else:
            s = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
            quat = [
                (matrix[0][2] + matrix[2][0]) / s,
                (matrix[1][2] + matrix[2][1]) / s,
                0.25 * s,
                (matrix[1][0] - matrix[0][1]) / s,
            ]
        return GasLeakMobileManipulatorTask.normalize_quaternion(quat)

    @staticmethod
    def normalize_quaternion(quat):
        length = math.sqrt(sum(float(value) * float(value) for value in quat))
        if length < 1e-9:
            return [0.0, 0.0, 0.0, 1.0]
        return [float(value) / length for value in quat]

    @staticmethod
    def unique_values(values):
        unique = []
        for value in values:
            rounded = round(float(value), 4)
            if rounded not in unique:
                unique.append(rounded)
        return unique

    @staticmethod
    def unique_links(values):
        unique = []
        for value in values:
            if value and value not in unique:
                unique.append(value)
        return unique

    def wait_future(self, future, timeout_sec):
        start = time.monotonic()
        while (
            rclpy.ok()
            and not self.shutdown_event.is_set()
            and not future.done()
        ):
            if time.monotonic() - start > timeout_sec:
                return False
            self.sleep_for(0.05)
        return future.done()


def main(args=None):
    rclpy.init(args=args)
    node = GasLeakMobileManipulatorTask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.request_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

import math
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint


class RebotSprayTask(Node):
    def __init__(self):
        super().__init__('rebot_spray_task')
        self.declare_parameter('move_group_action_name', 'move_action')
        self.declare_parameter('arm_controller_action_name', 'rebotarm_controller/follow_joint_trajectory')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('start_delay_sec', 4.0)
        self.declare_parameter('spray_duration_sec', 4.0)
        self.declare_parameter('leak_standoff', 0.12)
        self.declare_parameter('position_tolerance', 0.015)
        self.declare_parameter('orientation_tolerance', 0.18)
        self.declare_parameter('tip_link', 'spray_tip_link')

        self.latest_leak_pose = None
        self.spray_done = False
        self.started = False

        self.create_subscription(PoseStamped, 'leak_pose', self.on_leak_pose, 10)
        self.create_subscription(Bool, 'spray/done', self.on_spray_done, 10)
        self.spray_pub = self.create_publisher(Bool, 'spray/start', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, 'spray/target_pose', 10)

        self.move_group_client = ActionClient(
            self, MoveGroup, str(self.get_parameter('move_group_action_name').value))
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory,
            str(self.get_parameter('arm_controller_action_name').value))

        self.timer = self.create_timer(0.5, self.maybe_start)

    def on_leak_pose(self, msg):
        self.latest_leak_pose = msg

    def on_spray_done(self, msg):
        if msg.data:
            self.spray_done = True

    def maybe_start(self):
        if self.started or not bool(self.get_parameter('auto_start').value):
            return
        self.started = True
        thread = threading.Thread(target=self.run_task, daemon=True)
        thread.start()

    def run_task(self):
        delay = float(self.get_parameter('start_delay_sec').value)
        self.get_logger().info(f'等待 MoveIt/controller 启动稳定: {delay:.1f}s')
        time.sleep(delay)

        leak_pose = self.wait_for_leak_pose(timeout_sec=20.0)
        if leak_pose is None:
            self.get_logger().error('没有收到 /leak_pose，喷药任务停止。')
            return

        self.get_logger().info('机械臂先回 home。')
        home = self.home_joints()
        if not self.move_joint_goal_with_moveit(home):
            self.get_logger().warn('MoveIt home 规划失败，尝试直接发送 home 轨迹。')
            self.send_joint_trajectory(home, duration_sec=3.0)

        if not self.move_tip_to_leak_with_ik(leak_pose):
            self.get_logger().error('所有 spray_tip_link 位姿 IK 尝试均失败，停止喷药任务。')
            return

        self.get_logger().info('spray_tip_link 已对准泄漏点，发布 /spray/start。')
        self.publish_spray(True)
        self.wait_for_spray_done()
        self.publish_spray(False)

        self.get_logger().info('喷洒完成，机械臂回 home。')
        if not self.move_joint_goal_with_moveit(home):
            self.send_joint_trajectory(home, duration_sec=3.0)
        self.get_logger().info('Leak neutralization completed')

    def wait_for_leak_pose(self, timeout_sec):
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < timeout_sec:
            if self.latest_leak_pose is not None:
                return self.latest_leak_pose
            time.sleep(0.05)
        return None

    def wait_for_spray_done(self):
        duration = float(self.get_parameter('spray_duration_sec').value)
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < duration + 3.0:
            if self.spray_done:
                return
            time.sleep(0.05)

    def publish_spray(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)
        for _ in range(5):
            self.spray_pub.publish(msg)
            time.sleep(0.05)

    @staticmethod
    def home_joints():
        return {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0,
            'joint4': 0.0,
            'joint5': 0.0,
            'joint6': 0.0,
        }

    def move_tip_to_leak_with_ik(self, leak_pose):
        configured_standoff = float(self.get_parameter('leak_standoff').value)
        standoff_candidates = self.unique_values([
            configured_standoff, 0.10, 0.12, 0.15,
        ])
        roll_candidates = [0.0, math.pi / 2.0, -math.pi / 2.0, math.pi]

        for standoff in standoff_candidates:
            for roll in roll_candidates:
                target_pose = self.compute_spray_tip_pose(leak_pose, standoff, roll)
                self.target_pose_pub.publish(target_pose)
                self.get_logger().info(
                    '尝试 spray_tip_link IK: '
                    f'standoff={standoff:.3f}m, roll={roll:.2f}rad, '
                    f'pose=({target_pose.pose.position.x:.3f}, '
                    f'{target_pose.pose.position.y:.3f}, '
                    f'{target_pose.pose.position.z:.3f})')
                ok, code = self.move_tip_pose_with_moveit(target_pose)
                if ok:
                    return True
                self.get_logger().warn(
                    'spray_tip_link 位姿 IK/规划失败: '
                    f'standoff={standoff:.3f}, roll={roll:.2f}, '
                    f'MoveItErrorCode={code}')
        return False

    @staticmethod
    def unique_values(values):
        unique = []
        for value in values:
            rounded = round(float(value), 4)
            if rounded not in unique:
                unique.append(rounded)
        return unique

    def compute_spray_tip_pose(self, leak_pose, standoff, roll):
        frame_id = leak_pose.header.frame_id or 'world'
        leak = [
            float(leak_pose.pose.position.x),
            float(leak_pose.pose.position.y),
            float(leak_pose.pose.position.z),
        ]

        horizontal = [leak[0], leak[1], 0.0]
        if self.norm(horizontal) < 1e-6:
            horizontal = [1.0, 0.0, 0.0]
        spray_axis = self.normalize(horizontal)

        tip = [
            leak[0] - spray_axis[0] * standoff,
            leak[1] - spray_axis[1] * standoff,
            leak[2] - spray_axis[2] * standoff,
        ]

        quat = self.quaternion_with_x_axis(spray_axis, roll)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        pose.pose.position.x = tip[0]
        pose.pose.position.y = tip[1]
        pose.pose.position.z = tip[2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def quaternion_with_x_axis(self, x_axis, roll):
        x_axis = self.normalize(x_axis)
        up = [0.0, 0.0, 1.0]
        if self.norm(self.cross(up, x_axis)) < 1e-6:
            up = [0.0, 1.0, 0.0]
        y_axis = self.normalize(self.cross(up, x_axis))
        z_axis = self.normalize(self.cross(x_axis, y_axis))

        cos_r = math.cos(roll)
        sin_r = math.sin(roll)
        y_roll = [
            y_axis[i] * cos_r + z_axis[i] * sin_r
            for i in range(3)
        ]
        z_roll = [
            -y_axis[i] * sin_r + z_axis[i] * cos_r
            for i in range(3)
        ]

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
        return RebotSprayTask.normalize_quaternion(quat)

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

    @staticmethod
    def normalize_quaternion(quat):
        length = math.sqrt(sum(float(value) * float(value) for value in quat))
        if length < 1e-9:
            return [0.0, 0.0, 0.0, 1.0]
        return [float(value) / length for value in quat]

    def move_tip_pose_with_moveit(self, target_pose):
        if not self.move_group_client.wait_for_server(timeout_sec=12.0):
            self.get_logger().warn('找不到 MoveGroup action server。')
            return False, 'NO_MOVE_GROUP_SERVER'

        goal = self.base_move_group_goal()
        goal.request.goal_constraints.append(self.pose_constraints(target_pose))

        send_future = self.move_group_client.send_goal_async(goal)
        if not self.wait_future(send_future, 12.0):
            return False, 'SEND_TIMEOUT'
        handle = send_future.result()
        if not handle.accepted:
            return False, 'GOAL_REJECTED'

        result_future = handle.get_result_async()
        if not self.wait_future(result_future, 30.0):
            return False, 'RESULT_TIMEOUT'
        result = result_future.result().result
        return result.error_code.val == MoveItErrorCodes.SUCCESS, result.error_code.val

    def pose_constraints(self, target_pose):
        tip_link = str(self.get_parameter('tip_link').value)
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
        position_constraint.link_name = tip_link
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = target_pose.header
        orientation_constraint.link_name = tip_link
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = orientation_tolerance
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = 'spray_tip_pose_goal'
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        return constraints

    def move_joint_goal_with_moveit(self, joint_targets):
        if not self.move_group_client.wait_for_server(timeout_sec=12.0):
            self.get_logger().warn('找不到 MoveGroup action server。')
            return False

        goal = self.base_move_group_goal()
        constraints = Constraints()
        constraints.name = 'rebot_joint_goal'
        for joint_name, position in joint_targets.items():
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(position)
            joint_constraint.tolerance_above = 0.03
            joint_constraint.tolerance_below = 0.03
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        goal.request.goal_constraints.append(constraints)

        send_future = self.move_group_client.send_goal_async(goal)
        if not self.wait_future(send_future, 12.0):
            return False
        handle = send_future.result()
        if not handle.accepted:
            return False

        result_future = handle.get_result_async()
        if not self.wait_future(result_future, 25.0):
            return False
        result = result_future.result().result
        return result.error_code.val == MoveItErrorCodes.SUCCESS

    @staticmethod
    def base_move_group_goal():
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.pipeline_id = 'ompl'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 8.0
        goal.request.max_velocity_scaling_factor = 0.20
        goal.request.max_acceleration_scaling_factor = 0.20
        goal.request.start_state.is_diff = True
        goal.request.workspace_parameters.header.frame_id = 'world'
        goal.request.workspace_parameters.min_corner.x = -1.0
        goal.request.workspace_parameters.min_corner.y = -1.0
        goal.request.workspace_parameters.min_corner.z = -0.2
        goal.request.workspace_parameters.max_corner.x = 1.0
        goal.request.workspace_parameters.max_corner.y = 1.0
        goal.request.workspace_parameters.max_corner.z = 1.0
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def send_joint_trajectory(self, joint_targets, duration_sec=3.0):
        if not self.trajectory_client.wait_for_server(timeout_sec=8.0):
            self.get_logger().error('找不到 rebotarm_controller/follow_joint_trajectory。')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_targets.keys())
        point = JointTrajectoryPoint()
        point.positions = [float(joint_targets[name]) for name in goal.trajectory.joint_names]
        point.time_from_start = DurationMsg(sec=int(duration_sec))
        goal.trajectory.points = [point]

        send_future = self.trajectory_client.send_goal_async(goal)
        if not self.wait_future(send_future, 8.0):
            return False
        handle = send_future.result()
        if not handle.accepted:
            return False
        result_future = handle.get_result_async()
        return self.wait_future(result_future, duration_sec + 8.0)

    @staticmethod
    def wait_future(future, timeout_sec):
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if time.monotonic() - start > timeout_sec:
                return False
            time.sleep(0.05)
        return future.done()


def main(args=None):
    rclpy.init(args=args)
    node = RebotSprayTask()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

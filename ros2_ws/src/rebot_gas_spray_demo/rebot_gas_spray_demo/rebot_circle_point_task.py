import math
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray


class RebotCirclePointTask(Node):
    def __init__(self):
        super().__init__('rebot_circle_point_task')
        self.declare_parameter('move_group_action_name', 'move_action')
        self.declare_parameter('arm_controller_action_name', 'rebotarm_controller/follow_joint_trajectory')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('start_delay_sec', 4.0)
        self.declare_parameter('frame_id', 'rebot_base_link')
        self.declare_parameter('center', [0.35, 0.0, 0.05])
        self.declare_parameter('center_x', 0.35)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('center_z', 0.05)
        self.declare_parameter('radius', 0.25)
        self.declare_parameter('num_points', 6)
        self.declare_parameter('loop_count', 0)
        self.declare_parameter('hold_sec', 1.0)
        self.declare_parameter('tip_link', 'spray_tip_link')
        self.declare_parameter('position_tolerance', 0.025)
        self.declare_parameter('home_duration_sec', 3.0)
        self.declare_parameter('velocity_scaling', 0.25)
        self.declare_parameter('acceleration_scaling', 0.25)

        self.started = False
        self.status_text = 'Waiting to start'
        self.current_index = -1
        self.current_target_pose = None

        self.move_group_client = ActionClient(
            self, MoveGroup, str(self.get_parameter('move_group_action_name').value))
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory,
            str(self.get_parameter('arm_controller_action_name').value))

        self.marker_pub = self.create_publisher(MarkerArray, 'circle_points/markers', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, 'circle_points/current_target_pose', 10)
        self.status_pub = self.create_publisher(String, 'circle_points/status', 10)

        self.timer = self.create_timer(0.2, self.publish_visualization)
        self.start_timer = self.create_timer(0.5, self.maybe_start)

    def maybe_start(self):
        if self.started or not bool(self.get_parameter('auto_start').value):
            return
        self.started = True
        thread = threading.Thread(target=self.run_task, daemon=True)
        thread.start()

    def run_task(self):
        delay = float(self.get_parameter('start_delay_sec').value)
        self.set_status(f'Waiting for MoveIt/controller: {delay:.1f}s')
        time.sleep(delay)

        self.get_logger().info('圆形点位任务启动，机械臂先回 home。')
        self.move_home()

        loop_count = int(self.get_parameter('loop_count').value)
        completed_loops = 0
        while rclpy.ok() and (loop_count == 0 or completed_loops < loop_count):
            points = self.generate_points()
            for index, point in enumerate(points):
                if not rclpy.ok():
                    break
                self.current_index = index
                self.current_target_pose = self.pose_from_point(point)
                self.target_pose_pub.publish(self.current_target_pose)
                self.set_status(
                    f'Moving to P{index + 1}: '
                    f'({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})')
                ok = self.move_to_point(index, point)
                if ok:
                    self.set_status(f'Arrived at P{index + 1}, holding')
                    time.sleep(max(0.0, float(self.get_parameter('hold_sec').value)))
                else:
                    self.get_logger().warn(
                        f'P{index + 1} 所有 IK 尝试失败，跳过该点继续下一个点。')
                    self.set_status(f'P{index + 1} skipped after IK failure')

                self.set_status('Returning home')
                self.move_home()
            completed_loops += 1

        self.current_index = -1
        self.set_status('Circle point task completed')

    def move_to_point(self, point_index, point):
        center = self.center()
        base_radius = float(self.get_parameter('radius').value)
        angle = math.atan2(point[1] - center[1], point[0] - center[0])
        radius_candidates = self.unique_values([base_radius, base_radius * 0.85, base_radius * 0.70])
        z_candidates = self.unique_values([point[2], point[2] + 0.05, point[2] + 0.10])
        link_candidates = self.unique_links([
            str(self.get_parameter('tip_link').value),
            'spray_nozzle_link',
            'gripper_tcp',
        ])

        for link_name in link_candidates:
            if link_name != str(self.get_parameter('tip_link').value):
                self.get_logger().warn(f'spray_tip_link 点位失败，尝试备用末端 link: {link_name}')
            for radius in radius_candidates:
                for z in z_candidates:
                    candidate = [
                        center[0] + radius * math.cos(angle),
                        center[1] + radius * math.sin(angle),
                        max(0.03, z),
                    ]
                    target_pose = self.pose_from_point(candidate)
                    self.current_target_pose = target_pose
                    self.target_pose_pub.publish(target_pose)
                    self.get_logger().info(
                        f'尝试 P{point_index + 1} IK: link={link_name}, '
                        f'radius={radius:.3f}, '
                        f'point=({candidate[0]:.3f}, {candidate[1]:.3f}, {candidate[2]:.3f})')
                    ok, code = self.move_tip_position_with_moveit(target_pose, link_name)
                    if ok:
                        if candidate != point or link_name != str(self.get_parameter('tip_link').value):
                            self.get_logger().info(
                                f'P{point_index + 1} 使用调整后目标成功: '
                                f'link={link_name}, point={candidate}')
                        return True
                    self.get_logger().warn(
                        f'P{point_index + 1} IK/规划失败: '
                        f'link={link_name}, point={candidate}, MoveItErrorCode={code}')
        return False

    def move_home(self):
        home = self.home_joints()
        if not self.move_joint_goal_with_moveit(home):
            self.get_logger().warn('MoveIt home 规划失败，尝试直接发送 home 轨迹。')
            return self.send_joint_trajectory(
                home, duration_sec=float(self.get_parameter('home_duration_sec').value))
        return True

    def generate_points(self):
        center = self.center()
        radius = float(self.get_parameter('radius').value)
        num_points = max(1, int(self.get_parameter('num_points').value))
        points = []
        for index in range(num_points):
            angle = 2.0 * math.pi * index / num_points
            points.append([
                center[0] + radius * math.cos(angle),
                center[1] + radius * math.sin(angle),
                max(0.03, center[2]),
            ])
        return points

    def pose_from_point(self, point):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = str(self.get_parameter('frame_id').value)
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = float(point[2])
        pose.pose.orientation.w = 1.0
        return pose

    def center(self):
        try:
            value = list(self.get_parameter('center').value)
            if len(value) >= 3:
                return [float(value[0]), float(value[1]), max(0.03, float(value[2]))]
        except (TypeError, ValueError):
            pass

        return [
            float(self.get_parameter('center_x').value),
            float(self.get_parameter('center_y').value),
            max(0.03, float(self.get_parameter('center_z').value)),
        ]

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

    def move_tip_position_with_moveit(self, target_pose, link_name):
        if not self.move_group_client.wait_for_server(timeout_sec=12.0):
            self.get_logger().warn('找不到 MoveGroup action server。')
            return False, 'NO_MOVE_GROUP_SERVER'

        goal = self.base_move_group_goal()
        goal.request.goal_constraints.append(self.position_constraints(target_pose, link_name))

        send_future = self.move_group_client.send_goal_async(goal)
        if not self.wait_future(send_future, 12.0):
            return False, 'SEND_TIMEOUT'
        handle = send_future.result()
        if not handle.accepted:
            return False, 'GOAL_REJECTED'

        result_future = handle.get_result_async()
        if not self.wait_future(result_future, 35.0):
            return False, 'RESULT_TIMEOUT'
        result = result_future.result().result
        return result.error_code.val == MoveItErrorCodes.SUCCESS, result.error_code.val

    def position_constraints(self, target_pose, link_name):
        position_tolerance = float(self.get_parameter('position_tolerance').value)

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
        constraints.name = f'{link_name}_circle_point_position_goal'
        constraints.position_constraints.append(position_constraint)
        return constraints

    def move_joint_goal_with_moveit(self, joint_targets):
        if not self.move_group_client.wait_for_server(timeout_sec=12.0):
            self.get_logger().warn('找不到 MoveGroup action server。')
            return False

        goal = self.base_move_group_goal()
        constraints = Constraints()
        constraints.name = 'rebot_home_goal'
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

    def base_move_group_goal(self):
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.pipeline_id = 'ompl'
        goal.request.num_planning_attempts = 12
        goal.request.allowed_planning_time = 8.0
        goal.request.max_velocity_scaling_factor = float(self.get_parameter('velocity_scaling').value)
        goal.request.max_acceleration_scaling_factor = float(
            self.get_parameter('acceleration_scaling').value)
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

    def set_status(self, text):
        self.status_text = text
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_visualization(self):
        now = self.get_clock().now()
        markers = MarkerArray()
        points = self.generate_points()
        center = self.center()
        for index, point in enumerate(points):
            markers.markers.append(self.point_marker(now, index, point))
            markers.markers.append(self.label_marker(now, index, point))
        markers.markers.append(self.circle_marker(now, center, float(self.get_parameter('radius').value)))
        markers.markers.append(self.status_marker(now, center))
        self.marker_pub.publish(markers)
        if self.current_target_pose is not None:
            self.current_target_pose.header.stamp = now.to_msg()
            self.target_pose_pub.publish(self.current_target_pose)

    def base_marker(self, now, marker_id, marker_type, namespace='circle_points'):
        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = str(self.get_parameter('frame_id').value)
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        return marker

    def point_marker(self, now, index, point):
        marker = self.base_marker(now, index + 1, Marker.SPHERE)
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        active = index == self.current_index
        scale = 0.07 if active else 0.05
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = 1.0 if active else 0.10
        marker.color.g = 0.82 if active else 0.70
        marker.color.b = 0.05 if active else 1.0
        marker.color.a = 1.0
        return marker

    def label_marker(self, now, index, point):
        marker = self.base_marker(now, 100 + index, Marker.TEXT_VIEW_FACING, 'circle_point_labels')
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2] + 0.09
        marker.scale.z = 0.055
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f'P{index + 1}'
        return marker

    def circle_marker(self, now, center, radius):
        marker = self.base_marker(now, 200, Marker.LINE_STRIP, 'circle_point_path')
        marker.scale.x = 0.008
        marker.color.r = 0.20
        marker.color.g = 0.95
        marker.color.b = 1.0
        marker.color.a = 0.65
        point_count = max(24, int(self.get_parameter('num_points').value) * 8)
        for index in range(point_count + 1):
            angle = 2.0 * math.pi * index / point_count
            point = Point()
            point.x = center[0] + radius * math.cos(angle)
            point.y = center[1] + radius * math.sin(angle)
            point.z = center[2]
            marker.points.append(point)
        return marker

    def status_marker(self, now, center):
        marker = self.base_marker(now, 300, Marker.TEXT_VIEW_FACING, 'circle_point_status')
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2] + 0.32
        marker.scale.z = 0.06
        marker.color.r = 0.90
        marker.color.g = 1.0
        marker.color.b = 0.65
        marker.color.a = 1.0
        marker.text = self.status_text
        return marker

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
    node = RebotCirclePointTask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

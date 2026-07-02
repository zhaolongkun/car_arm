#!/usr/bin/env python3
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class Nav2WaypointCommander(Node):
    """Send a final goal, or an optional waypoint list, to Nav2."""

    def __init__(self):
        super().__init__('nav2_waypoint_commander')

        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('goal_x', 10.0)
        self.declare_parameter('goal_y', 10.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('waypoints', '[]')
        self.declare_parameter('use_through_poses', False)
        self.declare_parameter('wait_for_map', True)
        self.declare_parameter('wait_for_odom', True)
        self.declare_parameter('auto_start_navigation', True)
        self.declare_parameter('initial_delay_sec', 8.0)
        self.declare_parameter('retry_failed_goals', False)
        self.declare_parameter('goal_result_timeout_sec', 180.0)

        self.action_name = str(self.get_parameter('action_name').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.goal_frame = str(self.get_parameter('goal_frame').value)
        self.waypoints = self.parse_waypoints(
            str(self.get_parameter('waypoints').value))
        if not self.waypoints:
            self.waypoints = [(
                float(self.get_parameter('goal_x').value),
                float(self.get_parameter('goal_y').value),
                float(self.get_parameter('goal_yaw').value),
            )]
        self.wait_for_map = bool(self.get_parameter('wait_for_map').value)
        self.wait_for_odom = bool(self.get_parameter('wait_for_odom').value)
        self.use_through_poses = bool(
            self.get_parameter('use_through_poses').value)
        self.auto_start_navigation = bool(
            self.get_parameter('auto_start_navigation').value)
        self.initial_delay_sec = max(
            0.0, float(self.get_parameter('initial_delay_sec').value))
        self.retry_failed_goals = bool(self.get_parameter('retry_failed_goals').value)
        self.goal_result_timeout_sec = max(
            10.0, float(self.get_parameter('goal_result_timeout_sec').value))

        self.map_ready = not self.wait_for_map
        self.odom_ready = not self.wait_for_odom
        self.started = False
        self.current_index = 0
        self.active_goal_handle = None
        self.active_goal_start_time = None
        self.goal_request_pending = False
        self.wait_log_count = 0

        action_type = NavigateThroughPoses if self.use_through_poses else NavigateToPose
        self.client = ActionClient(self, action_type, self.action_name)

        if self.wait_for_map:
            self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, 2)
        if self.wait_for_odom:
            self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)

        self.create_timer(self.initial_delay_sec or 0.1, self.arm_once)
        self.create_timer(0.5, self.on_timer)

        self.get_logger().info(
            f'Nav2 waypoint commander ready: {len(self.waypoints)} waypoint(s), '
            f'action=/{self.action_name}, through_poses={self.use_through_poses}, '
            f'auto_start={self.auto_start_navigation}')

    def on_map(self, _msg):
        self.map_ready = True

    def on_odom(self, _msg):
        self.odom_ready = True

    def arm_once(self):
        self.started = True

    def on_timer(self):
        if not self.auto_start_navigation or not self.started:
            return
        if self.current_index >= len(self.waypoints):
            return
        if self.goal_request_pending:
            return
        if self.active_goal_handle is not None:
            self.check_goal_timeout()
            return
        if not (self.map_ready and self.odom_ready):
            self.wait_log_count += 1
            if self.wait_log_count % 4 == 1:
                self.get_logger().info(
                    f'Waiting before Nav2 goal: map_ready={self.map_ready}, '
                    f'odom_ready={self.odom_ready}')
            return
        if not self.client.server_is_ready():
            if not self.client.wait_for_server(timeout_sec=0.1):
                self.get_logger().info(
                    f'Waiting for Nav2 action server /{self.action_name}...',
                    throttle_duration_sec=2.0)
                return
        self.send_current_goal()

    def send_current_goal(self):
        if self.use_through_poses:
            goal_msg = NavigateThroughPoses.Goal()
            goal_msg.poses = [
                self.make_goal_pose(index)
                for index in range(self.current_index, len(self.waypoints))
            ]
            self.get_logger().info(
                f'Sending Nav2 route with {len(goal_msg.poses)} pose(s); '
                f'final x={self.waypoints[-1][0]:.2f}, y={self.waypoints[-1][1]:.2f}')
        else:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.make_goal_pose(self.current_index)
            x, y, _ = self.waypoints[self.current_index]
            self.get_logger().info(
                f'Sending Nav2 waypoint {self.current_index + 1}/{len(self.waypoints)}: '
                f'x={x:.2f}, y={y:.2f}')
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.on_goal_response)
        self.goal_request_pending = True
        self.active_goal_start_time = self.now_seconds()

    def on_goal_response(self, future):
        self.goal_request_pending = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f'Nav2 rejected waypoint {self.current_index + 1}.')
            self.active_goal_handle = None
            self.advance_after_failure()
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result()
        status = result.status
        self.active_goal_handle = None
        self.active_goal_start_time = None
        self.goal_request_pending = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.use_through_poses:
                self.get_logger().info(
                    f'Nav2 reached final pose after {len(self.waypoints)} route pose(s).')
                self.current_index = len(self.waypoints)
            else:
                self.get_logger().info(
                    f'Nav2 reached waypoint {self.current_index + 1}/{len(self.waypoints)}.')
                self.current_index += 1
            return

        self.get_logger().warn(
            f'Nav2 waypoint {self.current_index + 1} finished with status {status}.')
        self.advance_after_failure()

    def check_goal_timeout(self):
        if self.active_goal_start_time is None:
            return
        if self.now_seconds() - self.active_goal_start_time <= self.goal_result_timeout_sec:
            return
        self.get_logger().warn(
            f'Nav2 waypoint {self.current_index + 1} timed out; canceling goal.')
        self.active_goal_handle.cancel_goal_async()
        self.active_goal_handle = None
        self.active_goal_start_time = None
        self.goal_request_pending = False
        self.advance_after_failure()

    def advance_after_failure(self):
        if self.retry_failed_goals:
            return
        if self.use_through_poses:
            self.get_logger().warn('Stopping route after NavigateThroughPoses failure.')
            self.current_index = len(self.waypoints)
            return
        if self.current_index < len(self.waypoints) - 1:
            self.get_logger().warn('Skipping failed waypoint and continuing.')
            self.current_index += 1
            return
        self.get_logger().warn('Stopping navigation after final goal failure.')
        self.current_index = len(self.waypoints)

    def make_goal_pose(self, index):
        x, y, yaw = self.waypoints[index]
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.goal_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def parse_waypoints(self, raw):
        raw = raw.strip()
        if raw in ('', '[]'):
            return []
        waypoints = []
        for chunk in raw.split(';'):
            chunk = chunk.strip()
            if not chunk:
                continue
            parts = [part.strip() for part in chunk.split(',')]
            if len(parts) not in (2, 3):
                self.get_logger().warn(
                    f'Ignoring invalid waypoint "{chunk}". Use x,y or x,y,yaw.')
                continue
            try:
                x = float(parts[0])
                y = float(parts[1])
                yaw = float(parts[2]) if len(parts) == 3 else 0.0
            except ValueError:
                self.get_logger().warn(f'Ignoring invalid waypoint "{chunk}".')
                continue
            waypoints.append((x, y, yaw))
        return waypoints

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = Nav2WaypointCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

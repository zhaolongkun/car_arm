#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class AutoGoalPublisher(Node):
    """Publish a default navigation goal after map and odometry are ready."""

    def __init__(self):
        super().__init__('auto_goal_publisher')

        self.declare_parameter('goal_topic', 'goal_pose')
        self.declare_parameter('start_topic', 'start_navigation')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('goal_x', 10.0)
        self.declare_parameter('goal_y', 8.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('waypoints', '')
        self.declare_parameter('waypoint_tolerance', 0.8)
        self.declare_parameter('initial_delay_sec', 8.0)
        self.declare_parameter('repeat_period_sec', 0.5)
        self.declare_parameter('publish_repeats', 6)
        self.declare_parameter('wait_for_map', True)
        self.declare_parameter('wait_for_odom', True)
        self.declare_parameter('auto_start_navigation', True)

        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.start_topic = str(self.get_parameter('start_topic').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.goal_frame = str(self.get_parameter('goal_frame').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_yaw = float(self.get_parameter('goal_yaw').value)
        self.waypoints = self.parse_waypoints(
            str(self.get_parameter('waypoints').value))
        if not self.waypoints:
            self.waypoints = [(self.goal_x, self.goal_y, self.goal_yaw)]
        self.waypoint_tolerance = max(
            0.05, float(self.get_parameter('waypoint_tolerance').value))
        self.initial_delay_sec = max(
            0.0, float(self.get_parameter('initial_delay_sec').value))
        self.repeat_period_sec = max(
            0.05, float(self.get_parameter('repeat_period_sec').value))
        self.publish_repeats = max(1, int(self.get_parameter('publish_repeats').value))
        self.wait_for_map = bool(self.get_parameter('wait_for_map').value)
        self.wait_for_odom = bool(self.get_parameter('wait_for_odom').value)
        self.auto_start_navigation = bool(
            self.get_parameter('auto_start_navigation').value)

        self.map_ready = not self.wait_for_map
        self.odom_ready = not self.wait_for_odom
        self.odom_xy = None
        self.current_index = 0
        self.sent_count = 0
        self.started = False
        self.wait_log_count = 0

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 5)
        self.start_pub = self.create_publisher(Bool, self.start_topic, 5)

        if self.wait_for_map:
            self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, 2)
        if self.wait_for_odom:
            self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)

        self.create_timer(self.initial_delay_sec or 0.1, self.arm_once)
        self.timer = self.create_timer(self.repeat_period_sec, self.on_timer)

        self.get_logger().info(
            f'Auto goal publisher ready: {len(self.waypoints)} waypoint(s), '
            f'final=({self.waypoints[-1][0]:.2f}, {self.waypoints[-1][1]:.2f}), '
            f'auto_start={self.auto_start_navigation}')

    def on_map(self, _msg):
        self.map_ready = True

    def on_odom(self, msg):
        self.odom_ready = True
        self.odom_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def arm_once(self):
        self.started = True

    def on_timer(self):
        if not self.started:
            return
        if not (self.map_ready and self.odom_ready):
            self.wait_log_count += 1
            if self.wait_log_count % max(1, int(2.0 / self.repeat_period_sec)) == 1:
                self.get_logger().info(
                    f'Waiting before publishing goal: map_ready={self.map_ready}, '
                    f'odom_ready={self.odom_ready}')
            return

        self.advance_waypoint_if_reached()
        if self.current_index >= len(self.waypoints):
            return
        if self.sent_count >= self.publish_repeats:
            return

        self.goal_pub.publish(self.make_goal())
        if self.auto_start_navigation:
            start = Bool()
            start.data = True
            self.start_pub.publish(start)
        self.sent_count += 1
        goal_x, goal_y, _ = self.waypoints[self.current_index]
        self.get_logger().info(
            f'Published waypoint {self.current_index + 1}/{len(self.waypoints)} '
            f'({self.sent_count}/{self.publish_repeats}): '
            f'x={goal_x:.2f}, y={goal_y:.2f}')

    def make_goal(self):
        goal_x, goal_y, goal_yaw = self.waypoints[self.current_index]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.goal_frame
        msg.pose.position.x = goal_x
        msg.pose.position.y = goal_y
        msg.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(goal_yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def advance_waypoint_if_reached(self):
        if self.odom_xy is None or self.current_index >= len(self.waypoints):
            return
        goal_x, goal_y, _ = self.waypoints[self.current_index]
        distance = math.hypot(self.odom_xy[0] - goal_x, self.odom_xy[1] - goal_y)
        if distance > self.waypoint_tolerance:
            return
        self.get_logger().info(
            f'Reached waypoint {self.current_index + 1}/{len(self.waypoints)} '
            f'at distance {distance:.2f} m.')
        self.current_index += 1
        self.sent_count = 0

    def parse_waypoints(self, raw):
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


def main(args=None):
    rclpy.init(args=args)
    node = AutoGoalPublisher()
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

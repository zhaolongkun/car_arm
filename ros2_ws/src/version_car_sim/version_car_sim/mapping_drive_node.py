#!/usr/bin/env python3
"""Mapping drive helper for FAST-LIO 3D mapping simulation.

Publishes /cmd_vel to drive the car in a pattern that covers
the environment for FAST-LIO mapping.

Supports two modes:
  1. CIRCLE:  constant forward speed + slight angular speed (circle path)
  2. WAYPOINT: rectangle/square path covering the scene

Controllable via parameters:
  - mode: 'circle' (default) or 'waypoint'
  - linear_speed: forward speed (default 0.3 m/s)
  - angular_speed: turn rate (default 0.15 rad/s for circle)
  - waypoint_size: half-side of rectangle (default 5.0 m)
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class MappingDriveNode(Node):
    """Drive the car in a pattern for FAST-LIO 3D mapping."""

    def __init__(self):
        super().__init__('mapping_drive_node')

        self.declare_parameter('mode', 'circle')
        self.declare_parameter('linear_speed', 0.30)
        self.declare_parameter('angular_speed', 0.15)
        self.declare_parameter('waypoint_size', 5.0)
        self.declare_parameter('waypoint_linear_speed', 0.25)
        self.declare_parameter('waypoint_angular_speed', 0.20)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('enable_topic', 'mapping_drive_enable')
        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('auto_enable', True)

        self.mode = str(self.get_parameter('mode').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.waypoint_size = float(self.get_parameter('waypoint_size').value)
        self.waypoint_linear_speed = float(
            self.get_parameter('waypoint_linear_speed').value)
        self.waypoint_angular_speed = float(
            self.get_parameter('waypoint_angular_speed').value)
        self.auto_enable = bool(self.get_parameter('auto_enable').value)

        self.enabled = self.auto_enable
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_topic').value), 10)

        enable_topic = str(self.get_parameter('enable_topic').value)
        if enable_topic:
            self.create_subscription(
                Bool, enable_topic, self.on_enable, 5)

        publish_rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / publish_rate, self.on_timer)

        self.waypoint_phase = 0  # 0=forward, 1=turn, 2=forward, 3=turn, ...
        self.waypoint_timer = 0.0
        self.waypoint_duration_forward = 3.0  # seconds to go straight
        self.waypoint_duration_turn = 4.0     # seconds to turn

        self.get_logger().info(
            f'Mapping drive node ready: mode={self.mode}, '
            f'linear={self.linear_speed}, angular={self.angular_speed}, '
            f'auto_enable={self.auto_enable}')

    def on_enable(self, msg):
        self.enabled = msg.data
        state = 'enabled' if self.enabled else 'disabled'
        self.get_logger().info(f'Mapping drive {state}')

    def on_timer(self):
        if not self.enabled:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        if self.mode == 'circle':
            cmd = Twist()
            cmd.linear.x = self.linear_speed
            cmd.angular.z = self.angular_speed
            self.cmd_pub.publish(cmd)
        elif self.mode == 'waypoint':
            cmd = self.waypoint_pattern()
            self.cmd_pub.publish(cmd)

    def waypoint_pattern(self):
        """Square/rectangle drive pattern for area coverage."""
        cmd = Twist()
        dt = 1.0 / float(self.get_parameter('publish_rate').value)
        self.waypoint_timer += dt

        if self.waypoint_phase % 2 == 0:
            # Go straight
            self.waypoint_duration_forward = (
                self.waypoint_size / max(0.01, self.waypoint_linear_speed))
            if self.waypoint_timer >= self.waypoint_duration_forward:
                self.waypoint_phase += 1
                self.waypoint_timer = 0.0
            else:
                cmd.linear.x = self.waypoint_linear_speed
        else:
            # Turn ~90 degrees
            if self.waypoint_timer >= self.waypoint_duration_turn:
                self.waypoint_phase += 1
                self.waypoint_timer = 0.0
                if self.waypoint_phase >= 8:
                    self.waypoint_phase = 0
            else:
                cmd.angular.z = self.waypoint_angular_speed

        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = MappingDriveNode()
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

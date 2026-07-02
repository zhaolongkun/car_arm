#!/usr/bin/env python3
import json

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class EgoGoalBridge(Node):
    """Forward the car's /goal_pose into ego-planner's manual goal topic."""

    def __init__(self):
        super().__init__('ego_goal_bridge')

        self.declare_parameter('input_goal_topic', 'goal_pose')
        self.declare_parameter('ego_goal_topic', '/move_base_simple/goal')
        self.declare_parameter('ego_goal_frame', 'map')
        self.declare_parameter('ego_goal_z', 1.0)
        self.declare_parameter('debug_topic', 'ego_goal_bridge_debug')

        self.ego_goal_frame = str(self.get_parameter('ego_goal_frame').value)
        self.ego_goal_z = float(self.get_parameter('ego_goal_z').value)
        self.forward_count = 0
        self.latest_goal = None

        self.goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('ego_goal_topic').value), 5)
        self.debug_pub = self.create_publisher(
            String, str(self.get_parameter('debug_topic').value), 10)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('input_goal_topic').value),
            self.on_goal,
            5,
        )
        self.create_timer(1.0, self.publish_debug)
        self.get_logger().info(
            f'EGO goal bridge ready: /{self.get_parameter("input_goal_topic").value} '
            f'-> {self.get_parameter("ego_goal_topic").value}')

    def on_goal(self, msg):
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.ego_goal_frame or msg.header.frame_id or 'map'
        out.pose = msg.pose
        out.pose.position.z = self.ego_goal_z
        self.goal_pub.publish(out)

        self.forward_count += 1
        self.latest_goal = (
            float(out.pose.position.x),
            float(out.pose.position.y),
            float(out.pose.position.z),
        )
        self.get_logger().info(
            'Forwarded /goal_pose to ego-planner manual goal: '
            f'x={self.latest_goal[0]:.2f}, y={self.latest_goal[1]:.2f}, '
            f'z={self.latest_goal[2]:.2f}')

    def publish_debug(self):
        msg = String()
        msg.data = json.dumps({
            'forward_count': int(self.forward_count),
            'latest_goal': self.latest_goal,
            'ego_goal_frame': self.ego_goal_frame,
            'ego_goal_z': float(self.ego_goal_z),
        }, sort_keys=True)
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EgoGoalBridge()
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

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker


class GasFieldSimulator(Node):
    def __init__(self):
        super().__init__('gas_field_simulator')
        self.declare_parameter('leak_x', 9.2)
        self.declare_parameter('leak_y', 9.2)
        self.declare_parameter('leak_z', 0.3)
        self.declare_parameter('source_strength', 1.0)
        self.declare_parameter('publish_rate', 2.0)

        self.pose_pub = self.create_publisher(PoseStamped, 'leak_pose_map', 10)
        self.strength_pub = self.create_publisher(Float32, 'gas/source_strength', 10)
        self.concentration_pub = self.create_publisher(Float32, 'gas/concentration', 10)
        self.marker_pub = self.create_publisher(Marker, 'gas/leak_marker', 10)

        rate = max(0.2, float(self.get_parameter('publish_rate').value))
        self.timer = self.create_timer(1.0 / rate, self.publish_state)

    def publish_state(self):
        x = float(self.get_parameter('leak_x').value)
        y = float(self.get_parameter('leak_y').value)
        z = float(self.get_parameter('leak_z').value)
        strength = max(0.0, float(self.get_parameter('source_strength').value))

        now = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        msg = Float32()
        msg.data = float(strength)
        self.strength_pub.publish(msg)
        self.concentration_pub.publish(msg)

        marker = Marker()
        marker.header = pose.header
        marker.ns = 'gas_leak'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.25 + 0.45 * math.sqrt(strength)
        marker.scale.y = marker.scale.x
        marker.scale.z = marker.scale.x
        marker.color.r = 1.0
        marker.color.g = 0.15 + 0.55 * (1.0 - min(strength, 1.0))
        marker.color.b = 0.05
        marker.color.a = 0.35 + 0.55 * min(strength, 1.0)
        self.marker_pub.publish(marker)

        text = Marker()
        text.header = pose.header
        text.ns = 'gas_leak'
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = z + 0.55
        text.pose.orientation.w = 1.0
        text.scale.z = 0.22
        text.color.r = 1.0 if strength > 0.25 else 0.25
        text.color.g = 0.25 if strength > 0.25 else 1.0
        text.color.b = 0.08
        text.color.a = 1.0
        if strength <= 0.25:
            text.text = 'Leak neutralization completed'
        else:
            text.text = f'Gas Leak Source  concentration={strength:.2f}'
        self.marker_pub.publish(text)


def main(args=None):
    rclpy.init(args=args)
    node = GasFieldSimulator()
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

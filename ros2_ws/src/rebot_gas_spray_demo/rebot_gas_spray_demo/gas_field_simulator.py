import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker, MarkerArray


class GasFieldSimulator(Node):
    def __init__(self):
        super().__init__('gas_field_simulator')
        self.declare_parameter('leak_x', 0.50)
        self.declare_parameter('leak_y', 0.0)
        self.declare_parameter('leak_z', 0.28)
        self.declare_parameter('initial_concentration', 1.0)
        self.declare_parameter('neutralize_rate', 0.22)
        self.declare_parameter('alarm_threshold', 0.25)
        self.declare_parameter('publish_rate', 10.0)

        self.concentration = float(self.get_parameter('initial_concentration').value)
        self.spraying = False
        self.completed = False

        self.pose_pub = self.create_publisher(PoseStamped, 'leak_pose', 10)
        self.concentration_pub = self.create_publisher(Float32, 'gas/concentration', 10)
        self.alarm_pub = self.create_publisher(Bool, 'gas/alarm', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'gas/source_marker', 10)
        self.create_subscription(Bool, 'spray/start', self.on_spray_start, 10)

        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate, self.update)

    def on_spray_start(self, msg):
        self.spraying = bool(msg.data)

    def update(self):
        now = self.get_clock().now()
        dt = max(0.0, (now - self.last_time).nanoseconds / 1e9)
        self.last_time = now

        if self.spraying and self.concentration > 0.0:
            rate = float(self.get_parameter('neutralize_rate').value)
            self.concentration = max(0.0, self.concentration - rate * dt)

        self.publish_pose(now)
        self.publish_concentration()
        self.publish_scene_markers(now)

        threshold = float(self.get_parameter('alarm_threshold').value)
        if not self.completed and self.concentration <= threshold:
            self.completed = True
            self.get_logger().info('Leak neutralization completed')

    def publish_pose(self, now):
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(self.get_parameter('leak_x').value)
        pose.pose.position.y = float(self.get_parameter('leak_y').value)
        pose.pose.position.z = float(self.get_parameter('leak_z').value)
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

    def publish_concentration(self):
        concentration = Float32()
        concentration.data = float(self.concentration)
        self.concentration_pub.publish(concentration)

        alarm = Bool()
        alarm.data = self.concentration > float(self.get_parameter('alarm_threshold').value)
        self.alarm_pub.publish(alarm)

    def publish_scene_markers(self, now):
        leak_x = float(self.get_parameter('leak_x').value)
        leak_y = float(self.get_parameter('leak_y').value)
        leak_z = float(self.get_parameter('leak_z').value)

        markers = MarkerArray()
        markers.markers.append(self.pipe_marker(now, leak_x, leak_y, leak_z))
        markers.markers.append(self.crack_marker(now, leak_x, leak_y, leak_z))
        markers.markers.append(self.leak_point_marker(now, leak_x, leak_y, leak_z))
        markers.markers.append(self.gas_cloud_marker(now, leak_x, leak_y, leak_z))
        markers.markers.append(self.concentration_text_marker(now, leak_x, leak_y, leak_z))
        markers.markers.append(self.source_text_marker(now, leak_x, leak_y, leak_z))
        self.marker_pub.publish(markers)

    def base_marker(self, now, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = 'world'
        marker.ns = 'gas_scene'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        return marker

    def pipe_marker(self, now, x, y, z):
        marker = self.base_marker(now, 1, Marker.CYLINDER)
        marker.pose.position.x = x + 0.04
        marker.pose.position.y = y
        marker.pose.position.z = z - 0.03
        marker.pose.orientation.x = math.sqrt(0.5)
        marker.pose.orientation.w = math.sqrt(0.5)
        marker.scale.x = 0.07
        marker.scale.y = 0.07
        marker.scale.z = 0.65
        marker.color.r = 0.25
        marker.color.g = 0.28
        marker.color.b = 0.30
        marker.color.a = 1.0
        return marker

    def crack_marker(self, now, x, y, z):
        marker = self.base_marker(now, 2, Marker.CUBE)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = 0.012
        marker.scale.y = 0.16
        marker.scale.z = 0.025
        marker.color.r = 1.0
        marker.color.g = 0.12
        marker.color.b = 0.04
        marker.color.a = 1.0
        return marker

    def leak_point_marker(self, now, x, y, z):
        marker = self.base_marker(now, 3, Marker.SPHERE)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = 0.045
        marker.scale.y = 0.045
        marker.scale.z = 0.045
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

    def gas_cloud_marker(self, now, x, y, z):
        marker = self.base_marker(now, 4, Marker.SPHERE)
        marker.pose.position.x = x - 0.03
        marker.pose.position.y = y
        marker.pose.position.z = z + 0.13
        scale = 0.10 + 0.35 * math.sqrt(max(0.0, self.concentration))
        marker.scale.x = scale
        marker.scale.y = scale * 0.85
        marker.scale.z = scale * 0.75
        marker.color.r = 0.95
        marker.color.g = 0.72
        marker.color.b = 0.10
        marker.color.a = 0.18 + 0.45 * max(0.0, min(1.0, self.concentration))
        return marker

    def concentration_text_marker(self, now, x, y, z):
        marker = self.base_marker(now, 5, Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z + 0.55
        marker.scale.z = 0.07
        marker.color.r = 0.95
        marker.color.g = 0.95
        marker.color.b = 0.95
        marker.color.a = 1.0
        if self.concentration <= float(self.get_parameter('alarm_threshold').value):
            marker.text = 'Leak neutralization completed'
            marker.color.g = 1.0
            marker.color.r = 0.25
        else:
            marker.text = f'Gas concentration: {self.concentration:.2f}'
        return marker

    def source_text_marker(self, now, x, y, z):
        marker = self.base_marker(now, 6, Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z + 0.18
        marker.scale.z = 0.055
        marker.color.r = 1.0
        marker.color.g = 0.24
        marker.color.b = 0.08
        marker.color.a = 1.0
        marker.text = 'Gas Leak Source'
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = GasFieldSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

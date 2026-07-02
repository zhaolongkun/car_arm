import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class SpraySimulator(Node):
    def __init__(self):
        super().__init__('spray_simulator')
        self.declare_parameter('spray_duration_sec', 4.0)
        self.declare_parameter('nozzle_frame', 'spray_tip_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('particle_count', 9)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_leak_pose = None
        self.active_until = None
        self.done_sent = False

        self.create_subscription(Bool, 'spray/start', self.on_start, 10)
        self.create_subscription(PoseStamped, 'leak_pose', self.on_leak_pose, 10)
        self.status_pub = self.create_publisher(String, 'spray/status', 10)
        self.done_pub = self.create_publisher(Bool, 'spray/done', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'spray/marker', 10)

        self.timer = self.create_timer(0.08, self.update)

    def on_start(self, msg):
        if msg.data:
            duration = float(self.get_parameter('spray_duration_sec').value)
            self.active_until = self.get_clock().now() + Duration(seconds=duration)
            self.done_sent = False
            self.publish_status('spraying')
        else:
            self.active_until = None
            self.publish_status('idle')
            self.clear_markers()

    def on_leak_pose(self, msg):
        self.latest_leak_pose = msg

    def update(self):
        now = self.get_clock().now()
        active = self.active_until is not None and now < self.active_until
        if active:
            self.publish_status('spraying')
            self.publish_spray_markers(now)
            return

        if self.active_until is not None and not self.done_sent:
            self.active_until = None
            self.done_sent = True
            done = Bool()
            done.data = True
            self.done_pub.publish(done)
            self.publish_status('done')
            self.clear_markers()
        elif self.active_until is None:
            self.publish_status('idle')

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_spray_markers(self, now):
        if self.latest_leak_pose is None:
            return

        world_frame = str(self.get_parameter('world_frame').value)
        nozzle_frame = str(self.get_parameter('nozzle_frame').value)
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                world_frame, nozzle_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.03))
        except TransformException:
            return

        p0 = Point()
        p0.x = tf_msg.transform.translation.x
        p0.y = tf_msg.transform.translation.y
        p0.z = tf_msg.transform.translation.z

        p1 = Point()
        p1.x = self.latest_leak_pose.pose.position.x
        p1.y = self.latest_leak_pose.pose.position.y
        p1.z = self.latest_leak_pose.pose.position.z

        markers = MarkerArray()
        markers.markers.extend(self.spray_beams(now, world_frame, p0, p1))
        markers.markers.extend(self.spray_particles(now, world_frame, p0, p1))
        self.marker_pub.publish(markers)

    def spray_beams(self, now, frame, p0, p1):
        beams = []
        for i, offset in enumerate([-0.018, -0.009, 0.0, 0.009, 0.018]):
            marker = Marker()
            marker.header.stamp = now.to_msg()
            marker.header.frame_id = frame
            marker.ns = 'spray'
            marker.id = 1 + i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.010
            marker.color.r = 0.10
            marker.color.g = 0.85
            marker.color.b = 1.0
            marker.color.a = 0.70
            q0 = Point()
            q0.x = p0.x
            q0.y = p0.y
            q0.z = p0.z
            q1 = Point()
            q1.x = p1.x
            q1.y = p1.y + offset
            q1.z = p1.z + offset * 0.35
            marker.points = [q0, q1]
            beams.append(marker)
        return beams

    def spray_particles(self, now, frame, p0, p1):
        count = int(self.get_parameter('particle_count').value)
        count = max(3, min(20, count))
        phase = (now.nanoseconds / 1e9) * 6.0
        markers = []
        for i in range(count):
            ratio = (i + 1) / (count + 1)
            jitter = 0.018 * math.sin(phase + i * 1.7)
            marker = Marker()
            marker.header.stamp = now.to_msg()
            marker.header.frame_id = frame
            marker.ns = 'spray_particles'
            marker.id = 100 + i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = p0.x + (p1.x - p0.x) * ratio
            marker.pose.position.y = p0.y + (p1.y - p0.y) * ratio + jitter
            marker.pose.position.z = p0.z + (p1.z - p0.z) * ratio + jitter * 0.4
            marker.scale.x = 0.018
            marker.scale.y = 0.018
            marker.scale.z = 0.018
            marker.color.r = 0.10
            marker.color.g = 0.90
            marker.color.b = 1.0
            marker.color.a = 0.65
            markers.append(marker)
        return markers

    def clear_markers(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = str(self.get_parameter('world_frame').value)
        marker.action = Marker.DELETEALL
        array = MarkerArray()
        array.markers.append(marker)
        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = SpraySimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

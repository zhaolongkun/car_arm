from functools import partial

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class SpraySimulator(Node):
    def __init__(self):
        super().__init__('spray_simulator')
        self.declare_parameter('spray_duration_sec', 5.0)
        self.declare_parameter('neutralize_rate', 0.16)
        self.declare_parameter('source_parameter_service', '/gas_field_simulator/set_parameters')
        self.declare_parameter('nozzle_frame', 'spray_tip_link')
        self.declare_parameter('map_frame', 'map')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_leak_pose = None
        self.source_strength = 1.0
        self.active_until = None
        self.last_reduce_time = None
        self.pending_param_request = False

        self.create_subscription(Bool, 'spray/start', self.on_spray_start, 10)
        self.create_subscription(PoseStamped, 'leak_pose_map', self.on_leak_pose, 10)
        self.create_subscription(Float32, 'gas/source_strength', self.on_source_strength, 10)

        self.status_pub = self.create_publisher(String, 'spray/status', 10)
        self.done_pub = self.create_publisher(Bool, 'spray/done', 10)
        self.marker_pub = self.create_publisher(Marker, 'spray/marker', 10)

        service_name = str(self.get_parameter('source_parameter_service').value)
        self.set_params_client = self.create_client(SetParameters, service_name)

        self.timer = self.create_timer(0.1, self.update)

    def on_spray_start(self, msg):
        if not msg.data:
            self.active_until = None
            self.publish_status('idle')
            return

        duration_sec = float(self.get_parameter('spray_duration_sec').value)
        self.active_until = self.get_clock().now() + Duration(seconds=duration_sec)
        self.last_reduce_time = self.get_clock().now()
        self.publish_status('spraying')

    def on_leak_pose(self, msg):
        self.latest_leak_pose = msg

    def on_source_strength(self, msg):
        self.source_strength = max(0.0, float(msg.data))

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def update(self):
        now = self.get_clock().now()
        spraying = self.active_until is not None and now < self.active_until

        if spraying:
            self.publish_status(f'spraying source_strength={self.source_strength:.3f}')
            self.publish_spray_marker()
            self.reduce_source_strength(now)
        elif self.active_until is not None:
            self.active_until = None
            self.publish_status('spray_finished')
            done = Bool()
            done.data = True
            self.done_pub.publish(done)
            self.clear_marker()
        else:
            self.publish_status(f'idle source_strength={self.source_strength:.3f}')

    def publish_spray_marker(self):
        if self.latest_leak_pose is None:
            return

        map_frame = str(self.get_parameter('map_frame').value)
        nozzle_frame = str(self.get_parameter('nozzle_frame').value)
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                map_frame, nozzle_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except TransformException:
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = map_frame
        marker.ns = 'spray'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.035
        marker.color.r = 0.05
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 0.95

        p0 = Point()
        p0.x = tf_msg.transform.translation.x
        p0.y = tf_msg.transform.translation.y
        p0.z = tf_msg.transform.translation.z
        p1 = Point()
        p1.x = self.latest_leak_pose.pose.position.x
        p1.y = self.latest_leak_pose.pose.position.y
        p1.z = self.latest_leak_pose.pose.position.z
        marker.points = [p0, p1]
        self.marker_pub.publish(marker)

    def clear_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = str(self.get_parameter('map_frame').value)
        marker.ns = 'spray'
        marker.id = 1
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    def reduce_source_strength(self, now):
        if self.pending_param_request:
            return
        if self.last_reduce_time is None:
            self.last_reduce_time = now
            return

        dt = (now - self.last_reduce_time).nanoseconds / 1e9
        if dt < 0.4:
            return
        self.last_reduce_time = now

        rate = max(0.0, float(self.get_parameter('neutralize_rate').value))
        new_strength = max(0.0, self.source_strength - rate * dt)
        request = SetParameters.Request()
        value = ParameterValue()
        value.type = ParameterType.PARAMETER_DOUBLE
        value.double_value = float(new_strength)
        request.parameters = [Parameter(name='source_strength', value=value)]

        self.pending_param_request = True
        future = self.set_params_client.call_async(request)
        future.add_done_callback(partial(self.on_set_parameter_done, new_strength=new_strength))

    def on_set_parameter_done(self, future, new_strength):
        self.pending_param_request = False
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f'更新 gas source_strength 失败: {exc}')
            return
        if result.results and result.results[0].successful:
            self.source_strength = new_strength


def main(args=None):
    rclpy.init(args=args)
    node = SpraySimulator()
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

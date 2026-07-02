#!/usr/bin/env python3
import math
import subprocess
import time

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.node import Node


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class StartPoseSetter(Node):
    """Set the Gazebo vehicle start pose from ROS topics in dry-run simulation."""

    def __init__(self):
        super().__init__('start_pose_setter')

        self.declare_parameter('entity_name', 'version_car')
        self.declare_parameter('start_pose_topic', 'start_pose')
        self.declare_parameter('start_pose_2d_topic', 'start_pose_2d')
        self.declare_parameter('initialpose_topic', 'initialpose')
        self.declare_parameter('set_entity_state_service', '/gazebo/set_entity_state')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('default_z', 0.0)
        self.declare_parameter('stop_cmd_repeats', 5)
        self.declare_parameter('retry_period', 0.5)
        self.declare_parameter('prefer_gazebo_cli', True)
        self.declare_parameter('gazebo_cli_timeout', 3.0)

        self.entity_name = str(self.get_parameter('entity_name').value)
        self.reference_frame = str(self.get_parameter('reference_frame').value)
        self.default_z = float(self.get_parameter('default_z').value)
        self.stop_cmd_repeats = max(1, int(self.get_parameter('stop_cmd_repeats').value))
        retry_period = max(0.1, float(self.get_parameter('retry_period').value))
        self.prefer_gazebo_cli = bool(self.get_parameter('prefer_gazebo_cli').value)
        self.gazebo_cli_timeout = max(
            0.5, float(self.get_parameter('gazebo_cli_timeout').value))

        service_name = str(self.get_parameter('set_entity_state_service').value)
        self.set_state_client = self.create_client(SetEntityState, service_name)

        self.applied_pose_pub = self.create_publisher(PoseStamped, 'start_pose_applied', 10)

        self.pending_pose = None
        self.pending_source = ''
        self.pending_seq = 0
        self.request_in_flight = False
        self.wait_logged = False

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('start_pose_topic').value),
            self.on_start_pose,
            10,
        )
        self.create_subscription(
            Pose2D,
            str(self.get_parameter('start_pose_2d_topic').value),
            self.on_start_pose_2d,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('initialpose_topic').value),
            self.on_initial_pose,
            10,
        )
        self.create_timer(retry_period, self.retry_pending_pose)

        self.get_logger().info(
            'Start pose setter ready: publish /start_pose PoseStamped or '
            '/start_pose_2d Pose2D to move version_car in Gazebo.')

    def on_start_pose(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose
        if pose_msg.pose.position.z == 0.0:
            pose_msg.pose.position.z = self.default_z
        self.set_start_pose(pose_msg, 'start_pose')

    def on_start_pose_2d(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(msg.x)
        pose_msg.pose.position.y = float(msg.y)
        pose_msg.pose.position.z = self.default_z
        qx, qy, qz, qw = quaternion_from_yaw(float(msg.theta))
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.set_start_pose(pose_msg, 'start_pose_2d')

    def on_initial_pose(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        if pose_msg.pose.position.z == 0.0:
            pose_msg.pose.position.z = self.default_z
        self.set_start_pose(pose_msg, 'initialpose')

    def set_start_pose(self, pose_msg, source):
        self.publish_stop()
        self.pending_pose = pose_msg
        self.pending_source = source
        self.pending_seq += 1

        self.get_logger().info(
            f'Received start pose from /{source}: '
            f'x={pose_msg.pose.position.x:.2f}, y={pose_msg.pose.position.y:.2f}, '
            f'z={pose_msg.pose.position.z:.2f}')
        self.retry_pending_pose()

    def retry_pending_pose(self):
        if self.pending_pose is None or self.request_in_flight:
            return

        if self.prefer_gazebo_cli and self.apply_with_gazebo_cli(
                self.pending_pose, self.pending_source, self.pending_seq):
            return

        if not self.set_state_client.service_is_ready():
            if not self.wait_logged:
                self.wait_logged = True
                self.get_logger().warn(
                    'Waiting for Gazebo /set_entity_state service before applying '
                    'the requested start pose.')
            return

        self.wait_logged = False
        pose_msg = self.pending_pose
        source = self.pending_source
        seq = self.pending_seq

        if not self.set_state_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(
                'Gazebo /set_entity_state service is not ready yet; '
                'will retry the pending start pose.')
            return

        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.entity_name
        request.state.pose = pose_msg.pose
        request.state.twist = Twist()
        request.state.reference_frame = self.reference_frame

        self.request_in_flight = True
        future = self.set_state_client.call_async(request)
        future.add_done_callback(
            lambda done: self.on_set_state_done(done, pose_msg, source, seq))

        self.get_logger().info(
            f'Requested Gazebo start pose update from /{source}: '
            f'x={pose_msg.pose.position.x:.2f}, y={pose_msg.pose.position.y:.2f}, '
            f'z={pose_msg.pose.position.z:.2f}')

    def on_set_state_done(self, future, pose_msg, source, seq):
        self.request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to call /gazebo/set_entity_state: {exc}')
            return

        if not response.success:
            self.get_logger().warn(
                f'Gazebo rejected start pose from /{source} for entity '
                f'{self.entity_name}; will retry while this request remains pending.')
            if self.apply_with_gazebo_cli(pose_msg, source, seq):
                return
            return

        self.mark_pose_applied(pose_msg, seq)

        if self.pending_pose is not None:
            self.retry_pending_pose()

    def apply_with_gazebo_cli(self, pose_msg, source, seq):
        yaw = yaw_from_quaternion(pose_msg.pose.orientation)
        command = [
            'gz', 'model',
            '-m', self.entity_name,
            '-x', f'{pose_msg.pose.position.x:.6f}',
            '-y', f'{pose_msg.pose.position.y:.6f}',
            '-z', f'{pose_msg.pose.position.z:.6f}',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', f'{yaw:.6f}',
        ]

        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=self.gazebo_cli_timeout,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warn(
                f'Gazebo CLI start pose fallback is not ready for /{source}: {exc}')
            return False

        if result.returncode != 0:
            message = (result.stderr or result.stdout or '').strip()
            self.get_logger().warn(
                f'Gazebo CLI rejected start pose from /{source}; '
                f'will retry. {message}')
            return False

        self.mark_pose_applied(pose_msg, seq)
        return True

    def mark_pose_applied(self, pose_msg, seq):
        if seq == self.pending_seq:
            self.pending_pose = None
            self.pending_source = ''

        self.publish_stop()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.applied_pose_pub.publish(pose_msg)
        self.get_logger().info(
            f'Start pose applied to {self.entity_name}: '
            f'x={pose_msg.pose.position.x:.2f}, y={pose_msg.pose.position.y:.2f}')

    def publish_stop(self):
        stop = Twist()
        cmd_raw_pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        time.sleep(0.05)
        for _ in range(self.stop_cmd_repeats):
            cmd_raw_pub.publish(stop)
            cmd_pub.publish(stop)
            time.sleep(0.01)
        self.destroy_publisher(cmd_raw_pub)
        self.destroy_publisher(cmd_pub)


def main(args=None):
    rclpy.init(args=args)
    node = StartPoseSetter()
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

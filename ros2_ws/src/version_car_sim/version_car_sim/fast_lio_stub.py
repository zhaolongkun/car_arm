#!/usr/bin/env python3
import copy
import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener


def distance_xy(a, b):
    return math.hypot(a.pose.position.x - b.pose.position.x,
                      a.pose.position.y - b.pose.position.y)


class FastLioStub(Node):
    """Dry-run FAST-LIO interface stub for Gazebo-only Livox simulation."""

    def __init__(self):
        super().__init__('fast_lio_stub')

        self.declare_parameter('input_odom_topic', 'odom')
        self.declare_parameter('input_points_topic', 'mid360_points')
        self.declare_parameter('fast_lio_odom_topic', 'fast_lio/odom')
        self.declare_parameter('fast_lio_path_topic', 'fast_lio/path')
        self.declare_parameter('registered_cloud_topic', 'cloud_registered')
        self.declare_parameter('laser_map_topic', 'Laser_map')
        self.declare_parameter('debug_topic', 'fast_lio_stub_debug')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('transform_cloud_to_map', True)
        self.declare_parameter('tf_timeout_sec', 0.10)
        self.declare_parameter('path_sample_distance', 0.05)
        self.declare_parameter('max_path_length', 5000)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.transform_cloud_to_map = bool(
            self.get_parameter('transform_cloud_to_map').value)
        self.tf_timeout_sec = max(
            0.01, float(self.get_parameter('tf_timeout_sec').value))
        self.path_sample_distance = max(
            0.0, float(self.get_parameter('path_sample_distance').value))
        self.max_path_length = max(1, int(self.get_parameter('max_path_length').value))
        self.path = Path()
        self.path.header.frame_id = self.map_frame
        self.last_pose = None
        self.latest_odom = None
        self.latest_cloud_frame = ''
        self.latest_registered_frame = ''
        self.latest_cloud_points = 0
        self.latest_tf_success = False
        self.latest_tf_error = ''

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter('fast_lio_odom_topic').value), 10)
        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter('fast_lio_path_topic').value), 5)
        self.cloud_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('registered_cloud_topic').value), 5)
        self.map_cloud_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('laser_map_topic').value), 5)
        self.debug_pub = self.create_publisher(
            String, str(self.get_parameter('debug_topic').value), 10)

        self.create_subscription(
            Odometry,
            str(self.get_parameter('input_odom_topic').value),
            self.on_odom,
            20,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('input_points_topic').value),
            self.on_points,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self.publish_debug)

        self.get_logger().warn(
            'FAST-LIO is not installed in this workspace; running fast_lio_stub. '
            'Gazebo /odom is republished as FAST-LIO odometry, and /mid360_points '
            'is republished as /cloud_registered for the 3D-to-2D projection chain.')

    def on_odom(self, msg):
        out = copy.deepcopy(msg)
        out.header.frame_id = self.map_frame
        out.child_frame_id = msg.child_frame_id or 'base_link'
        self.latest_odom = out
        self.odom_pub.publish(out)

        pose = copy.deepcopy(out.pose.pose)
        pose_stamped = self.make_pose_stamped(out.header.stamp, pose)
        if self.last_pose is None or distance_xy(pose_stamped, self.last_pose) >= self.path_sample_distance:
            self.path.header.stamp = out.header.stamp
            self.path.poses.append(pose_stamped)
            if len(self.path.poses) > self.max_path_length:
                self.path.poses = self.path.poses[-self.max_path_length:]
            self.last_pose = pose_stamped
        self.path_pub.publish(self.path)

    def on_points(self, msg):
        registered = self.make_registered_cloud(msg)
        if registered is None:
            return
        self.latest_cloud_frame = registered.header.frame_id
        self.latest_cloud_points = int(
            registered.width * registered.height if registered.width and registered.height
            else registered.width)
        self.latest_registered_frame = registered.header.frame_id
        self.cloud_pub.publish(registered)
        self.map_cloud_pub.publish(registered)

    def make_registered_cloud(self, msg):
        if not self.transform_cloud_to_map:
            self.latest_tf_success = True
            self.latest_tf_error = ''
            return copy.deepcopy(msg)

        source_frame = msg.header.frame_id.strip()
        if not source_frame:
            self.latest_tf_success = False
            self.latest_tf_error = 'empty point cloud frame_id'
            self.get_logger().warn(
                'Skipping FAST-LIO stub cloud registration because frame_id is empty.',
                throttle_duration_sec=2.0)
            return None

        transform = self.lookup_transform(source_frame, msg.header.stamp)
        if transform is None:
            return None

        points = point_cloud2.read_points(
            msg, field_names=['x', 'y', 'z'], skip_nans=True)
        if len(points) == 0:
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = self.map_frame
            return point_cloud2.create_cloud_xyz32(header, [])

        source_xyz = np.column_stack((
            np.asarray(points['x'], dtype=np.float32),
            np.asarray(points['y'], dtype=np.float32),
            np.asarray(points['z'], dtype=np.float32),
        ))
        finite = np.all(np.isfinite(source_xyz), axis=1)
        source_xyz = source_xyz[finite]
        map_xyz = self.transform_points(source_xyz, transform)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.map_frame
        return point_cloud2.create_cloud_xyz32(
            header, map_xyz.astype(float).tolist())

    def lookup_transform(self, source_frame, stamp_msg):
        timeout = Duration(seconds=self.tf_timeout_sec)
        stamp = Time.from_msg(stamp_msg)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, source_frame, stamp, timeout=timeout)
            self.latest_tf_success = True
            self.latest_tf_error = ''
            return transform
        except TransformException as exact_exc:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, source_frame, Time(), timeout=timeout)
                self.latest_tf_success = True
                self.latest_tf_error = str(exact_exc)
                self.get_logger().warn(
                    'FAST-LIO stub is using latest available TF for cloud '
                    f'registration because exact TF is not ready: {exact_exc}',
                    throttle_duration_sec=2.0)
                return transform
            except TransformException as latest_exc:
                self.latest_tf_success = False
                self.latest_tf_error = f'exact={exact_exc}; latest={latest_exc}'
                self.get_logger().warn(
                    'FAST-LIO stub skipped cloud registration: cannot transform '
                    f'{source_frame} -> {self.map_frame}: {self.latest_tf_error}',
                    throttle_duration_sec=2.0)
                return None

    def make_pose_stamped(self, stamp, pose):
        from geometry_msgs.msg import PoseStamped

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = stamp
        pose_stamped.header.frame_id = self.map_frame
        pose_stamped.pose = pose
        return pose_stamped

    def publish_debug(self):
        msg = String()
        msg.data = json.dumps({
            'mode': 'fast_lio_stub',
            'map_frame': self.map_frame,
            'has_odom': self.latest_odom is not None,
            'latest_cloud_frame': self.latest_cloud_frame,
            'latest_registered_frame': self.latest_registered_frame,
            'latest_cloud_points': int(self.latest_cloud_points),
            'transform_cloud_to_map': bool(self.transform_cloud_to_map),
            'tf_lookup_success': bool(self.latest_tf_success),
            'tf_error': self.latest_tf_error,
            'using_gazebo_odom': True,
            'dry_run': True,
        }, sort_keys=True)
        self.debug_pub.publish(msg)

    @staticmethod
    def transform_points(points_xyz, transform):
        rotation = FastLioStub.rotation_matrix(transform.transform.rotation)
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float64)
        return points_xyz.astype(np.float64) @ rotation.T + translation

    @staticmethod
    def rotation_matrix(q):
        x = float(q.x)
        y = float(q.y)
        z = float(q.z)
        w = float(q.w)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 0.0:
            return np.eye(3, dtype=np.float64)
        x /= norm
        y /= norm
        z /= norm
        w /= norm
        return np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ], dtype=np.float64)


def main(args=None):
    rclpy.init(args=args)
    node = FastLioStub()
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

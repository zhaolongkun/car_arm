#!/usr/bin/env python3
import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from version_car_sim.vehicle_geometry import (
    declare_vehicle_safety_parameters,
    read_vehicle_safety_geometry,
)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class D435iDepthObstacleMapper(Node):
    """Project D435i depth points into a rolling local obstacle grid."""

    def __init__(self):
        super().__init__('d435i_depth_obstacle_mapper')

        self.declare_parameter('point_cloud_topic', '/d435i/depth/points')
        self.declare_parameter('point_cloud_qos', 'reliable')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('obstacle_grid_topic', 'vision_obstacle_grid')
        self.declare_parameter('local_costmap_topic', 'vision_local_costmap')
        self.declare_parameter('obstacle_points_topic', 'vision_obstacle_points')
        self.declare_parameter('debug_topic', 'vehicle_safety_debug')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_timeout_sec', 0.10)
        self.declare_parameter('min_depth', 0.20)
        self.declare_parameter('max_depth', 6.0)
        self.declare_parameter('lateral_limit', 4.0)
        self.declare_parameter('min_obstacle_height', 0.06)
        self.declare_parameter('max_obstacle_height', 1.35)
        self.declare_parameter('grid_size_m', 12.0)
        self.declare_parameter('resolution', 0.12)
        self.declare_parameter('inflation_radius', 0.45)
        self.declare_parameter('inflation_cost', 70)
        self.declare_parameter('point_stride', 4)
        self.declare_parameter('max_obstacle_points', 45000)
        declare_vehicle_safety_parameters(self)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tf_timeout_sec = max(0.01, float(self.get_parameter('tf_timeout_sec').value))
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.lateral_limit = float(self.get_parameter('lateral_limit').value)
        self.min_obstacle_height = float(
            self.get_parameter('min_obstacle_height').value)
        self.max_obstacle_height = float(
            self.get_parameter('max_obstacle_height').value)
        self.grid_size_m = float(self.get_parameter('grid_size_m').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.safety_geometry = read_vehicle_safety_geometry(self)
        self.vehicle_safety_radius = self.safety_geometry['vehicle_safety_radius']
        configured_inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.inflation_radius = max(configured_inflation_radius, self.vehicle_safety_radius)
        self.inflation_cost = int(self.get_parameter('inflation_cost').value)
        self.point_stride = max(1, int(self.get_parameter('point_stride').value))
        self.max_obstacle_points = max(
            1, int(self.get_parameter('max_obstacle_points').value))
        self.point_cloud_qos = str(self.get_parameter('point_cloud_qos').value).lower()

        self.odom_pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_tf_lookup_mode = 'none'
        self.last_tf_error = ''

        self.grid_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('obstacle_grid_topic').value), 5)
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('local_costmap_topic').value), 5)
        self.points_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('obstacle_points_topic').value), 5)
        self.debug_pub = self.create_publisher(
            String, str(self.get_parameter('debug_topic').value), 10)

        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self.on_odom, 30)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('point_cloud_topic').value),
            self.on_points,
            self.make_point_cloud_qos(),
        )

        self.get_logger().info(
            'D435i depth mapper ready: /d435i/depth/points -> '
            '/vision_obstacle_grid, /vision_local_costmap, /vision_obstacle_points')
        self.get_logger().info(
            f'Point clouds are transformed with tf2 from their header.frame_id to '
            f'{self.base_frame} and {self.map_frame}; tf_timeout={self.tf_timeout_sec:.2f}s, '
            f'sensor range=[{self.min_depth:.2f}, {self.max_depth:.2f}] m, '
            f'obstacle height in {self.base_frame}=[{self.min_obstacle_height:.2f}, '
            f'{self.max_obstacle_height:.2f}] m, grid_size={self.grid_size_m:.1f} m')
        self.get_logger().info(
            f'Using vehicle safety circle from base_link wheel centers: '
            f'max_wheel_distance={self.safety_geometry["max_distance"]:.3f} m, '
            f'scale={self.safety_geometry["scale"]:.2f}, '
            f'vehicle_safety_radius={self.vehicle_safety_radius:.3f} m, '
            f'inflation_radius={self.inflation_radius:.3f} m')

    def make_point_cloud_qos(self):
        profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        if self.point_cloud_qos in ('best_effort', 'besteffort', 'sensor_data'):
            profile.reliability = ReliabilityPolicy.BEST_EFFORT
        return profile

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_pose = (float(p.x), float(p.y), float(yaw))

    def on_points(self, msg):
        if self.odom_pose is None:
            self.get_logger().warn(
                'Waiting for /odom before projecting D435i points.',
                throttle_duration_sec=2.0)
            return

        source_frame = msg.header.frame_id.strip()
        if not source_frame:
            self.get_logger().warn(
                'Skipping D435i point cloud because header.frame_id is empty.',
                throttle_duration_sec=2.0)
            self.publish_debug('', False, 'empty point cloud frame_id')
            return

        base_tf, map_tf = self.lookup_point_transforms(source_frame, msg.header.stamp)
        if base_tf is None or map_tf is None:
            return

        points = point_cloud2.read_points(
            msg, field_names=['x', 'y', 'z'], skip_nans=True)
        if len(points) == 0:
            self.publish_debug(source_frame, True, input_points=0, valid_points=0)
            self.publish_outputs(np.array([]), np.array([]), np.array([]))
            return

        points = points[::self.point_stride]
        source_xyz = np.column_stack((
            np.asarray(points['x'], dtype=np.float32),
            np.asarray(points['y'], dtype=np.float32),
            np.asarray(points['z'], dtype=np.float32),
        ))
        input_point_count = int(source_xyz.shape[0])
        finite_source = np.all(np.isfinite(source_xyz), axis=1)
        if not np.any(finite_source):
            self.publish_debug(
                source_frame, True, input_points=input_point_count, valid_points=0)
            self.publish_outputs(np.array([]), np.array([]), np.array([]))
            return

        source_xyz = source_xyz[finite_source]
        base_xyz = self.transform_points(source_xyz, base_tf)
        map_xyz = self.transform_points(source_xyz, map_tf)
        sensor_range = np.linalg.norm(source_xyz, axis=1)

        valid = (
            np.isfinite(sensor_range) &
            np.all(np.isfinite(base_xyz), axis=1) &
            np.all(np.isfinite(map_xyz), axis=1) &
            (sensor_range >= self.min_depth) &
            (sensor_range <= self.max_depth) &
            (np.abs(base_xyz[:, 1]) <= self.lateral_limit) &
            (base_xyz[:, 2] >= self.min_obstacle_height) &
            (base_xyz[:, 2] <= self.max_obstacle_height)
        )

        if not np.any(valid):
            self.publish_debug(
                source_frame, True, input_points=input_point_count, valid_points=0)
            self.publish_outputs(np.array([]), np.array([]), np.array([]))
            return

        base_valid = base_xyz[valid]
        map_valid = map_xyz[valid]
        map_x = map_valid[:, 0]
        map_y = map_valid[:, 1]
        obstacle_z = map_valid[:, 2]

        if map_x.size > self.max_obstacle_points:
            step = int(math.ceil(map_x.size / self.max_obstacle_points))
            map_x = map_x[::step]
            map_y = map_y[::step]
            obstacle_z = obstacle_z[::step]
            base_valid = base_valid[::step]

        self.publish_debug(
            source_frame, True, input_points=input_point_count,
            valid_points=int(map_x.size), base_xyz=base_valid)
        self.publish_outputs(map_x, map_y, obstacle_z)

    def lookup_point_transforms(self, source_frame, stamp_msg):
        stamp = Time.from_msg(stamp_msg)
        timeout = Duration(seconds=self.tf_timeout_sec)
        try:
            base_tf = self.tf_buffer.lookup_transform(
                self.base_frame, source_frame, stamp, timeout=timeout)
            map_tf = self.tf_buffer.lookup_transform(
                self.map_frame, source_frame, stamp, timeout=timeout)
            self.last_tf_lookup_mode = 'exact'
            self.last_tf_error = ''
            return base_tf, map_tf
        except TransformException as exact_exc:
            try:
                base_tf = self.tf_buffer.lookup_transform(
                    self.base_frame, source_frame, Time(), timeout=timeout)
                map_tf = self.tf_buffer.lookup_transform(
                    self.map_frame, source_frame, Time(), timeout=timeout)
                self.last_tf_lookup_mode = 'latest_fallback'
                self.last_tf_error = str(exact_exc)
                self.get_logger().warn(
                    'Using latest available TF for D435i point cloud because exact '
                    f'stamp {stamp_msg.sec}.{stamp_msg.nanosec:09d} is not in the '
                    f'buffer yet: {exact_exc}',
                    throttle_duration_sec=2.0)
                return base_tf, map_tf
            except TransformException as latest_exc:
                latest_error = str(latest_exc)
                self.last_tf_lookup_mode = 'failed'
                self.last_tf_error = latest_error
            error = (
                f'cannot transform D435i point cloud from {source_frame} to '
                f'{self.base_frame}/{self.map_frame} at stamp '
                f'{stamp_msg.sec}.{stamp_msg.nanosec:09d}: exact={exact_exc}; '
                f'latest={latest_error}')
            self.get_logger().warn(f'Skipping frame: {error}', throttle_duration_sec=2.0)
            self.publish_debug(source_frame, False, error)
            return None, None

    def publish_debug(
            self, source_frame, tf_success, error='', input_points=None,
            valid_points=None, base_xyz=None):
        front_min, left_min, right_min = self.compute_sector_distances(base_xyz)
        msg = String()
        msg.data = json.dumps({
            'source_point_frame': source_frame,
            'target_frame': self.base_frame,
            'map_frame': self.map_frame,
            'tf_lookup_success': bool(tf_success),
            'tf_lookup_mode': self.last_tf_lookup_mode,
            'tf_error': error,
            'tf_exact_error': self.last_tf_error,
            'vehicle_safety_radius': float(self.vehicle_safety_radius),
            'max_distance_from_center_to_wheel': float(
                self.safety_geometry['max_distance']),
            'inflation_radius': float(self.inflation_radius),
            'input_point_count': input_points,
            'valid_obstacle_point_count': valid_points,
            'front_min_obstacle_distance_base_link': self.debug_distance(front_min),
            'left_min_obstacle_distance_base_link': self.debug_distance(left_min),
            'right_min_obstacle_distance_base_link': self.debug_distance(right_min),
            'distance_to_safety_boundary': self.distance_to_safety_boundary(front_min),
            'using_vehicle_safety_radius': True,
        }, sort_keys=True)
        self.debug_pub.publish(msg)

    def compute_sector_distances(self, base_xyz):
        if base_xyz is None or len(base_xyz) == 0:
            return math.inf, math.inf, math.inf

        x_body = base_xyz[:, 0]
        y_body = base_xyz[:, 1]
        ahead = x_body > 0.05
        if not np.any(ahead):
            return math.inf, math.inf, math.inf

        x_body = x_body[ahead]
        y_body = y_body[ahead]
        distances = np.hypot(x_body, y_body)
        angles = np.arctan2(y_body, x_body)
        front_angle = math.radians(25.0)
        side_angle = math.radians(90.0)
        return (
            self.sector_min(distances, angles, -front_angle, front_angle),
            self.sector_min(distances, angles, front_angle, side_angle),
            self.sector_min(distances, angles, -side_angle, -front_angle),
        )

    @staticmethod
    def sector_min(distances, angles, min_angle, max_angle):
        mask = (angles >= min_angle) & (angles <= max_angle)
        if not np.any(mask):
            return math.inf
        return float(np.min(distances[mask]))

    @staticmethod
    def transform_points(points_xyz, transform):
        rotation = D435iDepthObstacleMapper.rotation_matrix(
            transform.transform.rotation)
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

    @staticmethod
    def debug_distance(value):
        return None if not math.isfinite(value) else float(value)

    def distance_to_safety_boundary(self, obstacle_distance):
        if not math.isfinite(obstacle_distance):
            return None
        return float(obstacle_distance - self.vehicle_safety_radius)

    def publish_outputs(self, map_x, map_y, obstacle_z):
        stamp = self.get_clock().now().to_msg()
        raw_grid, cost_grid, origin_x, origin_y = self.build_grids(map_x, map_y)

        self.grid_pub.publish(self.make_grid_msg(raw_grid, origin_x, origin_y, stamp))
        self.costmap_pub.publish(self.make_grid_msg(cost_grid, origin_x, origin_y, stamp))
        self.points_pub.publish(self.make_points_msg(map_x, map_y, obstacle_z, stamp))

    def build_grids(self, map_x, map_y):
        width = max(1, int(math.ceil(self.grid_size_m / self.resolution)))
        height = width
        robot_x, robot_y, _ = self.odom_pose
        origin_x = robot_x - 0.5 * self.grid_size_m
        origin_y = robot_y - 0.5 * self.grid_size_m
        raw_grid = np.zeros((height, width), dtype=np.int8)

        if map_x.size:
            cols = ((map_x - origin_x) / self.resolution).astype(np.int32)
            rows = ((map_y - origin_y) / self.resolution).astype(np.int32)
            inside = (
                (cols >= 0) & (cols < width) &
                (rows >= 0) & (rows < height)
            )
            raw_grid[rows[inside], cols[inside]] = 100

        cost_grid = self.inflate_grid(raw_grid)
        return raw_grid, cost_grid, origin_x, origin_y

    def inflate_grid(self, raw_grid):
        occupied = raw_grid >= 100
        if not np.any(occupied):
            return raw_grid.copy()

        inflation_cells = max(1, int(math.ceil(
            self.inflation_radius / self.resolution)))
        cost_grid = raw_grid.copy()
        for dy in range(-inflation_cells, inflation_cells + 1):
            for dx in range(-inflation_cells, inflation_cells + 1):
                distance_cells = math.hypot(dx, dy)
                if distance_cells > inflation_cells:
                    continue
                src_y0 = max(0, -dy)
                src_y1 = occupied.shape[0] - max(0, dy)
                src_x0 = max(0, -dx)
                src_x1 = occupied.shape[1] - max(0, dx)
                dst_y0 = max(0, dy)
                dst_y1 = occupied.shape[0] - max(0, -dy)
                dst_x0 = max(0, dx)
                dst_x1 = occupied.shape[1] - max(0, -dx)
                shifted = occupied[src_y0:src_y1, src_x0:src_x1]
                target = cost_grid[dst_y0:dst_y1, dst_x0:dst_x1]
                inflated_cost = 100 if distance_cells < 0.5 else self.inflation_cost
                np.maximum(target, shifted.astype(np.int8) * inflated_cost, out=target)
        return cost_grid

    def make_grid_msg(self, grid, origin_x, origin_y, stamp):
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.resolution
        msg.info.width = int(grid.shape[1])
        msg.info.height = int(grid.shape[0])
        msg.info.origin.position.x = float(origin_x)
        msg.info.origin.position.y = float(origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.reshape(-1).astype(np.int8).tolist()
        return msg

    def make_points_msg(self, map_x, map_y, obstacle_z, stamp):
        ros_header = self._make_header(stamp)
        if map_x.size == 0:
            return point_cloud2.create_cloud_xyz32(ros_header, [])

        points = zip(
            map_x.astype(float).tolist(),
            map_y.astype(float).tolist(),
            obstacle_z.astype(float).tolist(),
        )
        return point_cloud2.create_cloud_xyz32(ros_header, points)

    def _make_header(self, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = self.map_frame
        return header


def main(args=None):
    rclpy.init(args=args)
    node = D435iDepthObstacleMapper()
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

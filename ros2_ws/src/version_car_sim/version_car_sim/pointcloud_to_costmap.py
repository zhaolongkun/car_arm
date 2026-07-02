#!/usr/bin/env python3
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from version_car_sim.vehicle_geometry import (
    declare_vehicle_safety_parameters,
    read_vehicle_safety_geometry,
)


class PointCloudToCostmap(Node):
    """Project Livox/MID360 style 3D point clouds into a 2D planning map."""

    def __init__(self):
        super().__init__('pointcloud_to_costmap')

        self.declare_parameter('point_cloud_topic', 'cloud_registered')
        self.declare_parameter('point_cloud_qos', 'best_effort')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('costmap_topic', 'costmap_2d')
        self.declare_parameter('obstacle_points_topic', 'obstacle_points_2d')
        self.declare_parameter('self_filtered_points_topic', 'self_filtered_points_debug')
        self.declare_parameter('debug_topic', 'pointcloud_costmap_debug')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_map_to_odom_tf', True)
        self.declare_parameter('tf_timeout_sec', 0.10)
        self.declare_parameter('resolution', 0.12)
        self.declare_parameter('width_m', 120.0)
        self.declare_parameter('height_m', 100.0)
        self.declare_parameter('origin_x', -60.0)
        self.declare_parameter('origin_y', -50.0)
        self.declare_parameter('min_obstacle_height', 0.08)
        self.declare_parameter('max_obstacle_height', 1.60)
        self.declare_parameter('min_range', 0.20)
        self.declare_parameter('max_range', 40.0)
        self.declare_parameter('self_filter_front', 0.75)
        self.declare_parameter('self_filter_back', 0.65)
        self.declare_parameter('self_filter_half_width', 0.55)
        self.declare_parameter('self_filter_min_height', -0.20)
        self.declare_parameter('self_filter_max_height', 0.80)
        self.declare_parameter('self_filter_x_min', -0.70)
        self.declare_parameter('self_filter_x_max', 0.85)
        self.declare_parameter('self_filter_y_min', -0.50)
        self.declare_parameter('self_filter_y_max', 0.50)
        self.declare_parameter('self_filter_z_min', -0.20)
        self.declare_parameter('self_filter_z_max', 1.25)
        self.declare_parameter('front_debug_x_min', 0.05)
        self.declare_parameter('front_debug_x_max', 2.00)
        self.declare_parameter('front_debug_abs_y', 0.80)
        self.declare_parameter('front_debug_warn_distance', 1.20)
        self.declare_parameter('point_stride', 2)
        self.declare_parameter('max_obstacle_points', 90000)
        self.declare_parameter('max_debug_points', 20000)
        self.declare_parameter('persistent_map', True)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('hard_collision_radius', 0.0)
        self.declare_parameter('soft_inflation_radius', 0.0)
        self.declare_parameter('enable_custom_soft_inflation', False)
        self.declare_parameter('use_vehicle_radius_as_hard_collision', True)
        self.declare_parameter('boundary_inflation_radius', 0.0)
        declare_vehicle_safety_parameters(self)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tf_timeout_sec = max(0.01, float(self.get_parameter('tf_timeout_sec').value))
        self.resolution = max(0.02, float(self.get_parameter('resolution').value))
        self.width_m = max(self.resolution, float(self.get_parameter('width_m').value))
        self.height_m = max(self.resolution, float(self.get_parameter('height_m').value))
        self.width = max(1, int(math.ceil(self.width_m / self.resolution)))
        self.height = max(1, int(math.ceil(self.height_m / self.resolution)))
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.min_obstacle_height = float(self.get_parameter('min_obstacle_height').value)
        self.max_obstacle_height = float(self.get_parameter('max_obstacle_height').value)
        self.min_range = max(0.0, float(self.get_parameter('min_range').value))
        self.max_range = max(self.min_range, float(self.get_parameter('max_range').value))
        self.self_filter_x_min = float(self.get_parameter('self_filter_x_min').value)
        self.self_filter_x_max = float(self.get_parameter('self_filter_x_max').value)
        self.self_filter_y_min = float(self.get_parameter('self_filter_y_min').value)
        self.self_filter_y_max = float(self.get_parameter('self_filter_y_max').value)
        self.self_filter_z_min = float(self.get_parameter('self_filter_z_min').value)
        self.self_filter_z_max = float(self.get_parameter('self_filter_z_max').value)
        if self.self_filter_x_min > self.self_filter_x_max:
            self.self_filter_x_min, self.self_filter_x_max = (
                self.self_filter_x_max, self.self_filter_x_min)
        if self.self_filter_y_min > self.self_filter_y_max:
            self.self_filter_y_min, self.self_filter_y_max = (
                self.self_filter_y_max, self.self_filter_y_min)
        if self.self_filter_z_min > self.self_filter_z_max:
            self.self_filter_z_min, self.self_filter_z_max = (
                self.self_filter_z_max, self.self_filter_z_min)
        self.self_filter_front = max(0.0, self.self_filter_x_max)
        self.self_filter_back = max(0.0, -self.self_filter_x_min)
        self.self_filter_half_width = max(
            abs(self.self_filter_y_min), abs(self.self_filter_y_max))
        self.self_filter_min_height = self.self_filter_z_min
        self.self_filter_max_height = self.self_filter_z_max
        self.front_debug_x_min = float(self.get_parameter('front_debug_x_min').value)
        self.front_debug_x_max = float(self.get_parameter('front_debug_x_max').value)
        self.front_debug_abs_y = max(
            0.0, float(self.get_parameter('front_debug_abs_y').value))
        self.front_debug_warn_distance = max(
            0.0, float(self.get_parameter('front_debug_warn_distance').value))
        self.point_stride = max(1, int(self.get_parameter('point_stride').value))
        self.max_obstacle_points = max(1, int(self.get_parameter('max_obstacle_points').value))
        self.max_debug_points = max(1, int(self.get_parameter('max_debug_points').value))
        self.persistent_map = bool(self.get_parameter('persistent_map').value)
        self.publish_map_to_odom_tf = bool(
            self.get_parameter('publish_map_to_odom_tf').value)
        self.point_cloud_qos = str(self.get_parameter('point_cloud_qos').value).lower()

        self.safety_geometry = read_vehicle_safety_geometry(self)
        self.vehicle_safety_radius = float(self.safety_geometry['vehicle_safety_radius'])
        configured_hard_radius = float(self.get_parameter('hard_collision_radius').value)
        configured_soft_radius = float(self.get_parameter('soft_inflation_radius').value)
        self.enable_custom_soft_inflation = bool(
            self.get_parameter('enable_custom_soft_inflation').value)
        use_vehicle_hard_radius = bool(
            self.get_parameter('use_vehicle_radius_as_hard_collision').value)
        configured_boundary_radius = float(
            self.get_parameter('boundary_inflation_radius').value)
        if use_vehicle_hard_radius:
            self.hard_collision_radius = max(
                configured_hard_radius, self.vehicle_safety_radius)
        else:
            self.hard_collision_radius = max(0.0, configured_hard_radius)
        if self.enable_custom_soft_inflation:
            self.soft_inflation_radius = max(
                configured_soft_radius, self.hard_collision_radius)
        else:
            self.soft_inflation_radius = 0.0
        if configured_boundary_radius > 0.0:
            self.boundary_inflation_radius = configured_boundary_radius
        else:
            self.boundary_inflation_radius = self.vehicle_safety_radius + 0.10

        self.raw_grid = np.zeros((self.height, self.width), dtype=np.int8)
        self.latest_obstacle_points = np.empty((0, 3), dtype=np.float32)
        self.latest_self_filtered_points = np.empty((0, 3), dtype=np.float32)
        self.latest_input_points = 0
        self.latest_valid_points = 0
        self.latest_height_range_points = 0
        self.latest_self_filtered_count = 0
        self.latest_source_frame = ''
        self.latest_source_stamp = ''
        self.latest_tf_success = False
        self.latest_tf_error = ''
        self.latest_base_front_min = math.inf
        self.latest_base_left_min = math.inf
        self.latest_base_right_min = math.inf
        self.latest_front_debug_stats = self.empty_stats()
        self.latest_self_filter_stats = self.empty_stats()
        self.latest_cleared_robot_cells = 0
        self.map_publish_count = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf_pub = StaticTransformBroadcaster(self)
        if self.publish_map_to_odom_tf:
            self.publish_static_map_to_odom()

        self.map_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('map_topic').value), 1)
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('costmap_topic').value), 1)
        self.points_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('obstacle_points_topic').value), 5)
        self.self_filtered_points_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('self_filtered_points_topic').value), 5)
        self.debug_pub = self.create_publisher(
            String, str(self.get_parameter('debug_topic').value), 10)

        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('point_cloud_topic').value),
            self.on_points,
            self.make_point_cloud_qos(),
        )
        publish_rate = max(0.5, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / publish_rate, self.publish_outputs)

        self.get_logger().info(
            'PointCloud to costmap ready: '
            f'/{self.get_parameter("point_cloud_topic").value} -> '
            f'/{self.get_parameter("map_topic").value}, '
            f'/{self.get_parameter("costmap_topic").value}, '
            f'/{self.get_parameter("obstacle_points_topic").value}')
        self.get_logger().info(
            f'3D projection map: frame={self.map_frame}, size={self.width_m:.1f}x'
            f'{self.height_m:.1f} m, resolution={self.resolution:.3f} m, '
            f'origin=({self.origin_x:.2f}, {self.origin_y:.2f}), '
            f'obstacle height in {self.base_frame}=[{self.min_obstacle_height:.2f}, '
            f'{self.max_obstacle_height:.2f}] m')
        self.get_logger().info(
            f'Self point filter in {self.base_frame}: '
            f'x=[{self.self_filter_x_min:.2f}, {self.self_filter_x_max:.2f}] m, '
            f'y=[{self.self_filter_y_min:.2f}, {self.self_filter_y_max:.2f}] m, '
            f'z=[{self.self_filter_min_height:.2f}, '
            f'{self.self_filter_max_height:.2f}] m')
        self.get_logger().info(
            f'Using vehicle safety circle: max_wheel_distance='
            f'{self.safety_geometry["max_distance"]:.3f} m, '
            f'scale={self.safety_geometry["scale"]:.2f}, '
            f'vehicle_safety_radius={self.vehicle_safety_radius:.3f} m, '
            f'use_vehicle_radius_as_hard_collision={use_vehicle_hard_radius}, '
            f'hard_collision_radius={self.hard_collision_radius:.3f} m, '
            f'enable_custom_soft_inflation={self.enable_custom_soft_inflation}, '
            f'soft_inflation_radius={self.soft_inflation_radius:.3f} m, '
            f'boundary_inflation_radius={self.boundary_inflation_radius:.3f} m')

    def make_point_cloud_qos(self):
        if self.point_cloud_qos in ('best_effort', 'besteffort', 'sensor_data'):
            return qos_profile_sensor_data
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )

    def publish_static_map_to_odom(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.rotation.w = 1.0
        self.static_tf_pub.sendTransform(tf)

    def on_points(self, msg):
        source_frame = msg.header.frame_id.strip()
        self.latest_source_frame = source_frame
        self.latest_source_stamp = f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
        if not source_frame:
            self.latest_tf_success = False
            self.latest_tf_error = 'empty point cloud frame_id'
            self.get_logger().warn(
                'Skipping Livox point cloud because header.frame_id is empty.',
                throttle_duration_sec=2.0)
            return

        base_tf, map_tf = self.lookup_point_transforms(source_frame, msg.header.stamp)
        if base_tf is None or map_tf is None:
            return

        if not self.persistent_map:
            self.raw_grid.fill(0)

        points = point_cloud2.read_points(
            msg, field_names=['x', 'y', 'z'], skip_nans=True)
        if len(points) == 0:
            self.latest_input_points = 0
            self.latest_valid_points = 0
            self.latest_obstacle_points = np.empty((0, 3), dtype=np.float32)
            self.latest_self_filtered_points = np.empty((0, 3), dtype=np.float32)
            self.latest_self_filtered_count = 0
            self.latest_height_range_points = 0
            return

        points = points[::self.point_stride]
        source_xyz = np.column_stack((
            np.asarray(points['x'], dtype=np.float32),
            np.asarray(points['y'], dtype=np.float32),
            np.asarray(points['z'], dtype=np.float32),
        ))
        self.latest_input_points = int(source_xyz.shape[0])
        finite = np.all(np.isfinite(source_xyz), axis=1)
        if not np.any(finite):
            self.latest_valid_points = 0
            self.latest_obstacle_points = np.empty((0, 3), dtype=np.float32)
            self.latest_self_filtered_points = np.empty((0, 3), dtype=np.float32)
            self.latest_self_filtered_count = 0
            self.latest_height_range_points = 0
            return

        source_xyz = source_xyz[finite]
        base_xyz = self.transform_points(source_xyz, base_tf)
        map_xyz = self.transform_points(source_xyz, map_tf)
        ranges = np.linalg.norm(source_xyz, axis=1)
        height_range_valid = (
            np.isfinite(ranges) &
            np.all(np.isfinite(base_xyz), axis=1) &
            np.all(np.isfinite(map_xyz), axis=1) &
            (ranges >= self.min_range) &
            (ranges <= self.max_range) &
            (base_xyz[:, 2] >= self.min_obstacle_height) &
            (base_xyz[:, 2] <= self.max_obstacle_height)
        )
        self.latest_height_range_points = int(np.count_nonzero(height_range_valid))
        self_mask = np.zeros(base_xyz.shape[0], dtype=bool)
        self_mask[height_range_valid] = self.inside_self_filter(base_xyz[height_range_valid])
        valid = (
            height_range_valid &
            ~self_mask
        )
        self_points = base_xyz[self_mask]
        self.latest_self_filtered_count = int(self_points.shape[0])
        self.latest_self_filter_stats = self.xyz_stats(self_points)
        self.latest_self_filtered_points = self.downsample_debug_points(self_points)
        if not np.any(valid):
            self.latest_valid_points = 0
            self.latest_obstacle_points = np.empty((0, 3), dtype=np.float32)
            self.latest_base_front_min = math.inf
            self.latest_base_left_min = math.inf
            self.latest_base_right_min = math.inf
            self.latest_front_debug_stats = self.empty_stats()
            return

        base_valid = base_xyz[valid]
        map_valid = map_xyz[valid]
        if map_valid.shape[0] > self.max_obstacle_points:
            step = int(math.ceil(map_valid.shape[0] / self.max_obstacle_points))
            map_valid = map_valid[::step]
            base_valid = base_valid[::step]
        self.latest_valid_points = int(map_valid.shape[0])
        self.latest_obstacle_points = base_valid.astype(np.float32)
        self.latest_base_front_min, self.latest_base_left_min, self.latest_base_right_min = (
            self.compute_sector_distances(base_valid))
        self.latest_front_debug_stats = self.compute_front_debug_stats(base_valid)
        self.log_front_debug_if_needed()

        self.insert_points(map_valid)
        base_pose = self.lookup_base_pose_in_map(msg.header.stamp)
        if base_pose is not None:
            self.clear_robot_self_filter_area(*base_pose)

    def lookup_point_transforms(self, source_frame, stamp_msg):
        stamp = Time.from_msg(stamp_msg)
        timeout = Duration(seconds=self.tf_timeout_sec)
        try:
            base_tf = self.tf_buffer.lookup_transform(
                self.base_frame, source_frame, stamp, timeout=timeout)
            map_tf = self.tf_buffer.lookup_transform(
                self.map_frame, source_frame, stamp, timeout=timeout)
            self.latest_tf_success = True
            self.latest_tf_error = ''
            return base_tf, map_tf
        except TransformException as exact_exc:
            try:
                base_tf = self.tf_buffer.lookup_transform(
                    self.base_frame, source_frame, Time(), timeout=timeout)
                map_tf = self.tf_buffer.lookup_transform(
                    self.map_frame, source_frame, Time(), timeout=timeout)
                self.latest_tf_success = True
                self.latest_tf_error = str(exact_exc)
                self.get_logger().warn(
                    'Using latest available TF for Livox point cloud because exact '
                    f'stamp {stamp_msg.sec}.{stamp_msg.nanosec:09d} is not in the '
                    f'buffer yet: {exact_exc}',
                    throttle_duration_sec=2.0)
                return base_tf, map_tf
            except TransformException as latest_exc:
                self.latest_tf_success = False
                self.latest_tf_error = (
                    f'exact={exact_exc}; latest={latest_exc}')
                self.get_logger().warn(
                    'Skipping Livox point cloud: cannot transform from '
                    f'{source_frame} to {self.base_frame}/{self.map_frame}: '
                    f'{self.latest_tf_error}',
                    throttle_duration_sec=2.0)
                return None, None

    def insert_points(self, map_xyz):
        if map_xyz.size == 0:
            return
        cols = ((map_xyz[:, 0] - self.origin_x) / self.resolution).astype(np.int32)
        rows = ((map_xyz[:, 1] - self.origin_y) / self.resolution).astype(np.int32)
        inside = (
            (cols >= 0) & (cols < self.width) &
            (rows >= 0) & (rows < self.height)
        )
        if np.any(inside):
            self.raw_grid[rows[inside], cols[inside]] = 100

    def lookup_base_pose_in_map(self, stamp_msg):
        stamp = Time.from_msg(stamp_msg)
        timeout = Duration(seconds=self.tf_timeout_sec)
        for lookup_time in (stamp, Time()):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, lookup_time, timeout=timeout)
                return (
                    float(tf.transform.translation.x),
                    float(tf.transform.translation.y),
                    self.yaw_from_quaternion(tf.transform.rotation),
                )
            except TransformException:
                continue
        return None

    def clear_robot_self_filter_area(self, x, y, yaw):
        radius = max(
            self.vehicle_safety_radius + 0.20,
            self.hard_collision_radius,
            math.hypot(
                max(abs(self.self_filter_x_min), abs(self.self_filter_x_max)),
                max(abs(self.self_filter_y_min), abs(self.self_filter_y_max)),
            ),
        )
        center_col = int((x - self.origin_x) / self.resolution)
        center_row = int((y - self.origin_y) / self.resolution)
        radius_cells = max(1, int(math.ceil(radius / self.resolution)))
        row0 = max(0, center_row - radius_cells)
        row1 = min(self.height, center_row + radius_cells + 1)
        col0 = max(0, center_col - radius_cells)
        col1 = min(self.width, center_col + radius_cells + 1)
        if row0 >= row1 or col0 >= col1:
            self.latest_cleared_robot_cells = 0
            return

        rows, cols = np.ogrid[row0:row1, col0:col1]
        map_x = self.origin_x + (cols + 0.5) * self.resolution
        map_y = self.origin_y + (rows + 0.5) * self.resolution
        dx = map_x - x
        dy = map_y - y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        base_x = cos_yaw * dx + sin_yaw * dy
        base_y = -sin_yaw * dx + cos_yaw * dy
        mask = (
            (base_x >= self.self_filter_x_min) &
            (base_x <= self.self_filter_x_max) &
            (base_y >= self.self_filter_y_min) &
            (base_y <= self.self_filter_y_max)
        )
        patch = self.raw_grid[row0:row1, col0:col1]
        self.latest_cleared_robot_cells = int(np.count_nonzero(mask & (patch >= 100)))
        patch[mask] = 0

    def publish_outputs(self):
        stamp = self.get_clock().now().to_msg()
        raw_with_boundary = self.apply_boundary(self.raw_grid)
        costmap = self.build_costmap(raw_with_boundary)
        self.map_pub.publish(self.make_grid_msg(raw_with_boundary, stamp))
        self.costmap_pub.publish(self.make_grid_msg(costmap, stamp))
        self.points_pub.publish(self.make_points_msg(stamp))
        self.self_filtered_points_pub.publish(
            self.make_points_msg(stamp, self.latest_self_filtered_points))
        self.publish_debug()
        self.map_publish_count += 1

    def apply_boundary(self, grid):
        out = grid.copy()
        cells = max(1, int(math.ceil(self.boundary_inflation_radius / self.resolution)))
        cells = min(cells, max(1, min(out.shape) // 2))
        out[:cells, :] = 100
        out[-cells:, :] = 100
        out[:, :cells] = 100
        out[:, -cells:] = 100
        return out

    def build_costmap(self, raw_grid):
        if not self.enable_custom_soft_inflation:
            return raw_grid.copy()

        occupied = raw_grid >= 100
        if not np.any(occupied):
            return raw_grid.copy()

        costmap = np.zeros_like(raw_grid, dtype=np.int16)
        soft_cells = max(1, int(math.ceil(
            self.soft_inflation_radius / self.resolution)))
        for dy in range(-soft_cells, soft_cells + 1):
            for dx in range(-soft_cells, soft_cells + 1):
                distance_cells = math.hypot(dx, dy)
                if distance_cells > soft_cells:
                    continue
                distance_m = distance_cells * self.resolution
                if distance_m <= self.hard_collision_radius:
                    value = 100
                else:
                    span = max(0.01, self.soft_inflation_radius - self.hard_collision_radius)
                    value = int(round(85.0 * (1.0 - (distance_m - self.hard_collision_radius) / span)))
                    value = max(1, min(85, value))
                self.shift_max(costmap, occupied, dx, dy, value)
        return costmap.astype(np.int8)

    @staticmethod
    def shift_max(target_grid, source_mask, dx, dy, value):
        src_y0 = max(0, -dy)
        src_y1 = source_mask.shape[0] - max(0, dy)
        src_x0 = max(0, -dx)
        src_x1 = source_mask.shape[1] - max(0, dx)
        dst_y0 = max(0, dy)
        dst_y1 = source_mask.shape[0] - max(0, -dy)
        dst_x0 = max(0, dx)
        dst_x1 = source_mask.shape[1] - max(0, -dx)
        source = source_mask[src_y0:src_y1, src_x0:src_x1]
        target = target_grid[dst_y0:dst_y1, dst_x0:dst_x1]
        target[source] = np.maximum(target[source], value)

    def make_grid_msg(self, grid, stamp):
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.width)
        msg.info.height = int(self.height)
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.reshape(-1).astype(np.int8).tolist()
        return msg

    def make_points_msg(self, stamp, points=None):
        header = Header()
        header.stamp = stamp
        header.frame_id = self.base_frame
        if points is None:
            points = self.latest_obstacle_points
        if points.size == 0:
            return point_cloud2.create_cloud_xyz32(header, [])
        return point_cloud2.create_cloud_xyz32(
            header,
            points.astype(float).tolist(),
        )

    def publish_debug(self):
        occupied_cells = int(np.count_nonzero(self.raw_grid >= 100))
        msg = String()
        msg.data = json.dumps({
            'source_point_frame': self.latest_source_frame,
            'source_stamp': self.latest_source_stamp,
            'target_frame': self.base_frame,
            'map_frame': self.map_frame,
            'obstacle_points_frame': self.base_frame,
            'tf_lookup_success': bool(self.latest_tf_success),
            'tf_error': self.latest_tf_error,
            'input_point_count': int(self.latest_input_points),
            'height_range_candidate_count': int(self.latest_height_range_points),
            'self_filtered_point_count': int(self.latest_self_filtered_count),
            'valid_obstacle_point_count': int(self.latest_valid_points),
            'occupied_cell_count': occupied_cells,
            'cleared_robot_cells': int(self.latest_cleared_robot_cells),
            'map_width_m': float(self.width * self.resolution),
            'map_height_m': float(self.height * self.resolution),
            'map_origin_x': float(self.origin_x),
            'map_origin_y': float(self.origin_y),
            'vehicle_safety_radius': float(self.vehicle_safety_radius),
            'max_distance_from_center_to_wheel': float(
                self.safety_geometry['max_distance']),
            'hard_collision_radius': float(self.hard_collision_radius),
            'use_vehicle_radius_as_hard_collision': bool(
                self.get_parameter('use_vehicle_radius_as_hard_collision').value),
            'enable_custom_soft_inflation': bool(self.enable_custom_soft_inflation),
            'soft_inflation_radius': float(self.soft_inflation_radius),
            'boundary_inflation_radius': float(self.boundary_inflation_radius),
            'front_min_obstacle_distance_base_link': self.debug_distance(
                self.latest_base_front_min),
            'left_min_obstacle_distance_base_link': self.debug_distance(
                self.latest_base_left_min),
            'right_min_obstacle_distance_base_link': self.debug_distance(
                self.latest_base_right_min),
            'distance_to_safety_boundary': self.distance_to_safety_boundary(
                self.latest_base_front_min),
            'self_filter_box_base_link': {
                'x_min': float(self.self_filter_x_min),
                'x_max': float(self.self_filter_x_max),
                'y_min': float(self.self_filter_y_min),
                'y_max': float(self.self_filter_y_max),
                'z_min': float(self.self_filter_z_min),
                'z_max': float(self.self_filter_z_max),
            },
            'self_filtered_points_bounds_base_link': self.latest_self_filter_stats,
            'front_obstacle_bounds_base_link': self.latest_front_debug_stats,
            'using_vehicle_safety_radius': True,
            'persistent_map': bool(self.persistent_map),
            'map_publish_count': int(self.map_publish_count),
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

    def inside_self_filter(self, base_xyz):
        return (
            (base_xyz[:, 0] >= self.self_filter_x_min) &
            (base_xyz[:, 0] <= self.self_filter_x_max) &
            (base_xyz[:, 1] >= self.self_filter_y_min) &
            (base_xyz[:, 1] <= self.self_filter_y_max) &
            (base_xyz[:, 2] >= self.self_filter_z_min) &
            (base_xyz[:, 2] <= self.self_filter_z_max)
        )

    def compute_front_debug_stats(self, base_xyz):
        if base_xyz is None or len(base_xyz) == 0:
            return self.empty_stats()
        mask = (
            (base_xyz[:, 0] >= self.front_debug_x_min) &
            (base_xyz[:, 0] <= self.front_debug_x_max) &
            (np.abs(base_xyz[:, 1]) <= self.front_debug_abs_y)
        )
        return self.xyz_stats(base_xyz[mask])

    def log_front_debug_if_needed(self):
        stats = self.latest_front_debug_stats
        if int(stats['count']) <= 0:
            return
        front_min = self.latest_base_front_min
        if not math.isfinite(front_min) or front_min > self.front_debug_warn_distance:
            return
        self.get_logger().warn(
            'Front obstacle points after self-filter: '
            f'count={stats["count"]}, '
            f'x=[{stats["x_min"]:.3f}, {stats["x_max"]:.3f}], '
            f'y=[{stats["y_min"]:.3f}, {stats["y_max"]:.3f}], '
            f'z=[{stats["z_min"]:.3f}, {stats["z_max"]:.3f}], '
            f'source_frame={self.latest_source_frame}, '
            f'stamp={self.latest_source_stamp}',
            throttle_duration_sec=1.5)

    def downsample_debug_points(self, points):
        if points is None or len(points) == 0:
            return np.empty((0, 3), dtype=np.float32)
        if points.shape[0] <= self.max_debug_points:
            return points.astype(np.float32)
        step = int(math.ceil(points.shape[0] / self.max_debug_points))
        return points[::step].astype(np.float32)

    @staticmethod
    def empty_stats():
        return {
            'count': 0,
            'x_min': None,
            'x_max': None,
            'y_min': None,
            'y_max': None,
            'z_min': None,
            'z_max': None,
        }

    @staticmethod
    def xyz_stats(points):
        if points is None or len(points) == 0:
            return PointCloudToCostmap.empty_stats()
        return {
            'count': int(points.shape[0]),
            'x_min': float(np.min(points[:, 0])),
            'x_max': float(np.max(points[:, 0])),
            'y_min': float(np.min(points[:, 1])),
            'y_max': float(np.max(points[:, 1])),
            'z_min': float(np.min(points[:, 2])),
            'z_max': float(np.max(points[:, 2])),
        }

    @staticmethod
    def sector_min(distances, angles, min_angle, max_angle):
        mask = (angles >= min_angle) & (angles <= max_angle)
        if not np.any(mask):
            return math.inf
        return float(np.min(distances[mask]))

    @staticmethod
    def transform_points(points_xyz, transform):
        rotation = PointCloudToCostmap.rotation_matrix(transform.transform.rotation)
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
    def yaw_from_quaternion(q):
        x = float(q.x)
        y = float(q.y)
        z = float(q.z)
        w = float(q.w)
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    @staticmethod
    def debug_distance(value):
        return None if not math.isfinite(value) else float(value)

    def distance_to_safety_boundary(self, obstacle_distance):
        if not math.isfinite(obstacle_distance):
            return None
        return float(obstacle_distance - self.vehicle_safety_radius)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToCostmap()
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

#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OccupancyMapper(Node):
    """Small odometry-assisted occupancy-grid mapper for Gazebo bring-up."""

    def __init__(self):
        super().__init__('occupancy_mapper')

        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('resolution', 0.12)
        self.declare_parameter('width_m', 124.0)
        self.declare_parameter('height_m', 104.0)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('ray_stride', 2)
        self.declare_parameter('free_logodds', -0.35)
        self.declare_parameter('occupied_logodds', 0.85)
        self.declare_parameter('min_logodds', -4.0)
        self.declare_parameter('max_logodds', 4.0)
        self.declare_parameter('hit_margin', 0.10)
        self.declare_parameter('boundary_inflation_radius', 0.80)

        self.resolution = float(self.get_parameter('resolution').value)
        width_m = float(self.get_parameter('width_m').value)
        height_m = float(self.get_parameter('height_m').value)
        self.width = max(10, int(round(width_m / self.resolution)))
        self.height = max(10, int(round(height_m / self.resolution)))
        self.origin_x = -0.5 * self.width * self.resolution
        self.origin_y = -0.5 * self.height * self.resolution

        self.ray_stride = max(1, int(self.get_parameter('ray_stride').value))
        self.free_logodds = float(self.get_parameter('free_logodds').value)
        self.occupied_logodds = float(self.get_parameter('occupied_logodds').value)
        self.min_logodds = float(self.get_parameter('min_logodds').value)
        self.max_logodds = float(self.get_parameter('max_logodds').value)
        self.hit_margin = float(self.get_parameter('hit_margin').value)
        self.boundary_inflation_radius = max(
            0.0, float(self.get_parameter('boundary_inflation_radius').value))
        self.boundary_inflation_cells = max(
            1, int(math.ceil(self.boundary_inflation_radius / self.resolution)))

        self.logodds = np.zeros((self.height, self.width), dtype=np.float32)
        self.seen = np.zeros((self.height, self.width), dtype=np.bool_)
        self.pose = None

        self.map_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('map_topic').value), 1)
        self.static_tf_pub = StaticTransformBroadcaster(self)
        self.publish_static_map_tf()

        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self.on_odom, 20)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value), self.on_scan, 10)

        publish_rate = max(0.2, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / publish_rate, self.publish_map)

        self.get_logger().info(
            f'Occupancy mapper ready: {self.width}x{self.height} cells, '
            f'{self.resolution:.3f} m/cell, origin=({self.origin_x:.2f}, '
            f'{self.origin_y:.2f}), boundary inflation='
            f'{self.boundary_inflation_radius:.2f} m')

    def publish_static_map_tf(self):
        tf = TransformStamped()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.rotation.w = 1.0
        self.static_tf_pub.sendTransform(tf)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose = (p.x, p.y, yaw)

    def on_scan(self, msg):
        if self.pose is None:
            return

        start_cell = self.world_to_cell(self.pose[0], self.pose[1])
        if start_cell is None:
            return

        robot_x, robot_y, robot_yaw = self.pose
        max_range = float(msg.range_max)
        min_range = float(msg.range_min)

        for index in range(0, len(msg.ranges), self.ray_stride):
            distance = float(msg.ranges[index])
            if not math.isfinite(distance) or distance < min_range:
                continue

            is_hit = distance < max_range - self.hit_margin
            ray_distance = min(distance, max_range)
            angle = robot_yaw + msg.angle_min + index * msg.angle_increment
            end_x = robot_x + ray_distance * math.cos(angle)
            end_y = robot_y + ray_distance * math.sin(angle)
            end_cell = self.world_to_cell(end_x, end_y)
            if end_cell is None:
                end_cell = self.clip_world_to_cell(end_x, end_y)
                if end_cell is None:
                    continue

            self.update_ray(start_cell, end_cell, is_hit)

    def update_ray(self, start_cell, end_cell, is_hit):
        cells = self.bresenham(start_cell[0], start_cell[1], end_cell[0], end_cell[1])
        if not cells:
            return

        free_cells = cells[:-1] if is_hit else cells
        for col, row in free_cells:
            if self.in_bounds(col, row):
                self.logodds[row, col] = max(
                    self.min_logodds, self.logodds[row, col] + self.free_logodds)
                self.seen[row, col] = True

        if is_hit:
            col, row = cells[-1]
            if self.in_bounds(col, row):
                self.logodds[row, col] = min(
                    self.max_logodds, self.logodds[row, col] + self.occupied_logodds)
                self.seen[row, col] = True

    def publish_map(self):
        stamp = self.get_clock().now().to_msg()

        probabilities = 1.0 / (1.0 + np.exp(-self.logodds))
        values = np.clip(np.rint(probabilities * 100.0), 0, 100).astype(np.int16)
        grid = np.full((self.height, self.width), -1, dtype=np.int16)
        grid[self.seen] = values[self.seen]
        self.apply_boundary_obstacles(grid)

        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.reshape(-1).tolist()
        self.map_pub.publish(msg)

    def world_to_cell(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        if not self.in_bounds(col, row):
            return None
        return col, row

    def clip_world_to_cell(self, x, y):
        max_x = self.origin_x + self.width * self.resolution
        max_y = self.origin_y + self.height * self.resolution
        clipped_x = min(max(x, self.origin_x + 0.5 * self.resolution),
                        max_x - 0.5 * self.resolution)
        clipped_y = min(max(y, self.origin_y + 0.5 * self.resolution),
                        max_y - 0.5 * self.resolution)
        return self.world_to_cell(clipped_x, clipped_y)

    def apply_boundary_obstacles(self, grid):
        cells = min(
            self.boundary_inflation_cells,
            max(1, min(self.width, self.height) // 2),
        )
        grid[:cells, :] = 100
        grid[-cells:, :] = 100
        grid[:, :cells] = 100
        grid[:, -cells:] = 100

    def in_bounds(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height

    @staticmethod
    def bresenham(x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x = x0
        y = y0

        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

            if len(cells) > 10000:
                break

        return cells


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapper()
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

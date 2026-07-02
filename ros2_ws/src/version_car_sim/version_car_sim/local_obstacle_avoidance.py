#!/usr/bin/env python3
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


class LocalObstacleAvoidance(Node):
    """Reactive LaserScan safety layer for Gazebo-only local obstacle avoidance."""

    def __init__(self):
        super().__init__('local_obstacle_avoidance')

        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('input_cmd_topic', 'cmd_vel_raw')
        self.declare_parameter('output_cmd_topic', 'cmd_vel')
        self.declare_parameter('debug_topic', 'local_avoidance_debug')
        self.declare_parameter('safety_marker_topic', 'vehicle_safety_radius_marker')
        self.declare_parameter('goal_topic', 'goal_pose')
        self.declare_parameter('replan_request_topic', 'replan_requested')
        self.declare_parameter('slow_distance', 0.90)
        self.declare_parameter('stop_distance', 0.55)
        self.declare_parameter('emergency_stop_distance', 0.30)
        self.declare_parameter('vehicle_safety_radius', 0.0)
        self.declare_parameter('vehicle_half_width', 0.42)
        self.declare_parameter('footprint_clearance_margin', 0.10)
        self.declare_parameter('slow_margin', 0.35)
        self.declare_parameter('self_filter_distance', 0.42)
        self.declare_parameter('self_filter_front', 0.75)
        self.declare_parameter('self_filter_back', 0.65)
        self.declare_parameter('self_filter_half_width', 0.55)
        self.declare_parameter('boundary_margin', 0.25)
        self.declare_parameter('enforce_map_boundaries', True)
        self.declare_parameter('front_angle_deg', 25.0)
        self.declare_parameter('center_clear_angle_deg', 9.0)
        self.declare_parameter('side_angle_deg', 90.0)
        self.declare_parameter('rear_angle_deg', 25.0)
        self.declare_parameter('avoid_turn_speed', 0.5)
        self.declare_parameter('recovery_reverse_speed', 0.08)
        self.declare_parameter('reverse_recovery_delay_sec', 1.0)
        self.declare_parameter('edge_clearance_speed', 0.10)
        self.declare_parameter('edge_clearance_front_margin', 0.15)
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('narrow_passage_extra_clearance', 0.22)
        self.declare_parameter('narrow_passage_speed', 0.16)
        self.declare_parameter('narrow_passage_max_angular_speed', 0.12)
        self.declare_parameter('narrow_passage_centering_gain', 0.55)
        self.declare_parameter('narrow_passage_lateral_window', 1.60)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('scan_timeout', 0.5)
        self.declare_parameter('obstacle_points_topic', '')
        self.declare_parameter('obstacle_points_timeout', 1.0)
        self.declare_parameter('prefer_obstacle_points', True)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_continuous_avoidance_sec', 3.0)

        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.emergency_stop_distance = float(
            self.get_parameter('emergency_stop_distance').value)
        self.vehicle_safety_radius = max(
            0.0, float(self.get_parameter('vehicle_safety_radius').value))
        self.vehicle_half_width = max(
            0.0, float(self.get_parameter('vehicle_half_width').value))
        self.footprint_clearance_margin = max(
            0.0, float(self.get_parameter('footprint_clearance_margin').value))
        self.footprint_corridor_half_width = max(
            self.vehicle_safety_radius,
            self.vehicle_half_width + self.footprint_clearance_margin,
        )
        slow_margin = max(0.0, float(self.get_parameter('slow_margin').value))
        self.self_filter_distance = max(
            0.0, float(self.get_parameter('self_filter_distance').value))
        self.self_filter_front = max(
            0.0, float(self.get_parameter('self_filter_front').value))
        self.self_filter_back = max(
            0.0, float(self.get_parameter('self_filter_back').value))
        self.self_filter_half_width = max(
            0.0, float(self.get_parameter('self_filter_half_width').value))
        self.boundary_margin = max(
            0.0, float(self.get_parameter('boundary_margin').value))
        self.enforce_map_boundaries = bool(
            self.get_parameter('enforce_map_boundaries').value)
        if self.vehicle_safety_radius > 0.0:
            self.stop_distance = max(self.stop_distance, self.vehicle_safety_radius)
            self.slow_distance = max(
                self.slow_distance, self.vehicle_safety_radius + slow_margin)
            self.emergency_stop_distance = max(
                self.emergency_stop_distance, 0.5 * self.vehicle_safety_radius)
            self.emergency_stop_distance = min(
                self.emergency_stop_distance, max(0.05, self.stop_distance - 0.05))
        self.side_clearance_distance = self.vehicle_safety_radius + self.boundary_margin
        self.front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        self.center_clear_angle = math.radians(
            float(self.get_parameter('center_clear_angle_deg').value))
        self.side_angle = math.radians(float(self.get_parameter('side_angle_deg').value))
        self.rear_angle = math.radians(float(self.get_parameter('rear_angle_deg').value))
        self.avoid_turn_speed = float(self.get_parameter('avoid_turn_speed').value)
        self.recovery_reverse_speed = abs(
            float(self.get_parameter('recovery_reverse_speed').value))
        self.reverse_recovery_delay_sec = max(
            0.1, float(self.get_parameter('reverse_recovery_delay_sec').value))
        self.edge_clearance_speed = max(
            0.02, float(self.get_parameter('edge_clearance_speed').value))
        self.edge_clearance_front_margin = max(
            0.0, float(self.get_parameter('edge_clearance_front_margin').value))
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.narrow_passage_extra_clearance = max(
            0.0, float(self.get_parameter('narrow_passage_extra_clearance').value))
        self.narrow_passage_speed = max(
            0.02, float(self.get_parameter('narrow_passage_speed').value))
        self.narrow_passage_max_angular_speed = max(
            0.0, float(self.get_parameter('narrow_passage_max_angular_speed').value))
        self.narrow_passage_centering_gain = max(
            0.0, float(self.get_parameter('narrow_passage_centering_gain').value))
        self.narrow_passage_lateral_window = max(
            self.narrow_center_clearance(),
            float(self.get_parameter('narrow_passage_lateral_window').value),
        )
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.obstacle_points_topic = str(
            self.get_parameter('obstacle_points_topic').value)
        self.obstacle_points_timeout = float(
            self.get_parameter('obstacle_points_timeout').value)
        self.prefer_obstacle_points = bool(
            self.get_parameter('prefer_obstacle_points').value)
        self.max_continuous_avoidance_sec = max(
            0.5, float(self.get_parameter('max_continuous_avoidance_sec').value))

        self.latest_cmd = Twist()
        self.latest_cmd_time = None
        self.latest_scan_time = None
        self.latest_odom_time = None
        self.center_min = math.inf
        self.footprint_corridor_min = math.inf
        self.left_lateral_min = math.inf
        self.right_lateral_min = math.inf
        self.front_min = math.inf
        self.left_min = math.inf
        self.right_min = math.inf
        self.rear_min = math.inf
        self.map_info = None
        self.odom_pose = None
        self.goal = None
        self.last_mode = None
        self.avoidance_start_time = None
        self.close_obstacle_start_time = None
        self.last_replan_request_time = None
        self.replan_requested_now = False
        self.latest_obstacle_points_map = np.empty((0, 2), dtype=np.float32)
        self.latest_obstacle_points_time = None

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('output_cmd_topic').value), 10)
        self.debug_pub = self.create_publisher(
            String, str(self.get_parameter('debug_topic').value), 10)
        self.marker_pub = self.create_publisher(
            Marker, str(self.get_parameter('safety_marker_topic').value), 1)
        self.replan_pub = self.create_publisher(
            Bool, str(self.get_parameter('replan_request_topic').value), 5)

        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self.on_map,
            5,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.on_odom,
            20,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('input_cmd_topic').value),
            self.on_cmd,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self.on_goal,
            5,
        )

        if self.obstacle_points_topic:
            self.create_subscription(
                PointCloud2,
                self.obstacle_points_topic,
                self.on_obstacle_points,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                f'Will use obstacle_points_2d from /{self.obstacle_points_topic} '
                'as fallback obstacle source when /scan is unavailable.')

        publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / publish_rate, self.on_timer)

        self.get_logger().info(
            'Local obstacle avoidance ready: /scan + /cmd_vel_raw -> /cmd_vel')
        self.get_logger().info(
            f'Obstacle safety distances: emergency={self.emergency_stop_distance:.2f} m, '
            f'stop={self.stop_distance:.2f} m, slow={self.slow_distance:.2f} m, '
            f'vehicle_safety_radius={self.vehicle_safety_radius:.2f} m, '
            f'side_clearance={self.side_clearance_distance:.2f} m, '
            f'center_clearance={self.narrow_center_clearance():.2f} m, '
            f'footprint_corridor_half_width={self.footprint_corridor_half_width:.2f} m, '
            f'self_filter_distance={self.self_filter_distance:.2f} m, '
            f'self_filter_box=[-{self.self_filter_back:.2f}, '
            f'{self.self_filter_front:.2f}]x+/-{self.self_filter_half_width:.2f} m, '
            f'boundary_margin={self.boundary_margin:.2f} m, '
            f'reverse_recovery_delay={self.reverse_recovery_delay_sec:.2f} s, '
            f'edge_clearance_speed={self.edge_clearance_speed:.2f} m/s, '
            f'prefer_obstacle_points={self.prefer_obstacle_points}')

    def on_scan(self, msg):
        self.update_footprint_clearance_from_scan(msg)
        self.center_min = self.sector_min(
            msg, -self.center_clear_angle, self.center_clear_angle)
        self.front_min = self.sector_min(msg, -self.front_angle, self.front_angle)
        self.left_min = self.sector_min(msg, self.front_angle, self.side_angle)
        self.right_min = self.sector_min(msg, -self.side_angle, -self.front_angle)
        self.rear_min = min(
            self.sector_min(msg, math.pi - self.rear_angle, math.pi),
            self.sector_min(msg, -math.pi, -math.pi + self.rear_angle),
        )
        self.latest_scan_time = self.now_seconds()

    def on_cmd(self, msg):
        self.latest_cmd = msg
        self.latest_cmd_time = self.now_seconds()

    def on_goal(self, msg):
        self.goal = (float(msg.pose.position.x), float(msg.pose.position.y))

    def on_obstacle_points(self, msg):
        """Store 2D obstacle points (from pointcloud_to_costmap projection) for sector distance computation."""
        points = point_cloud2.read_points(
            msg, field_names=['x', 'y'], skip_nans=True)
        pts = [(float(p[0]), float(p[1])) for p in points]
        if not pts:
            self.latest_obstacle_points_map = np.empty((0, 2), dtype=np.float32)
        else:
            self.latest_obstacle_points_map = np.array(pts, dtype=np.float32)
        self.latest_obstacle_points_time = self.now_seconds()

    def on_map(self, msg):
        self.map_info = msg.info

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_pose = (float(p.x), float(p.y), float(yaw))
        self.latest_odom_time = self.now_seconds()

    def on_timer(self):
        now = self.now_seconds()
        if self.latest_cmd_time is None or now - self.latest_cmd_time > self.cmd_timeout:
            self.publish_limited(Twist(), 'NO_CMD')
            return

        scan_available = (
            self.latest_scan_time is not None and
            now - self.latest_scan_time <= self.scan_timeout
        )
        obstacle_points_available = (
            self.obstacle_points_topic and
            self.latest_obstacle_points_time is not None and
            now - self.latest_obstacle_points_time <= self.obstacle_points_timeout
        )

        if not scan_available and not obstacle_points_available:
            self.publish_limited(Twist(), 'NO_SENSOR')
            return

        # MID360 obstacle points are height-filtered 3D returns, so prefer them when available.
        if obstacle_points_available and (self.prefer_obstacle_points or not scan_available):
            self.compute_distances_from_obstacle_points()
        elif not scan_available:
            self.publish_limited(Twist(), 'NO_SENSOR')
            return

        cmd, mode = self.apply_avoidance(self.latest_cmd)
        self.publish_limited(cmd, mode)

    def reset_footprint_clearance(self):
        self.footprint_corridor_min = math.inf
        self.left_lateral_min = math.inf
        self.right_lateral_min = math.inf

    def update_footprint_clearance_from_scan(self, msg):
        self.reset_footprint_clearance()
        angle = msg.angle_min
        for distance in msg.ranges:
            if self.valid_range(distance, msg) and float(distance) >= self.self_filter_distance:
                local_x = float(distance) * math.cos(angle)
                local_y = float(distance) * math.sin(angle)
                self.update_footprint_clearance_from_point(local_x, local_y)
            angle += msg.angle_increment

    def update_footprint_clearance_from_point(self, local_x, local_y):
        if self.inside_self_filter(local_x, local_y):
            return
        if local_x < -0.10 or local_x > self.narrow_passage_lateral_window:
            return

        lateral_abs = abs(local_y)
        if local_x >= 0.0 and lateral_abs <= self.footprint_corridor_half_width:
            self.footprint_corridor_min = min(self.footprint_corridor_min, local_x)

        if local_y > 0.0:
            self.left_lateral_min = min(self.left_lateral_min, local_y)
        elif local_y < 0.0:
            self.right_lateral_min = min(self.right_lateral_min, -local_y)

    def compute_distances_from_obstacle_points(self):
        """Compute sector obstacle distances from 2D obstacle points in map frame.

        Uses current odom pose to transform points into the robot body frame,
        then bins them into front/left/right/rear sectors.
        """
        self.reset_footprint_clearance()
        if self.odom_pose is None or self.latest_obstacle_points_map.size == 0:
            self.center_min = math.inf
            self.front_min = math.inf
            self.left_min = math.inf
            self.right_min = math.inf
            self.rear_min = math.inf
            return

        x, y, yaw = self.odom_pose
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        pts = self.latest_obstacle_points_map

        # Translate to robot position, then rotate into body frame.
        dx = pts[:, 0] - x
        dy = pts[:, 1] - y
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw

        distances = np.hypot(local_x, local_y)
        angles = np.arctan2(local_y, local_x)

        # Filter out self-points near the robot and inside the vehicle footprint.
        valid = (
            (distances >= self.self_filter_distance) &
            ~self.inside_self_filter_array(local_x, local_y)
        )
        if not np.any(valid):
            self.center_min = math.inf
            self.front_min = math.inf
            self.left_min = math.inf
            self.right_min = math.inf
            self.rear_min = math.inf
            return

        distances = distances[valid]
        angles = angles[valid]
        local_x = local_x[valid]
        local_y = local_y[valid]

        def sector_min(dist_arr, angle_arr, min_a, max_a):
            mask = (angle_arr >= min_a) & (angle_arr <= max_a)
            if not np.any(mask):
                return math.inf
            return float(np.min(dist_arr[mask]))

        self.center_min = sector_min(
            distances, angles, -self.center_clear_angle, self.center_clear_angle)
        self.front_min = sector_min(distances, angles, -self.front_angle, self.front_angle)
        self.left_min = sector_min(distances, angles, self.front_angle, self.side_angle)
        self.right_min = sector_min(distances, angles, -self.side_angle, -self.front_angle)
        self.rear_min = min(
            sector_min(distances, angles, math.pi - self.rear_angle, math.pi),
            sector_min(distances, angles, -math.pi, -math.pi + self.rear_angle),
        )
        for x_value, y_value in zip(local_x, local_y):
            self.update_footprint_clearance_from_point(float(x_value), float(y_value))

    def inside_self_filter(self, local_x, local_y):
        return (
            -self.self_filter_back <= local_x <= self.self_filter_front and
            abs(local_y) <= self.self_filter_half_width
        )

    def inside_self_filter_array(self, local_x, local_y):
        return (
            (local_x >= -self.self_filter_back) &
            (local_x <= self.self_filter_front) &
            (np.abs(local_y) <= self.self_filter_half_width)
        )

    def apply_avoidance(self, raw):
        boundary_cmd, boundary_mode = self.apply_boundary_guard(raw)
        if boundary_mode in ('OUT_OF_MAP', 'BOUNDARY_RECOVERY'):
            return boundary_cmd, boundary_mode

        raw_is_stop = abs(raw.linear.x) < 1e-4 and abs(raw.angular.z) < 1e-4
        if raw_is_stop:
            self.update_close_obstacle_timer(False)
            return Twist(), 'HOLD_POSITION'

        forward_min = self.forward_clearance_min()
        self.update_close_obstacle_timer(forward_min <= self.stop_distance)
        if forward_min <= self.emergency_stop_distance:
            direction = self.choose_turn_direction(raw)
            if direction is not None and self.reverse_recovery_is_allowed(
                    direction, self.reverse_recovery_delay_sec):
                return self.make_reverse_recovery_cmd(direction), 'REVERSE_RECOVERY'
            return Twist(), 'EMERGENCY_STOP'

        if forward_min <= self.stop_distance:
            edge_direction = self.choose_edge_clearance_direction(raw)
            if edge_direction is not None and self.edge_clearance_is_allowed():
                return self.make_edge_clearance_cmd(raw, edge_direction), 'EDGE_CLEARANCE'

            direction = self.choose_turn_direction(raw)
            if direction is None:
                return Twist(), 'BLOCKED'
            if self.reverse_recovery_is_allowed(direction, self.reverse_recovery_delay_sec):
                return self.make_reverse_recovery_cmd(direction), 'REVERSE_RECOVERY'
            cmd = Twist()
            if direction > 0.0:
                cmd.angular.z = self.avoid_turn_speed
                return cmd, 'TURN_LEFT'
            cmd.angular.z = -self.avoid_turn_speed
            return cmd, 'TURN_RIGHT'

        cmd = self.copy_twist(boundary_cmd if boundary_mode == 'NEAR_MAP_BOUNDARY' else raw)
        side_cmd, side_mode = self.apply_side_clearance(cmd)
        if side_mode is not None:
            if side_mode == 'BLOCKED':
                direction = self.choose_turn_direction(raw)
                if direction is not None and self.reverse_recovery_is_allowed(
                        direction, self.reverse_recovery_delay_sec):
                    return self.make_reverse_recovery_cmd(direction), 'REVERSE_RECOVERY'
            return side_cmd, side_mode

        if self.front_min <= self.slow_distance:
            if self.center_path_is_clear():
                if cmd.linear.x > self.narrow_passage_speed:
                    cmd.linear.x = self.narrow_passage_speed
                cmd.angular.z = self.narrow_passage_angular(cmd.angular.z)
                return cmd, 'CENTER_CLEAR_SLOW'

            denom = max(0.01, self.slow_distance - self.stop_distance)
            scale = clamp((self.front_min - self.stop_distance) / denom, 0.0, 1.0)
            if cmd.linear.x > 0.0:
                cmd.linear.x *= scale
            # Steer toward the clearer side while slowing down;
            # turn strength grows as the obstacle gets closer.
            turn_strength = self.avoid_turn_speed * (1.0 - scale) * 0.5
            direction = self.choose_turn_direction(raw)
            if direction is not None:
                cmd.angular.z = direction * turn_strength
            return cmd, 'SLOW_DOWN'

        if boundary_mode == 'NEAR_MAP_BOUNDARY':
            return cmd, boundary_mode
        return cmd, 'PASS_THROUGH'

    def apply_side_clearance(self, cmd):
        left_close = self.left_min <= self.side_clearance_distance
        right_close = self.right_min <= self.side_clearance_distance
        if not left_close and not right_close:
            return cmd, None

        adjusted = self.copy_twist(cmd)
        center_clear = self.center_path_is_clear()
        if adjusted.linear.x > 0.0:
            if center_clear:
                adjusted.linear.x = min(adjusted.linear.x, self.narrow_passage_speed)
            else:
                adjusted.linear.x *= 0.25

        if left_close and right_close:
            if center_clear:
                adjusted.angular.z = self.narrow_passage_angular(adjusted.angular.z)
                return adjusted, 'NARROW_PASSAGE'

            adjusted.linear.x = 0.0
            direction = self.choose_turn_direction(cmd)
            if direction is None:
                return Twist(), 'BLOCKED'
            if direction > 0.0:
                adjusted.angular.z = self.avoid_turn_speed
                return adjusted, 'TURN_LEFT'
            adjusted.angular.z = -self.avoid_turn_speed
            return adjusted, 'TURN_RIGHT'

        if left_close:
            if center_clear:
                adjusted.angular.z = self.narrow_passage_angular(
                    adjusted.angular.z - 0.35 * self.avoid_turn_speed)
                return adjusted, 'SIDE_CLEARANCE'
            if self.turn_is_safe(-1.0):
                adjusted.angular.z = -self.avoid_turn_speed
                return adjusted, 'TURN_RIGHT'
            return Twist(), 'BLOCKED'

        if center_clear:
            adjusted.angular.z = self.narrow_passage_angular(
                adjusted.angular.z + 0.35 * self.avoid_turn_speed)
            return adjusted, 'SIDE_CLEARANCE'

        if self.turn_is_safe(1.0):
            adjusted.angular.z = self.avoid_turn_speed
            return adjusted, 'TURN_LEFT'
        return Twist(), 'BLOCKED'

    def narrow_center_clearance(self):
        return max(
            self.stop_distance + self.narrow_passage_extra_clearance,
            self.vehicle_safety_radius + self.narrow_passage_extra_clearance,
        )

    def forward_clearance_min(self):
        return min(self.center_min, self.footprint_corridor_min)

    def center_path_is_clear(self):
        return self.forward_clearance_min() > self.narrow_center_clearance()

    def centerline_is_clear(self):
        return (
            self.center_min > self.narrow_center_clearance() and
            self.front_min > self.stop_distance + self.edge_clearance_front_margin
        )

    def edge_clearance_is_allowed(self):
        left_close = self.left_lateral_min <= self.side_clearance_distance
        right_close = self.right_lateral_min <= self.side_clearance_distance
        return self.centerline_is_clear() and left_close != right_close

    def narrow_passage_angular(self, base_angular):
        correction = 0.0
        left_known = math.isfinite(self.left_lateral_min)
        right_known = math.isfinite(self.right_lateral_min)
        if left_known and right_known:
            correction = self.narrow_passage_centering_gain * (
                self.left_lateral_min - self.right_lateral_min)
        elif left_known and self.left_lateral_min < self.side_clearance_distance:
            correction = self.narrow_passage_centering_gain * (
                self.left_lateral_min - self.side_clearance_distance)
        elif right_known and self.right_lateral_min < self.side_clearance_distance:
            correction = self.narrow_passage_centering_gain * (
                self.side_clearance_distance - self.right_lateral_min)

        return clamp(
            base_angular + correction,
            -self.narrow_passage_max_angular_speed,
            self.narrow_passage_max_angular_speed,
        )

    def make_reverse_recovery_cmd(self, direction):
        cmd = Twist()
        cmd.linear.x = -self.recovery_reverse_speed
        cmd.angular.z = direction * 0.6 * self.avoid_turn_speed
        return cmd

    def make_edge_clearance_cmd(self, raw, direction):
        cmd = self.copy_twist(raw)
        requested_speed = cmd.linear.x if cmd.linear.x > 0.0 else self.edge_clearance_speed
        cmd.linear.x = min(requested_speed, self.edge_clearance_speed)
        cmd.angular.z = direction * self.avoid_turn_speed
        return cmd

    def choose_edge_clearance_direction(self, raw):
        left_close = self.left_lateral_min <= self.side_clearance_distance
        right_close = self.right_lateral_min <= self.side_clearance_distance
        if right_close and not left_close and self.turn_is_safe(1.0):
            return 1.0
        if left_close and not right_close and self.turn_is_safe(-1.0):
            return -1.0
        return self.choose_turn_direction(raw)

    def choose_turn_direction(self, raw):
        candidates = []
        if self.turn_is_safe(1.0):
            candidates.append(1.0)
        if self.turn_is_safe(-1.0):
            candidates.append(-1.0)
        if not candidates:
            return None

        raw_direction = 0.0
        if abs(raw.angular.z) > 0.03:
            raw_direction = 1.0 if raw.angular.z > 0.0 else -1.0
        if raw_direction in candidates and self.goal_direction_is_reasonable(raw_direction):
            return raw_direction

        reasonable = [
            direction for direction in candidates
            if self.goal_direction_is_reasonable(direction)
        ]
        if reasonable:
            candidates = reasonable

        if len(candidates) == 1:
            return candidates[0]
        if self.left_min > self.right_min:
            return 1.0 if 1.0 in candidates else candidates[0]
        return -1.0 if -1.0 in candidates else candidates[0]

    def goal_direction_is_reasonable(self, direction):
        if self.goal is None or self.odom_pose is None:
            return True
        x, y, yaw = self.odom_pose
        goal_yaw = math.atan2(self.goal[1] - y, self.goal[0] - x)
        current_error = abs(angle_diff(goal_yaw, yaw))
        candidate_error = abs(angle_diff(goal_yaw, yaw + direction * 0.8))
        if candidate_error <= current_error + 0.35:
            return True
        return candidate_error < math.radians(115.0)

    def publish_limited(self, cmd, mode):
        limited = self.copy_twist(cmd)
        limited.linear.x = clamp(
            limited.linear.x, -self.max_linear_speed, self.max_linear_speed)
        limited.linear.y = 0.0
        limited.linear.z = 0.0
        limited.angular.x = 0.0
        limited.angular.y = 0.0
        limited.angular.z = clamp(
            limited.angular.z, -self.max_angular_speed, self.max_angular_speed)

        if mode != self.last_mode:
            self.last_mode = mode
            self.get_logger().info(
                f'mode={mode}, forward={self.forward_clearance_min():.2f}, '
                f'center={self.center_min:.2f}, footprint={self.footprint_corridor_min:.2f}, '
                f'front={self.front_min:.2f}, '
                f'left={self.left_min:.2f}, right={self.right_min:.2f}, '
                f'boundary={self.format_distance(self.nearest_map_boundary_distance())}')

        self.update_avoidance_duration(mode)
        self.cmd_pub.publish(limited)
        self.publish_safety_marker(mode)
        self.publish_debug(mode, limited)

    def update_avoidance_duration(self, mode):
        now = self.now_seconds()
        self.replan_requested_now = False
        if mode in (
            'SLOW_DOWN', 'TURN_LEFT', 'TURN_RIGHT', 'BLOCKED',
            'EMERGENCY_STOP', 'NEAR_MAP_BOUNDARY', 'BOUNDARY_RECOVERY',
            'REVERSE_RECOVERY', 'EDGE_CLEARANCE',
        ):
            if self.avoidance_start_time is None:
                self.avoidance_start_time = now
            duration = now - self.avoidance_start_time
            recently_requested = (
                self.last_replan_request_time is not None and
                now - self.last_replan_request_time < self.max_continuous_avoidance_sec)
            if duration >= self.max_continuous_avoidance_sec and not recently_requested:
                msg = Bool()
                msg.data = True
                self.replan_pub.publish(msg)
                self.last_replan_request_time = now
                self.replan_requested_now = True
                self.get_logger().info(
                    'Continuous local avoidance exceeded '
                    f'{self.max_continuous_avoidance_sec:.1f}s; requested replanning.')
            return

        self.avoidance_start_time = None

    def update_close_obstacle_timer(self, close_now):
        now = self.now_seconds()
        if close_now:
            if self.close_obstacle_start_time is None:
                self.close_obstacle_start_time = now
            return
        self.close_obstacle_start_time = None

    def sector_min(self, msg, min_angle, max_angle):
        best = math.inf
        angle = msg.angle_min
        for distance in msg.ranges:
            if (
                min_angle <= angle <= max_angle and
                self.valid_range(distance, msg) and
                float(distance) >= self.self_filter_distance
            ):
                best = min(best, float(distance))
            angle += msg.angle_increment
        return best

    @staticmethod
    def valid_range(distance, msg):
        if not math.isfinite(distance):
            return False
        return msg.range_min <= distance <= msg.range_max

    def reverse_recovery_is_allowed(self, direction, required_duration=None):
        if required_duration is None:
            required_duration = self.max_continuous_avoidance_sec
        now = self.now_seconds()
        avoidance_duration = (
            now - self.avoidance_start_time
            if self.avoidance_start_time is not None else 0.0
        )
        close_duration = (
            now - self.close_obstacle_start_time
            if self.close_obstacle_start_time is not None else 0.0
        )
        if max(avoidance_duration, close_duration) < required_duration:
            return False
        if self.odom_pose is None or self.map_info is None:
            return False
        required_rear_clearance = self.vehicle_safety_radius + self.boundary_margin + 0.25
        if self.rear_min <= required_rear_clearance:
            return False

        x, y, yaw = self.odom_pose
        probe_distance = 0.35
        probe_x = x - math.cos(yaw) * probe_distance
        probe_y = y - math.sin(yaw) * probe_distance
        if not self.point_inside_map(
                probe_x, probe_y, self.vehicle_safety_radius + self.boundary_margin):
            return False

        turn_probe_yaw = yaw + direction * 0.4
        turn_probe_x = x - math.cos(turn_probe_yaw) * probe_distance
        turn_probe_y = y - math.sin(turn_probe_yaw) * probe_distance
        return self.point_inside_map(
            turn_probe_x, turn_probe_y, self.vehicle_safety_radius + self.boundary_margin)

    def apply_boundary_guard(self, raw):
        cmd = self.copy_twist(raw)
        if not self.enforce_map_boundaries or self.map_info is None or self.odom_pose is None:
            return cmd, None

        x, y, yaw = self.odom_pose
        boundary_distance = self.nearest_map_boundary_distance(x, y)
        if boundary_distance is None:
            return cmd, None
        if boundary_distance < 0.0:
            return Twist(), 'OUT_OF_MAP'

        limit = self.vehicle_safety_radius + self.boundary_margin
        if boundary_distance > limit:
            return cmd, None

        future_x = x + math.cos(yaw) * max(0.0, cmd.linear.x) * 0.5
        future_y = y + math.sin(yaw) * max(0.0, cmd.linear.x) * 0.5
        future_distance = self.nearest_map_boundary_distance(future_x, future_y)
        moving_toward_boundary = (
            future_distance is not None and future_distance < boundary_distance + 0.02
        )

        if moving_toward_boundary:
            recovery = Twist()
            recovery.angular.z = self.turn_toward_map_center(x, y, yaw)
            return recovery, 'BOUNDARY_RECOVERY'

        cmd.linear.x = min(cmd.linear.x, 0.25 * self.max_linear_speed)
        return cmd, 'NEAR_MAP_BOUNDARY'

    def map_bounds(self):
        if self.map_info is None:
            return None
        min_x = self.map_info.origin.position.x
        min_y = self.map_info.origin.position.y
        max_x = min_x + self.map_info.width * self.map_info.resolution
        max_y = min_y + self.map_info.height * self.map_info.resolution
        return min_x, min_y, max_x, max_y

    def point_inside_map(self, x, y, margin=0.0):
        bounds = self.map_bounds()
        if bounds is None:
            return False
        min_x, min_y, max_x, max_y = bounds
        return (
            min_x + margin <= x <= max_x - margin and
            min_y + margin <= y <= max_y - margin
        )

    def nearest_map_boundary_distance(self, x=None, y=None):
        bounds = self.map_bounds()
        if bounds is None:
            return None
        if x is None or y is None:
            if self.odom_pose is None:
                return None
            x, y = self.odom_pose[0], self.odom_pose[1]
        min_x, min_y, max_x, max_y = bounds
        if min_x <= x <= max_x and min_y <= y <= max_y:
            return min(x - min_x, max_x - x, y - min_y, max_y - y)
        outside_x = max(min_x - x, 0.0, x - max_x)
        outside_y = max(min_y - y, 0.0, y - max_y)
        return -math.hypot(outside_x, outside_y)

    def turn_is_safe(self, direction):
        if self.map_info is None or self.odom_pose is None:
            return True
        x, y, yaw = self.odom_pose
        candidate_yaw = yaw + direction * 0.8
        probe_distance = self.vehicle_safety_radius + self.boundary_margin + 0.7
        probe_x = x + math.cos(candidate_yaw) * probe_distance
        probe_y = y + math.sin(candidate_yaw) * probe_distance
        return self.point_inside_map(
            probe_x, probe_y, self.vehicle_safety_radius + self.boundary_margin)

    def turn_toward_map_center(self, x, y, yaw):
        bounds = self.map_bounds()
        if bounds is None:
            return 0.0
        min_x, min_y, max_x, max_y = bounds
        target_yaw = math.atan2(0.5 * (min_y + max_y) - y,
                               0.5 * (min_x + max_x) - x)
        error = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
        return clamp(error, -self.max_angular_speed, self.max_angular_speed)

    def publish_safety_marker(self, mode):
        if self.vehicle_safety_radius <= 0.0:
            return
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'base_link'
        marker.ns = 'vehicle_safety_radius'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.z = 0.025
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0 * self.vehicle_safety_radius
        marker.scale.y = 2.0 * self.vehicle_safety_radius
        marker.scale.z = 0.05
        if mode in (
            'OUT_OF_MAP', 'BOUNDARY_RECOVERY', 'EMERGENCY_STOP', 'BLOCKED',
            'REVERSE_RECOVERY',
        ):
            marker.color.r = 1.0
            marker.color.g = 0.10
            marker.color.b = 0.02
            marker.color.a = 0.30
        elif mode == 'NEAR_MAP_BOUNDARY':
            marker.color.r = 1.0
            marker.color.g = 0.70
            marker.color.b = 0.05
            marker.color.a = 0.25
        else:
            marker.color.r = 0.10
            marker.color.g = 0.85
            marker.color.b = 0.35
            marker.color.a = 0.18
        self.marker_pub.publish(marker)

    def publish_debug(self, mode, final):
        bounds = self.map_bounds()
        now = self.now_seconds()
        avoidance_duration = 0.0
        if self.avoidance_start_time is not None:
            avoidance_duration = max(0.0, now - self.avoidance_start_time)
        data = {
            'avoidance_state': mode,
            'forward_clearance_min': self.debug_distance(self.forward_clearance_min()),
            'center_min': self.debug_distance(self.center_min),
            'footprint_corridor_min': self.debug_distance(self.footprint_corridor_min),
            'left_lateral_min': self.debug_distance(self.left_lateral_min),
            'right_lateral_min': self.debug_distance(self.right_lateral_min),
            'front_min': self.debug_distance(self.front_min),
            'left_min': self.debug_distance(self.left_min),
            'right_min': self.debug_distance(self.right_min),
            'rear_min': self.debug_distance(self.rear_min),
            'vehicle_safety_radius': float(self.vehicle_safety_radius),
            'self_filter_distance': float(self.self_filter_distance),
            'side_clearance_distance': float(self.side_clearance_distance),
            'center_clearance_distance': float(self.narrow_center_clearance()),
            'footprint_corridor_half_width': float(self.footprint_corridor_half_width),
            'boundary_margin': float(self.boundary_margin),
            'nearest_map_boundary_distance': self.nearest_map_boundary_distance(),
            'inside_map': bool(
                self.odom_pose is not None and
                self.point_inside_map(self.odom_pose[0], self.odom_pose[1], 0.0)),
            'inside_safe_boundary': bool(
                self.odom_pose is not None and self.point_inside_map(
                    self.odom_pose[0], self.odom_pose[1],
                    self.vehicle_safety_radius + self.boundary_margin)),
            'cmd_vel_final': {
                'linear_x': float(final.linear.x),
                'angular_z': float(final.angular.z),
            },
            'continuous_avoidance_sec': float(avoidance_duration),
            'max_continuous_avoidance_sec': float(self.max_continuous_avoidance_sec),
            'replan_requested': bool(self.replan_requested_now),
            'enforce_map_boundaries': bool(self.enforce_map_boundaries),
        }
        if self.goal is not None:
            data.update({
                'goal_x': float(self.goal[0]),
                'goal_y': float(self.goal[1]),
            })
        if self.odom_pose is not None:
            data.update({
                'odom_x': float(self.odom_pose[0]),
                'odom_y': float(self.odom_pose[1]),
                'odom_yaw': float(self.odom_pose[2]),
            })
        if bounds is not None:
            data.update({
                'map_min_x': float(bounds[0]),
                'map_min_y': float(bounds[1]),
                'map_max_x': float(bounds[2]),
                'map_max_y': float(bounds[3]),
            })
        msg = String()
        msg.data = json.dumps(data, sort_keys=True)
        self.debug_pub.publish(msg)

    @staticmethod
    def debug_distance(value):
        return None if not math.isfinite(value) else float(value)

    @staticmethod
    def format_distance(value):
        if value is None or not math.isfinite(value):
            return 'inf'
        return f'{value:.2f}'

    @staticmethod
    def copy_twist(msg):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z
        return out

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = LocalObstacleAvoidance()
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

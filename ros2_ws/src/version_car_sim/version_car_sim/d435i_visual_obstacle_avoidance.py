#!/usr/bin/env python3
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from version_car_sim.vehicle_geometry import (
    declare_vehicle_safety_parameters,
    read_vehicle_safety_geometry,
)


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class D435iVisualObstacleAvoidance(Node):
    """Reactive safety layer driven by D435i visual obstacle points."""

    PASS_THROUGH = 'PASS_THROUGH'
    SLOW_DOWN = 'SLOW_DOWN'
    TURN_LEFT = 'TURN_LEFT'
    TURN_RIGHT = 'TURN_RIGHT'
    EMERGENCY_STOP = 'EMERGENCY_STOP'
    START_UNSAFE = 'START_UNSAFE'
    BLOCKED = 'BLOCKED'

    def __init__(self):
        super().__init__('d435i_visual_obstacle_avoidance')

        self.declare_parameter('obstacle_points_topic', 'vision_obstacle_points')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('input_cmd_topic', 'cmd_vel_raw')
        self.declare_parameter('output_cmd_topic', 'cmd_vel')
        self.declare_parameter('debug_topic', 'd435i_avoidance_debug')
        self.declare_parameter('safety_marker_topic', 'vehicle_safety_radius_marker')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('inflation_radius', 0.45)
        self.declare_parameter('slow_distance', 1.10)
        self.declare_parameter('stop_distance', 0.65)
        self.declare_parameter('emergency_stop_distance', 0.35)
        self.declare_parameter('slow_margin', 0.80)
        self.declare_parameter('stop_margin', 0.40)
        self.declare_parameter('emergency_margin', 0.15)
        self.declare_parameter('turn_clearance_margin', 0.50)
        self.declare_parameter('turn_distance_margin', 0.35)
        self.declare_parameter('turn_release_margin', 0.20)
        self.declare_parameter('front_angle_deg', 25.0)
        self.declare_parameter('side_angle_deg', 90.0)
        self.declare_parameter('avoid_turn_speed', 0.30)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 0.35)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('points_timeout', 0.7)
        self.declare_parameter('odom_timeout', 0.7)
        self.declare_parameter('publish_rate', 20.0)
        declare_vehicle_safety_parameters(self)

        self.safety_geometry = read_vehicle_safety_geometry(self)
        self.vehicle_safety_radius = self.safety_geometry['vehicle_safety_radius']
        self.slow_margin = max(0.0, float(self.get_parameter('slow_margin').value))
        self.stop_margin = max(0.0, float(self.get_parameter('stop_margin').value))
        self.emergency_margin = max(
            0.0, float(self.get_parameter('emergency_margin').value))
        self.turn_clearance_margin = max(
            0.0, float(self.get_parameter('turn_clearance_margin').value))
        self.turn_distance_margin = max(
            0.0, float(self.get_parameter('turn_distance_margin').value))
        self.turn_release_margin = max(
            0.0, float(self.get_parameter('turn_release_margin').value))
        configured_slow_distance = float(self.get_parameter('slow_distance').value)
        configured_stop_distance = float(self.get_parameter('stop_distance').value)
        configured_emergency_distance = float(
            self.get_parameter('emergency_stop_distance').value)
        self.slow_distance = max(
            configured_slow_distance,
            self.vehicle_safety_radius + self.slow_margin,
        )
        self.stop_distance = max(
            configured_stop_distance,
            self.vehicle_safety_radius + self.stop_margin,
        )
        self.emergency_stop_distance = max(
            configured_emergency_distance,
            self.vehicle_safety_radius + self.emergency_margin,
        )
        if self.slow_distance <= self.stop_distance:
            self.slow_distance = self.stop_distance + 0.10
        if self.stop_distance <= self.emergency_stop_distance:
            self.stop_distance = self.emergency_stop_distance + 0.10
        self.turn_clearance_distance = max(
            self.stop_distance,
            self.vehicle_safety_radius + self.turn_clearance_margin,
        )
        self.turn_distance = max(
            self.stop_distance + self.turn_distance_margin,
            self.turn_clearance_distance,
        )
        self.turn_release_distance = max(
            self.turn_distance,
            min(self.slow_distance, self.turn_distance + self.turn_release_margin),
        )
        self.front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        self.side_angle = math.radians(float(self.get_parameter('side_angle_deg').value))
        self.avoid_turn_speed = float(self.get_parameter('avoid_turn_speed').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        configured_inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.inflation_radius = max(configured_inflation_radius, self.vehicle_safety_radius)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.points_timeout = float(self.get_parameter('points_timeout').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)

        self.latest_cmd = Twist()
        self.latest_cmd_time = None
        self.latest_points_time = None
        self.latest_odom_time = None
        self.odom_pose = None
        self.front_min = math.inf
        self.left_min = math.inf
        self.right_min = math.inf
        self.nearest_min = math.inf
        self.last_state = None
        self.last_turn_state = None
        self.latest_point_frame = ''

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('output_cmd_topic').value), 10)
        self.debug_pub = self.create_publisher(
            String, str(self.get_parameter('debug_topic').value), 10)
        self.marker_pub = self.create_publisher(
            Marker, str(self.get_parameter('safety_marker_topic').value), 1)

        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('obstacle_points_topic').value),
            self.on_points,
            10,
        )
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self.on_odom, 30)
        self.create_subscription(
            Twist, str(self.get_parameter('input_cmd_topic').value), self.on_cmd, 10)

        publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / publish_rate, self.on_timer)

        self.get_logger().info(
            'D435i visual obstacle avoidance ready: '
            '/vision_obstacle_points + /cmd_vel_raw -> /cmd_vel')
        self.get_logger().info(
            f'Using base_link vehicle safety circle: '
            f'front_left={self.safety_geometry["wheel_distances"]["front_left"]:.3f} m, '
            f'front_right={self.safety_geometry["wheel_distances"]["front_right"]:.3f} m, '
            f'rear_left={self.safety_geometry["wheel_distances"]["rear_left"]:.3f} m, '
            f'rear_right={self.safety_geometry["wheel_distances"]["rear_right"]:.3f} m, '
            f'max={self.safety_geometry["max_distance"]:.3f} m, '
            f'scale={self.safety_geometry["scale"]:.2f}, '
            f'vehicle_safety_radius={self.vehicle_safety_radius:.3f} m')
        self.get_logger().info(
            f'Obstacle safety thresholds: emergency={self.emergency_stop_distance:.3f} m, '
            f'stop={self.stop_distance:.3f} m, slow={self.slow_distance:.3f} m, '
            f'turn_distance={self.turn_distance:.3f} m, '
            f'turn_release={self.turn_release_distance:.3f} m, '
            f'turn_clearance={self.turn_clearance_distance:.3f} m')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_pose = (float(p.x), float(p.y), float(yaw))
        self.latest_odom_time = self.now_seconds()

    def on_cmd(self, msg):
        self.latest_cmd = msg
        self.latest_cmd_time = self.now_seconds()

    def on_points(self, msg):
        self.latest_points_time = self.now_seconds()
        self.latest_point_frame = msg.header.frame_id
        if self.odom_pose is None:
            self.reset_obstacle_distances()
            return

        points = point_cloud2.read_points(
            msg, field_names=['x', 'y', 'z'], skip_nans=True)
        if len(points) == 0:
            self.reset_obstacle_distances()
            return

        map_x = np.asarray(points['x'], dtype=np.float32)
        map_y = np.asarray(points['y'], dtype=np.float32)
        x_body, y_body = self.map_to_body(map_x, map_y)
        all_distances = np.hypot(x_body, y_body)
        if all_distances.size:
            self.nearest_min = float(np.min(all_distances))
        else:
            self.nearest_min = math.inf
        ahead = x_body > 0.05
        if not np.any(ahead):
            self.front_min = math.inf
            self.left_min = math.inf
            self.right_min = math.inf
            return

        x_body = x_body[ahead]
        y_body = y_body[ahead]
        distances = np.hypot(x_body, y_body)
        angles = np.arctan2(y_body, x_body)

        self.front_min = self.sector_min(distances, angles, -self.front_angle, self.front_angle)
        self.left_min = self.sector_min(distances, angles, self.front_angle, self.side_angle)
        self.right_min = self.sector_min(distances, angles, -self.side_angle, -self.front_angle)

    def reset_obstacle_distances(self):
        self.front_min = math.inf
        self.left_min = math.inf
        self.right_min = math.inf
        self.nearest_min = math.inf

    def map_to_body(self, map_x, map_y):
        robot_x, robot_y, yaw = self.odom_pose
        dx = map_x - robot_x
        dy = map_y - robot_y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x_body = cos_yaw * dx + sin_yaw * dy
        y_body = -sin_yaw * dx + cos_yaw * dy
        return x_body, y_body

    @staticmethod
    def sector_min(distances, angles, min_angle, max_angle):
        mask = (angles >= min_angle) & (angles <= max_angle)
        if not np.any(mask):
            return math.inf
        return float(np.min(distances[mask]))

    def on_timer(self):
        now = self.now_seconds()
        raw = self.latest_cmd
        if self.latest_cmd_time is None or now - self.latest_cmd_time > self.cmd_timeout:
            self.publish_limited(Twist(), self.EMERGENCY_STOP, raw)
            return
        if self.latest_odom_time is None or now - self.latest_odom_time > self.odom_timeout:
            self.publish_limited(Twist(), self.EMERGENCY_STOP, raw)
            return
        if self.latest_points_time is None or now - self.latest_points_time > self.points_timeout:
            self.publish_limited(Twist(), self.EMERGENCY_STOP, raw)
            return

        cmd, state = self.apply_avoidance(raw)
        self.publish_limited(cmd, state, raw)

    def apply_avoidance(self, raw):
        raw_is_stop = abs(raw.linear.x) < 1e-4 and abs(raw.angular.z) < 1e-4
        start_clearance_limit = self.vehicle_safety_radius + self.turn_clearance_margin

        if self.nearest_min <= self.vehicle_safety_radius:
            return Twist(), self.EMERGENCY_STOP

        if raw_is_stop and self.nearest_min <= start_clearance_limit:
            return Twist(), self.START_UNSAFE

        if self.front_min <= self.emergency_stop_distance:
            return Twist(), self.EMERGENCY_STOP

        turn_trigger_distance = self.turn_release_distance if self.last_turn_state else self.turn_distance
        if self.front_min <= turn_trigger_distance:
            left_safe = self.side_is_safe(self.left_min)
            right_safe = self.side_is_safe(self.right_min)
            if not left_safe and not right_safe:
                self.last_turn_state = None
                return Twist(), self.BLOCKED

            cmd = Twist()
            state = self.choose_turn_state(left_safe, right_safe)
            if state == self.TURN_LEFT:
                cmd.angular.z = self.avoid_turn_speed
            else:
                cmd.angular.z = -self.avoid_turn_speed
            self.last_turn_state = state
            return cmd, state

        self.last_turn_state = None
        cmd = self.copy_twist(raw)
        if self.front_min <= self.slow_distance:
            denom = max(0.01, self.slow_distance - self.turn_distance)
            scale = clamp((self.front_min - self.turn_distance) / denom, 0.0, 1.0)
            if cmd.linear.x > 0.0:
                cmd.linear.x *= scale
            return cmd, self.SLOW_DOWN

        return cmd, self.PASS_THROUGH

    def choose_turn_state(self, left_safe, right_safe):
        if self.last_turn_state == self.TURN_LEFT and left_safe:
            return self.TURN_LEFT
        if self.last_turn_state == self.TURN_RIGHT and right_safe:
            return self.TURN_RIGHT
        if left_safe and not right_safe:
            return self.TURN_LEFT
        if right_safe and not left_safe:
            return self.TURN_RIGHT
        if self.left_min >= self.right_min:
            return self.TURN_LEFT
        return self.TURN_RIGHT

    def side_is_safe(self, distance):
        return (not math.isfinite(distance)) or distance >= self.turn_clearance_distance

    def publish_limited(self, cmd, state, raw):
        limited = self.copy_twist(cmd)
        limited.linear.x = clamp(
            limited.linear.x, -self.max_linear_speed, self.max_linear_speed)
        limited.linear.y = 0.0
        limited.linear.z = 0.0
        limited.angular.x = 0.0
        limited.angular.y = 0.0
        limited.angular.z = clamp(
            limited.angular.z, -self.max_angular_speed, self.max_angular_speed)

        if state != self.last_state:
            self.last_state = state
            self.get_logger().info(
                f'state={state}, front={self.format_distance(self.front_min)}, '
                f'left={self.format_distance(self.left_min)}, '
                f'right={self.format_distance(self.right_min)}')

        self.cmd_pub.publish(limited)
        self.publish_safety_marker(state)
        self.publish_debug(state, raw, limited)

    def publish_debug(self, state, raw, final):
        msg = String()
        front_boundary = self.distance_to_safety_boundary(self.front_min)
        nearest_boundary = self.distance_to_safety_boundary(self.nearest_min)
        left_safe_for_turn = self.side_is_safe(self.left_min)
        right_safe_for_turn = self.side_is_safe(self.right_min)
        start_unsafe = (
            abs(raw.linear.x) < 1e-4 and abs(raw.angular.z) < 1e-4 and
            math.isfinite(self.nearest_min) and
            self.nearest_min <= self.vehicle_safety_radius + self.turn_clearance_margin
        )
        msg.data = json.dumps({
            'source_point_frame': self.latest_point_frame,
            'target_frame': self.target_frame,
            'tf_lookup_success': self.points_are_fresh(),
            'vehicle_safety_radius': float(self.vehicle_safety_radius),
            'max_distance_from_center_to_wheel': float(self.safety_geometry['max_distance']),
            'inflation_radius': float(self.inflation_radius),
            'front_left_distance': float(self.safety_geometry['wheel_distances']['front_left']),
            'front_right_distance': float(self.safety_geometry['wheel_distances']['front_right']),
            'rear_left_distance': float(self.safety_geometry['wheel_distances']['rear_left']),
            'rear_right_distance': float(self.safety_geometry['wheel_distances']['rear_right']),
            'front_min': self.debug_distance(self.front_min),
            'left_min': self.debug_distance(self.left_min),
            'right_min': self.debug_distance(self.right_min),
            'nearest_min': self.debug_distance(self.nearest_min),
            'front_min_obstacle_distance_base_link': self.debug_distance(self.front_min),
            'left_min_obstacle_distance_base_link': self.debug_distance(self.left_min),
            'right_min_obstacle_distance_base_link': self.debug_distance(self.right_min),
            'nearest_obstacle_distance': self.debug_distance(self.nearest_min),
            'front_min_obstacle_distance': self.debug_distance(self.front_min),
            'distance_to_safety_boundary': front_boundary,
            'nearest_distance_to_safety_boundary': nearest_boundary,
            'slow_distance': float(self.slow_distance),
            'stop_distance': float(self.stop_distance),
            'emergency_stop_distance': float(self.emergency_stop_distance),
            'turn_distance': float(self.turn_distance),
            'turn_release_distance': float(self.turn_release_distance),
            'turn_trigger_distance': float(
                self.turn_release_distance if self.last_turn_state else self.turn_distance),
            'turn_clearance_distance': float(self.turn_clearance_distance),
            'last_turn_state': self.last_turn_state,
            'left_safe_for_turn': bool(left_safe_for_turn),
            'right_safe_for_turn': bool(right_safe_for_turn),
            'start_unsafe': bool(start_unsafe),
            'avoidance_state': state,
            'using_vehicle_safety_radius': True,
            'cmd_vel_raw': {
                'linear_x': float(raw.linear.x),
                'angular_z': float(raw.angular.z),
            },
            'cmd_vel_final': {
                'linear_x': float(final.linear.x),
                'angular_z': float(final.angular.z),
            },
            'raw_linear': float(raw.linear.x),
            'raw_angular': float(raw.angular.z),
            'final_linear': float(final.linear.x),
            'final_angular': float(final.angular.z),
        }, sort_keys=True)
        self.debug_pub.publish(msg)

    def publish_safety_marker(self, state):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'base_link'
        marker.ns = 'vehicle_safety_radius'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.025
        marker.pose.orientation.w = 1.0
        diameter = 2.0 * self.vehicle_safety_radius
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = 0.05
        if state == self.EMERGENCY_STOP:
            marker.color.r = 1.0
            marker.color.g = 0.12
            marker.color.b = 0.05
            marker.color.a = 0.30
        elif state in (self.START_UNSAFE, self.BLOCKED):
            marker.color.r = 1.0
            marker.color.g = 0.36
            marker.color.b = 0.02
            marker.color.a = 0.28
        elif state in (self.TURN_LEFT, self.TURN_RIGHT, self.SLOW_DOWN):
            marker.color.r = 1.0
            marker.color.g = 0.70
            marker.color.b = 0.05
            marker.color.a = 0.24
        else:
            marker.color.r = 0.10
            marker.color.g = 0.85
            marker.color.b = 0.35
            marker.color.a = 0.18
        self.marker_pub.publish(marker)

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

    @staticmethod
    def debug_distance(value):
        return None if not math.isfinite(value) else float(value)

    def distance_to_safety_boundary(self, obstacle_distance):
        if not math.isfinite(obstacle_distance):
            return None
        return float(obstacle_distance - self.vehicle_safety_radius)

    def points_are_fresh(self):
        if self.latest_points_time is None:
            return False
        return self.now_seconds() - self.latest_points_time <= self.points_timeout

    @staticmethod
    def format_distance(value):
        if not math.isfinite(value):
            return 'inf'
        return f'{value:.2f}'

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = D435iVisualObstacleAvoidance()
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

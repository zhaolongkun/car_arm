#!/usr/bin/env python3
import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    from quadrotor_msgs.msg import PositionCommand
except ImportError:
    PositionCommand = None

try:
    from traj_utils.msg import Bspline
except ImportError:
    Bspline = None


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


class EgoPositionCmdToTwist(Node):
    """Convert ego-planner PositionCommand into ground-car Twist commands."""

    def __init__(self):
        super().__init__('ego_position_cmd_to_twist')

        self.declare_parameter('position_cmd_topic', 'drone_0_planning/pos_cmd')
        self.declare_parameter('bspline_topic', 'drone_0_planning/bspline')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('output_cmd_topic', 'cmd_vel_raw')
        self.declare_parameter('planned_path_topic', 'planned_path')
        self.declare_parameter('start_topic', 'start_navigation')
        self.declare_parameter('debug_topic', 'ego_cmd_bridge_debug')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('cmd_timeout', 0.50)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 0.55)
        self.declare_parameter('position_kp', 0.65)
        self.declare_parameter('yaw_kp', 1.8)
        self.declare_parameter('large_yaw_error', 1.15)
        self.declare_parameter('min_follow_distance', 0.08)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.enabled = bool(self.get_parameter('auto_start').value)
        self.cmd_timeout = max(0.05, float(self.get_parameter('cmd_timeout').value))
        self.max_linear_speed = abs(float(self.get_parameter('max_linear_speed').value))
        self.max_angular_speed = abs(float(self.get_parameter('max_angular_speed').value))
        self.position_kp = float(self.get_parameter('position_kp').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.large_yaw_error = abs(float(self.get_parameter('large_yaw_error').value))
        self.min_follow_distance = max(
            0.0, float(self.get_parameter('min_follow_distance').value))

        self.odom_pose = None
        self.latest_position_cmd = None
        self.latest_position_cmd_time = None
        self.latest_state = 'INIT'
        self.latest_output = Twist()
        self.latest_path_point_count = 0

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('output_cmd_topic').value), 10)
        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter('planned_path_topic').value), 5)
        self.debug_pub = self.create_publisher(
            String, str(self.get_parameter('debug_topic').value), 10)

        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.on_odom,
            20,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('start_topic').value),
            self.on_start,
            5,
        )

        if PositionCommand is None:
            self.get_logger().error(
                'quadrotor_msgs/msg/PositionCommand is not available. '
                'Source the ego-planner workspace before using planner_backend:=ego.')
        else:
            self.create_subscription(
                PositionCommand,
                str(self.get_parameter('position_cmd_topic').value),
                self.on_position_cmd,
                20,
            )

        if Bspline is None:
            self.get_logger().warn(
                'traj_utils/msg/Bspline is not available; /planned_path will not '
                'be populated from ego-planner B-splines.')
        else:
            self.create_subscription(
                Bspline,
                str(self.get_parameter('bspline_topic').value),
                self.on_bspline,
                10,
            )

        publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / publish_rate, self.on_timer)

        self.get_logger().info(
            'EGO command bridge ready: '
            f'/{self.get_parameter("position_cmd_topic").value} -> '
            f'/{self.get_parameter("output_cmd_topic").value}, '
            f'auto_start={self.enabled}')

    def on_start(self, msg):
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.publish_stop('STOPPED_BY_START_TOPIC')
        self.get_logger().info(f'EGO command bridge enabled={self.enabled}')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_pose = (float(p.x), float(p.y), float(yaw))

    def on_position_cmd(self, msg):
        self.latest_position_cmd = msg
        self.latest_position_cmd_time = self.now_seconds()

    def on_bspline(self, msg):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame
        for point in msg.pos_pts:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(point.x)
            pose.pose.position.y = float(point.y)
            pose.pose.position.z = 0.05
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.latest_path_point_count = len(path.poses)
        self.path_pub.publish(path)

    def on_timer(self):
        if PositionCommand is None:
            self.publish_stop('MISSING_QUADROTOR_MSGS')
            return
        if not self.enabled:
            self.publish_stop('WAIT_START')
            return
        if self.odom_pose is None:
            self.publish_stop('NO_ODOM')
            return
        if self.latest_position_cmd is None or self.latest_position_cmd_time is None:
            self.publish_stop('NO_POSITION_CMD')
            return
        if self.now_seconds() - self.latest_position_cmd_time > self.cmd_timeout:
            self.publish_stop('POSITION_CMD_TIMEOUT')
            return

        cmd, state = self.make_twist(self.latest_position_cmd)
        self.latest_state = state
        self.latest_output = cmd
        self.cmd_pub.publish(cmd)
        self.publish_debug()

    def make_twist(self, position_cmd):
        x, y, yaw = self.odom_pose
        target_x = float(position_cmd.position.x)
        target_y = float(position_cmd.position.y)
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)

        vel_x = float(position_cmd.velocity.x)
        vel_y = float(position_cmd.velocity.y)
        desired_speed = math.hypot(vel_x, vel_y)
        if desired_speed > 0.03:
            desired_yaw = math.atan2(vel_y, vel_x)
            speed = min(self.max_linear_speed, desired_speed)
            state = 'TRACK_EGO_VELOCITY'
        elif distance > self.min_follow_distance:
            desired_yaw = math.atan2(dy, dx)
            speed = min(self.max_linear_speed, self.position_kp * distance)
            state = 'TRACK_EGO_POSITION'
        else:
            return Twist(), 'EGO_TARGET_REACHED'

        yaw_error = angle_diff(desired_yaw, yaw)
        cmd = Twist()
        cmd.angular.z = clamp(
            self.yaw_kp * yaw_error + float(position_cmd.yaw_dot),
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        if abs(yaw_error) > self.large_yaw_error:
            cmd.linear.x = 0.0
            state = 'TURN_TO_EGO_COMMAND'
        else:
            heading_scale = max(0.0, math.cos(yaw_error))
            cmd.linear.x = clamp(speed * heading_scale, 0.0, self.max_linear_speed)
        return cmd, state

    def publish_stop(self, state):
        self.latest_state = state
        self.latest_output = Twist()
        self.cmd_pub.publish(self.latest_output)
        self.publish_debug()

    def publish_debug(self):
        target = None
        velocity = None
        flag = None
        if self.latest_position_cmd is not None:
            target = [
                float(self.latest_position_cmd.position.x),
                float(self.latest_position_cmd.position.y),
                float(self.latest_position_cmd.position.z),
            ]
            velocity = [
                float(self.latest_position_cmd.velocity.x),
                float(self.latest_position_cmd.velocity.y),
                float(self.latest_position_cmd.velocity.z),
            ]
            flag = int(self.latest_position_cmd.trajectory_flag)
        msg = String()
        msg.data = json.dumps({
            'state': self.latest_state,
            'enabled_by_start_navigation': bool(self.enabled),
            'has_odom': self.odom_pose is not None,
            'has_position_cmd': self.latest_position_cmd is not None,
            'position_cmd_age_sec': self.command_age(),
            'target_position': target,
            'target_velocity': velocity,
            'trajectory_flag': flag,
            'cmd_vel_raw': {
                'linear_x': float(self.latest_output.linear.x),
                'angular_z': float(self.latest_output.angular.z),
            },
            'planned_path_point_count': int(self.latest_path_point_count),
        }, sort_keys=True)
        self.debug_pub.publish(msg)

    def command_age(self):
        if self.latest_position_cmd_time is None:
            return None
        return float(self.now_seconds() - self.latest_position_cmd_time)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = EgoPositionCmdToTwist()
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

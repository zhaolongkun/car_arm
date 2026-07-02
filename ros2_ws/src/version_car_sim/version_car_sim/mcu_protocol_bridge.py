#!/usr/bin/env python3
from dataclasses import dataclass
import math
import socket
import struct

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


HEAD0 = 0xAE
HEAD1 = 0xEA
TAIL0 = 0xEF
TAIL1 = 0xFE
CMD_SPEED = 0xF3
CMD_ODOM = 0xA7
SPEED_FRAME_LEN = 0x0B
MICK_OFFSET = 10.0


@dataclass
class MickChassisCommand:
    vx: float
    vy: float
    wz: float
    x_raw: int
    y_raw: int
    w_raw: int


def clamp(value, low, high):
    return max(low, min(high, value))


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class McuProtocolBridge(Node):
    """Bridge Twist commands to the MickX4-V3 chassis protocol."""

    def __init__(self):
        super().__init__('mcu_protocol_bridge')

        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('mick_cmd_topic', '/mickrobot/chassis/cmd_vel')
        self.declare_parameter('publish_mick_cmd', True)
        self.declare_parameter('tcp_enabled', False)
        self.declare_parameter('tcp_host', '192.168.0.7')
        self.declare_parameter('tcp_port', 8234)
        self.declare_parameter('max_linear_speed', 0.75)
        self.declare_parameter('max_lateral_speed', 0.0)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('allow_lateral_speed', False)
        self.declare_parameter('deadman_timeout', 0.4)
        self.declare_parameter('hex_log_period', 1.0)
        self.declare_parameter('odom_topic', '/mickrobot/chassis/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_odom', True)

        # Legacy parameters are declared so older launch files keep starting.
        self.declare_parameter('serial_enabled', False)
        self.declare_parameter('serial_port', '')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_rear_speed', 0.75)
        self.declare_parameter('max_speed_mps', 0.75)
        self.declare_parameter('max_steering_angle_deg', 0.0)
        self.declare_parameter('max_steering_rate_deg_s', 0.0)
        self.declare_parameter('max_steer_deg', 0.0)
        self.declare_parameter('low_speed_epsilon', 0.0)
        self.declare_parameter('low_speed_steering_limit_deg', 0.0)
        self.declare_parameter('use_front_steering_feedback', False)
        self.declare_parameter('front_steering_feedback_topic', 'front_steering_feedback')
        self.declare_parameter('steering_kp', 0.0)
        self.declare_parameter('max_steering_speed_cmd_deg_s', 0.0)
        self.declare_parameter('wheel_base', 0.40)

        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.mick_cmd_topic = str(self.get_parameter('mick_cmd_topic').value)
        self.publish_mick_cmd = bool(self.get_parameter('publish_mick_cmd').value)
        self.tcp_enabled = bool(self.get_parameter('tcp_enabled').value)
        self.tcp_host = str(self.get_parameter('tcp_host').value)
        self.tcp_port = int(self.get_parameter('tcp_port').value)
        self.max_linear_speed = self.read_speed_limit()
        self.max_lateral_speed = max(
            0.0, abs(float(self.get_parameter('max_lateral_speed').value)))
        self.max_angular_speed = max(
            0.01, abs(float(self.get_parameter('max_angular_speed').value)))
        self.allow_lateral_speed = bool(
            self.get_parameter('allow_lateral_speed').value)
        self.deadman_timeout = max(
            0.05, float(self.get_parameter('deadman_timeout').value))
        self.hex_log_period = max(
            0.0, float(self.get_parameter('hex_log_period').value))
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_odom = bool(self.get_parameter('publish_odom').value)

        self.sock = None
        self.rx_buffer = bytearray()
        self.last_cmd_time = None
        self.last_log_time = 0.0
        self.stop_sent = False
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_odom_time = None

        self.frame_pub = self.create_publisher(String, 'mcu_frame_hex', 10)
        self.mick_cmd_pub = None
        if self.publish_mick_cmd:
            self.mick_cmd_pub = self.create_publisher(
                Twist, self.mick_cmd_topic, 10)
        self.odom_pub = None
        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
        self.create_timer(0.05, self.on_deadman_timer)
        self.create_timer(0.02, self.on_tcp_timer)

        if bool(self.get_parameter('serial_enabled').value):
            self.get_logger().warn(
                'serial_enabled is ignored for MickX4; use tcp_enabled instead.')

        mode = (
            f'TCP {self.tcp_host}:{self.tcp_port}'
            if self.tcp_enabled else 'dry-run hex output')
        self.get_logger().info(
            f'MickX4 bridge ready: /{self.cmd_topic} -> F3 speed frame '
            f'({mode}), max_v={self.max_linear_speed:.2f} m/s, '
            f'max_w={self.max_angular_speed:.2f} rad/s.')

    def read_speed_limit(self):
        max_linear = float(self.get_parameter('max_linear_speed').value)
        legacy_rear = float(self.get_parameter('max_rear_speed').value)
        legacy_speed = float(self.get_parameter('max_speed_mps').value)
        if abs(max_linear - 0.75) < 1e-9:
            legacy_overrides = [
                value for value in (legacy_rear, legacy_speed)
                if abs(value - 0.75) >= 1e-9
            ]
            if legacy_overrides:
                max_linear = max(legacy_overrides)
        return max(0.01, abs(max_linear))

    def on_cmd(self, msg):
        self.last_cmd_time = self.now_seconds()
        self.stop_sent = False
        normalized = self.normalize_twist(msg)
        frame, command = self.twist_to_mick_chassis_frame(normalized)
        self.publish_mick_twist(normalized)
        self.send_frame(frame, command)

    def normalize_twist(self, msg):
        normalized = Twist()
        normalized.linear.x = clamp(
            float(msg.linear.x), -self.max_linear_speed, self.max_linear_speed)
        if self.allow_lateral_speed:
            normalized.linear.y = clamp(
                float(msg.linear.y), -self.max_lateral_speed, self.max_lateral_speed)
        else:
            normalized.linear.y = 0.0
        normalized.angular.z = clamp(
            float(msg.angular.z), -self.max_angular_speed, self.max_angular_speed)
        return normalized

    def twist_to_mick_chassis_frame(self, cmd):
        x_raw = self.encode_offset_speed(cmd.linear.x)
        y_raw = self.encode_offset_speed(cmd.linear.y)
        w_raw = self.encode_offset_speed(cmd.angular.z)

        frame = bytearray(15)
        frame[0] = HEAD0
        frame[1] = HEAD1
        frame[2] = SPEED_FRAME_LEN
        frame[3] = CMD_SPEED
        frame[4] = (x_raw >> 8) & 0xFF
        frame[5] = x_raw & 0xFF
        frame[6] = (y_raw >> 8) & 0xFF
        frame[7] = y_raw & 0xFF
        frame[8] = (w_raw >> 8) & 0xFF
        frame[9] = w_raw & 0xFF
        frame[10] = 0x00
        frame[11] = 0x00
        frame[12] = sum(frame[2:12]) & 0xFF
        frame[13] = TAIL0
        frame[14] = TAIL1

        command = MickChassisCommand(
            vx=cmd.linear.x,
            vy=cmd.linear.y,
            wz=cmd.angular.z,
            x_raw=x_raw,
            y_raw=y_raw,
            w_raw=w_raw,
        )
        return bytes(frame), command

    def encode_offset_speed(self, value):
        value = clamp(float(value), -MICK_OFFSET, MICK_OFFSET)
        return int(round((value + MICK_OFFSET) * 100.0)) & 0xFFFF

    def publish_mick_twist(self, msg):
        if self.mick_cmd_pub is None:
            return
        self.mick_cmd_pub.publish(msg)

    def on_deadman_timer(self):
        if self.last_cmd_time is None:
            return
        if self.now_seconds() - self.last_cmd_time < self.deadman_timeout:
            return
        if self.stop_sent:
            return
        stop = Twist()
        frame, command = self.twist_to_mick_chassis_frame(stop)
        self.publish_mick_twist(stop)
        self.send_frame(frame, command)
        self.stop_sent = True

    def send_frame(self, frame, command):
        hex_text = frame.hex(' ')
        self.frame_pub.publish(String(data=hex_text))
        if self.tcp_enabled:
            self.send_tcp(frame)

        now = self.now_seconds()
        if now - self.last_log_time >= self.hex_log_period:
            self.last_log_time = now
            self.get_logger().info(
                f'MickX4 vx={command.vx:.2f} m/s, vy={command.vy:.2f} m/s, '
                f'wz={command.wz:.2f} rad/s, raw=({command.x_raw}, '
                f'{command.y_raw}, {command.w_raw}); frame: {hex_text}')

    def send_tcp(self, frame):
        if not self.ensure_tcp_connected():
            return
        try:
            self.sock.sendall(frame)
        except OSError as exc:
            self.get_logger().warn(f'MickX4 TCP send failed: {exc}')
            self.close_tcp()

    def on_tcp_timer(self):
        if not self.tcp_enabled:
            return
        if not self.ensure_tcp_connected():
            return
        try:
            while True:
                chunk = self.sock.recv(256)
                if not chunk:
                    self.close_tcp()
                    return
                self.rx_buffer.extend(chunk)
        except BlockingIOError:
            pass
        except OSError as exc:
            self.get_logger().warn(f'MickX4 TCP receive failed: {exc}')
            self.close_tcp()
            return
        self.consume_rx_buffer()

    def ensure_tcp_connected(self):
        if self.sock is not None:
            return True
        try:
            sock = socket.create_connection(
                (self.tcp_host, self.tcp_port), timeout=0.2)
            sock.setblocking(False)
            self.sock = sock
            self.get_logger().info(
                f'Connected to MickX4 chassis TCP {self.tcp_host}:{self.tcp_port}.')
            return True
        except OSError as exc:
            self.get_logger().warn(
                f'Waiting for MickX4 TCP {self.tcp_host}:{self.tcp_port}: {exc}',
                throttle_duration_sec=2.0)
            return False

    def close_tcp(self):
        if self.sock is None:
            return
        try:
            self.sock.close()
        finally:
            self.sock = None

    def consume_rx_buffer(self):
        while len(self.rx_buffer) >= 5:
            if self.rx_buffer[0] != HEAD0 or self.rx_buffer[1] != HEAD1:
                del self.rx_buffer[0]
                continue

            frame_len = self.rx_buffer[2]
            total_len = 2 + frame_len + 2
            if len(self.rx_buffer) < total_len:
                return

            frame = bytes(self.rx_buffer[:total_len])
            del self.rx_buffer[:total_len]
            if frame[-2] != TAIL0 or frame[-1] != TAIL1:
                continue

            check_index = 2 + frame_len - 1
            expected = sum(frame[2:check_index]) & 0xFF
            if frame[check_index] != expected:
                self.get_logger().warn('Dropped MickX4 feedback frame with bad checksum.')
                continue

            if frame[3] == CMD_ODOM:
                self.handle_odom_feedback(frame)

    def handle_odom_feedback(self, frame):
        if len(frame) < 13:
            return
        vx_raw, vy_raw, wz_raw = struct.unpack('>hhh', frame[4:10])
        vx = vx_raw * 0.001
        vy = vy_raw * 0.001
        wz = wz_raw * 0.001
        self.publish_odom_feedback(vx, vy, wz)

    def publish_odom_feedback(self, vx, vy, wz):
        if self.odom_pub is None:
            return

        now = self.get_clock().now()
        now_sec = self.now_seconds()
        if self.last_odom_time is None:
            dt = 0.0
        else:
            dt = clamp(now_sec - self.last_odom_time, 0.0, 0.2)
        self.last_odom_time = now_sec

        if dt > 0.0:
            self.odom_yaw += wz * dt
            cos_yaw = math.cos(self.odom_yaw)
            sin_yaw = math.sin(self.odom_yaw)
            self.odom_x += (vx * cos_yaw - vy * sin_yaw) * dt
            self.odom_y += (vx * sin_yaw + vy * cos_yaw) * dt

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.odom_x
        msg.pose.pose.position.y = self.odom_y
        qx, qy, qz, qw = quaternion_from_yaw(self.odom_yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        self.odom_pub.publish(msg)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = McuProtocolBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close_tcp()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

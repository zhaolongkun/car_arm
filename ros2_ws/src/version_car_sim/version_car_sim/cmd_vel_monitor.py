import json
import math

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class CmdVelMonitor(Node):
    def __init__(self):
        super().__init__('cmd_vel_monitor')
        self.declare_parameter('nav_cmd_topic', 'cmd_vel_nav')
        self.declare_parameter('final_cmd_topic', 'cmd_vel')
        self.declare_parameter('nav_status_topic', 'navigate_to_pose/_action/status')
        self.declare_parameter('log_period_sec', 1.0)
        self.declare_parameter('stale_timeout_sec', 1.0)
        self.declare_parameter('warn_only_during_active_goal', True)

        self.nav_cmd_topic = self.get_parameter('nav_cmd_topic').value
        self.final_cmd_topic = self.get_parameter('final_cmd_topic').value
        self.nav_status_topic = self.get_parameter('nav_status_topic').value
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)
        self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)
        self.warn_only_during_active_goal = bool(
            self.get_parameter('warn_only_during_active_goal').value)

        self.nav_cmd = None
        self.nav_stamp = None
        self.final_cmd = None
        self.final_stamp = None
        self.active_nav_goal = False

        self.create_subscription(Twist, self.nav_cmd_topic, self._on_nav_cmd, 10)
        self.create_subscription(Twist, self.final_cmd_topic, self._on_final_cmd, 10)
        self.create_subscription(
            GoalStatusArray, self.nav_status_topic, self._on_nav_status, 10)
        self.debug_pub = self.create_publisher(String, 'cmd_vel_debug', 10)
        self.create_timer(self.log_period_sec, self._on_timer)

        self.get_logger().info(
            f'Watching cmd_vel chain: {self.nav_cmd_topic} -> {self.final_cmd_topic}, '
            f'nav_status={self.nav_status_topic}')

    def _on_nav_cmd(self, msg):
        self.nav_cmd = msg
        self.nav_stamp = self.get_clock().now()

    def _on_final_cmd(self, msg):
        self.final_cmd = msg
        self.final_stamp = self.get_clock().now()

    def _on_nav_status(self, msg):
        active_states = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        self.active_nav_goal = any(
            status.status in active_states for status in msg.status_list)

    def _age(self, stamp):
        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    @staticmethod
    def _twist_payload(msg, age):
        if msg is None:
            return {'vx': None, 'wz': None, 'age': None}
        return {
            'vx': round(float(msg.linear.x), 4),
            'wz': round(float(msg.angular.z), 4),
            'age': round(age, 3),
        }

    def _on_timer(self):
        nav_age = self._age(self.nav_stamp)
        final_age = self._age(self.final_stamp)
        payload = {
            'nav_cmd_topic': self.nav_cmd_topic,
            'final_cmd_topic': self.final_cmd_topic,
            'active_nav_goal': self.active_nav_goal,
            'nav': self._twist_payload(self.nav_cmd, nav_age),
            'final': self._twist_payload(self.final_cmd, final_age),
        }
        self.debug_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

        nav_stale = nav_age > self.stale_timeout_sec
        final_stale = final_age > self.stale_timeout_sec
        nav_text = self._format_twist(self.nav_cmd, nav_age)
        final_text = self._format_twist(self.final_cmd, final_age)
        if nav_stale or final_stale:
            if self.active_nav_goal or not self.warn_only_during_active_goal:
                self.get_logger().warn(
                    f'cmd_vel chain stale while navigation active: '
                    f'nav={nav_text}, final={final_text}')
            else:
                self.get_logger().info(
                    f'cmd_vel chain idle: no active NavigateToPose goal; '
                    f'nav={nav_text}, final={final_text}')
        else:
            self.get_logger().info(
                f'cmd_vel chain: nav={nav_text}, final={final_text}')

    @staticmethod
    def _format_twist(msg, age):
        if msg is None:
            return 'none'
        return f'vx={msg.linear.x:+.3f}, wz={msg.angular.z:+.3f}, age={age:.2f}s'


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMonitor()
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

import argparse
import math
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import PerformanceMetrics
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class RebotArmMinimalTest(Node):
    def __init__(
        self,
        target_name,
        duration_sec,
        mirror_to_gazebo,
        joint_state_timeout,
        start_delay,
    ):
        super().__init__('rebot_arm_minimal_test')
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.latest_joint_positions = {}
        self.latest_joint_state_names = []
        self.latest_real_time_factor = None
        self.target_name = target_name
        self.duration_sec = max(0.5, float(duration_sec))
        self.mirror_to_gazebo = bool(mirror_to_gazebo)
        self.joint_state_timeout = max(1.0, float(joint_state_timeout))
        self.start_delay = max(0.0, float(start_delay))
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            'rebotarm_controller/follow_joint_trajectory')
        self.gazebo_pub = self.create_publisher(
            JointTrajectory,
            '/gazebo_arm/set_joint_trajectory',
            10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.create_subscription(
            PerformanceMetrics,
            '/gazebo/performance_metrics',
            self.on_performance_metrics,
            10)

    def on_joint_states(self, msg):
        self.latest_joint_state_names = list(msg.name)
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names:
                self.latest_joint_positions[name] = float(position)

    def on_performance_metrics(self, msg):
        self.latest_real_time_factor = float(msg.real_time_factor)

    def run(self):
        if not self.wait_for_joint_states(self.joint_state_timeout):
            missing = [
                name for name in self.joint_names
                if name not in self.latest_joint_positions
            ]
            topics = dict(self.get_topic_names_and_types())
            if '/joint_states' not in topics and 'joint_states' not in topics:
                self.get_logger().error(
                    '/joint_states 当前没有发布。请先启动完整仿真：\n'
                    '  cd <project-root>\n'
                    '  ./start_car_arm.sh\n'
                    '等 Gazebo、robot_state_publisher、ros2_control_node 和 '
                    'rebotarm_controller 都启动后，再运行这个最小测试。')
            else:
                self.get_logger().error(
                    '/joint_states 已发布，但没有完整 joint1~joint6，'
                    f'missing={missing}, received_names={self.latest_joint_state_names}')
            return False
        if not self.trajectory_client.wait_for_server(timeout_sec=8.0):
            self.get_logger().error(
                '找不到 /rebotarm_controller/follow_joint_trajectory action。'
                '请检查 ros2 control list_controllers 里 rebotarm_controller 是否 active。')
            return False

        if self.start_delay > 0.0:
            self.get_logger().info(
                f'最小机械臂调试入口已就绪，等待 {self.start_delay:.1f}s 后发送轨迹。')
            end_time = time.monotonic() + self.start_delay
            while rclpy.ok() and time.monotonic() < end_time:
                rclpy.spin_once(self, timeout_sec=0.05)

        target = self.target_joints(self.target_name)
        trajectory = self.build_trajectory(target)
        first_time = self.point_time_sec(trajectory.points[0]) if trajectory.points else 0.0
        last_time = self.point_time_sec(trajectory.points[-1]) if trajectory.points else 0.0
        self.get_logger().info(
            f'发送最小机械臂测试轨迹: target={self.target_name}, '
            f'duration={self.duration_sec:.2f}s, '
            f'current={self.format_joints(self.latest_joint_positions)}, '
            f'target={self.format_joints(target)}, '
            f'point_count={len(trajectory.points)}, '
            f'first_time_from_start={first_time:.3f}s, '
            f'last_time_from_start={last_time:.3f}s')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        sent_wall = time.monotonic()
        self.get_logger().info(
            f'wall time: trajectory sent at {sent_wall:.6f}; '
            'controller action name=/rebotarm_controller/follow_joint_trajectory; '
            f'{self.time_status_text()}')
        send_future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=8.0)
        if not send_future.done():
            self.get_logger().error('发送 FollowJointTrajectory goal 超时。')
            return False
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error('FollowJointTrajectory goal 被拒绝。')
            return False
        accepted_wall = time.monotonic()
        self.get_logger().info(
            f'FollowJointTrajectory goal accepted; '
            f'wall time: controller accepted at {accepted_wall:.6f}; '
            f'send_to_accept_elapsed={accepted_wall - sent_wall:.3f}s; '
            f'{self.time_status_text()}')

        result_future = handle.get_result_async()
        if self.mirror_to_gazebo:
            self.publish_gazebo_mirror(trajectory)
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=self.duration_sec + 8.0)
        if not result_future.done():
            self.get_logger().error('FollowJointTrajectory 执行等待超时。')
            return False
        result = result_future.result().result
        ok = int(result.error_code) == 0
        done_wall = time.monotonic()
        self.get_logger().info(
            f'最小机械臂测试完成: success={ok}, '
            f'error_code={result.error_code}, error_string="{result.error_string}"')
        self.get_logger().info(
            f'wall time: target {self.target_name} reached at {done_wall:.6f}; '
            f'elapsed_since_sent={done_wall - sent_wall:.3f}s; '
            f'{self.time_status_text()}')
        return ok

    def wait_for_joint_states(self, timeout_sec):
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < timeout_sec:
            if all(name in self.latest_joint_positions for name in self.joint_names):
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def target_joints(self, target_name):
        targets = {
            'home': {
                'joint1': 0.0,
                'joint2': 0.0,
                'joint3': 0.0,
                'joint4': 0.0,
                'joint5': 0.0,
                'joint6': 0.0,
            },
            'ready': {
                'joint1': 0.0,
                'joint2': -0.55,
                'joint3': -1.20,
                'joint4': 0.25,
                'joint5': 0.65,
                'joint6': 0.0,
            },
            'initial_pose': {
                'joint1': 0.0,
                'joint2': -0.65,
                'joint3': -1.45,
                'joint4': 0.15,
                'joint5': 0.75,
                'joint6': 0.0,
            },
            'spray_demo': {
                'joint1': 0.0,
                'joint2': -0.40,
                'joint3': -1.10,
                'joint4': 0.35,
                'joint5': 0.55,
                'joint6': 0.0,
            },
            'target_test_pose': {
                'joint1': 0.25,
                'joint2': -0.48,
                'joint3': -1.12,
                'joint4': 0.42,
                'joint5': 0.58,
                'joint6': 0.0,
            },
        }
        return targets.get(target_name, targets['ready'])

    def build_trajectory(self, target):
        trajectory = JointTrajectory()
        trajectory.header.frame_id = 'world'
        trajectory.joint_names = list(self.joint_names)
        start_positions = [
            self.latest_joint_positions[name]
            for name in self.joint_names
        ]
        target_positions = [target[name] for name in self.joint_names]

        waypoints = 20
        for idx in range(1, waypoints + 1):
            ratio = float(idx) / float(waypoints)
            smooth = ratio * ratio * (3.0 - 2.0 * ratio)
            point = JointTrajectoryPoint()
            point.positions = [
                start + (goal - start) * smooth
                for start, goal in zip(start_positions, target_positions)
            ]
            point.time_from_start = self.duration_msg(self.duration_sec * ratio)
            trajectory.points.append(point)
        return trajectory

    def publish_gazebo_mirror(self, trajectory):
        for _ in range(5):
            self.gazebo_pub.publish(trajectory)
            time.sleep(0.05)
        self.get_logger().info('已把同一条测试轨迹同步到 /gazebo_arm/set_joint_trajectory。')

    @staticmethod
    def duration_msg(duration_sec):
        sec = int(math.floor(duration_sec))
        nanosec = int(round((duration_sec - sec) * 1e9))
        if nanosec >= 1000000000:
            sec += 1
            nanosec -= 1000000000
        return DurationMsg(sec=sec, nanosec=nanosec)

    @staticmethod
    def point_time_sec(point):
        return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9

    def time_status_text(self):
        use_sim_time = (
            self.get_parameter('use_sim_time').value
            if self.has_parameter('use_sim_time')
            else False)
        now_msg = self.get_clock().now().to_msg()
        rtf = (
            'unavailable'
            if self.latest_real_time_factor is None
            else f'{self.latest_real_time_factor:.2f}')
        return (
            f'use_sim_time={use_sim_time}, '
            f'ros_time={now_msg.sec}.{now_msg.nanosec:09d}, '
            f'gazebo_real_time_factor={rtf}')

    @staticmethod
    def format_joints(joints):
        return '[' + ', '.join(
            f'{name}={float(joints[name]):+.3f}'
            for name in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        ) + ']'


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--target',
        choices=['home', 'ready', 'initial_pose', 'spray_demo', 'target_test_pose'],
        default='ready')
    parser.add_argument('--duration', type=float, default=2.5)
    parser.add_argument('--start-delay', type=float, default=0.0)
    parser.add_argument('--joint-state-timeout', type=float, default=20.0)
    parser.add_argument('--no-gazebo-mirror', action='store_true')
    return parser.parse_args(argv)


def main():
    rclpy.init()
    args = parse_args(remove_ros_args(sys.argv)[1:])
    node = RebotArmMinimalTest(
        args.target,
        args.duration,
        mirror_to_gazebo=not args.no_gazebo_mirror,
        joint_state_timeout=args.joint_state_timeout,
        start_delay=args.start_delay)
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()

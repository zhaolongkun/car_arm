#!/usr/bin/env python3
import csv
from datetime import datetime
import json
import math
import os
import time

import numpy as np
import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node

try:
    from PIL import Image, ImageDraw
except ImportError:
    Image = None
    ImageDraw = None


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class TrajectoryRecorder(Node):
    """Record actual odometry path, show a Gazebo trail, and save simulation results."""

    def __init__(self):
        super().__init__('trajectory_recorder')

        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('planned_path_topic', 'planned_path')
        self.declare_parameter('goal_topic', 'goal_pose')
        self.declare_parameter('start_pose_applied_topic', 'start_pose_applied')
        self.declare_parameter('actual_path_topic', 'actual_path')
        self.declare_parameter('stop_cmd_topic', 'cmd_vel_raw')
        self.declare_parameter('results_root', 'sim_results')
        self.declare_parameter('save_results', True)
        self.declare_parameter('show_gazebo_trail', True)
        self.declare_parameter('path_sample_distance', 0.05)
        self.declare_parameter('path_sample_period', 0.2)
        self.declare_parameter('gazebo_marker_distance', 0.15)
        self.declare_parameter('gazebo_marker_radius', 0.04)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('max_saved_gazebo_markers', 1500)

        self.results_root = str(self.get_parameter('results_root').value)
        self.save_results = bool(self.get_parameter('save_results').value)
        self.show_gazebo_trail = bool(self.get_parameter('show_gazebo_trail').value)
        self.path_sample_distance = max(
            0.001, float(self.get_parameter('path_sample_distance').value))
        self.path_sample_period = max(
            0.0, float(self.get_parameter('path_sample_period').value))
        self.gazebo_marker_distance = max(
            self.path_sample_distance,
            float(self.get_parameter('gazebo_marker_distance').value),
        )
        self.gazebo_marker_radius = max(
            0.005, float(self.get_parameter('gazebo_marker_radius').value))
        self.goal_tolerance = max(0.01, float(self.get_parameter('goal_tolerance').value))
        self.max_saved_gazebo_markers = max(
            1, int(self.get_parameter('max_saved_gazebo_markers').value))

        self.actual_path_pub = self.create_publisher(
            Path, str(self.get_parameter('actual_path_topic').value), 10)
        self.stop_cmd_topic = str(self.get_parameter('stop_cmd_topic').value)

        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self.on_odom, 30)
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('map_topic').value), self.on_map, 2)
        self.create_subscription(
            Path, str(self.get_parameter('planned_path_topic').value),
            self.on_planned_path, 5)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('goal_topic').value), self.on_goal, 5)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('start_pose_applied_topic').value),
            self.on_start_pose_applied,
            5,
        )

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        self.latest_map = None
        self.latest_planned_path = None
        self.goal = None
        self.actual_samples = []
        self.actual_path_msg = Path()
        self.actual_path_msg.header.frame_id = 'map'
        self.last_sample = None
        self.last_marker_sample = None
        self.marker_count = 0
        self.result_saved = False

        self.get_logger().info(
            f'Trajectory recorder ready: publishes /{self.get_parameter("actual_path_topic").value}, '
            f'save_results={self.save_results}, show_gazebo_trail={self.show_gazebo_trail}')

    def on_map(self, msg):
        self.latest_map = msg

    def on_planned_path(self, msg):
        self.latest_planned_path = msg

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.reset_actual_path()
        self.get_logger().info(
            f'Trajectory recorder got new goal: x={self.goal[0]:.2f}, y={self.goal[1]:.2f}')

    def on_start_pose_applied(self, msg):
        self.reset_actual_path()
        self.get_logger().info(
            f'Trajectory reset for new start pose: '
            f'x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')

    def reset_actual_path(self):
        self.result_saved = False
        self.actual_samples = []
        self.actual_path_msg = Path()
        self.actual_path_msg.header.frame_id = 'map'
        self.last_sample = None
        self.last_marker_sample = None
        self.actual_path_pub.publish(self.actual_path_msg)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        sample = {
            'time': stamp_seconds(stamp),
            'x': float(p.x),
            'y': float(p.y),
            'yaw': float(yaw),
        }

        if self.should_append_sample(sample):
            self.append_sample(sample, stamp)

        self.check_goal_reached(sample)

    def should_append_sample(self, sample):
        if self.last_sample is None:
            return True

        distance = math.hypot(
            sample['x'] - self.last_sample['x'],
            sample['y'] - self.last_sample['y'],
        )
        elapsed = sample['time'] - self.last_sample['time']
        if distance >= self.path_sample_distance:
            return True
        return distance > 0.005 and elapsed >= self.path_sample_period

    def append_sample(self, sample, stamp):
        self.actual_samples.append(sample)
        self.last_sample = sample

        self.actual_path_msg.header.stamp = stamp
        self.actual_path_msg.header.frame_id = 'map'
        pose = PoseStamped()
        pose.header = self.actual_path_msg.header
        pose.pose.position.x = sample['x']
        pose.pose.position.y = sample['y']
        pose.pose.orientation.z = math.sin(0.5 * sample['yaw'])
        pose.pose.orientation.w = math.cos(0.5 * sample['yaw'])
        self.actual_path_msg.poses.append(pose)
        self.actual_path_pub.publish(self.actual_path_msg)

        if self.show_gazebo_trail and self.should_spawn_marker(sample):
            self.spawn_gazebo_marker(sample)
            self.last_marker_sample = sample

    def should_spawn_marker(self, sample):
        if self.last_marker_sample is None:
            return True
        distance = math.hypot(
            sample['x'] - self.last_marker_sample['x'],
            sample['y'] - self.last_marker_sample['y'],
        )
        return distance >= self.gazebo_marker_distance

    def spawn_gazebo_marker(self, sample):
        if not self.spawn_client.service_is_ready():
            return
        name = f'actual_path_marker_{self.marker_count:05d}'
        self.marker_count += 1
        request = SpawnEntity.Request()
        request.name = name
        request.xml = self.marker_sdf(
            name, sample['x'], sample['y'], self.gazebo_marker_radius)
        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda _: None)

    @staticmethod
    def marker_sdf(name, x, y, radius):
        return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <pose>{x:.4f} {y:.4f} {radius:.4f} 0 0 0</pose>
    <link name="link">
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>{radius:.4f}</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1.0 0.78 0.12 1</ambient>
          <diffuse>1.0 0.78 0.12 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''

    def check_goal_reached(self, sample):
        if self.goal is None or self.result_saved:
            return
        distance = math.hypot(sample['x'] - self.goal[0], sample['y'] - self.goal[1])
        if distance > self.goal_tolerance:
            return

        self.result_saved = True
        self.publish_stop()
        self.get_logger().info(
            f'Goal reached by actual odom path: distance={distance:.3f} m. '
            'Saving simulation results.')
        if self.save_results:
            self.save_all_results()

    def publish_stop(self):
        stop = Twist()
        stop_pub = self.create_publisher(Twist, self.stop_cmd_topic, 10)
        time.sleep(0.05)
        for _ in range(5):
            stop_pub.publish(stop)
            time.sleep(0.01)
        self.destroy_publisher(stop_pub)

    def save_all_results(self):
        run_name = datetime.now().strftime('run_%Y%m%d_%H%M%S')
        run_dir = os.path.join(self.results_root, run_name)
        os.makedirs(run_dir, exist_ok=True)

        self.save_actual_path(run_dir)
        self.save_planned_path(run_dir)
        self.save_gazebo_path_sdf(run_dir)
        if self.latest_map is not None:
            self.save_map(run_dir)
            self.save_map_with_path_image(run_dir)
        else:
            self.get_logger().warn('No /map has been received; map files were not saved.')

        self.get_logger().info(f'Simulation results saved to: {run_dir}')

    def save_actual_path(self, run_dir):
        csv_path = os.path.join(run_dir, 'actual_path.csv')
        with open(csv_path, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=['time', 'x', 'y', 'yaw'])
            writer.writeheader()
            writer.writerows(self.actual_samples)

        json_path = os.path.join(run_dir, 'actual_path.json')
        with open(json_path, 'w', encoding='utf-8') as json_file:
            json.dump({
                'frame_id': 'map',
                'goal': None if self.goal is None else {'x': self.goal[0], 'y': self.goal[1]},
                'samples': self.actual_samples,
            }, json_file, indent=2)

    def save_planned_path(self, run_dir):
        csv_path = os.path.join(run_dir, 'planned_path.csv')
        with open(csv_path, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(['x', 'y'])
            if self.latest_planned_path is None:
                return
            for pose in self.latest_planned_path.poses:
                writer.writerow([pose.pose.position.x, pose.pose.position.y])

    def save_map(self, run_dir):
        msg = self.latest_map
        width = int(msg.info.width)
        height = int(msg.info.height)
        grid = np.array(msg.data, dtype=np.int16).reshape((height, width))
        image = self.occupancy_to_gray(grid)

        pgm_path = os.path.join(run_dir, 'map.pgm')
        with open(pgm_path, 'wb') as pgm:
            pgm.write(f'P5\n# version_car_sim\n{width} {height}\n255\n'.encode('ascii'))
            pgm.write(np.flipud(image).astype(np.uint8).tobytes())

        yaml_path = os.path.join(run_dir, 'map.yaml')
        origin = msg.info.origin.position
        with open(yaml_path, 'w', encoding='utf-8') as yaml_file:
            yaml_file.write('image: map.pgm\n')
            yaml_file.write(f'resolution: {msg.info.resolution:.6f}\n')
            yaml_file.write(
                f'origin: [{origin.x:.6f}, {origin.y:.6f}, 0.000000]\n')
            yaml_file.write('negate: 0\n')
            yaml_file.write('occupied_thresh: 0.65\n')
            yaml_file.write('free_thresh: 0.196\n')

    @staticmethod
    def occupancy_to_gray(grid):
        image = np.full(grid.shape, 205, dtype=np.uint8)
        image[grid >= 65] = 0
        image[(grid >= 0) & (grid < 65)] = 254
        return image

    def save_map_with_path_image(self, run_dir):
        if Image is None or ImageDraw is None:
            self.get_logger().warn('Pillow is not available; map_with_path.png was not saved.')
            return

        msg = self.latest_map
        width = int(msg.info.width)
        height = int(msg.info.height)
        grid = np.array(msg.data, dtype=np.int16).reshape((height, width))
        gray = np.flipud(self.occupancy_to_gray(grid))
        image = Image.fromarray(gray, mode='L').convert('RGB')
        draw = ImageDraw.Draw(image)

        planned_pixels = []
        if self.latest_planned_path is not None:
            for pose in self.latest_planned_path.poses:
                planned_pixels.append(self.world_to_pixel(
                    pose.pose.position.x, pose.pose.position.y, msg))
        self.draw_polyline(draw, planned_pixels, fill=(30, 120, 255), width=3)

        actual_pixels = [
            self.world_to_pixel(sample['x'], sample['y'], msg)
            for sample in self.actual_samples
        ]
        self.draw_polyline(draw, actual_pixels, fill=(255, 60, 40), width=3)

        if self.goal is not None:
            gx, gy = self.world_to_pixel(self.goal[0], self.goal[1], msg)
            draw.ellipse((gx - 5, gy - 5, gx + 5, gy + 5), fill=(40, 220, 80))

        image.save(os.path.join(run_dir, 'map_with_path.png'))

    @staticmethod
    def draw_polyline(draw, pixels, fill, width):
        pixels = [point for point in pixels if point is not None]
        if len(pixels) >= 2:
            draw.line(pixels, fill=fill, width=width)
        elif len(pixels) == 1:
            x, y = pixels[0]
            draw.ellipse((x - width, y - width, x + width, y + width), fill=fill)

    @staticmethod
    def world_to_pixel(x, y, map_msg):
        resolution = float(map_msg.info.resolution)
        origin = map_msg.info.origin.position
        col = int(round((x - origin.x) / resolution))
        row = int(round((y - origin.y) / resolution))
        if col < 0 or row < 0 or col >= map_msg.info.width or row >= map_msg.info.height:
            return None
        return col, int(map_msg.info.height) - 1 - row

    def save_gazebo_path_sdf(self, run_dir):
        samples = self.downsample_for_gazebo_file()
        path = os.path.join(run_dir, 'gazebo_actual_path.sdf')
        with open(path, 'w', encoding='utf-8') as sdf:
            sdf.write('<?xml version="1.0"?>\n')
            sdf.write('<sdf version="1.6">\n')
            sdf.write('  <model name="saved_actual_path">\n')
            sdf.write('    <static>true</static>\n')
            for index, sample in enumerate(samples):
                sdf.write(f'    <link name="trail_{index:04d}">\n')
                sdf.write(
                    f'      <pose>{sample["x"]:.4f} {sample["y"]:.4f} '
                    f'{self.gazebo_marker_radius:.4f} 0 0 0</pose>\n')
                sdf.write('      <visual name="visual">\n')
                sdf.write('        <geometry><sphere>')
                sdf.write(f'<radius>{self.gazebo_marker_radius:.4f}</radius>')
                sdf.write('</sphere></geometry>\n')
                sdf.write('        <material>')
                sdf.write('<ambient>1.0 0.78 0.12 1</ambient>')
                sdf.write('<diffuse>1.0 0.78 0.12 1</diffuse>')
                sdf.write('</material>\n')
                sdf.write('      </visual>\n')
                sdf.write('    </link>\n')
            sdf.write('  </model>\n')
            sdf.write('</sdf>\n')

    def downsample_for_gazebo_file(self):
        if len(self.actual_samples) <= self.max_saved_gazebo_markers:
            return self.actual_samples
        stride = int(math.ceil(len(self.actual_samples) / self.max_saved_gazebo_markers))
        return self.actual_samples[::stride]


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
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

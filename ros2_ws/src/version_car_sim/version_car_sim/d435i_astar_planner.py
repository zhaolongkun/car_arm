#!/usr/bin/env python3
import heapq
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, String

from version_car_sim.vehicle_geometry import (
    declare_vehicle_safety_parameters,
    read_vehicle_safety_geometry,
)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class D435iAStarPlanner(Node):
    """Local A* planner over the D435i rolling visual costmap."""

    NEIGHBORS = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
    )

    def __init__(self):
        super().__init__('d435i_astar_planner')

        self.declare_parameter('map_topic', 'vision_local_costmap')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('goal_topic', 'goal_pose')
        self.declare_parameter('start_topic', 'start_navigation')
        self.declare_parameter('path_topic', 'planned_path')
        self.declare_parameter('cmd_topic', 'cmd_vel_raw')
        self.declare_parameter('debug_topic', 'planner_debug')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('wheel_base', 0.62)
        self.declare_parameter('max_speed', 0.32)
        self.declare_parameter('max_steer', 0.35)
        self.declare_parameter('max_yaw_rate', 0.35)
        self.declare_parameter('lookahead', 0.85)
        self.declare_parameter('goal_tolerance', 0.30)
        self.declare_parameter('local_goal_distance', 4.5)
        self.declare_parameter('min_local_goal_distance', 0.8)
        self.declare_parameter('obstacle_threshold', 55)
        self.declare_parameter('unknown_cost', 0.0)
        self.declare_parameter('replan_period', 0.35)
        self.declare_parameter('start_clearance_margin', 0.50)
        declare_vehicle_safety_parameters(self)

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.local_goal_distance = float(
            self.get_parameter('local_goal_distance').value)
        self.min_local_goal_distance = float(
            self.get_parameter('min_local_goal_distance').value)
        self.obstacle_threshold = int(self.get_parameter('obstacle_threshold').value)
        self.unknown_cost = float(self.get_parameter('unknown_cost').value)
        self.replan_period = float(self.get_parameter('replan_period').value)
        self.start_clearance_margin = max(
            0.0, float(self.get_parameter('start_clearance_margin').value))
        self.safety_geometry = read_vehicle_safety_geometry(self)
        self.vehicle_safety_radius = self.safety_geometry['vehicle_safety_radius']

        self.map_msg = None
        self.grid = None
        self.map_version = 0
        self.planned_map_version = -1
        self.odom_pose = None
        self.goal = None
        self.local_goal = None
        self.path = []
        self.last_plan_time = 0.0
        self.goal_reached = False
        self.navigation_started = bool(self.get_parameter('auto_start').value)

        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter('path_topic').value), 1)
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_topic').value), 10)
        self.debug_pub = self.create_publisher(
            String, str(self.get_parameter('debug_topic').value), 10)

        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('map_topic').value), self.on_map, 5)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self.on_odom, 30)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('goal_topic').value), self.on_goal, 5)
        self.create_subscription(
            Bool, str(self.get_parameter('start_topic').value),
            self.on_start_navigation, 5)

        self.create_timer(0.1, self.on_timer)

        self.get_logger().info(
            'D435i local A* planner ready: /vision_local_costmap + /goal_pose '
            '-> /planned_path + /cmd_vel_raw')
        self.get_logger().info(
            f'A* uses inflated visual costmap built with vehicle_safety_radius='
            f'{self.vehicle_safety_radius:.3f} m '
            f'(max_wheel_distance={self.safety_geometry["max_distance"]:.3f} m, '
            f'scale={self.safety_geometry["scale"]:.2f}); start requires an extra '
            f'{self.start_clearance_margin:.2f} m clearance from the inflated boundary.')
        if self.navigation_started:
            self.get_logger().info('Navigation auto_start is enabled.')
        else:
            self.get_logger().info(
                'Navigation is waiting. Publish std_msgs/Bool true on '
                f'/{self.get_parameter("start_topic").value} to start moving.')

    def on_map(self, msg):
        self.map_msg = msg
        self.grid = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width))
        self.map_version += 1

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_pose = (float(p.x), float(p.y), float(yaw))

    def on_goal(self, msg):
        self.goal = (float(msg.pose.position.x), float(msg.pose.position.y))
        self.local_goal = None
        self.path = []
        self.planned_map_version = -1
        self.last_plan_time = 0.0
        self.goal_reached = False
        self.get_logger().info(f'New D435i visual goal: x={self.goal[0]:.2f}, y={self.goal[1]:.2f}')

    def on_start_navigation(self, msg):
        if msg.data:
            if self.can_check_start_safety():
                blocked = self.build_blocked_grid()
                status = self.evaluate_start_goal(blocked, require_clearance=True)
                if status['start_unsafe']:
                    self.navigation_started = False
                    self.path = []
                    self.publish_stop()
                    self.publish_planner_debug(
                        'START_UNSAFE',
                        status,
                        path_found=False,
                        message=(
                            'base_link safety circle is already occupied or too close '
                            'to an inflated D435i obstacle; move the start pose or '
                            'the obstacle before starting navigation.'),
                    )
                    self.get_logger().warn(
                        'Refusing /start_navigation: base_link safety circle is unsafe. '
                        'Move the obstacle/start pose and try again.',
                        throttle_duration_sec=1.0)
                    return

            if not self.navigation_started:
                self.navigation_started = True
                self.path = []
                self.last_plan_time = 0.0
                self.goal_reached = False
                self.get_logger().info('D435i visual navigation started by /start_navigation.')
            return

        if self.navigation_started:
            self.get_logger().info('D435i visual navigation paused by /start_navigation=false.')
        self.navigation_started = False
        self.path = []
        self.publish_stop()

    def on_timer(self):
        if not self.navigation_started:
            return
        if self.map_msg is None or self.grid is None or self.odom_pose is None:
            self.publish_stop()
            return
        blocked = self.build_blocked_grid()
        status = self.evaluate_start_goal(blocked)
        if status['start_unsafe']:
            self.path = []
            self.publish_stop()
            self.publish_planner_debug(
                'START_UNSAFE',
                status,
                path_found=False,
                message=(
                    'base_link safety circle is unsafe at navigation start/current pose; '
                    'holding zero cmd_vel_raw.'),
            )
            self.get_logger().warn(
                'base_link safety circle overlaps or is too close to an inflated '
                'D435i obstacle; holding position.',
                throttle_duration_sec=1.0)
            return
        if self.goal is None:
            self.get_logger().warn(
                'Navigation start requested but no /goal_pose has been received yet.',
                throttle_duration_sec=2.0)
            self.publish_stop()
            self.publish_planner_debug(
                'NO_GOAL',
                status,
                path_found=False,
                message='navigation is started but no /goal_pose has been received.')
            return
        if self.goal_reached:
            self.publish_stop()
            return

        x, y, _ = self.odom_pose
        if math.hypot(self.goal[0] - x, self.goal[1] - y) <= self.goal_tolerance:
            self.goal_reached = True
            self.publish_stop()
            self.get_logger().info('D435i visual goal reached.')
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        needs_plan = (
            not self.path or
            (self.map_version != self.planned_map_version and
             now - self.last_plan_time >= self.replan_period)
        )
        if needs_plan:
            self.plan_from_current_pose(now)

        self.follow_path()

    def plan_from_current_pose(self, now):
        blocked = self.build_blocked_grid()
        status = self.evaluate_start_goal(blocked)
        start = status['start_cell']
        if start is None:
            self.get_logger().warn('Current pose is outside the D435i local costmap.')
            self.path = []
            self.publish_stop()
            self.publish_planner_debug(
                'START_OUT_OF_COSTMAP',
                status,
                path_found=False,
                message='current base_link pose is outside /vision_local_costmap.')
            return

        if status['start_unsafe']:
            state = 'START_IN_COLLISION' if status['start_in_collision'] else 'START_UNSAFE'
            self.get_logger().warn(
                'Current base_link center is inside the inflated D435i safety costmap; '
                'holding position until the vehicle safety circle is clear.',
                throttle_duration_sec=1.0)
            self.path = []
            self.publish_stop()
            self.publish_planner_debug(
                state,
                status,
                path_found=False,
                message=(
                    'start cell is occupied or the start pose is too close to '
                    'the inflated vehicle safety boundary.'),
            )
            return

        if self.goal is not None and not status['goal_valid']:
            self.get_logger().warn(
                'D435i goal is inside the inflated safety map; '
                'holding zero cmd_vel_raw.',
                throttle_duration_sec=1.0)
            self.path = []
            self.publish_stop()
            self.publish_planner_debug(
                'GOAL_IN_COLLISION' if status['goal_in_collision'] else 'GOAL_INVALID',
                status,
                path_found=False,
                message='goal pose is not safely reachable in the current visual costmap.')
            return

        goal = self.choose_local_goal(blocked)
        if goal is None:
            self.get_logger().warn('No reachable local D435i planning goal is available.')
            self.path = []
            self.publish_stop()
            self.publish_planner_debug(
                'NO_LOCAL_GOAL',
                status,
                path_found=False,
                message='no free local planning target was found in the inflated map.')
            return

        cell_path = self.astar(start, goal, blocked)
        if not cell_path:
            self.get_logger().warn('D435i local A* did not find a path.')
            self.path = []
            self.publish_stop()
            self.publish_planner_debug(
                'NO_PATH',
                status,
                path_found=False,
                local_goal_cell=goal,
                message='A* could not find a collision-free path through the inflated map.')
            return
        if not self.path_is_safe(cell_path, blocked):
            self.get_logger().warn(
                'D435i local A* produced a path that touches the inflated safety map; '
                'discarding it and publishing zero velocity.',
                throttle_duration_sec=1.0)
            self.path = []
            self.publish_stop()
            self.publish_planner_debug(
                'PATH_COLLIDES',
                status,
                path_found=False,
                local_goal_cell=goal,
                message='computed path touches the inflated vehicle safety costmap.')
            return

        self.path = self.cells_to_sparse_world_path(cell_path)
        self.local_goal = self.cell_to_world(goal[0], goal[1])
        self.last_plan_time = now
        self.planned_map_version = self.map_version
        self.publish_path()
        self.publish_planner_debug(
            'PATH_FOUND',
            status,
            path_found=True,
            local_goal_cell=goal,
            path_length=len(cell_path),
            message='A* found a path that stays outside the inflated vehicle safety map.')

    def choose_local_goal(self, blocked):
        x, y, _ = self.odom_pose
        goal_dx = self.goal[0] - x
        goal_dy = self.goal[1] - y
        goal_distance = math.hypot(goal_dx, goal_dy)
        if goal_distance <= 1e-6:
            return self.world_to_cell(x, y)

        direction_x = goal_dx / goal_distance
        direction_y = goal_dy / goal_distance
        max_distance = min(self.local_goal_distance, goal_distance)
        min_distance = min(self.min_local_goal_distance, max_distance)
        samples = np.linspace(max_distance, min_distance, 24)

        best = None
        for distance in samples:
            wx = x + direction_x * float(distance)
            wy = y + direction_y * float(distance)
            cell = self.world_to_cell(wx, wy)
            if cell is None:
                continue
            free = self.nearest_free(cell, blocked, max_radius=8)
            if free is not None:
                best = free
                break

        if best is not None:
            return best

        goal_cell = self.world_to_cell(self.goal[0], self.goal[1])
        if goal_cell is not None:
            return self.nearest_free(goal_cell, blocked, max_radius=8)
        return None

    def build_blocked_grid(self):
        return self.grid >= self.obstacle_threshold

    def can_check_start_safety(self):
        return self.map_msg is not None and self.grid is not None and self.odom_pose is not None

    def evaluate_start_goal(self, blocked, require_clearance=False):
        start_cell = None
        if self.odom_pose is not None:
            start_cell = self.world_to_cell(self.odom_pose[0], self.odom_pose[1])
        start_in_collision = (
            start_cell is not None and bool(blocked[start_cell[1], start_cell[0]])
        )
        start_valid = start_cell is not None and not start_in_collision
        nearest_inflated_distance = self.nearest_blocked_distance(start_cell, blocked)
        start_clearance_valid = (
            math.isinf(nearest_inflated_distance) or
            nearest_inflated_distance >= self.start_clearance_margin
        )
        start_unsafe = (not start_valid) or (
            bool(require_clearance) and not start_clearance_valid)
        nearest_obstacle_distance = (
            math.inf if math.isinf(nearest_inflated_distance)
            else nearest_inflated_distance + self.vehicle_safety_radius
        )

        goal_cell = None
        goal_in_costmap = False
        goal_in_collision = False
        goal_valid = False
        if self.goal is not None:
            goal_cell = self.world_to_cell(self.goal[0], self.goal[1])
            goal_in_costmap = goal_cell is not None
            if goal_cell is None:
                goal_valid = True
            else:
                goal_in_collision = bool(blocked[goal_cell[1], goal_cell[0]])
                goal_valid = not goal_in_collision

        return {
            'start_cell': start_cell,
            'goal_cell': goal_cell,
            'start_valid': start_valid,
            'goal_valid': goal_valid,
            'goal_in_costmap': goal_in_costmap,
            'start_in_collision': start_in_collision,
            'goal_in_collision': goal_in_collision,
            'start_clearance_valid': start_clearance_valid,
            'start_clearance_required': bool(require_clearance),
            'start_unsafe': start_unsafe,
            'nearest_inflated_obstacle_distance': nearest_inflated_distance,
            'nearest_obstacle_distance': nearest_obstacle_distance,
            'distance_to_safety_boundary': nearest_inflated_distance,
        }

    def nearest_blocked_distance(self, cell, blocked):
        if cell is None:
            return math.inf
        rows, cols = np.nonzero(blocked)
        if rows.size == 0:
            return math.inf
        d_cells = np.hypot(cols.astype(np.float32) - cell[0],
                           rows.astype(np.float32) - cell[1])
        return float(np.min(d_cells) * self.map_msg.info.resolution)

    @staticmethod
    def path_is_safe(cell_path, blocked):
        for col, row in cell_path:
            if blocked[row, col]:
                return False
        return True

    def nearest_free(self, cell, blocked, max_radius=20):
        col, row = cell
        if self.in_bounds(col, row) and not blocked[row, col]:
            return cell

        for radius in range(1, max_radius + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    c = col + dx
                    r = row + dy
                    if self.in_bounds(c, r) and not blocked[r, c]:
                        return c, r
        return None

    def astar(self, start, goal, blocked):
        width = int(self.map_msg.info.width)
        height = int(self.map_msg.info.height)
        g_score = np.full((height, width), np.inf, dtype=np.float32)
        closed = np.zeros((height, width), dtype=np.bool_)
        came_from = np.full((height, width, 2), -1, dtype=np.int32)

        sx, sy = start
        gx, gy = goal
        g_score[sy, sx] = 0.0
        queue = []
        counter = 0
        heapq.heappush(queue, (self.heuristic(start, goal), counter, sx, sy))

        while queue:
            _, _, col, row = heapq.heappop(queue)
            if closed[row, col]:
                continue
            if (col, row) == goal:
                return self.reconstruct_path(came_from, start, goal)

            closed[row, col] = True
            for dx, dy, step_cost in self.NEIGHBORS:
                next_col = col + dx
                next_row = row + dy
                if not self.in_bounds(next_col, next_row):
                    continue
                if blocked[next_row, next_col] or closed[next_row, next_col]:
                    continue

                extra = self.unknown_cost if self.grid[next_row, next_col] < 0 else 0.0
                tentative = g_score[row, col] + step_cost + extra
                if tentative < g_score[next_row, next_col]:
                    came_from[next_row, next_col] = (col, row)
                    g_score[next_row, next_col] = tentative
                    counter += 1
                    f_score = tentative + self.heuristic((next_col, next_row), goal)
                    heapq.heappush(queue, (f_score, counter, next_col, next_row))

        return []

    @staticmethod
    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def reconstruct_path(came_from, start, goal):
        path = [goal]
        current = goal
        while current != start:
            prev = came_from[current[1], current[0]]
            if prev[0] < 0:
                return []
            current = (int(prev[0]), int(prev[1]))
            path.append(current)
        path.reverse()
        return path

    def cells_to_sparse_world_path(self, cell_path):
        stride = max(1, int(round(0.18 / self.map_msg.info.resolution)))
        sparse = []
        for index, cell in enumerate(cell_path):
            if index % stride == 0 or index == len(cell_path) - 1:
                sparse.append(self.cell_to_world(cell[0], cell[1]))
        return sparse

    def follow_path(self):
        if not self.path:
            self.publish_stop()
            return

        current_cell = self.world_to_cell(self.odom_pose[0], self.odom_pose[1])
        if current_cell is None:
            self.publish_stop()
            self.publish_planner_debug(
                'START_OUT_OF_COSTMAP',
                self.evaluate_start_goal(self.build_blocked_grid()),
                path_found=False,
                message='current base_link pose left /vision_local_costmap while following path.')
            return
        blocked = self.build_blocked_grid()
        status = self.evaluate_start_goal(blocked)
        if status['start_unsafe']:
            self.get_logger().warn(
                'Vehicle safety circle is blocked in the current visual costmap; '
                'stopping and waiting for a safe replan.',
                throttle_duration_sec=1.0)
            self.path = []
            self.publish_stop()
            self.publish_planner_debug(
                'START_IN_COLLISION' if status['start_in_collision'] else 'START_UNSAFE',
                status,
                path_found=False,
                message='current path following pose is no longer safe.')
            return

        x, y, yaw = self.odom_pose
        nearest_index = min(
            range(len(self.path)),
            key=lambda i: math.hypot(self.path[i][0] - x, self.path[i][1] - y))
        target = self.path[-1]
        for point in self.path[nearest_index:]:
            if math.hypot(point[0] - x, point[1] - y) >= self.lookahead:
                target = point
                break

        dx = target[0] - x
        dy = target[1] - y
        x_body = math.cos(yaw) * dx + math.sin(yaw) * dy
        y_body = -math.sin(yaw) * dx + math.cos(yaw) * dy
        lookahead_distance = max(0.1, math.hypot(x_body, y_body))
        alpha = math.atan2(y_body, max(0.05, x_body))
        global_goal_distance = math.hypot(self.goal[0] - x, self.goal[1] - y)
        local_goal_distance = math.hypot(self.path[-1][0] - x, self.path[-1][1] - y)
        goal_slowdown = max(0.25, min(1.0, global_goal_distance / 1.2))
        local_slowdown = max(0.35, min(1.0, local_goal_distance / 0.8))
        base_speed = self.max_speed * goal_slowdown * local_slowdown
        curvature = 2.0 * math.sin(alpha) / lookahead_distance
        desired_yaw_rate = max(
            -self.max_yaw_rate,
            min(self.max_yaw_rate, base_speed * curvature))
        angular_ratio = abs(desired_yaw_rate) / max(1e-6, self.max_yaw_rate)
        turn_slowdown = 1.0 - 0.50 * min(1.0, angular_ratio)
        speed = base_speed * turn_slowdown
        yaw_rate = speed * curvature
        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate))

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = yaw_rate
        self.cmd_pub.publish(cmd)

    def publish_planner_debug(
            self, state, status, path_found=False, local_goal_cell=None,
            path_length=0, message=''):
        msg = String()
        local_goal = None
        if local_goal_cell is not None:
            local_goal = self.cell_to_world(local_goal_cell[0], local_goal_cell[1])
        start_cell = status.get('start_cell')
        goal_cell = status.get('goal_cell')
        msg.data = json.dumps({
            'planner_state': state,
            'message': message,
            'start_valid': bool(status.get('start_valid', False)),
            'goal_valid': bool(status.get('goal_valid', False)),
            'goal_in_costmap': bool(status.get('goal_in_costmap', False)),
            'path_found': bool(path_found),
            'start_in_collision': bool(status.get('start_in_collision', False)),
            'goal_in_collision': bool(status.get('goal_in_collision', False)),
            'start_unsafe': bool(status.get('start_unsafe', False)),
            'start_clearance_valid': bool(status.get('start_clearance_valid', False)),
            'start_clearance_required': bool(status.get('start_clearance_required', False)),
            'nearest_obstacle_distance': self.debug_distance(
                status.get('nearest_obstacle_distance', math.inf)),
            'nearest_inflated_obstacle_distance': self.debug_distance(
                status.get('nearest_inflated_obstacle_distance', math.inf)),
            'distance_to_safety_boundary': self.debug_distance(
                status.get('distance_to_safety_boundary', math.inf)),
            'vehicle_safety_radius': float(self.vehicle_safety_radius),
            'start_clearance_margin': float(self.start_clearance_margin),
            'required_start_distance_from_obstacle': float(
                self.vehicle_safety_radius + self.start_clearance_margin),
            'obstacle_threshold': int(self.obstacle_threshold),
            'map_frame': self.map_msg.header.frame_id if self.map_msg else '',
            'start_cell': [int(start_cell[0]), int(start_cell[1])]
            if start_cell is not None else None,
            'goal_cell': [int(goal_cell[0]), int(goal_cell[1])]
            if goal_cell is not None else None,
            'local_goal_cell': [int(local_goal_cell[0]), int(local_goal_cell[1])]
            if local_goal_cell is not None else None,
            'start_pose': {
                'x': float(self.odom_pose[0]),
                'y': float(self.odom_pose[1]),
            } if self.odom_pose is not None else None,
            'goal_pose': {
                'x': float(self.goal[0]),
                'y': float(self.goal[1]),
            } if self.goal is not None else None,
            'local_goal_pose': {
                'x': float(local_goal[0]),
                'y': float(local_goal[1]),
            } if local_goal is not None else None,
            'path_length_cells': int(path_length),
            'using_vehicle_safety_radius': True,
        }, sort_keys=True)
        self.debug_pub.publish(msg)

    def publish_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_msg.header.frame_id or 'map'
        for x, y in self.path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def world_to_cell(self, x, y):
        info = self.map_msg.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        if not self.in_bounds(col, row):
            return None
        return col, row

    def cell_to_world(self, col, row):
        info = self.map_msg.info
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return x, y

    def in_bounds(self, col, row):
        if self.map_msg is None:
            return False
        return 0 <= col < self.map_msg.info.width and 0 <= row < self.map_msg.info.height

    @staticmethod
    def debug_distance(value):
        if value is None or not math.isfinite(value):
            return None
        return float(value)


def main(args=None):
    rclpy.init(args=args)
    node = D435iAStarPlanner()
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

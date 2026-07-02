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
from visualization_msgs.msg import Marker


def clamp01(value):
    return max(0.0, min(1.0, value))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class AStarPlanner(Node):
    """A* global planner plus pure-pursuit controller publishing raw cmd_vel."""

    NEIGHBORS = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
    )

    def __init__(self):
        super().__init__('astar_planner')

        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('goal_topic', 'goal_pose')
        self.declare_parameter('cmd_topic', 'cmd_vel_raw')
        self.declare_parameter('path_topic', 'planned_path')
        self.declare_parameter('direct_path_topic', 'direct_path')
        self.declare_parameter('planning_corridor_topic', 'planning_corridor')
        self.declare_parameter('inflated_map_topic', 'inflated_map')
        self.declare_parameter('costmap_debug_topic', 'costmap_debug')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('start_topic', 'start_navigation')
        self.declare_parameter('replan_request_topic', 'replan_requested')
        self.declare_parameter('wheel_base', 0.62)
        self.declare_parameter('track_width', 0.62)
        self.declare_parameter('safety_diameter_scale', 1.5)
        self.declare_parameter('max_speed', 0.75)
        self.declare_parameter('max_steer', 0.6458)
        self.declare_parameter('max_yaw_rate', 1.2)
        self.declare_parameter('lookahead', 0.75)
        self.declare_parameter('min_lookahead', 0.75)
        self.declare_parameter('max_lookahead', 1.80)
        self.declare_parameter('lookahead_clearance_distance', 2.50)
        self.declare_parameter('steering_smoothing_alpha', 0.50)
        self.declare_parameter('goal_tolerance', 0.28)
        self.declare_parameter('robot_radius', 0.0)
        self.declare_parameter('safety_margin', 0.0)
        self.declare_parameter('inflation_radius', 0.0)
        self.declare_parameter('obstacle_threshold', 55)
        self.declare_parameter('allow_unknown', True)
        self.declare_parameter('unknown_cost', 0.15)
        self.declare_parameter('replan_period', 1.0)
        self.declare_parameter('debug_topic', 'planner_debug')
        self.declare_parameter('boundary_margin', 0.20)
        self.declare_parameter('enforce_map_boundaries', True)
        self.declare_parameter('pass_margin', 0.25)
        self.declare_parameter('planning_corridor_half_width', 3.0)
        self.declare_parameter('max_planning_corridor_half_width', 8.0)
        self.declare_parameter('corridor_expansion_step', 1.0)
        self.declare_parameter('soft_inflation_radius', 0.0)
        self.declare_parameter('soft_cost_weight', 1.2)
        self.declare_parameter('corridor_center_weight', 0.7)

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.safety_diameter_scale = float(
            self.get_parameter('safety_diameter_scale').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.min_lookahead = max(
            0.30, float(self.get_parameter('min_lookahead').value))
        self.max_lookahead = max(
            self.min_lookahead, float(self.get_parameter('max_lookahead').value))
        self.lookahead_clearance_distance = max(
            self.max_lookahead,
            float(self.get_parameter('lookahead_clearance_distance').value))
        self.steering_smoothing_alpha = clamp01(
            float(self.get_parameter('steering_smoothing_alpha').value))
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        configured_robot_radius = float(self.get_parameter('robot_radius').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        configured_inflation_radius = float(
            self.get_parameter('inflation_radius').value)
        self.vehicle_safety_radius = self.compute_vehicle_safety_radius(
            configured_robot_radius)
        self.inflation_radius = configured_inflation_radius
        minimum_inflation_radius = self.vehicle_safety_radius + self.safety_margin
        if self.inflation_radius <= 0.0:
            self.inflation_radius = minimum_inflation_radius
        else:
            self.inflation_radius = max(self.inflation_radius, minimum_inflation_radius)
        self.obstacle_threshold = int(self.get_parameter('obstacle_threshold').value)
        self.allow_unknown = bool(self.get_parameter('allow_unknown').value)
        self.unknown_cost = float(self.get_parameter('unknown_cost').value)
        self.replan_period = float(self.get_parameter('replan_period').value)
        self.boundary_margin = max(
            0.0, float(self.get_parameter('boundary_margin').value))
        self.enforce_map_boundaries = bool(
            self.get_parameter('enforce_map_boundaries').value)
        self.pass_margin = max(0.0, float(self.get_parameter('pass_margin').value))
        self.planning_corridor_half_width = max(
            self.vehicle_safety_radius + 0.25,
            float(self.get_parameter('planning_corridor_half_width').value),
        )
        self.max_planning_corridor_half_width = max(
            self.planning_corridor_half_width,
            float(self.get_parameter('max_planning_corridor_half_width').value),
        )
        self.corridor_expansion_step = max(
            self.map_resolution_floor(),
            float(self.get_parameter('corridor_expansion_step').value),
        )
        configured_soft_inflation_radius = float(
            self.get_parameter('soft_inflation_radius').value)
        self.soft_inflation_radius = configured_soft_inflation_radius
        if self.soft_inflation_radius <= self.inflation_radius:
            self.soft_inflation_radius = self.inflation_radius + 0.3
        self.soft_cost_weight = max(
            0.0, float(self.get_parameter('soft_cost_weight').value))
        self.corridor_center_weight = max(
            0.0, float(self.get_parameter('corridor_center_weight').value))

        self.map_msg = None
        self.grid = None
        self.odom_pose = None
        self.path = []
        self.goal = None
        self.active_goal = None
        self.map_version = 0
        self.planned_map_version = -1
        self.last_plan_time = 0.0
        self.goal_reached = False
        self.navigation_started = bool(self.get_parameter('auto_start').value)
        self.last_plan_debug = {}
        self.filtered_steer = 0.0

        self.get_logger().info(
            'No default goal is active. Set /goal_pose from RViz 2D Goal Pose '
            'or publish geometry_msgs/PoseStamped before starting navigation.')

        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter('path_topic').value), 1)
        self.direct_path_pub = self.create_publisher(
            Path, str(self.get_parameter('direct_path_topic').value), 1)
        self.corridor_pub = self.create_publisher(
            Marker, str(self.get_parameter('planning_corridor_topic').value), 1)
        self.inflated_map_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('inflated_map_topic').value), 1)
        self.costmap_debug_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('costmap_debug_topic').value), 1)
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_topic').value), 10)
        self.debug_pub = self.create_publisher(
            String, str(self.get_parameter('debug_topic').value), 10)

        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('map_topic').value), self.on_map, 5)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self.on_odom, 20)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('goal_topic').value), self.on_goal, 5)
        self.create_subscription(
            Bool, str(self.get_parameter('start_topic').value), self.on_start_navigation, 5)
        self.create_subscription(
            Bool, str(self.get_parameter('replan_request_topic').value),
            self.on_replan_request, 5)

        self.create_timer(0.1, self.on_timer)

        if self.navigation_started:
            self.get_logger().info('Navigation auto_start is enabled.')
        else:
            self.get_logger().info(
                'Navigation is waiting. Publish std_msgs/Bool true on '
                f'/{self.get_parameter("start_topic").value} to start moving.')
        self.get_logger().info(
            f'Vehicle safety radius={self.vehicle_safety_radius:.3f} m, '
            f'A* inflation_radius={self.inflation_radius:.3f} m '
            f'(wheel_base={self.wheel_base:.3f}, track_width={self.track_width:.3f}, '
            f'scale={self.safety_diameter_scale:.2f}); map boundary margin='
            f'{self.boundary_margin:.3f} m, corridor half-width='
            f'{self.planning_corridor_half_width:.2f}-{self.max_planning_corridor_half_width:.2f} m, '
            f'adaptive lookahead={self.min_lookahead:.2f}-{self.max_lookahead:.2f} m.')

    def compute_vehicle_safety_radius(self, configured_robot_radius):
        if configured_robot_radius > 0.0:
            return configured_robot_radius
        diagonal = math.hypot(self.wheel_base, self.track_width)
        return 0.5 * self.safety_diameter_scale * diagonal

    @staticmethod
    def map_resolution_floor():
        return 0.05

    def on_map(self, msg):
        self.map_msg = msg
        self.grid = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width))
        self.map_version += 1

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_pose = (p.x, p.y, yaw)

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.active_goal = self.goal
        self.path = []
        self.planned_map_version = -1
        self.last_plan_time = 0.0
        self.goal_reached = False
        self.filtered_steer = 0.0
        self.get_logger().info(f'New goal: x={self.goal[0]:.2f}, y={self.goal[1]:.2f}')

    def on_start_navigation(self, msg):
        if msg.data:
            if not self.navigation_started:
                self.navigation_started = True
                self.last_plan_time = 0.0
                self.path = []
                self.goal_reached = False
                self.get_logger().info('Navigation started by /start_navigation.')
            return

        if self.navigation_started:
            self.get_logger().info('Navigation paused by /start_navigation=false.')
        self.navigation_started = False
        self.path = []
        self.active_goal = None
        self.filtered_steer = 0.0
        self.publish_stop()

    def on_replan_request(self, msg):
        if not msg.data:
            return
        if not self.navigation_started:
            return
        self.path = []
        self.planned_map_version = -1
        self.last_plan_time = 0.0
        self.get_logger().info(
            'Received /replan_requested from local avoidance; replanning from current pose.',
            throttle_duration_sec=1.0)

    def on_timer(self):
        if not self.navigation_started:
            return
        if self.map_msg is None or self.grid is None or self.odom_pose is None:
            self.publish_stop()
            return
        if self.goal is None:
            self.get_logger().warn(
                'Navigation start requested but no /goal_pose has been received yet.',
                throttle_duration_sec=2.0)
            self.publish_stop()
            return
        if self.goal_reached:
            self.publish_stop()
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
        start_x = self.odom_pose[0]
        start_y = self.odom_pose[1]
        goal_x = self.goal[0]
        goal_y = self.goal[1]
        safety_clearance = self.vehicle_safety_radius + self.boundary_margin
        start_inside_map = self.point_inside_map(start_x, start_y, safety_clearance)
        goal_inside_map = self.point_inside_map(goal_x, goal_y, safety_clearance)

        if not start_inside_map or not goal_inside_map:
            state = 'START_OUT_OF_MAP' if not start_inside_map else 'GOAL_OUT_OF_MAP'
            self.get_logger().warn(
                f'{state}: refusing to plan outside the safe map boundary.')
            self.path = []
            self.publish_stop()
            self.publish_debug(
                planner_state=state,
                start_valid=start_inside_map,
                goal_valid=goal_inside_map,
                path_found=False,
                path_has_out_of_map_point=False,
            )
            return

        start = self.world_to_cell(self.odom_pose[0], self.odom_pose[1])
        goal = self.world_to_cell(self.goal[0], self.goal[1])
        if start is None or goal is None:
            self.get_logger().warn('Start or goal is outside the mapping area.')
            self.path = []
            self.publish_stop()
            self.publish_debug(
                planner_state='START_OR_GOAL_OUT_OF_MAP',
                start_valid=start is not None,
                goal_valid=goal is not None,
                path_found=False,
                path_has_out_of_map_point=False,
            )
            return

        blocked = self.build_blocked_grid()
        occupied = self.build_occupied_grid()
        soft_cost = self.build_soft_costmap(occupied, blocked)
        self.publish_inflated_map(blocked)
        self.publish_costmap_debug(soft_cost)

        required_pass_width = 2.0 * self.vehicle_safety_radius + self.pass_margin
        plan_debug = {
            'direct_path_available': False,
            'direct_path_blocked': None,
            'nearest_gap_width': None,
            'required_pass_width': float(required_pass_width),
            'gap_passable': None,
            'selected_passage_type': 'UNSELECTED',
            'hard_collision_radius': float(self.inflation_radius),
            'soft_inflation_radius': float(self.soft_inflation_radius),
            'boundary_state': 'INSIDE_MAP',
            'adjusted_goal_x': None,
            'adjusted_goal_y': None,
        }

        start_valid = not blocked[start[1], start[0]]
        goal_valid = not blocked[goal[1], goal[0]]
        if not start_valid and not occupied[start[1], start[0]]:
            recovery_start = self.nearest_free(
                start,
                blocked,
                max_radius=max(20, int(math.ceil(
                    2.0 * self.vehicle_safety_radius / self.map_msg.info.resolution))),
            )
            if recovery_start is not None:
                self.get_logger().warn(
                    'Current pose is inside an inflated obstacle buffer, but not an '
                    'occupied cell; replanning from the nearest safe cell.')
                start = recovery_start
                start_valid = True
            else:
                self.get_logger().warn(
                    'Current pose is inside an inflated obstacle buffer and no nearby '
                    'safe recovery cell is available.')
                self.path = []
                self.publish_stop()
                self.publish_debug(
                    planner_state='START_IN_INFLATED_ZONE',
                    start_valid=False,
                    goal_valid=goal_valid,
                    path_found=False,
                    path_has_out_of_map_point=False,
                    **dict(plan_debug, selected_passage_type='NO_SAFE_START'),
                )
                return

        if not goal_valid:
            recovery_goal = self.nearest_free(
                goal,
                blocked,
                max_radius=max(20, int(math.ceil(
                    2.5 * self.vehicle_safety_radius / self.map_msg.info.resolution))),
            )
            if recovery_goal is not None:
                adjusted_goal_x, adjusted_goal_y = self.cell_to_world(
                    recovery_goal[0], recovery_goal[1])
                self.get_logger().warn(
                    'Requested goal is inside an obstacle safety buffer; planning to '
                    f'the nearest safe point ({adjusted_goal_x:.2f}, {adjusted_goal_y:.2f}).',
                    throttle_duration_sec=1.0)
                goal = recovery_goal
                goal_x = adjusted_goal_x
                goal_y = adjusted_goal_y
                goal_valid = True
                plan_debug['adjusted_goal_x'] = float(adjusted_goal_x)
                plan_debug['adjusted_goal_y'] = float(adjusted_goal_y)

        if (not start_valid and occupied[start[1], start[0]]) or not goal_valid:
            state = 'START_IN_COLLISION' if not start_valid else 'GOAL_IN_COLLISION'
            self.get_logger().warn(
                f'{state}: start/goal is occupied, inflated, or too close to map boundary.')
            self.path = []
            self.publish_stop()
            self.publish_debug(
                planner_state=state,
                start_valid=start_valid,
                goal_valid=goal_valid,
                path_found=False,
                path_has_out_of_map_point=False,
                **dict(plan_debug, selected_passage_type=state),
            )
            return

        direct_cells = self.bresenham_cells(start[0], start[1], goal[0], goal[1])
        direct_path = self.cells_to_sparse_world_path(direct_cells)
        if direct_path:
            direct_path[0] = (start_x, start_y)
            direct_path[-1] = (goal_x, goal_y)
        self.publish_direct_path(direct_path)

        direct_path_available, direct_blocked_cell = self.direct_cells_available(
            direct_cells, blocked)
        nearest_gap_width = self.estimate_gap_width(direct_blocked_cell, occupied)
        gap_passable = (
            nearest_gap_width is not None and nearest_gap_width >= required_pass_width)
        plan_debug.update({
            'direct_path_available': bool(direct_path_available),
            'direct_path_blocked': bool(not direct_path_available),
            'nearest_gap_width': (
                None if nearest_gap_width is None else float(nearest_gap_width)),
            'gap_passable': bool(gap_passable),
        })

        selected_corridor_half_width = None
        if direct_path_available:
            self.path = self.make_direct_world_path((start_x, start_y), (goal_x, goal_y))
            plan_debug['selected_passage_type'] = 'DIRECT'
            selected_corridor_half_width = max(
                self.planning_corridor_half_width, 0.5 * required_pass_width)
            self.publish_planning_corridor(
                (start_x, start_y), (goal_x, goal_y), selected_corridor_half_width)
        else:
            cell_path = []
            half_width = self.planning_corridor_half_width
            selected_corridor_half_width = half_width
            start_world = self.cell_to_world(start[0], start[1])
            goal_world = self.cell_to_world(goal[0], goal[1])
            while half_width <= self.max_planning_corridor_half_width + 1e-6:
                selected_corridor_half_width = half_width
                corridor_mask = self.build_corridor_mask(
                    start_world, goal_world, half_width, safety_clearance)
                corridor_blocked = np.logical_or(blocked, np.logical_not(corridor_mask))
                corridor_blocked[start[1], start[0]] = False
                corridor_blocked[goal[1], goal[0]] = False
                candidate = self.astar(
                    start,
                    goal,
                    corridor_blocked,
                    soft_cost=soft_cost,
                    corridor_line=(start_world, goal_world),
                    corridor_half_width=half_width,
                )
                if candidate:
                    cell_path = self.shortcut_cell_path(candidate, corridor_blocked)
                    plan_debug['selected_passage_type'] = f'CORRIDOR_{half_width:.1f}M'
                    break
                half_width += self.corridor_expansion_step

            self.publish_planning_corridor(
                (start_x, start_y), (goal_x, goal_y), selected_corridor_half_width)

            if not cell_path:
                self.get_logger().warn(
                    'No safe path inside the planning corridor; refusing to route '
                    'around the map edge.')
                self.path = []
                self.publish_stop()
                self.publish_debug(
                    planner_state='NO_CORRIDOR_PATH',
                    start_valid=True,
                    goal_valid=True,
                    path_found=False,
                    path_has_out_of_map_point=False,
                    **dict(
                        plan_debug,
                        selected_passage_type='NO_CORRIDOR_PATH',
                        corridor_half_width=float(selected_corridor_half_width),
                    ),
                )
                return

            self.path = self.cells_to_sparse_world_path(cell_path)

        if not self.path:
            self.get_logger().warn('Planner produced an empty path.')
            self.path = []
            self.publish_stop()
            self.publish_debug(
                planner_state='EMPTY_PATH',
                start_valid=True,
                goal_valid=True,
                path_found=False,
                path_has_out_of_map_point=False,
                **dict(plan_debug, selected_passage_type='EMPTY_PATH'),
            )
            return

        path_has_out_of_map_point = any(
            not self.point_inside_map(x, y, safety_clearance)
            for x, y in self.path
        )
        if path_has_out_of_map_point:
            self.get_logger().warn(
                'A* produced a path that would violate the map boundary; rejecting it.')
            self.path = []
            self.publish_stop()
            self.publish_debug(
                planner_state='PATH_OUT_OF_MAP',
                start_valid=True,
                goal_valid=True,
                path_found=False,
                path_has_out_of_map_point=True,
                **dict(plan_debug, selected_passage_type='PATH_OUT_OF_MAP'),
            )
            return

        self.last_plan_time = now
        self.planned_map_version = self.map_version
        self.active_goal = (goal_x, goal_y)
        self.goal_reached = False
        self.publish_path()
        self.publish_debug(
            planner_state='PATH_FOUND',
            start_valid=True,
            goal_valid=True,
            path_found=True,
            path_has_out_of_map_point=False,
            **dict(
                plan_debug,
                corridor_half_width=(
                    None if selected_corridor_half_width is None
                    else float(selected_corridor_half_width)),
            ),
        )
        self.get_logger().info(f'Planned path with {len(self.path)} waypoints.')

    def build_blocked_grid(self):
        occupied = self.build_occupied_grid()

        inflation_cells = max(1, int(math.ceil(
            self.inflation_radius / self.map_msg.info.resolution)))
        inflated = occupied.copy()
        for dy in range(-inflation_cells, inflation_cells + 1):
            for dx in range(-inflation_cells, inflation_cells + 1):
                if dx * dx + dy * dy > inflation_cells * inflation_cells:
                    continue
                src_y0 = max(0, -dy)
                src_y1 = occupied.shape[0] - max(0, dy)
                src_x0 = max(0, -dx)
                src_x1 = occupied.shape[1] - max(0, dx)
                dst_y0 = max(0, dy)
                dst_y1 = occupied.shape[0] - max(0, -dy)
                dst_x0 = max(0, dx)
                dst_x1 = occupied.shape[1] - max(0, -dx)
                inflated[dst_y0:dst_y1, dst_x0:dst_x1] |= occupied[src_y0:src_y1, src_x0:src_x1]
        if self.enforce_map_boundaries:
            boundary_cells = max(1, int(math.ceil(
                (self.vehicle_safety_radius + self.boundary_margin) /
                self.map_msg.info.resolution)))
            boundary_cells = min(boundary_cells, max(1, min(inflated.shape) // 2))
            inflated[:boundary_cells, :] = True
            inflated[-boundary_cells:, :] = True
            inflated[:, :boundary_cells] = True
            inflated[:, -boundary_cells:] = True
        return inflated

    def build_occupied_grid(self):
        occupied = self.grid >= self.obstacle_threshold
        if not self.allow_unknown:
            occupied = np.logical_or(occupied, self.grid < 0)
        return occupied

    def build_soft_costmap(self, occupied, blocked):
        cost = np.zeros_like(self.grid, dtype=np.int16)
        if self.allow_unknown:
            cost[self.grid < 0] = int(clamp01(self.unknown_cost) * 100.0)
        else:
            cost[self.grid < 0] = 100

        soft_cells = max(1, int(math.ceil(
            self.soft_inflation_radius / self.map_msg.info.resolution)))
        for dy in range(-soft_cells, soft_cells + 1):
            for dx in range(-soft_cells, soft_cells + 1):
                distance_cells = math.hypot(dx, dy)
                if distance_cells > soft_cells:
                    continue
                distance_m = distance_cells * self.map_msg.info.resolution
                if distance_m <= self.inflation_radius:
                    value = 100
                else:
                    span = max(0.01, self.soft_inflation_radius - self.inflation_radius)
                    value = int(round(85.0 * (1.0 - (distance_m - self.inflation_radius) / span)))
                    value = max(1, min(85, value))

                src_y0 = max(0, -dy)
                src_y1 = occupied.shape[0] - max(0, dy)
                src_x0 = max(0, -dx)
                src_x1 = occupied.shape[1] - max(0, dx)
                dst_y0 = max(0, dy)
                dst_y1 = occupied.shape[0] - max(0, -dy)
                dst_x0 = max(0, dx)
                dst_x1 = occupied.shape[1] - max(0, -dx)

                source = occupied[src_y0:src_y1, src_x0:src_x1]
                target = cost[dst_y0:dst_y1, dst_x0:dst_x1]
                target[source] = np.maximum(target[source], value)

        cost[blocked] = 100
        return cost

    def direct_cells_available(self, cells, blocked):
        if not cells:
            return False, None
        for col, row in cells:
            if not self.in_bounds(col, row) or blocked[row, col]:
                return False, (col, row)
        return True, None

    def make_direct_world_path(self, start, goal):
        sx, sy = start
        gx, gy = goal
        distance = math.hypot(gx - sx, gy - sy)
        steps = max(1, int(math.ceil(distance / 0.35)))
        path = []
        for index in range(steps + 1):
            t = index / steps
            path.append((sx + (gx - sx) * t, sy + (gy - sy) * t))
        return path

    def build_corridor_mask(self, start, goal, half_width, safety_clearance):
        height, width = self.grid.shape
        rows, cols = np.indices((height, width))
        info = self.map_msg.info
        xs = info.origin.position.x + (cols + 0.5) * info.resolution
        ys = info.origin.position.y + (rows + 0.5) * info.resolution

        sx, sy = start
        gx, gy = goal
        vx = gx - sx
        vy = gy - sy
        length_sq = max(1e-6, vx * vx + vy * vy)
        projection = ((xs - sx) * vx + (ys - sy) * vy) / length_sq
        projection = np.clip(projection, 0.0, 1.0)
        closest_x = sx + projection * vx
        closest_y = sy + projection * vy
        distance = np.hypot(xs - closest_x, ys - closest_y)

        bounds = self.map_bounds()
        if bounds is None:
            return distance <= half_width
        min_x, min_y, max_x, max_y = bounds
        inside_safe_map = (
            (xs >= min_x + safety_clearance) &
            (xs <= max_x - safety_clearance) &
            (ys >= min_y + safety_clearance) &
            (ys <= max_y - safety_clearance)
        )
        return (distance <= half_width) & inside_safe_map

    def shortcut_cell_path(self, cell_path, blocked):
        if len(cell_path) <= 2:
            return cell_path
        result = [cell_path[0]]
        index = 0
        while index < len(cell_path) - 1:
            next_index = len(cell_path) - 1
            while next_index > index + 1:
                if self.cell_segment_clear(cell_path[index], cell_path[next_index], blocked):
                    break
                next_index -= 1
            result.append(cell_path[next_index])
            index = next_index
        return result

    def cell_segment_clear(self, start, goal, blocked):
        for col, row in self.bresenham_cells(start[0], start[1], goal[0], goal[1]):
            if not self.in_bounds(col, row) or blocked[row, col]:
                return False
        return True

    def estimate_gap_width(self, cell, occupied):
        if cell is None or self.goal is None or self.odom_pose is None:
            return None
        col, row = cell
        if not self.in_bounds(col, row):
            return 0.0

        sx, sy = self.odom_pose[0], self.odom_pose[1]
        gx, gy = self.goal
        dx = gx - sx
        dy = gy - sy
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return None
        normal_x = -dy / length
        normal_y = dx / length
        center_x, center_y = self.cell_to_world(col, row)

        max_probe = self.max_planning_corridor_half_width + self.vehicle_safety_radius
        left = self.distance_to_occupied_along_ray(
            center_x, center_y, normal_x, normal_y, occupied, max_probe)
        right = self.distance_to_occupied_along_ray(
            center_x, center_y, -normal_x, -normal_y, occupied, max_probe)
        return left + right

    def distance_to_occupied_along_ray(self, x, y, ux, uy, occupied, max_distance):
        step = max(0.05, self.map_msg.info.resolution)
        safety_clearance = self.vehicle_safety_radius + self.boundary_margin
        distance = 0.0
        while distance <= max_distance:
            probe_x = x + ux * distance
            probe_y = y + uy * distance
            if not self.point_inside_map(probe_x, probe_y, safety_clearance):
                break
            cell = self.world_to_cell(probe_x, probe_y)
            if cell is None:
                break
            if occupied[cell[1], cell[0]]:
                break
            distance += step
        return distance

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

    def astar(self, start, goal, blocked, soft_cost=None, corridor_line=None,
              corridor_half_width=None):
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
                if soft_cost is not None:
                    extra += self.soft_cost_weight * float(soft_cost[next_row, next_col]) / 100.0
                if corridor_line is not None and corridor_half_width is not None:
                    wx, wy = self.cell_to_world(next_col, next_row)
                    line_distance = self.distance_to_segment(
                        wx, wy, corridor_line[0], corridor_line[1])
                    extra += self.corridor_center_weight * min(
                        1.0, line_distance / max(0.1, corridor_half_width))
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
    def distance_to_segment(x, y, start, goal):
        sx, sy = start
        gx, gy = goal
        vx = gx - sx
        vy = gy - sy
        length_sq = vx * vx + vy * vy
        if length_sq <= 1e-9:
            return math.hypot(x - sx, y - sy)
        t = ((x - sx) * vx + (y - sy) * vy) / length_sq
        t = max(0.0, min(1.0, t))
        closest_x = sx + t * vx
        closest_y = sy + t * vy
        return math.hypot(x - closest_x, y - closest_y)

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
        stride = max(1, int(round(0.25 / self.map_msg.info.resolution)))
        sparse = []
        for index, cell in enumerate(cell_path):
            if index % stride == 0 or index == len(cell_path) - 1:
                sparse.append(self.cell_to_world(cell[0], cell[1]))
        return sparse

    def follow_path(self):
        if not self.path:
            self.publish_stop()
            return

        x, y, yaw = self.odom_pose
        goal_for_control = self.active_goal if self.active_goal is not None else self.goal
        goal_distance = math.hypot(goal_for_control[0] - x, goal_for_control[1] - y)
        if goal_distance <= self.goal_tolerance:
            if not self.goal_reached:
                self.get_logger().info('Goal reached.')
            self.goal_reached = True
            self.publish_stop()
            return

        nearest_index = min(
            range(len(self.path)),
            key=lambda i: math.hypot(self.path[i][0] - x, self.path[i][1] - y))
        effective_lookahead = self.select_lookahead(x, y, goal_distance)
        target = self.path[-1]
        for point in self.path[nearest_index:]:
            if math.hypot(point[0] - x, point[1] - y) >= effective_lookahead:
                target = point
                break

        dx = target[0] - x
        dy = target[1] - y
        x_body = math.cos(yaw) * dx + math.sin(yaw) * dy
        y_body = -math.sin(yaw) * dx + math.cos(yaw) * dy
        lookahead_distance = max(0.1, math.hypot(x_body, y_body))
        alpha = math.atan2(y_body, max(0.05, x_body))
        goal_slowdown = max(0.25, min(1.0, goal_distance / 1.2))
        base_speed = self.max_speed * goal_slowdown
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
        cmd, boundary_state = self.apply_boundary_guard(cmd, x, y, yaw)
        if boundary_state:
            self.publish_debug(
                planner_state=boundary_state,
                start_valid=True,
                goal_valid=True,
                path_found=bool(self.path),
                path_has_out_of_map_point=False,
            )
        self.cmd_pub.publish(cmd)

    def select_lookahead(self, x, y, goal_distance):
        clearance = self.nearest_occupied_distance(
            x, y, self.lookahead_clearance_distance)
        if clearance is None:
            clear_ratio = 1.0
        else:
            near_clearance = self.inflation_radius + 0.20
            far_clearance = max(near_clearance + 0.05,
                                self.lookahead_clearance_distance)
            clear_ratio = clamp01(
                (clearance - near_clearance) / (far_clearance - near_clearance))

        lookahead = (
            self.min_lookahead +
            clear_ratio * (self.max_lookahead - self.min_lookahead)
        )
        return max(
            0.30,
            min(lookahead, max(self.min_lookahead, goal_distance)))

    def nearest_occupied_distance(self, x, y, max_distance):
        if self.map_msg is None or self.grid is None:
            return None

        cell = self.world_to_cell(x, y)
        if cell is None:
            return 0.0

        resolution = self.map_msg.info.resolution
        radius_cells = max(1, int(math.ceil(max_distance / resolution)))
        col, row = cell
        row0 = max(0, row - radius_cells)
        row1 = min(self.grid.shape[0], row + radius_cells + 1)
        col0 = max(0, col - radius_cells)
        col1 = min(self.grid.shape[1], col + radius_cells + 1)
        occupied_window = self.build_occupied_grid()[row0:row1, col0:col1]

        best = None
        boundary_distance = self.nearest_map_boundary_distance(x, y)
        if boundary_distance is not None:
            best = max(0.0, boundary_distance)

        if np.any(occupied_window):
            rows, cols = np.nonzero(occupied_window)
            world_x = (
                self.map_msg.info.origin.position.x +
                (cols + col0 + 0.5) * resolution
            )
            world_y = (
                self.map_msg.info.origin.position.y +
                (rows + row0 + 0.5) * resolution
            )
            occupied_distance = float(np.min(np.hypot(world_x - x, world_y - y)))
            best = occupied_distance if best is None else min(best, occupied_distance)

        if best is None or best > max_distance:
            return None
        return best

    def publish_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for x, y in self.path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_direct_path(self, path):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for x, y in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.05
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.direct_path_pub.publish(msg)

    def publish_planning_corridor(self, start, goal, half_width):
        sx, sy = start
        gx, gy = goal
        length = math.hypot(gx - sx, gy - sy)
        if length < 1e-6:
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'planning_corridor'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.5 * (sx + gx)
        marker.pose.position.y = 0.5 * (sy + gy)
        marker.pose.position.z = 0.015
        qx, qy, qz, qw = quaternion_from_yaw(math.atan2(gy - sy, gx - sx))
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = length
        marker.scale.y = 2.0 * half_width
        marker.scale.z = 0.03
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.16
        self.corridor_pub.publish(marker)

    def publish_inflated_map(self, blocked):
        data = np.where(blocked, 100, 0).astype(np.int16)
        self.publish_debug_grid(self.inflated_map_pub, data)

    def publish_costmap_debug(self, cost):
        self.publish_debug_grid(self.costmap_debug_pub, cost)

    def publish_debug_grid(self, publisher, data):
        if self.map_msg is None:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info = self.map_msg.info
        msg.data = data.reshape(-1).astype(np.int16).tolist()
        publisher.publish(msg)

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
    def bresenham_cells(x0, y0, x1, y1):
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
            if len(cells) > 100000:
                break
        return cells

    def map_bounds(self):
        if self.map_msg is None:
            return None
        info = self.map_msg.info
        min_x = info.origin.position.x
        min_y = info.origin.position.y
        max_x = min_x + info.width * info.resolution
        max_y = min_y + info.height * info.resolution
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

    def apply_boundary_guard(self, cmd, x, y, yaw):
        if not self.enforce_map_boundaries:
            return cmd, None
        bounds = self.map_bounds()
        if bounds is None:
            return cmd, None

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

        guarded = self.copy_twist(cmd)
        if moving_toward_boundary:
            guarded.linear.x = 0.0
            guarded.angular.z = self.turn_toward_map_center(x, y, yaw)
            return guarded, 'BOUNDARY_RECOVERY'

        guarded.linear.x = min(guarded.linear.x, 0.25 * self.max_speed)
        return guarded, 'NEAR_MAP_BOUNDARY'

    def turn_toward_map_center(self, x, y, yaw):
        min_x, min_y, max_x, max_y = self.map_bounds()
        center_x = 0.5 * (min_x + max_x)
        center_y = 0.5 * (min_y + max_y)
        target_yaw = math.atan2(center_y - y, center_x - x)
        error = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
        return max(-self.max_yaw_rate, min(self.max_yaw_rate, error))

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

    def publish_debug(self, planner_state, start_valid, goal_valid, path_found,
                      path_has_out_of_map_point, **extra):
        if self.map_msg is None:
            return
        bounds = self.map_bounds()
        start_x = None
        start_y = None
        if self.odom_pose is not None:
            start_x = float(self.odom_pose[0])
            start_y = float(self.odom_pose[1])
        goal_x = None
        goal_y = None
        if self.goal is not None:
            goal_x = float(self.goal[0])
            goal_y = float(self.goal[1])

        safety_clearance = self.vehicle_safety_radius + self.boundary_margin
        boundary_distance = self.nearest_map_boundary_distance()
        inside_map = bool(
            self.odom_pose is not None and
            self.point_inside_map(self.odom_pose[0], self.odom_pose[1], 0.0))
        safe_inside_map = bool(
            self.odom_pose is not None and
            self.point_inside_map(self.odom_pose[0], self.odom_pose[1], safety_clearance))
        default_plan_data = {
            'direct_path_available': False,
            'direct_path_blocked': None,
            'nearest_gap_width': None,
            'required_pass_width': float(2.0 * self.vehicle_safety_radius + self.pass_margin),
            'gap_passable': None,
            'selected_passage_type': None,
            'boundary_state': (
                'OUT_OF_MAP' if boundary_distance is not None and boundary_distance < 0.0
                else 'NEAR_MAP_BOUNDARY' if not safe_inside_map
                else 'INSIDE_MAP'),
            'hard_collision_radius': float(self.inflation_radius),
            'soft_inflation_radius': float(self.soft_inflation_radius),
            'corridor_half_width': None,
        }
        data = {
            'planner_state': planner_state,
            'start_x': start_x,
            'start_y': start_y,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'map_origin_x': float(self.map_msg.info.origin.position.x),
            'map_origin_y': float(self.map_msg.info.origin.position.y),
            'map_width_m': float(self.map_msg.info.width * self.map_msg.info.resolution),
            'map_height_m': float(self.map_msg.info.height * self.map_msg.info.resolution),
            'start_inside_map': bool(
                self.odom_pose is not None and
                self.point_inside_map(self.odom_pose[0], self.odom_pose[1], safety_clearance)),
            'goal_inside_map': bool(
                self.goal is not None and
                self.point_inside_map(self.goal[0], self.goal[1], safety_clearance)),
            'start_valid': bool(start_valid),
            'goal_valid': bool(goal_valid),
            'path_found': bool(path_found),
            'path_has_out_of_map_point': bool(path_has_out_of_map_point),
            'inside_map': inside_map,
            'nearest_map_boundary_distance': boundary_distance,
            'vehicle_safety_radius': float(self.vehicle_safety_radius),
            'boundary_margin': float(self.boundary_margin),
            'required_boundary_clearance': float(safety_clearance),
            'enforce_map_boundaries': bool(self.enforce_map_boundaries),
        }
        data.update(default_plan_data)
        data.update(self.last_plan_debug)
        data.update(extra)
        self.last_plan_debug = {
            key: data[key]
            for key in (
                'direct_path_available',
                'direct_path_blocked',
                'nearest_gap_width',
                'required_pass_width',
                'gap_passable',
                'selected_passage_type',
                'boundary_state',
                'hard_collision_radius',
                'soft_inflation_radius',
                'corridor_half_width',
            )
        }
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


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
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

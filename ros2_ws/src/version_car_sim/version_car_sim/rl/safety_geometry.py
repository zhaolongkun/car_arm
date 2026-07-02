import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np


JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")

JOINT_LIMITS = np.array([
    (-2.8, 2.8),
    (-3.14, 0.0),
    (-3.14, 0.0),
    (-1.87, 1.57),
    (-1.57, 1.57),
    (-3.14, 3.14),
], dtype=np.float64)


@dataclass(frozen=True)
class JointSpec:
    name: str
    parent: str
    child: str
    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]
    axis: Tuple[float, float, float]


@dataclass(frozen=True)
class LinkSphere:
    name: str
    radius: float


@dataclass(frozen=True)
class BoxObstacle:
    name: str
    center: Tuple[float, float, float]
    half_extents: Tuple[float, float, float]


@dataclass(frozen=True)
class CylinderObstacle:
    name: str
    center: Tuple[float, float, float]
    radius: float
    half_height: float


@dataclass
class DistanceReport:
    min_obstacle_distance: float
    min_self_distance: float
    min_distance: float
    obstacle_distances: Dict[str, Dict[str, float]]
    link_min_distances: Dict[str, float]
    self_pair_distances: Dict[str, float]
    collision: bool
    collision_pairs: List[str]
    cost: float


@dataclass
class ShieldResult:
    accepted: bool
    scale: float
    raw_action: np.ndarray
    clipped_action: np.ndarray
    applied_action: np.ndarray
    target_joints: np.ndarray
    report: DistanceReport
    reason: str


def _translation(xyz: Sequence[float]) -> np.ndarray:
    mat = np.eye(4, dtype=np.float64)
    mat[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return mat


def _rpy_matrix(rpy: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = [float(v) for v in rpy]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])

    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rz @ ry @ rx
    return mat


def _axis_angle_matrix(axis: Sequence[float], angle: float) -> np.ndarray:
    axis_array = np.asarray(axis, dtype=np.float64)
    norm = np.linalg.norm(axis_array)
    if norm <= 1e-12:
        return np.eye(4, dtype=np.float64)
    x, y, z = axis_array / norm
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    one_c = 1.0 - c
    rot = np.array([
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ], dtype=np.float64)
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rot
    return mat


def _fixed_transform(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    return _translation(xyz) @ _rpy_matrix(rpy)


def _joint_transform(spec: JointSpec, position: float) -> np.ndarray:
    return _fixed_transform(spec.xyz, spec.rpy) @ _axis_angle_matrix(spec.axis, position)


def _signed_distance_box(point: np.ndarray, box: BoxObstacle, sphere_radius: float) -> float:
    center = np.asarray(box.center, dtype=np.float64)
    half = np.asarray(box.half_extents, dtype=np.float64)
    q = np.abs(point - center) - half
    outside = np.linalg.norm(np.maximum(q, 0.0))
    inside = min(float(np.max(q)), 0.0)
    return outside + inside - sphere_radius


def _signed_distance_cylinder(point: np.ndarray, cylinder: CylinderObstacle, sphere_radius: float) -> float:
    center = np.asarray(cylinder.center, dtype=np.float64)
    radial = math.hypot(float(point[0] - center[0]), float(point[1] - center[1])) - cylinder.radius
    vertical = abs(float(point[2] - center[2])) - cylinder.half_height
    outside = math.hypot(max(radial, 0.0), max(vertical, 0.0))
    inside = min(max(radial, vertical), 0.0)
    return outside + inside - sphere_radius


class SimplifiedArmSafetyModel:
    """Approximate B601-DM geometry used by RL cost and runtime shielding.

    The model uses URDF joint origins for forward kinematics and represents
    moving arm links as conservative spheres. It is intentionally isolated so
    a MoveIt PlanningScene/FCL backend can replace it later without touching
    the RL algorithm or controller API.
    """

    def __init__(
        self,
        safe_distance: float = 0.10,
        hard_clearance: float = 0.0,
        max_action_delta: float = 0.025,
    ):
        self.safe_distance = float(safe_distance)
        self.hard_clearance = float(hard_clearance)
        self.max_action_delta = float(max_action_delta)
        self.ground_min_z: Optional[float] = None
        self.joint_specs = (
            JointSpec("joint1", "rebot_base_link", "link1", (-8.416e-05, 0.0, 0.08465), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            JointSpec("joint2", "link1", "link2", (0.020084, 0.031625, 0.05555), (-1.5708, 0.0, 0.0), (0.0, 0.0, -1.0)),
            JointSpec("joint3", "link2", "link3", (-0.264, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            JointSpec("joint4", "link3", "link4", (0.2426, -0.054, -0.001625), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            JointSpec("joint5", "link4", "link5", (0.078308, -0.0375, -0.03), (-1.5708, 0.0, 0.0), (0.0, 0.0, 1.0)),
            JointSpec("joint6", "link5", "link6", (0.023692, 0.0, 0.04), (0.0, 1.5708, 0.0), (0.0, 0.0, 1.0)),
        )
        self.link_spheres = (
            LinkSphere("link1", 0.050),
            LinkSphere("link2", 0.045),
            LinkSphere("link3", 0.045),
            LinkSphere("link4", 0.035),
            LinkSphere("link5", 0.030),
            LinkSphere("link6", 0.026),
            LinkSphere("gripper_link", 0.045),
            LinkSphere("spray_nozzle_link", 0.025),
            LinkSphere("spray_tip_link", 0.018),
        )
        self.disabled_self_collision_pairs = {
            tuple(sorted(("link2", "link4"))),
        }
        self.obstacles = (
            BoxObstacle("car_body", (0.0, 0.0, -0.08), (0.31, 0.19, 0.08)),
            BoxObstacle("front_left_wheel", (0.20, 0.20, -0.06), (0.08, 0.08, 0.11)),
            BoxObstacle("front_right_wheel", (0.20, -0.20, -0.06), (0.08, 0.08, 0.11)),
            BoxObstacle("rear_left_wheel", (-0.20, 0.20, -0.06), (0.08, 0.08, 0.11)),
            BoxObstacle("rear_right_wheel", (-0.20, -0.20, -0.06), (0.08, 0.08, 0.11)),
            CylinderObstacle("mid360_lidar", (0.24, 0.0, 0.035), 0.08, 0.07),
            BoxObstacle("arm_mount_base", (0.0, 0.0, -0.005), (0.12, 0.11, 0.025)),
        )

    def set_ground_min_z(self, min_z: Optional[float]) -> None:
        self.ground_min_z = None if min_z is None else float(min_z)

    @property
    def observation_link_names(self) -> Tuple[str, ...]:
        return tuple(sphere.name for sphere in self.link_spheres)

    @property
    def obstacle_names(self) -> Tuple[str, ...]:
        return tuple(obstacle.name for obstacle in self.obstacles)

    def clip_joints(self, joints: Sequence[float]) -> np.ndarray:
        values = np.asarray(joints, dtype=np.float64).reshape(6)
        return np.clip(values, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])

    def clip_action(self, action: Sequence[float]) -> np.ndarray:
        values = np.asarray(action, dtype=np.float64).reshape(6)
        return np.clip(values, -self.max_action_delta, self.max_action_delta)

    def forward_kinematics(self, joints: Sequence[float]) -> Dict[str, np.ndarray]:
        joint_values = self.clip_joints(joints)
        transforms: Dict[str, np.ndarray] = {"rebot_base_link": np.eye(4, dtype=np.float64)}
        for idx, spec in enumerate(self.joint_specs):
            transforms[spec.child] = transforms[spec.parent] @ _joint_transform(spec, joint_values[idx])

        transforms["gripper_link"] = (
            transforms["link6"] @ _fixed_transform((0.0, 0.0, 0.15971), (0.0, -1.5708, 0.0))
        )
        transforms["gripper_tcp"] = transforms["gripper_link"] @ _fixed_transform((-0.0443, 0.0, 0.0), (0.0, 0.0, 0.0))
        transforms["spray_nozzle_link"] = transforms["gripper_tcp"] @ _fixed_transform((0.060, 0.0, 0.0), (0.0, 0.0, 0.0))
        transforms["spray_tip_link"] = transforms["spray_nozzle_link"] @ _fixed_transform((0.060, 0.0, 0.0), (0.0, 0.0, 0.0))
        return {name: transform[:3, 3].copy() for name, transform in transforms.items()}

    def distance_report(self, joints: Sequence[float]) -> DistanceReport:
        positions = self.forward_kinematics(joints)
        obstacle_distances: Dict[str, Dict[str, float]] = {}
        link_min_distances: Dict[str, float] = {}
        collision_pairs: List[str] = []
        min_obstacle = float("inf")

        for sphere in self.link_spheres:
            point = positions[sphere.name]
            per_link: Dict[str, float] = {}
            for obstacle in self.obstacles:
                if isinstance(obstacle, BoxObstacle):
                    distance = _signed_distance_box(point, obstacle, sphere.radius)
                else:
                    distance = _signed_distance_cylinder(point, obstacle, sphere.radius)
                per_link[obstacle.name] = float(distance)
                min_obstacle = min(min_obstacle, float(distance))
                if distance <= 0.0:
                    collision_pairs.append(f"{sphere.name}:{obstacle.name}")
            obstacle_distances[sphere.name] = per_link
            link_min = min(per_link.values())
            if self.ground_min_z is not None:
                ground_distance = float(point[2] - sphere.radius - self.ground_min_z)
                link_min = min(link_min, ground_distance)
                min_obstacle = min(min_obstacle, ground_distance)
                if ground_distance <= 0.0:
                    collision_pairs.append(f"{sphere.name}:ground_plane")
            link_min_distances[sphere.name] = link_min

        self_distances: Dict[str, float] = {}
        min_self = float("inf")
        spheres = list(self.link_spheres)
        for i, first in enumerate(spheres):
            for j in range(i + 1, len(spheres)):
                second = spheres[j]
                if j - i <= 1:
                    continue
                if first.name.startswith("spray") and second.name.startswith("spray"):
                    continue
                unordered_pair = tuple(sorted((first.name, second.name)))
                if unordered_pair in self.disabled_self_collision_pairs:
                    continue
                distance = (
                    float(np.linalg.norm(positions[first.name] - positions[second.name]))
                    - first.radius
                    - second.radius
                )
                pair = f"{first.name}:{second.name}"
                self_distances[pair] = distance
                min_self = min(min_self, distance)
                if distance <= 0.0:
                    collision_pairs.append(pair)

        min_distance = min(min_obstacle, min_self)
        collision = bool(collision_pairs)
        cost = self.compute_cost_from_distances(min_obstacle, min_self, collision)
        return DistanceReport(
            min_obstacle_distance=float(min_obstacle),
            min_self_distance=float(min_self),
            min_distance=float(min_distance),
            obstacle_distances=obstacle_distances,
            link_min_distances=link_min_distances,
            self_pair_distances=self_distances,
            collision=collision,
            collision_pairs=collision_pairs,
            cost=float(cost),
        )

    def compute_cost_from_distances(self, min_obstacle: float, min_self: float, collision: bool) -> float:
        if collision or min_obstacle <= 0.0 or min_self <= 0.0:
            return 1.0
        nearest = min(min_obstacle, min_self)
        if nearest >= self.safe_distance:
            return 0.0
        return float((self.safe_distance - nearest) / max(self.safe_distance, 1e-6))

    def flatten_distances(self, report: DistanceReport) -> np.ndarray:
        values: List[float] = []
        for link_name in self.observation_link_names:
            per_link = report.obstacle_distances.get(link_name, {})
            for obstacle_name in self.obstacle_names:
                values.append(float(per_link.get(obstacle_name, self.safe_distance)))
        values.append(float(report.min_self_distance))
        return np.asarray(values, dtype=np.float32)

    def apply_safety_shield(
        self,
        current_joints: Sequence[float],
        raw_action: Sequence[float],
        backtrack_scales: Iterable[float] = (1.0, 0.5, 0.25, 0.1, 0.0),
    ) -> ShieldResult:
        current = self.clip_joints(current_joints)
        raw = np.asarray(raw_action, dtype=np.float64).reshape(6)
        clipped = self.clip_action(raw)
        current_report = self.distance_report(current)

        last_report: Optional[DistanceReport] = None
        for scale in backtrack_scales:
            applied = clipped * float(scale)
            target = self.clip_joints(current + applied)
            report = self.distance_report(target)
            last_report = report
            if report.collision:
                continue
            if report.min_distance < self.hard_clearance:
                continue
            if report.cost > max(current_report.cost, 0.98) + 1e-6:
                continue
            return ShieldResult(
                accepted=scale > 0.0,
                scale=float(scale),
                raw_action=raw,
                clipped_action=clipped,
                applied_action=target - current,
                target_joints=target,
                report=report,
                reason="accepted" if scale > 0.0 else "zero_action",
            )

        if last_report is None:
            last_report = current_report
        return ShieldResult(
            accepted=False,
            scale=0.0,
            raw_action=raw,
            clipped_action=clipped,
            applied_action=np.zeros(6, dtype=np.float64),
            target_joints=current,
            report=last_report,
            reason="unsafe_action_rejected",
        )


def dict_to_joint_array(joints: Mapping[str, float], default: Optional[Sequence[float]] = None) -> np.ndarray:
    if default is None:
        values = np.zeros(6, dtype=np.float64)
    else:
        values = np.asarray(default, dtype=np.float64).reshape(6).copy()
    for idx, name in enumerate(JOINT_NAMES):
        if name in joints:
            values[idx] = float(joints[name])
    return values


def joint_array_to_dict(values: Sequence[float]) -> Dict[str, float]:
    array = np.asarray(values, dtype=np.float64).reshape(6)
    return {name: float(array[idx]) for idx, name in enumerate(JOINT_NAMES)}

import math
from typing import Callable, Dict, Optional, Sequence, Tuple

import numpy as np

from version_car_sim.rl.safety_geometry import (
    JOINT_NAMES,
    SimplifiedArmSafetyModel,
    ShieldResult,
    dict_to_joint_array,
    joint_array_to_dict,
)

try:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseStamped
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception:  # pragma: no cover - lets offline training import without ROS sourced.
    rclpy = None
    Duration = None
    Time = None
    JointState = None
    PoseStamped = None
    Buffer = None
    TransformException = Exception
    TransformListener = None


OBSERVATION_LAYOUT = (
    ("joint_positions", 6),
    ("joint_velocities", 6),
    ("spray_tip_position_rebot_base", 3),
    ("target_position_rebot_base", 3),
    ("tip_to_target_delta", 3),
    ("link_to_obstacle_distances", 45),
    ("self_collision_min_distance", 1),
)


def observation_dim() -> int:
    return int(sum(size for _, size in OBSERVATION_LAYOUT))


def _layout_size(name: str) -> int:
    for layout_name, size in OBSERVATION_LAYOUT:
        if layout_name == name:
            return int(size)
    raise KeyError(name)


def _quat_rotate(vector: Sequence[float], quat: Sequence[float]) -> np.ndarray:
    qx, qy, qz, qw = [float(v) for v in quat]
    vx, vy, vz = [float(v) for v in vector]
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return np.asarray([
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    ], dtype=np.float64)


class RebotSafeRLEnv:
    """Gym-like safe RL environment for the Seeed reBot B601-DM arm.

    Observation layout:
      joint1..joint6 position, joint1..joint6 velocity,
      spray_tip_link position in rebot_base_link,
      leak target position in rebot_base_link,
      target - tip relative vector,
      per-link distances to car_chassis, car_top_surface, mid360_lidar,
      minimum self-collision distance.

    Action:
      six joint deltas [dq1..dq6]. The safety model clips and shields them
      before they can be executed.
    """

    def __init__(
        self,
        node=None,
        target_position: Sequence[float] = (0.45, 0.0, 0.16),
        safe_distance: float = 0.10,
        hard_clearance: float = 0.0,
        max_action_delta: float = 0.025,
        success_tolerance: float = 0.035,
        max_episode_steps: int = 240,
        control_dt: float = 0.10,
        execute_callback: Optional[Callable[[Dict[str, float]], bool]] = None,
        random_reset: bool = False,
        subscribe_leak_pose: bool = True,
        seed: Optional[int] = None,
    ):
        self.node = node
        self.target_frame = "rebot_base_link"
        self.safety_model = SimplifiedArmSafetyModel(
            safe_distance=safe_distance,
            hard_clearance=hard_clearance,
            max_action_delta=max_action_delta,
        )
        self.success_tolerance = float(success_tolerance)
        self.max_episode_steps = int(max_episode_steps)
        self.control_dt = float(control_dt)
        self.execute_callback = execute_callback
        self.random_reset = bool(random_reset)
        self.rng = np.random.default_rng(seed)

        self.home_joints = np.zeros(6, dtype=np.float64)
        self.joint_positions = self.safety_model.clip_joints(self.home_joints)
        self.joint_velocities = np.zeros(6, dtype=np.float64)
        self.target_position = np.asarray(target_position, dtype=np.float64).reshape(3)
        self.last_action = np.zeros(6, dtype=np.float64)
        self.last_shield_result: Optional[ShieldResult] = None
        self.step_count = 0
        self.latest_joint_stamp = None

        self.tf_buffer = None
        self.tf_listener = None
        if self.node is not None and Buffer is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self.node)
            self.node.create_subscription(JointState, "joint_states", self.on_joint_states, 10)
            if subscribe_leak_pose:
                self.node.create_subscription(PoseStamped, "leak_pose_map", self.on_leak_pose, 10)

    @property
    def action_dim(self) -> int:
        return 6

    @property
    def observation_dim(self) -> int:
        return observation_dim()

    def reset(self) -> np.ndarray:
        self.step_count = 0
        self.last_action = np.zeros(6, dtype=np.float64)
        self.last_shield_result = None
        if self.random_reset:
            noise = self.rng.normal(0.0, 0.08, size=6)
            self.joint_positions = self.safety_model.clip_joints(self.home_joints + noise)
        self.joint_velocities = np.zeros(6, dtype=np.float64)
        return self.get_observation()

    def step(self, action: Sequence[float]) -> Tuple[np.ndarray, float, float, bool, Dict[str, object]]:
        shield = self.filter_action(action)
        executed = True
        if self.execute_callback is not None:
            executed = bool(self.execute_callback(joint_array_to_dict(shield.target_joints)))

        previous = self.joint_positions.copy()
        self.joint_positions = shield.target_joints.copy()
        self.joint_velocities = (self.joint_positions - previous) / max(self.control_dt, 1e-6)
        self.last_action = shield.applied_action.copy()
        self.last_shield_result = shield
        self.step_count += 1

        obs = self.get_observation()
        reward = self.compute_reward(shield.applied_action)
        cost = self.compute_cost()
        done = self.check_done()
        info = self.info_dict()
        info["executed"] = executed
        return obs, reward, cost, done, info

    def filter_action(self, action: Sequence[float]) -> ShieldResult:
        return self.safety_model.apply_safety_shield(self.joint_positions, action)

    def get_observation(self) -> np.ndarray:
        positions = self.safety_model.forward_kinematics(self.joint_positions)
        tip = positions["spray_tip_link"].astype(np.float64)
        target = self.target_position.astype(np.float64)
        delta = target - tip
        report = self.safety_model.distance_report(self.joint_positions)
        distances = self._policy_distance_features(report)
        obs = np.concatenate([
            self.joint_positions.astype(np.float32),
            self.joint_velocities.astype(np.float32),
            tip.astype(np.float32),
            target.astype(np.float32),
            delta.astype(np.float32),
            distances.astype(np.float32),
        ])
        return obs.astype(np.float32)

    def _policy_distance_features(self, report) -> np.ndarray:
        """Adapt runtime safety distances to the deployed policy shape.

        The current safety geometry checks more obstacles than the policy was
        originally trained with. The shield still uses the full geometry, but
        the neural policy must receive the same 67-D observation shape stored
        in policy.pt.
        """
        raw = self.safety_model.flatten_distances(report)
        obstacle_count = _layout_size("link_to_obstacle_distances")
        self_count = _layout_size("self_collision_min_distance")
        expected = obstacle_count + self_count
        if raw.size == expected:
            return raw.astype(np.float32)

        obstacle_features = raw[:-self_count]
        self_features = raw[-self_count:]
        if obstacle_features.size < obstacle_count:
            obstacle_features = np.pad(
                obstacle_features,
                (0, obstacle_count - obstacle_features.size),
                mode="constant",
                constant_values=self.safety_model.safe_distance,
            )
        else:
            obstacle_features = obstacle_features[:obstacle_count]
        return np.concatenate([obstacle_features, self_features]).astype(np.float32)

    def compute_reward(self, action: Optional[Sequence[float]] = None) -> float:
        positions = self.safety_model.forward_kinematics(self.joint_positions)
        tip = positions["spray_tip_link"]
        distance = float(np.linalg.norm(self.target_position - tip))
        applied = self.last_action if action is None else np.asarray(action, dtype=np.float64).reshape(6)
        action_norm = float(np.linalg.norm(applied))
        velocity_norm = float(np.linalg.norm(self.joint_velocities))
        success_bonus = 8.0 if distance <= self.success_tolerance else 0.0
        return float(-2.0 * distance - 0.05 * action_norm - 0.05 * velocity_norm + success_bonus)

    def compute_cost(self) -> float:
        report = self.safety_model.distance_report(self.joint_positions)
        return float(report.cost)

    def check_done(self) -> bool:
        positions = self.safety_model.forward_kinematics(self.joint_positions)
        tip = positions["spray_tip_link"]
        distance = float(np.linalg.norm(self.target_position - tip))
        report = self.safety_model.distance_report(self.joint_positions)
        return bool(
            distance <= self.success_tolerance
            or report.collision
            or self.step_count >= self.max_episode_steps
        )

    def set_target_position(self, xyz: Sequence[float]) -> None:
        self.target_position = np.asarray(xyz, dtype=np.float64).reshape(3)

    def set_ground_min_z(self, min_z: Optional[float]) -> None:
        self.safety_model.set_ground_min_z(min_z)

    def set_joint_state(
        self,
        positions: Sequence[float],
        velocities: Optional[Sequence[float]] = None,
    ) -> None:
        self.joint_positions = self.safety_model.clip_joints(positions)
        if velocities is None:
            self.joint_velocities = np.zeros(6, dtype=np.float64)
        else:
            self.joint_velocities = np.asarray(velocities, dtype=np.float64).reshape(6)

    def info_dict(self) -> Dict[str, object]:
        positions = self.safety_model.forward_kinematics(self.joint_positions)
        tip = positions["spray_tip_link"]
        report = self.safety_model.distance_report(self.joint_positions)
        distance = float(np.linalg.norm(self.target_position - tip))
        return {
            "distance_to_target": distance,
            "success": distance <= self.success_tolerance,
            "cost": report.cost,
            "collision": report.collision,
            "collision_pairs": list(report.collision_pairs),
            "min_obstacle_distance": report.min_obstacle_distance,
            "min_self_distance": report.min_self_distance,
            "target_joint_positions": self.joint_positions.copy(),
            "last_shield": self.last_shield_result,
        }

    def on_joint_states(self, msg) -> None:
        joint_map = {}
        velocity_map = {}
        for idx, name in enumerate(msg.name):
            if name not in JOINT_NAMES:
                continue
            if idx < len(msg.position):
                joint_map[name] = float(msg.position[idx])
            if idx < len(msg.velocity):
                velocity_map[name] = float(msg.velocity[idx])
        if not joint_map:
            return
        self.joint_positions = dict_to_joint_array(joint_map, self.joint_positions)
        if velocity_map:
            self.joint_velocities = dict_to_joint_array(velocity_map, self.joint_velocities)
        self.latest_joint_stamp = msg.header.stamp

    def on_leak_pose(self, msg) -> None:
        transformed = self._pose_to_target_frame(msg)
        if transformed is not None:
            self.target_position = transformed

    def _pose_to_target_frame(self, msg) -> Optional[np.ndarray]:
        source_frame = msg.header.frame_id or self.target_frame
        point = np.asarray([
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        ], dtype=np.float64)
        if source_frame == self.target_frame or self.tf_buffer is None or Time is None:
            return point
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return None
        translation = tf_msg.transform.translation
        rotation = tf_msg.transform.rotation
        quat = (rotation.x, rotation.y, rotation.z, rotation.w)
        return _quat_rotate(point, quat) + np.asarray(
            [translation.x, translation.y, translation.z], dtype=np.float64)

    def observation_dict(self) -> Dict[str, np.ndarray]:
        obs = self.get_observation()
        result = {}
        offset = 0
        for name, size in OBSERVATION_LAYOUT:
            result[name] = obs[offset:offset + size].copy()
            offset += size
        return result

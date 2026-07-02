import math
import os
import time
import traceback
from typing import Optional, Sequence

import numpy as np

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import PerformanceMetrics
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import TransformException
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from version_car_sim.rl.rebot_safe_rl_env import RebotSafeRLEnv
from version_car_sim.rl.safety_geometry import JOINT_NAMES


class HeuristicJacobianPolicy:
    """Small fallback policy used before a learned policy exists."""

    def __init__(self, env: RebotSafeRLEnv, gain: float = 0.80, epsilon: float = 1e-3):
        self.env = env
        self.gain = float(gain)
        self.epsilon = float(epsilon)

    def __call__(self, _observation: np.ndarray) -> np.ndarray:
        model = self.env.safety_model
        joints = self.env.joint_positions.copy()
        target = self.env.target_position
        tip = model.forward_kinematics(joints)["spray_tip_link"]
        error = target - tip
        jacobian = np.zeros((3, 6), dtype=np.float64)
        for idx in range(6):
            perturbed = joints.copy()
            perturbed[idx] += self.epsilon
            perturbed = model.clip_joints(perturbed)
            tip_perturbed = model.forward_kinematics(perturbed)["spray_tip_link"]
            jacobian[:, idx] = (tip_perturbed - tip) / max(self.epsilon, 1e-9)
        action = self.gain * jacobian.T @ error
        return model.clip_action(action)


class TorchPolicy:
    def __init__(self, policy_path: str, action_limit: float):
        import torch
        import torch.nn as nn

        checkpoint = torch.load(policy_path, map_location="cpu", weights_only=False)
        obs_dim = int(checkpoint["obs_dim"])
        act_dim = int(checkpoint.get("act_dim", 6))
        hidden_sizes = tuple(checkpoint.get("hidden_sizes", (128, 128)))
        self.obs_dim = obs_dim

        layers = []
        last_dim = obs_dim
        for hidden in hidden_sizes:
            layers.append(nn.Linear(last_dim, int(hidden)))
            layers.append(nn.Tanh())
            last_dim = int(hidden)
        layers.append(nn.Linear(last_dim, act_dim))
        self.actor = nn.Sequential(*layers)
        self.actor.load_state_dict(checkpoint["actor_state_dict"])
        self.actor.eval()
        self.torch = torch
        self.action_limit = float(action_limit)
        self.obs_mean = np.asarray(checkpoint.get("obs_mean", np.zeros(obs_dim)), dtype=np.float32)
        self.obs_std = np.asarray(checkpoint.get("obs_std", np.ones(obs_dim)), dtype=np.float32)

    def __call__(self, observation: np.ndarray) -> np.ndarray:
        obs = (observation.astype(np.float32) - self.obs_mean) / np.maximum(self.obs_std, 1e-6)
        with self.torch.no_grad():
            tensor = self.torch.as_tensor(obs, dtype=self.torch.float32).unsqueeze(0)
            action = self.actor(tensor).squeeze(0).cpu().numpy()
        return np.tanh(action) * self.action_limit


class RebotSafeRLController(Node):
    def __init__(self):
        super().__init__("rebot_safe_rl_controller")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("control_rate_hz", 8.0)
        self.declare_parameter("trajectory_duration_sec", 0.32)
        self.declare_parameter("trajectory_min_duration_sec", 0.22)
        self.declare_parameter("trajectory_max_duration_sec", 0.55)
        self.declare_parameter("trajectory_nominal_joint_speed", 0.18)
        self.declare_parameter("trajectory_waypoints", 6)
        self.declare_parameter("max_steps", 360)
        self.declare_parameter("target_x", 0.45)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_z", 0.22)
        self.declare_parameter("safe_distance", 0.10)
        self.declare_parameter("hard_clearance", 0.0)
        self.declare_parameter("max_action_delta", 0.04)
        self.declare_parameter("action_low_pass_alpha", 0.55)
        self.declare_parameter("ground_frame", "map")
        self.declare_parameter("min_spray_tip_z", 0.24)
        self.declare_parameter("ground_clearance", 0.10)
        self.declare_parameter("spray_tip_work_height", 0.30)
        self.declare_parameter("success_tolerance", 0.035)
        self.declare_parameter("tf_success_tolerance", 0.07)
        self.declare_parameter("spray_on_success", False)
        self.declare_parameter("enable_teacher_fallback", True)
        self.declare_parameter("teacher_min_progress", 1e-4)
        self.declare_parameter("move_home_on_start", True)
        self.declare_parameter("home_trajectory_duration_sec", 1.0)
        self.declare_parameter("joint_velocity_limit", 1.6)
        self.declare_parameter("joint_acceleration_limit", 2.5)
        self.declare_parameter("done_topic", "rebot_safe_rl/done")
        self.declare_parameter("target_pose_topic", "spray/target_pose")
        self.declare_parameter("gazebo_arm_trajectory_topic", "/gazebo_arm/set_joint_trajectory")
        self.declare_parameter("arm_controller_action_name", "rebotarm_controller/follow_joint_trajectory")

        target = (
            float(self.get_parameter("target_x").value),
            float(self.get_parameter("target_y").value),
            float(self.get_parameter("target_z").value),
        )
        self.env = RebotSafeRLEnv(
            node=self,
            target_position=target,
            safe_distance=float(self.get_parameter("safe_distance").value),
            hard_clearance=float(self.get_parameter("hard_clearance").value),
            max_action_delta=float(self.get_parameter("max_action_delta").value),
            success_tolerance=float(self.get_parameter("success_tolerance").value),
            max_episode_steps=int(self.get_parameter("max_steps").value),
            subscribe_leak_pose=False,
        )
        self.policy = self.load_policy()
        self.teacher_policy = HeuristicJacobianPolicy(self.env)

        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("arm_controller_action_name").value),
        )
        self.latest_real_time_factor = None
        self.spray_start_pub = self.create_publisher(Bool, "spray/start", 10)
        self.done_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("done_topic").value),
            10)
        self.gazebo_arm_pub = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("gazebo_arm_trajectory_topic").value),
            10)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("target_pose_topic").value),
            self.on_target_pose,
            10)
        self.create_subscription(
            PerformanceMetrics,
            "/gazebo/performance_metrics",
            self.on_performance_metrics,
            10)
        self.create_subscription(Bool, "rebot_safe_rl/start", self.on_start, 10)
        self.create_subscription(Bool, "rebot_safe_rl/stop", self.on_stop, 10)

        self.active = bool(self.get_parameter("auto_start").value)
        self.in_flight = False
        self.finished = False
        self.step_index = 0
        self.first_policy_step_logged = False
        self.last_start_wall_time = 0.0
        self.home_joints = np.zeros(6, dtype=np.float64)
        self.filtered_action = np.zeros(6, dtype=np.float64)
        period = 1.0 / max(float(self.get_parameter("control_rate_hz").value), 0.1)
        self.timer = self.create_timer(period, self.on_timer)
        self.get_logger().info(
            "Safe RL controller ready. target_in_rebot_base="
            f"({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}), "
            f"auto_start={self.active}, "
            f"max_action_delta={float(self.get_parameter('max_action_delta').value):.3f}, "
            f"trajectory_duration={float(self.get_parameter('trajectory_duration_sec').value):.2f}s, "
            f"trajectory_duration_range="
            f"[{float(self.get_parameter('trajectory_min_duration_sec').value):.2f}, "
            f"{float(self.get_parameter('trajectory_max_duration_sec').value):.2f}]s"
        )

    def load_policy(self):
        policy_path = str(self.get_parameter("policy_path").value).strip()
        action_limit = float(self.get_parameter("max_action_delta").value)
        if policy_path and os.path.isfile(policy_path):
            try:
                policy = TorchPolicy(policy_path, action_limit)
                if policy.obs_dim != self.env.observation_dim:
                    raise ValueError(
                        f"policy obs_dim={policy.obs_dim} but current env obs_dim={self.env.observation_dim}; "
                        "please retrain the Safe RL policy")
                self.get_logger().info(f"Loaded Safe PPO-Lagrangian policy: {policy_path}")
                return policy
            except Exception as exc:
                self.get_logger().warn(
                    f"Could not load policy '{policy_path}', using heuristic fallback: {exc}")
        else:
            self.get_logger().warn("No trained policy_path provided; using heuristic fallback policy.")
        return HeuristicJacobianPolicy(self.env)

    def on_start(self, msg: Bool) -> None:
        if msg.data:
            now = time.monotonic()
            if now - self.last_start_wall_time < 1.0:
                return
            self.last_start_wall_time = now
            self.active = True
            self.finished = False
            self.step_index = 0
            self.first_policy_step_logged = False
            self.filtered_action = np.zeros(6, dtype=np.float64)
            self.update_ground_safety()
            self.publish_done(False)
            self.get_logger().info(
                "Safe RL arm controller started. target_in_rebot_base="
                f"({self.env.target_position[0]:.3f}, "
                f"{self.env.target_position[1]:.3f}, "
                f"{self.env.target_position[2]:.3f})")
            if bool(self.get_parameter("move_home_on_start").value):
                self.get_logger().info("Moving arm to training home pose before Safe RL rollout.")
                self.send_joint_trajectory(
                    self.home_joints,
                    duration_sec=float(self.get_parameter("home_trajectory_duration_sec").value))

    def on_stop(self, msg: Bool) -> None:
        if msg.data:
            self.active = False
            self.get_logger().info("Safe RL arm controller stopped.")

    def on_target_pose(self, msg: PoseStamped) -> None:
        transformed = self.env._pose_to_target_frame(msg)
        if transformed is None:
            self.get_logger().warn(
                f"Could not transform target pose from {msg.header.frame_id} "
                f"to {self.env.target_frame}; keeping previous target.")
            return
        transformed = self.validate_target_height(transformed)
        self.env.set_target_position(transformed)
        self.filtered_action = np.zeros(6, dtype=np.float64)
        self.get_logger().info(
            "Updated Safe RL target from spray/target_pose: "
            f"({transformed[0]:.3f}, {transformed[1]:.3f}, {transformed[2]:.3f})")

    def on_performance_metrics(self, msg: PerformanceMetrics) -> None:
        self.latest_real_time_factor = float(msg.real_time_factor)

    def on_timer(self) -> None:
        try:
            self._on_timer_impl()
        except Exception as exc:
            self.active = False
            self.finished = True
            self.in_flight = False
            self.get_logger().error(
                "Safe RL controller timer failed; publishing done=false. "
                f"error={exc}\n{traceback.format_exc()}")
            self.publish_done(False)

    def _on_timer_impl(self) -> None:
        if not self.active or self.finished or self.in_flight:
            return
        if not self.trajectory_client.server_is_ready():
            self.trajectory_client.wait_for_server(timeout_sec=0.01)
            return

        self.update_ground_safety()
        observation = self.env.get_observation()
        raw_action = self.policy(observation)
        if not self.first_policy_step_logged:
            self.first_policy_step_logged = True
            self.get_logger().info(
                "Safe RL policy is controlling the arm: "
                f"obs_dim={observation.size}, "
                f"target=({self.env.target_position[0]:.3f}, "
                f"{self.env.target_position[1]:.3f}, {self.env.target_position[2]:.3f})")
        raw_action = self.low_pass_action(raw_action)
        shield = self.env.filter_action(raw_action)
        if bool(self.get_parameter("enable_teacher_fallback").value):
            shield = self.apply_teacher_fallback_if_needed(observation, shield)
        self.env.last_shield_result = shield
        if not shield.accepted and np.linalg.norm(shield.applied_action) <= 1e-9:
            actual_distance = self.actual_tip_distance()
            if (
                actual_distance is not None
                and actual_distance <= float(self.get_parameter("tf_success_tolerance").value)
            ):
                self.finished = True
                self.active = False
                self.get_logger().info(
                    "spray_tip_link reached the safe RL target by TF distance: "
                    f"{actual_distance:.3f} m")
                self.publish_done(True)
                return
            self.get_logger().warn(
                "RL action rejected by safety shield: "
                f"reason={shield.reason}, cost={shield.report.cost:.3f}, "
                f"min_dist={shield.report.min_distance:.3f}, pairs={shield.report.collision_pairs}")
            self.finished = True
            self.publish_done(False)
            return

        sent = self.send_joint_trajectory(shield.target_joints)
        if not sent:
            return

        self.step_index += 1
        distance = self.env.info_dict()["distance_to_target"]
        actual_distance = self.actual_tip_distance()
        if self.step_index % 10 == 0:
            actual_text = "unavailable" if actual_distance is None else f"{actual_distance:.3f}"
            self.get_logger().info(
                f"Safe RL step={self.step_index}, distance={distance:.3f}, "
                f"tf_distance={actual_text}, cost={shield.report.cost:.3f}, "
                f"shield_scale={shield.scale:.2f}")
        tf_success = (
            actual_distance is not None
            and actual_distance <= float(self.get_parameter("tf_success_tolerance").value)
        )
        if distance <= self.env.success_tolerance or tf_success:
            self.finished = True
            self.active = False
            if tf_success:
                self.get_logger().info(
                    "spray_tip_link reached the safe RL target by TF distance: "
                    f"{actual_distance:.3f} m")
            else:
                self.get_logger().info("spray_tip_link reached the safe RL target.")
            self.publish_done(True)
            if bool(self.get_parameter("spray_on_success").value):
                msg = Bool()
                msg.data = True
                self.spray_start_pub.publish(msg)
        elif self.step_index >= int(self.get_parameter("max_steps").value):
            self.finished = True
            self.active = False
            self.get_logger().warn("Safe RL controller reached max_steps before success.")
            self.publish_done(False)

    def low_pass_action(self, raw_action: Sequence[float]) -> np.ndarray:
        alpha = float(self.get_parameter("action_low_pass_alpha").value)
        alpha = min(1.0, max(0.0, alpha))
        action = np.asarray(raw_action, dtype=np.float64).reshape(6)
        self.filtered_action = alpha * action + (1.0 - alpha) * self.filtered_action
        return self.filtered_action.copy()

    def update_ground_safety(self) -> None:
        ground_z = self.ground_z_in_target_frame()
        if ground_z is None:
            return
        clearance = max(0.0, float(self.get_parameter("ground_clearance").value))
        self.env.set_ground_min_z(ground_z + clearance)
        self.env.set_target_position(self.validate_target_height(self.env.target_position, ground_z))

    def validate_target_height(
        self,
        target_xyz: Sequence[float],
        ground_z: Optional[float] = None,
    ) -> np.ndarray:
        target = np.asarray(target_xyz, dtype=np.float64).reshape(3).copy()
        if ground_z is None:
            ground_z = self.ground_z_in_target_frame()
        if ground_z is None:
            return target

        min_tip_height = max(
            float(self.get_parameter("min_spray_tip_z").value),
            float(self.get_parameter("ground_clearance").value),
            float(self.get_parameter("spray_tip_work_height").value),
        )
        min_target_z = float(ground_z) + min_tip_height
        if target[2] < min_target_z:
            old_z = float(target[2])
            target[2] = min_target_z
            self.get_logger().warn(
                "Safe RL target z raised above ground: "
                f"old_z={old_z:.3f}, new_z={target[2]:.3f}, "
                f"ground_z_in_{self.env.target_frame}={ground_z:.3f}, "
                f"min_tip_height={min_tip_height:.3f}")
        return target

    def ground_z_in_target_frame(self) -> Optional[float]:
        if self.env.tf_buffer is None:
            return None
        ground_frame = str(self.get_parameter("ground_frame").value)
        try:
            tf_msg = self.env.tf_buffer.lookup_transform(
                self.env.target_frame,
                ground_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return None
        return float(tf_msg.transform.translation.z)

    def send_joint_trajectory(self, target_positions: Sequence[float], duration_sec: Optional[float] = None) -> bool:
        goal = FollowJointTrajectory.Goal()
        target = np.asarray(target_positions, dtype=np.float64).reshape(6)
        start = np.asarray(self.env.joint_positions, dtype=np.float64).reshape(6)
        delta = target - start
        if duration_sec is None:
            duration = self.compute_adaptive_trajectory_duration(delta)
        else:
            duration = max(0.05, float(duration_sec))
        goal.trajectory = self.build_joint_trajectory(target, duration)
        self.log_trajectory_debug("Safe RL JointTrajectory", start, target, delta, goal.trajectory, duration)

        self.publish_gazebo_arm_trajectory(goal.trajectory)
        self.in_flight = True
        future = self.trajectory_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)
        return True

    def compute_adaptive_trajectory_duration(self, joint_delta: Sequence[float]) -> float:
        max_delta = float(np.max(np.abs(np.asarray(joint_delta, dtype=np.float64))))
        fallback = float(self.get_parameter("trajectory_duration_sec").value)
        nominal_speed = max(
            1e-3,
            float(self.get_parameter("trajectory_nominal_joint_speed").value))
        min_duration = max(
            0.05,
            float(self.get_parameter("trajectory_min_duration_sec").value))
        max_duration = max(
            min_duration,
            float(self.get_parameter("trajectory_max_duration_sec").value))
        adaptive = max_delta / nominal_speed if max_delta > 1e-6 else fallback
        return min(max(adaptive, min_duration), max_duration)

    def build_joint_trajectory(
        self,
        target_positions: Sequence[float],
        duration_sec: float,
    ) -> JointTrajectory:
        target = np.asarray(target_positions, dtype=np.float64).reshape(6)
        start = np.asarray(self.env.joint_positions, dtype=np.float64).reshape(6)
        duration = max(0.05, float(duration_sec))
        waypoint_count = max(2, int(self.get_parameter("trajectory_waypoints").value))

        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        for idx in range(1, waypoint_count + 1):
            ratio = float(idx) / float(waypoint_count)
            smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
            point = JointTrajectoryPoint()
            point.positions = [
                float(value)
                for value in (start + (target - start) * smooth_ratio)
            ]
            point.time_from_start = self.duration_msg(duration * ratio)
            traj.points.append(point)
        return traj

    def log_trajectory_debug(
        self,
        label: str,
        start: np.ndarray,
        target: np.ndarray,
        delta: np.ndarray,
        trajectory: JointTrajectory,
        duration: float,
    ) -> None:
        rtf = (
            "unavailable"
            if self.latest_real_time_factor is None
            else f"{self.latest_real_time_factor:.2f}")
        self.get_logger().info(
            f"{label}: current={self.format_joints(start)}, "
            f"target={self.format_joints(target)}, "
            f"delta={self.format_joints(delta)}, "
            f"points={len(trajectory.points)}, duration={duration:.2f}s, "
            f"max_action_delta={float(self.get_parameter('max_action_delta').value):.3f}rad, "
            f"low_pass_alpha={float(self.get_parameter('action_low_pass_alpha').value):.2f}, "
            f"joint_limits={self.joint_limit_text()}, "
            f"gazebo_real_time_factor={rtf}")

    @staticmethod
    def format_joints(values: Sequence[float]) -> str:
        return "[" + ", ".join(f"{float(v):+.3f}" for v in values) + "]"

    def joint_limit_text(self) -> str:
        velocity = float(self.get_parameter("joint_velocity_limit").value)
        acceleration = float(self.get_parameter("joint_acceleration_limit").value)
        return (
            "{joint1..joint6: "
            f"max_velocity={velocity:.2f}rad/s, "
            f"max_acceleration={acceleration:.2f}rad/s^2"
            "}")

    def apply_teacher_fallback_if_needed(self, observation: np.ndarray, shield):
        current_tip = self.env.safety_model.forward_kinematics(
            self.env.joint_positions)["spray_tip_link"]
        candidate_tip = self.env.safety_model.forward_kinematics(
            shield.target_joints)["spray_tip_link"]
        current_distance = float(np.linalg.norm(self.env.target_position - current_tip))
        candidate_distance = float(np.linalg.norm(self.env.target_position - candidate_tip))
        min_progress = float(self.get_parameter("teacher_min_progress").value)
        if (
            shield.accepted
            and not shield.report.collision
            and candidate_distance <= current_distance - min_progress
        ):
            return shield
        teacher_action = self.teacher_policy(observation)
        teacher_shield = self.env.filter_action(teacher_action)
        teacher_tip = self.env.safety_model.forward_kinematics(
            teacher_shield.target_joints)["spray_tip_link"]
        teacher_distance = float(np.linalg.norm(self.env.target_position - teacher_tip))
        self.get_logger().debug(
            "Teacher fallback used: "
            f"nn_distance={candidate_distance:.4f}, current_distance={current_distance:.4f}, "
            f"teacher_distance={teacher_distance:.4f}, teacher_scale={teacher_shield.scale:.2f}")
        if (
            teacher_shield.accepted
            and not teacher_shield.report.collision
            and teacher_distance <= current_distance - min_progress
        ):
            return teacher_shield
        search_shield = self.search_safe_improving_action(current_distance, min_progress)
        if search_shield is not None:
            return search_shield
        if shield.accepted and not shield.report.collision:
            return shield
        return teacher_shield

    def search_safe_improving_action(self, current_distance: float, min_progress: float):
        """Last-resort local action search when the learned policy stalls."""
        action_limit = float(self.get_parameter("max_action_delta").value)
        candidates = []
        for scale in (1.0, 0.5, 0.25):
            for joint_idx in range(6):
                for sign in (-1.0, 1.0):
                    action = np.zeros(6, dtype=np.float64)
                    action[joint_idx] = sign * action_limit * scale
                    candidates.append(action)

        # A few deterministic coupled moves help the wrist keep advancing after
        # single-axis moves are blocked by the car/lidar safety geometry.
        coupled_patterns = (
            (0.0, -1.0, 1.0, 0.5, 0.0, 0.0),
            (0.0, 1.0, -1.0, -0.5, 0.0, 0.0),
            (0.4, -1.0, 1.0, 0.5, -0.4, 0.0),
            (-0.4, -1.0, 1.0, 0.5, 0.4, 0.0),
            (0.4, 1.0, -1.0, -0.5, -0.4, 0.0),
            (-0.4, 1.0, -1.0, -0.5, 0.4, 0.0),
        )
        for scale in (1.0, 0.5):
            for pattern in coupled_patterns:
                candidates.append(np.asarray(pattern, dtype=np.float64) * action_limit * scale)

        best_shield = None
        best_distance = current_distance
        for action in candidates:
            candidate = self.env.filter_action(action)
            if (
                not candidate.accepted
                or candidate.report.collision
                or np.linalg.norm(candidate.applied_action) <= 1e-9
            ):
                continue
            tip = self.env.safety_model.forward_kinematics(
                candidate.target_joints)["spray_tip_link"]
            distance = float(np.linalg.norm(self.env.target_position - tip))
            if distance <= best_distance - min_progress:
                best_distance = distance
                best_shield = candidate

        if best_shield is not None:
            self.get_logger().debug(
                "Safe local search fallback selected: "
                f"distance {current_distance:.4f} -> {best_distance:.4f}, "
                f"scale={best_shield.scale:.2f}")
        return best_shield

    def actual_tip_distance(self) -> Optional[float]:
        if self.env.tf_buffer is None:
            return None
        try:
            tf_msg = self.env.tf_buffer.lookup_transform(
                self.env.target_frame,
                "spray_tip_link",
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return None
        translation = tf_msg.transform.translation
        tip = np.asarray(
            [translation.x, translation.y, translation.z],
            dtype=np.float64)
        return float(np.linalg.norm(self.env.target_position - tip))

    def on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.in_flight = False
            self.get_logger().warn(f"Trajectory goal send failed: {exc}")
            return
        if not handle.accepted:
            self.in_flight = False
            self.get_logger().warn("Trajectory goal rejected by controller.")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future) -> None:
        self.in_flight = False
        try:
            future.result()
        except Exception as exc:
            self.get_logger().warn(f"Trajectory execution failed: {exc}")

    def publish_done(self, done: bool) -> None:
        msg = Bool()
        msg.data = bool(done)
        for _ in range(3):
            self.done_pub.publish(msg)

    @staticmethod
    def duration_msg(duration_sec: float) -> DurationMsg:
        duration = max(0.0, float(duration_sec))
        sec = int(math.floor(duration))
        nanosec = int(round((duration - sec) * 1e9))
        if nanosec >= 1000000000:
            sec += 1
            nanosec -= 1000000000
        return DurationMsg(sec=sec, nanosec=nanosec)

    def publish_gazebo_arm_trajectory(self, trajectory: JointTrajectory) -> None:
        traj = JointTrajectory()
        # Zero stamp means "start immediately" for the Gazebo joint trajectory
        # plugin. A wall-time stamp would be far in the future in sim time.
        traj.header.frame_id = "world"
        traj.joint_names = list(trajectory.joint_names)
        traj.points = list(trajectory.points)
        for _ in range(2):
            self.gazebo_arm_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = RebotSafeRLController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

import math
import os
import time
from pathlib import Path

import numpy as np

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from version_car_sim.rl.rebot_safe_rl_controller import HeuristicJacobianPolicy
from version_car_sim.rl.rebot_safe_rl_env import RebotSafeRLEnv, observation_dim
from version_car_sim.rl.safety_geometry import JOINT_LIMITS, JOINT_NAMES


class GazeboActor:
    def __init__(self, obs_dim, act_dim, hidden_sizes=(128, 128), action_limit=0.025):
        import torch
        import torch.nn as nn

        self.torch = torch
        self.nn = nn
        self.hidden_sizes = hidden_sizes
        self.action_limit = float(action_limit)
        layers = []
        last = int(obs_dim)
        for hidden in hidden_sizes:
            layers.append(nn.Linear(last, int(hidden)))
            layers.append(nn.Tanh())
            last = int(hidden)
        layers.append(nn.Linear(last, int(act_dim)))
        self.model = nn.Sequential(*layers)

    def predict_tensor(self, obs_tensor):
        return self.torch.tanh(self.model(obs_tensor)) * self.action_limit

    def save(self, path, config):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.torch.save(
            {
                "obs_dim": observation_dim(),
                "act_dim": 6,
                "hidden_sizes": self.hidden_sizes,
                "actor_state_dict": self.model.state_dict(),
                "obs_mean": np.zeros(observation_dim(), dtype=np.float32),
                "obs_std": np.ones(observation_dim(), dtype=np.float32),
                "config": dict(config),
                "training_source": "gazebo_whole_car_teacher_warmstart",
            },
            path,
        )


class RebotSafeRLGazeboTrainer(Node):
    """Collect teacher data in the full Gazebo car+arm model and fit a policy.

    This is intentionally a whole-system trainer: observations come from live
    /joint_states and TF while the arm is mounted on the car model in Gazebo.
    The safety shield remains active for every executed teacher step.
    """

    def __init__(self):
        super().__init__("rebot_safe_rl_gazebo_trainer")
        self.declare_parameter("output_dir", "ros2_ws/src/version_car_sim/trained_policies/rebot_safe_ppo_lagrangian_gazebo")
        self.declare_parameter("teacher_episodes", 80)
        self.declare_parameter("max_episode_steps", 120)
        self.declare_parameter("ik_teacher_enabled", True)
        self.declare_parameter("ik_teacher_maxiter", 60)
        self.declare_parameter("unfold_before_target", True)
        self.declare_parameter("unfold_steps", 48)
        self.declare_parameter("return_lift_before_escape", True)
        self.declare_parameter("return_lift_steps", 120)
        self.declare_parameter("return_lift_delta_z", 0.45)
        self.declare_parameter("return_lift_min_z", 0.45)
        self.declare_parameter("return_escape_steps", 96)
        self.declare_parameter("return_home_via_escape", True)
        self.declare_parameter("return_home_via_unfold", True)
        self.declare_parameter("unfold_joint1_abs", 0.18)
        self.declare_parameter("unfold_joint2", -0.50)
        self.declare_parameter("unfold_joint3", -0.70)
        self.declare_parameter("unfold_joint4_abs", 0.20)
        self.declare_parameter("unfold_joint5", 0.18)
        self.declare_parameter("unfold_joint6_abs", 0.14)
        self.declare_parameter("return_escape_joint1_abs", 0.55)
        self.declare_parameter("return_escape_joint2", -0.75)
        self.declare_parameter("return_escape_joint3", -0.95)
        self.declare_parameter("return_escape_joint4_abs", 0.45)
        self.declare_parameter("return_escape_joint5", 0.25)
        self.declare_parameter("return_escape_joint6_abs", 0.35)
        self.declare_parameter("bc_updates", 500)
        self.declare_parameter("bc_batch_size", 512)
        self.declare_parameter("learning_rate", 1e-3)
        self.declare_parameter("safe_distance", 0.10)
        self.declare_parameter("max_action_delta", 0.025)
        self.declare_parameter("success_tolerance", 0.035)
        self.declare_parameter("trajectory_duration_sec", 0.18)
        self.declare_parameter("target_x", 0.45)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_z", 0.16)
        self.declare_parameter("target_mode", "fixed")
        self.declare_parameter("car_center_x", -3.0)
        self.declare_parameter("car_center_y", 0.0)
        self.declare_parameter("car_center_z", 0.0)
        self.declare_parameter("car_yaw", 0.0)
        self.declare_parameter("car_safety_radius", 0.452548)
        self.declare_parameter("arm_base_offset_x", 0.0)
        self.declare_parameter("arm_base_offset_y", 0.0)
        self.declare_parameter("arm_base_height", 0.24)
        self.declare_parameter("ground_target_z", 0.0)
        self.declare_parameter("spray_standoff", 0.20)
        self.declare_parameter("ground_target_clearances", "0.35")
        self.declare_parameter("ground_target_angles_deg", "-90,-60,-30,0,30,60,90")
        self.declare_parameter("use_live_leak_pose_target", True)
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
            max_action_delta=float(self.get_parameter("max_action_delta").value),
            success_tolerance=float(self.get_parameter("success_tolerance").value),
            max_episode_steps=int(self.get_parameter("max_episode_steps").value),
            subscribe_leak_pose=bool(self.get_parameter("use_live_leak_pose_target").value),
        )
        self.training_targets = self.build_training_targets()
        self.teacher = HeuristicJacobianPolicy(self.env)
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("arm_controller_action_name").value),
        )
        self.gazebo_arm_pub = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("gazebo_arm_trajectory_topic").value),
            10)
        self.dataset_obs = []
        self.dataset_actions = []
        self.ik_cache = {}
        self.done = False

    def run(self):
        if not self.trajectory_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("找不到 rebotarm_controller/follow_joint_trajectory。")
            return False
        if not self.wait_for_joint_state(15.0):
            self.get_logger().error("一直没有收到完整 /joint_states，无法在 Gazebo 里训练。")
            return False

        episodes = int(self.get_parameter("teacher_episodes").value)
        for episode in range(episodes):
            if not rclpy.ok():
                return False
            self.select_episode_target(episode)
            self.reset_arm_home()
            success = self.collect_episode()
            self.return_arm_home()
            if (episode + 1) % max(episodes // 10, 1) == 0:
                target = self.training_targets[episode % len(self.training_targets)]
                self.get_logger().info(
                    f"Gazebo whole-car teacher data {episode + 1}/{episodes}, "
                    f"samples={len(self.dataset_obs)}, last_success={success}, "
                    f"target_map=({target['map'][0]:.3f}, {target['map'][1]:.3f}, {target['map'][2]:.3f}), "
                    f"leak_rebot=({target['leak_rebot'][0]:.3f}, {target['leak_rebot'][1]:.3f}, {target['leak_rebot'][2]:.3f}), "
                    f"target_rebot=({target['rebot'][0]:.3f}, {target['rebot'][1]:.3f}, {target['rebot'][2]:.3f})")
        if not self.dataset_obs:
            self.get_logger().error("没有采集到训练样本。")
            return False
        self.train_actor()
        return True

    def wait_for_joint_state(self, timeout_sec):
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(abs(value) < 100.0 for value in self.env.joint_positions):
                return True
        return False

    def build_training_targets(self):
        mode = str(self.get_parameter("target_mode").value).strip().lower()
        if mode in ("ground", "ground_ring", "ground_points", "ring"):
            targets = self.build_ground_ring_targets()
        else:
            fixed = np.asarray([
                float(self.get_parameter("target_x").value),
                float(self.get_parameter("target_y").value),
                float(self.get_parameter("target_z").value),
            ], dtype=np.float64)
            targets = [{
                "map": np.asarray([math.nan, math.nan, math.nan], dtype=np.float64),
                "leak_rebot": fixed,
                "rebot": fixed,
                "angle_deg": math.nan,
                "clearance": math.nan,
            }]
        self.get_logger().info(f"训练目标模式: {mode}, 目标点数量: {len(targets)}")
        for idx, target in enumerate(targets):
            map_xyz = target["map"]
            rebot_xyz = target["rebot"]
            self.get_logger().info(
                f"  target[{idx}]: map=({map_xyz[0]:.3f}, {map_xyz[1]:.3f}, {map_xyz[2]:.3f}), "
                f"leak_rebot_base_link=({target['leak_rebot'][0]:.3f}, {target['leak_rebot'][1]:.3f}, {target['leak_rebot'][2]:.3f}), "
                f"spray_tip_target=({rebot_xyz[0]:.3f}, {rebot_xyz[1]:.3f}, {rebot_xyz[2]:.3f}), "
                f"angle={target['angle_deg']:.1f} deg, clearance={target['clearance']:.3f} m")
        return targets

    def build_ground_ring_targets(self):
        clearances = self.parse_float_list(
            str(self.get_parameter("ground_target_clearances").value),
            default=[0.35],
        )
        angles = self.parse_float_list(
            str(self.get_parameter("ground_target_angles_deg").value),
            default=[-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0],
        )
        car_center = np.asarray([
            float(self.get_parameter("car_center_x").value),
            float(self.get_parameter("car_center_y").value),
            float(self.get_parameter("car_center_z").value),
        ], dtype=np.float64)
        car_yaw = float(self.get_parameter("car_yaw").value)
        car_radius = max(0.0, float(self.get_parameter("car_safety_radius").value))
        ground_z = float(self.get_parameter("ground_target_z").value)
        targets = []
        for clearance in clearances:
            center_distance = car_radius + max(0.0, float(clearance))
            for angle_deg in angles:
                local_angle = math.radians(float(angle_deg))
                local_xy = np.asarray([
                    center_distance * math.cos(local_angle),
                    center_distance * math.sin(local_angle),
                ], dtype=np.float64)
                world_xy = self.rotate_xy(local_xy, car_yaw) + car_center[:2]
                map_xyz = np.asarray([world_xy[0], world_xy[1], ground_z], dtype=np.float64)
                leak_rebot = self.map_point_to_rebot_base(map_xyz)
                targets.append({
                    "map": map_xyz,
                    "leak_rebot": leak_rebot,
                    "rebot": self.compute_spray_tip_target(leak_rebot),
                    "angle_deg": float(angle_deg),
                    "clearance": float(clearance),
                })
        return targets

    def compute_spray_tip_target(self, leak_rebot):
        leak = np.asarray(leak_rebot, dtype=np.float64).reshape(3)
        standoff = max(0.0, float(self.get_parameter("spray_standoff").value))
        norm = float(np.linalg.norm(leak))
        if norm <= 1e-6 or standoff <= 0.0:
            return leak.copy()
        axis = leak / norm
        return leak - axis * min(standoff, max(norm - 1e-3, 0.0))

    def map_point_to_rebot_base(self, map_xyz):
        car_center = np.asarray([
            float(self.get_parameter("car_center_x").value),
            float(self.get_parameter("car_center_y").value),
            float(self.get_parameter("car_center_z").value),
        ], dtype=np.float64)
        car_yaw = float(self.get_parameter("car_yaw").value)
        arm_offset_xy = self.rotate_xy(np.asarray([
            float(self.get_parameter("arm_base_offset_x").value),
            float(self.get_parameter("arm_base_offset_y").value),
        ], dtype=np.float64), car_yaw)
        arm_base_map = car_center + np.asarray([
            arm_offset_xy[0],
            arm_offset_xy[1],
            float(self.get_parameter("arm_base_height").value),
        ], dtype=np.float64)
        delta = np.asarray(map_xyz, dtype=np.float64) - arm_base_map
        xy = self.rotate_xy(delta[:2], -car_yaw)
        return np.asarray([xy[0], xy[1], delta[2]], dtype=np.float64)

    def select_episode_target(self, episode):
        target = self.training_targets[episode % len(self.training_targets)]
        self.env.set_target_position(target["rebot"])

    @staticmethod
    def rotate_xy(xy, yaw):
        c = math.cos(float(yaw))
        s = math.sin(float(yaw))
        return np.asarray([c * xy[0] - s * xy[1], s * xy[0] + c * xy[1]], dtype=np.float64)

    @staticmethod
    def parse_float_list(raw_value, default):
        values = []
        for item in raw_value.replace(";", ",").split(","):
            item = item.strip()
            if not item:
                continue
            values.append(float(item))
        return values if values else list(default)

    def collect_episode(self):
        max_steps = int(self.get_parameter("max_episode_steps").value)
        success = False
        if bool(self.get_parameter("unfold_before_target").value):
            unfold_pose = self.make_unfold_pose(self.env.target_position)
            self.move_toward_joint_target(
                unfold_pose,
                max_steps=int(self.get_parameter("unfold_steps").value),
                record_dataset=True,
            )
        if bool(self.get_parameter("ik_teacher_enabled").value):
            ik_target = self.solve_ik_for_target(self.env.target_position)
            if ik_target is not None:
                self.move_toward_joint_target(
                    ik_target,
                    max_steps=max_steps,
                    record_dataset=True,
                )
                return bool(self.env.info_dict()["success"])
        for _step in range(max_steps):
            rclpy.spin_once(self, timeout_sec=0.01)
            obs = self.env.get_observation()
            action = self.teacher(obs)
            shield = self.env.filter_action(action)
            if not shield.accepted and np.linalg.norm(shield.applied_action) <= 1e-9:
                break
            self.dataset_obs.append(obs.copy())
            self.dataset_actions.append(shield.applied_action.astype(np.float32))
            if not self.send_joint_positions(shield.target_joints):
                break
            self.env.set_joint_state(shield.target_joints)
            info = self.env.info_dict()
            success = bool(info["success"])
            if success:
                break
        return success

    def solve_ik_for_target(self, target_position):
        target = np.asarray(target_position, dtype=np.float64).reshape(3)
        cache_key = tuple(float(f"{value:.4f}") for value in target)
        if cache_key in self.ik_cache:
            cached = self.ik_cache[cache_key]
            return None if cached is None else cached.copy()
        try:
            from scipy.optimize import differential_evolution, minimize
        except Exception as exc:
            self.get_logger().warn(f"IK teacher unavailable, fallback to Jacobian teacher: {exc}")
            self.ik_cache[cache_key] = None
            return None

        model = self.env.safety_model
        min_obstacle_clearance = 0.025

        def objective(values):
            joints = model.clip_joints(values)
            report = model.distance_report(joints)
            tip = model.forward_kinematics(joints)["spray_tip_link"]
            distance = float(np.linalg.norm(tip - target))
            penalty = 0.0
            if report.collision:
                penalty += 100.0
            penalty += 30.0 * max(0.0, min_obstacle_clearance - report.min_obstacle_distance)
            return distance + penalty

        seed = int(abs(sum((idx + 1) * value for idx, value in enumerate(target))) * 10000) % 100000
        try:
            result = differential_evolution(
                objective,
                JOINT_LIMITS.tolist(),
                maxiter=int(self.get_parameter("ik_teacher_maxiter").value),
                popsize=8,
                seed=seed,
                polish=False,
                workers=1,
                updating="immediate",
            )
            refined = minimize(
                objective,
                result.x,
                bounds=JOINT_LIMITS.tolist(),
                method="Nelder-Mead",
                options={"maxiter": 500},
            )
            joints = model.clip_joints(refined.x if refined.fun <= result.fun else result.x)
        except Exception as exc:
            self.get_logger().warn(f"IK teacher failed, fallback to Jacobian teacher: {exc}")
            self.ik_cache[cache_key] = None
            return None

        report = model.distance_report(joints)
        tip = model.forward_kinematics(joints)["spray_tip_link"]
        distance = float(np.linalg.norm(tip - target))
        if report.collision or distance > max(float(self.get_parameter("success_tolerance").value), 0.05):
            self.get_logger().warn(
                "IK teacher did not find a good safe pose: "
                f"target=({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}), "
                f"distance={distance:.3f}, collision={report.collision}, "
                f"pairs={report.collision_pairs}")
            self.ik_cache[cache_key] = None
            return None

        self.get_logger().info(
            "IK teacher target solved: "
            f"target=({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}), "
            f"distance={distance:.3f}, joints="
            f"[{', '.join(f'{value:.3f}' for value in joints)}]")
        self.ik_cache[cache_key] = joints.copy()
        return joints

    def reset_arm_home(self):
        home = np.zeros(6, dtype=np.float64)
        self.send_joint_positions(home, duration_sec=0.6)
        self.env.set_joint_state(home)

    def return_arm_home(self):
        if bool(self.get_parameter("return_lift_before_escape").value):
            lift_pose = self.make_return_lift_pose()
            if lift_pose is not None:
                self.move_toward_joint_target(
                    lift_pose,
                    max_steps=int(self.get_parameter("return_lift_steps").value),
                    record_dataset=False,
                )
        if bool(self.get_parameter("return_home_via_escape").value):
            escape_pose = self.make_return_escape_pose(self.env.target_position)
            self.move_toward_joint_target(
                escape_pose,
                max_steps=int(self.get_parameter("return_escape_steps").value),
                record_dataset=False,
            )
        elif bool(self.get_parameter("return_home_via_unfold").value):
            return_pose = self.make_unfold_pose(self.env.target_position)
            self.move_toward_joint_target(
                return_pose,
                max_steps=int(self.get_parameter("unfold_steps").value),
                record_dataset=False,
            )
        self.move_toward_joint_target(
            np.zeros(6, dtype=np.float64),
            max_steps=int(self.get_parameter("unfold_steps").value),
            record_dataset=False,
        )

    def make_unfold_pose(self, target_position):
        target = np.asarray(target_position, dtype=np.float64).reshape(3)
        side_sign = 1.0 if target[1] >= 0.0 else -1.0
        if abs(float(target[1])) < 1e-4:
            side_sign = 1.0
        return self.env.safety_model.clip_joints(np.asarray([
            side_sign * float(self.get_parameter("unfold_joint1_abs").value),
            float(self.get_parameter("unfold_joint2").value),
            float(self.get_parameter("unfold_joint3").value),
            side_sign * float(self.get_parameter("unfold_joint4_abs").value),
            float(self.get_parameter("unfold_joint5").value),
            side_sign * float(self.get_parameter("unfold_joint6_abs").value),
        ], dtype=np.float64))

    def make_return_lift_pose(self):
        current_tip = self.env.safety_model.forward_kinematics(
            self.env.joint_positions)["spray_tip_link"]
        lift_target = current_tip.copy()
        lift_target[2] = max(
            float(current_tip[2]) + float(self.get_parameter("return_lift_delta_z").value),
            float(self.get_parameter("return_lift_min_z").value),
        )
        lift_pose = self.solve_ik_for_target(lift_target)
        if lift_pose is None:
            self.get_logger().warn(
                "回收抬高 IK 失败，继续尝试侧向撤离: "
                f"current_tip=({current_tip[0]:.3f}, {current_tip[1]:.3f}, {current_tip[2]:.3f}), "
                f"lift_target=({lift_target[0]:.3f}, {lift_target[1]:.3f}, {lift_target[2]:.3f})")
            return None
        self.get_logger().info(
            "回收第一步: 先抬高 spray_tip_link: "
            f"current_tip=({current_tip[0]:.3f}, {current_tip[1]:.3f}, {current_tip[2]:.3f}), "
            f"lift_target=({lift_target[0]:.3f}, {lift_target[1]:.3f}, {lift_target[2]:.3f})")
        return lift_pose

    def make_return_escape_pose(self, target_position):
        target = np.asarray(target_position, dtype=np.float64).reshape(3)
        side_sign = 1.0 if target[1] >= 0.0 else -1.0
        if abs(float(target[1])) < 1e-4:
            side_sign = 1.0
        return self.env.safety_model.clip_joints(np.asarray([
            side_sign * float(self.get_parameter("return_escape_joint1_abs").value),
            float(self.get_parameter("return_escape_joint2").value),
            float(self.get_parameter("return_escape_joint3").value),
            side_sign * float(self.get_parameter("return_escape_joint4_abs").value),
            float(self.get_parameter("return_escape_joint5").value),
            side_sign * float(self.get_parameter("return_escape_joint6_abs").value),
        ], dtype=np.float64))

    def move_toward_joint_target(self, target_joints, max_steps, record_dataset):
        target = self.env.safety_model.clip_joints(target_joints)
        for _step in range(max(1, int(max_steps))):
            rclpy.spin_once(self, timeout_sec=0.01)
            current = self.env.joint_positions.copy()
            error = target - current
            if np.linalg.norm(error, ord=np.inf) <= 1e-3:
                return True
            obs = self.env.get_observation()
            shield = self.env.filter_action(error)
            if not shield.accepted and np.linalg.norm(shield.applied_action) <= 1e-9:
                self.get_logger().warn(
                    "安全展开/回 home 动作被 shield 拒绝: "
                    f"min_dist={shield.report.min_distance:.3f}, "
                    f"pairs={shield.report.collision_pairs}")
                return False
            if record_dataset:
                self.dataset_obs.append(obs.copy())
                self.dataset_actions.append(shield.applied_action.astype(np.float32))
            if not self.send_joint_positions(shield.target_joints):
                return False
            self.env.set_joint_state(shield.target_joints)
        return bool(np.linalg.norm(target - self.env.joint_positions, ord=np.inf) <= 2e-2)

    def send_joint_positions(self, positions, duration_sec=None):
        if duration_sec is None:
            duration_sec = float(self.get_parameter("trajectory_duration_sec").value)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start = self.duration_msg(float(duration_sec))
        goal.trajectory.points = [point]
        self.publish_gazebo_arm_trajectory(positions, float(duration_sec))
        send_future = self.trajectory_client.send_goal_async(goal)
        if not self.wait_future(send_future, 5.0):
            return False
        handle = send_future.result()
        if not handle.accepted:
            return False
        result_future = handle.get_result_async()
        return self.wait_future(result_future, float(duration_sec) + 4.0)

    def publish_gazebo_arm_trajectory(self, positions, duration_sec):
        traj = JointTrajectory()
        # Gazebo's joint pose trajectory plugin treats header.stamp as the
        # trajectory start time. Leave it at zero so commands execute now,
        # even when this trainer is using wall time while Gazebo uses sim time.
        traj.header.frame_id = "world"
        traj.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start = self.duration_msg(float(duration_sec))
        traj.points.append(point)
        for _ in range(2):
            self.gazebo_arm_pub.publish(traj)

    def train_actor(self):
        import torch
        import torch.optim as optim

        obs = np.asarray(self.dataset_obs, dtype=np.float32)
        actions = np.asarray(self.dataset_actions, dtype=np.float32)
        actor = GazeboActor(
            observation_dim(),
            6,
            action_limit=float(self.get_parameter("max_action_delta").value),
        )
        optimizer = optim.Adam(actor.model.parameters(), lr=float(self.get_parameter("learning_rate").value))
        updates = int(self.get_parameter("bc_updates").value)
        batch_size = int(self.get_parameter("bc_batch_size").value)
        rng = np.random.default_rng(7)
        for update in range(updates):
            indices = rng.integers(0, len(obs), size=batch_size)
            obs_tensor = torch.as_tensor(obs[indices], dtype=torch.float32)
            action_tensor = torch.as_tensor(actions[indices], dtype=torch.float32)
            predicted = actor.predict_tensor(obs_tensor)
            loss = ((predicted - action_tensor) ** 2).mean()
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            if (update + 1) % max(updates // 10, 1) == 0:
                self.get_logger().info(f"Gazebo BC update {update + 1}/{updates}, mse={loss.item():.8f}")

        output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        policy_path = output_dir / "policy.pt"
        actor.save(
            policy_path,
            {
                "teacher_episodes": int(self.get_parameter("teacher_episodes").value),
                "samples": int(len(obs)),
                "safe_distance": float(self.get_parameter("safe_distance").value),
                "max_action_delta": float(self.get_parameter("max_action_delta").value),
            },
        )
        self.get_logger().info(f"整车 Gazebo 训练策略已保存: {policy_path}")

    def wait_future(self, future, timeout_sec):
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.02)
            if time.monotonic() - start > timeout_sec:
                return False
        return future.done()

    @staticmethod
    def duration_msg(duration_sec):
        sec = int(math.floor(duration_sec))
        nanosec = int((duration_sec - sec) * 1e9)
        return DurationMsg(sec=sec, nanosec=nanosec)


def main(args=None):
    rclpy.init(args=args)
    node = RebotSafeRLGazeboTrainer()
    try:
        ok = node.run()
        if not ok:
            raise SystemExit(2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

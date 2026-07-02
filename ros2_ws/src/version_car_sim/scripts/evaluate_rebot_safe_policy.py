#!/usr/bin/env python3
import argparse
import json
import sys
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from version_car_sim.rl.rebot_safe_rl_env import RebotSafeRLEnv


class HeuristicJacobianPolicy:
    def __init__(self, env: RebotSafeRLEnv, gain: float = 0.80, epsilon: float = 1e-3):
        self.env = env
        self.gain = float(gain)
        self.epsilon = float(epsilon)

    def __call__(self, _observation):
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
        return model.clip_action(self.gain * jacobian.T @ error)


class TorchActorPolicy:
    def __init__(self, policy_path: str, action_limit: float):
        import torch
        import torch.nn as nn

        checkpoint = torch.load(policy_path, map_location="cpu", weights_only=False)
        obs_dim = int(checkpoint["obs_dim"])
        act_dim = int(checkpoint.get("act_dim", 6))
        hidden_sizes = tuple(checkpoint.get("hidden_sizes", (128, 128)))
        layers = []
        last = obs_dim
        for hidden in hidden_sizes:
            layers += [nn.Linear(last, int(hidden)), nn.Tanh()]
            last = int(hidden)
        layers.append(nn.Linear(last, act_dim))
        self.actor = nn.Sequential(*layers)
        self.actor.load_state_dict(checkpoint["actor_state_dict"])
        self.actor.eval()
        self.torch = torch
        self.action_limit = float(action_limit)
        self.obs_mean = np.asarray(checkpoint.get("obs_mean", np.zeros(obs_dim)), dtype=np.float32)
        self.obs_std = np.asarray(checkpoint.get("obs_std", np.ones(obs_dim)), dtype=np.float32)

    def __call__(self, observation):
        obs = (observation.astype(np.float32) - self.obs_mean) / np.maximum(self.obs_std, 1e-6)
        with self.torch.no_grad():
            tensor = self.torch.as_tensor(obs, dtype=self.torch.float32).unsqueeze(0)
            action = self.actor(tensor).squeeze(0).cpu().numpy()
        return np.tanh(action) * self.action_limit


class HybridTorchTeacherPolicy:
    def __init__(self, env: RebotSafeRLEnv, torch_policy: TorchActorPolicy, min_progress: float = 1e-4):
        self.env = env
        self.torch_policy = torch_policy
        self.teacher = HeuristicJacobianPolicy(env)
        self.min_progress = float(min_progress)
        self.teacher_fallback_count = 0

    def __call__(self, observation):
        action = self.torch_policy(observation)
        current_tip = self.env.safety_model.forward_kinematics(
            self.env.joint_positions)["spray_tip_link"]
        current_distance = float(np.linalg.norm(self.env.target_position - current_tip))
        shield = self.env.filter_action(action)
        candidate_tip = self.env.safety_model.forward_kinematics(
            shield.target_joints)["spray_tip_link"]
        candidate_distance = float(np.linalg.norm(self.env.target_position - candidate_tip))
        if (
            not shield.accepted
            or shield.report.collision
            or candidate_distance > current_distance - self.min_progress
        ):
            self.teacher_fallback_count += 1
            return self.teacher(observation)
        return action


def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate safe reBot RL policy.")
    parser.add_argument("--policy-path", default="")
    parser.add_argument("--disable-teacher-fallback", action="store_true")
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--random-targets", action="store_true")
    parser.add_argument("--target-x", type=float, default=0.45)
    parser.add_argument("--target-y", type=float, default=0.0)
    parser.add_argument("--target-z", type=float, default=0.16)
    parser.add_argument("--target-x-range", type=float, nargs=2, default=(0.28, 0.55))
    parser.add_argument("--target-y-range", type=float, nargs=2, default=(-0.16, 0.16))
    parser.add_argument("--target-z-range", type=float, nargs=2, default=(0.05, 0.25))
    parser.add_argument("--max-episode-steps", type=int, default=240)
    parser.add_argument("--max-action-delta", type=float, default=0.025)
    parser.add_argument("--safe-distance", type=float, default=0.10)
    parser.add_argument("--success-threshold", type=float, default=0.98)
    parser.add_argument("--json-out", default="")
    return parser.parse_args()


def sample_target(args, rng):
    if not args.random_targets:
        return np.asarray([args.target_x, args.target_y, args.target_z], dtype=np.float64)
    return np.asarray([
        rng.uniform(args.target_x_range[0], args.target_x_range[1]),
        rng.uniform(args.target_y_range[0], args.target_y_range[1]),
        rng.uniform(args.target_z_range[0], args.target_z_range[1]),
    ], dtype=np.float64)


def main():
    args = parse_args()
    rng = np.random.default_rng(args.seed)
    env = RebotSafeRLEnv(
        target_position=(args.target_x, args.target_y, args.target_z),
        safe_distance=args.safe_distance,
        max_action_delta=args.max_action_delta,
        random_reset=True,
        seed=args.seed,
        max_episode_steps=args.max_episode_steps,
    )
    if args.policy_path:
        torch_policy = TorchActorPolicy(args.policy_path, args.max_action_delta)
        if args.disable_teacher_fallback:
            policy = torch_policy
            policy_name = f"torch:{args.policy_path}"
        else:
            policy = HybridTorchTeacherPolicy(env, torch_policy)
            policy_name = f"hybrid(torch+teacher):{args.policy_path}"
    else:
        policy = HeuristicJacobianPolicy(env)
        policy_name = "heuristic_jacobian"

    successes = 0
    safe_successes = 0
    collisions = 0
    final_distances = []
    episode_costs = []
    episode_steps = []
    min_distances = []

    for episode in range(args.episodes):
        obs = env.reset()
        env.set_target_position(sample_target(args, rng))
        total_cost = 0.0
        info = {}
        for step in range(args.max_episode_steps):
            obs, reward, cost, done, info = env.step(policy(obs))
            total_cost += float(cost)
            if done:
                break
        success = bool(info.get("success", False))
        collision = bool(info.get("collision", False))
        successes += int(success)
        safe_successes += int(success and not collision)
        collisions += int(collision)
        final_distances.append(float(info.get("distance_to_target", np.nan)))
        episode_costs.append(total_cost)
        episode_steps.append(step + 1)
        min_distances.append(float(info.get("min_obstacle_distance", np.nan)))

    metrics = {
        "policy": policy_name,
        "episodes": args.episodes,
        "success_rate": successes / max(args.episodes, 1),
        "safe_success_rate": safe_successes / max(args.episodes, 1),
        "collision_rate": collisions / max(args.episodes, 1),
        "mean_final_distance": float(np.nanmean(final_distances)),
        "p95_final_distance": float(np.nanpercentile(final_distances, 95)),
        "mean_episode_cost": float(np.mean(episode_costs)),
        "mean_steps": float(np.mean(episode_steps)),
        "mean_min_obstacle_distance": float(np.nanmean(min_distances)),
        "passed_98_percent": safe_successes / max(args.episodes, 1) >= args.success_threshold,
    }
    if hasattr(policy, "teacher_fallback_count"):
        metrics["teacher_fallback_count"] = int(policy.teacher_fallback_count)
    print(json.dumps(metrics, indent=2, ensure_ascii=False))
    if args.json_out:
        path = Path(args.json_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(metrics, indent=2, ensure_ascii=False) + "\n")
    if not metrics["passed_98_percent"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()

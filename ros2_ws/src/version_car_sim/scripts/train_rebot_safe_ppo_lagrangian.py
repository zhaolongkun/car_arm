#!/usr/bin/env python3
import argparse
import csv
import os
import sys
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from version_car_sim.rl.rebot_safe_rl_env import RebotSafeRLEnv, observation_dim
from version_car_sim.rl.safety_geometry import JOINT_LIMITS


def require_torch():
    try:
        import torch
        import torch.nn as nn
        import torch.optim as optim
        from torch.distributions.normal import Normal
    except Exception as exc:
        raise SystemExit(
            "PyTorch is required for Safe PPO-Lagrangian training. "
            "Install/source a Python environment with torch first. "
            f"Original error: {exc}"
        )
    return torch, nn, optim, Normal


torch, nn, optim, Normal = require_torch()


class ActorCritic(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden_sizes=(128, 128), action_limit=0.025):
        super().__init__()
        self.action_limit = float(action_limit)
        self.actor = self._mlp(obs_dim, act_dim, hidden_sizes)
        self.reward_critic = self._mlp(obs_dim, 1, hidden_sizes)
        self.cost_critic = self._mlp(obs_dim, 1, hidden_sizes)
        self.log_std = nn.Parameter(torch.full((act_dim,), -2.0))

    @staticmethod
    def _mlp(input_dim, output_dim, hidden_sizes):
        layers = []
        last = input_dim
        for hidden in hidden_sizes:
            layers += [nn.Linear(last, hidden), nn.Tanh()]
            last = hidden
        layers.append(nn.Linear(last, output_dim))
        return nn.Sequential(*layers)

    def distribution(self, obs):
        mean = torch.tanh(self.actor(obs)) * self.action_limit
        std = torch.exp(self.log_std).expand_as(mean)
        return Normal(mean, std)

    def act(self, obs):
        dist = self.distribution(obs)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(axis=-1)
        return action, log_prob, self.reward_critic(obs).squeeze(-1), self.cost_critic(obs).squeeze(-1)

    def evaluate(self, obs, action):
        dist = self.distribution(obs)
        log_prob = dist.log_prob(action).sum(axis=-1)
        entropy = dist.entropy().sum(axis=-1)
        reward_value = self.reward_critic(obs).squeeze(-1)
        cost_value = self.cost_critic(obs).squeeze(-1)
        return log_prob, entropy, reward_value, cost_value


def compute_gae(rewards, values, dones, last_value, gamma, gae_lambda):
    advantages = np.zeros_like(rewards, dtype=np.float32)
    last_gae = 0.0
    for step in reversed(range(len(rewards))):
        if step == len(rewards) - 1:
            next_nonterminal = 1.0 - dones[step]
            next_value = last_value
        else:
            next_nonterminal = 1.0 - dones[step]
            next_value = values[step + 1]
        delta = rewards[step] + gamma * next_value * next_nonterminal - values[step]
        last_gae = delta + gamma * gae_lambda * next_nonterminal * last_gae
        advantages[step] = last_gae
    returns = advantages + values
    return advantages, returns


def heuristic_action(env: RebotSafeRLEnv, gain: float = 0.80, epsilon: float = 1e-3) -> np.ndarray:
    model = env.safety_model
    joints = env.joint_positions.copy()
    target = env.target_position
    tip = model.forward_kinematics(joints)["spray_tip_link"]
    error = target - tip
    jacobian = np.zeros((3, 6), dtype=np.float64)
    for idx in range(6):
        perturbed = joints.copy()
        perturbed[idx] += epsilon
        perturbed = model.clip_joints(perturbed)
        tip_perturbed = model.forward_kinematics(perturbed)["spray_tip_link"]
        jacobian[:, idx] = (tip_perturbed - tip) / max(epsilon, 1e-9)
    return model.clip_action(gain * jacobian.T @ error)


def sample_target(args, rng) -> np.ndarray:
    if not args.random_targets:
        return np.asarray([args.target_x, args.target_y, args.target_z], dtype=np.float64)
    return np.asarray([
        rng.uniform(args.target_x_range[0], args.target_x_range[1]),
        rng.uniform(args.target_y_range[0], args.target_y_range[1]),
        rng.uniform(args.target_z_range[0], args.target_z_range[1]),
    ], dtype=np.float64)


def run_bc_warmstart(model, env, args, rng):
    if args.bc_warmstart_steps <= 0:
        return
    optimizer = optim.Adam(model.actor.parameters(), lr=args.bc_learning_rate)
    dataset_obs = None
    dataset_actions = None
    if args.bc_trajectory_episodes > 0:
        obs_list = []
        action_list = []
        for episode in range(args.bc_trajectory_episodes):
            obs = env.reset()
            env.set_target_position(sample_target(args, rng))
            for _step in range(args.bc_max_trajectory_steps):
                action = heuristic_action(env)
                obs_list.append(obs.copy())
                action_list.append(action.copy())
                obs, _reward, _cost, done, _info = env.step(action)
                if done:
                    break
            if (episode + 1) % max(args.bc_trajectory_episodes // 10, 1) == 0:
                print(
                    f"collected teacher trajectories "
                    f"{episode + 1}/{args.bc_trajectory_episodes}, samples={len(obs_list)}"
                )
        dataset_obs = np.asarray(obs_list, dtype=np.float32)
        dataset_actions = np.asarray(action_list, dtype=np.float32)

    for update in range(args.bc_warmstart_steps):
        if dataset_obs is not None and len(dataset_obs) > 0:
            indices = rng.integers(0, len(dataset_obs), size=args.bc_batch_size)
            obs_tensor = torch.as_tensor(dataset_obs[indices], dtype=torch.float32)
            target_actions = torch.as_tensor(dataset_actions[indices], dtype=torch.float32)
        else:
            obs_batch = []
            action_batch = []
            for _ in range(args.bc_batch_size):
                if rng.random() < 0.55:
                    joints = env.home_joints + rng.normal(0.0, 0.65, size=6)
                else:
                    lower = JOINT_LIMITS[:, 0]
                    upper = JOINT_LIMITS[:, 1]
                    joints = rng.uniform(lower, upper)
                env.set_joint_state(joints, np.zeros(6, dtype=np.float64))
                env.set_target_position(sample_target(args, rng))
                obs = env.get_observation()
                obs_batch.append(obs)
                action_batch.append(heuristic_action(env))
            obs_tensor = torch.as_tensor(np.asarray(obs_batch), dtype=torch.float32)
            target_actions = torch.as_tensor(np.asarray(action_batch), dtype=torch.float32)
        predicted = torch.tanh(model.actor(obs_tensor)) * model.action_limit
        loss = ((predicted - target_actions) ** 2).mean()
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        if (update + 1) % max(args.bc_warmstart_steps // 10, 1) == 0:
            print(f"bc warmstart {update + 1}/{args.bc_warmstart_steps}, mse={loss.item():.8f}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Train a Safe PPO / PPO-Lagrangian policy for the reBot arm scaffold.")
    parser.add_argument("--total-steps", type=int, default=20000)
    parser.add_argument("--rollout-steps", type=int, default=1024)
    parser.add_argument("--update-epochs", type=int, default=6)
    parser.add_argument("--minibatch-size", type=int, default=256)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--cost-gamma", type=float, default=0.99)
    parser.add_argument("--gae-lambda", type=float, default=0.95)
    parser.add_argument("--clip-coef", type=float, default=0.2)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--vf-coef", type=float, default=0.5)
    parser.add_argument("--ent-coef", type=float, default=0.01)
    parser.add_argument("--cost-limit", type=float, default=8.0)
    parser.add_argument("--lagrange-lr", type=float, default=0.02)
    parser.add_argument("--max-action-delta", type=float, default=0.025)
    parser.add_argument("--safe-distance", type=float, default=0.10)
    parser.add_argument("--target-x", type=float, default=0.45)
    parser.add_argument("--target-y", type=float, default=0.0)
    parser.add_argument("--target-z", type=float, default=0.16)
    parser.add_argument("--random-targets", action="store_true")
    parser.add_argument("--target-x-range", type=float, nargs=2, default=(0.28, 0.55))
    parser.add_argument("--target-y-range", type=float, nargs=2, default=(-0.16, 0.16))
    parser.add_argument("--target-z-range", type=float, nargs=2, default=(0.05, 0.25))
    parser.add_argument("--bc-warmstart-steps", type=int, default=0)
    parser.add_argument("--bc-batch-size", type=int, default=512)
    parser.add_argument("--bc-trajectory-episodes", type=int, default=0)
    parser.add_argument("--bc-max-trajectory-steps", type=int, default=240)
    parser.add_argument("--bc-learning-rate", type=float, default=1e-3)
    parser.add_argument("--output-dir", default="ros2_ws/src/version_car_sim/trained_policies/rebot_safe_ppo_lagrangian")
    return parser.parse_args()


def main():
    args = parse_args()
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    rng = np.random.default_rng(args.seed)

    env = RebotSafeRLEnv(
        target_position=(args.target_x, args.target_y, args.target_z),
        safe_distance=args.safe_distance,
        max_action_delta=args.max_action_delta,
        random_reset=True,
        seed=args.seed,
    )
    obs_dim = observation_dim()
    act_dim = env.action_dim
    hidden_sizes = (128, 128)
    model = ActorCritic(obs_dim, act_dim, hidden_sizes, action_limit=args.max_action_delta)
    optimizer = optim.Adam(model.parameters(), lr=args.learning_rate)
    lagrange_multiplier = 0.0

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    log_path = output_dir / "training_log.csv"

    run_bc_warmstart(model, env, args, rng)

    obs = env.reset()
    env.set_target_position(sample_target(args, rng))
    global_step = 0
    episode_return = 0.0
    episode_cost = 0.0
    episode_len = 0
    completed_episode_costs = []

    with log_path.open("w", newline="") as log_file:
        writer = csv.DictWriter(
            log_file,
            fieldnames=[
                "global_step",
                "mean_episode_cost",
                "lagrange_multiplier",
                "mean_reward",
                "mean_cost",
                "policy_loss",
                "value_loss",
            ],
        )
        writer.writeheader()

        while global_step < args.total_steps:
            observations = np.zeros((args.rollout_steps, obs_dim), dtype=np.float32)
            actions = np.zeros((args.rollout_steps, act_dim), dtype=np.float32)
            log_probs = np.zeros(args.rollout_steps, dtype=np.float32)
            rewards = np.zeros(args.rollout_steps, dtype=np.float32)
            costs = np.zeros(args.rollout_steps, dtype=np.float32)
            dones = np.zeros(args.rollout_steps, dtype=np.float32)
            reward_values = np.zeros(args.rollout_steps, dtype=np.float32)
            cost_values = np.zeros(args.rollout_steps, dtype=np.float32)

            for step in range(args.rollout_steps):
                global_step += 1
                observations[step] = obs
                obs_tensor = torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0)
                with torch.no_grad():
                    action_tensor, log_prob, reward_value, cost_value = model.act(obs_tensor)
                action = action_tensor.squeeze(0).cpu().numpy()
                next_obs, reward, cost, done, info = env.step(action)

                actions[step] = action
                log_probs[step] = float(log_prob.item())
                reward_values[step] = float(reward_value.item())
                cost_values[step] = float(cost_value.item())
                rewards[step] = float(reward)
                costs[step] = float(cost)
                dones[step] = float(done)

                episode_return += float(reward)
                episode_cost += float(cost)
                episode_len += 1
                obs = next_obs
                if done:
                    completed_episode_costs.append(episode_cost)
                    print(
                        f"episode done: step={global_step}, return={episode_return:.3f}, "
                        f"cost={episode_cost:.3f}, len={episode_len}, "
                        f"success={info.get('success')}, collision={info.get('collision')}"
                    )
                    obs = env.reset()
                    env.set_target_position(sample_target(args, rng))
                    episode_return = 0.0
                    episode_cost = 0.0
                    episode_len = 0
                if global_step >= args.total_steps:
                    break

            rollout_len = step + 1
            observations = observations[:rollout_len]
            actions = actions[:rollout_len]
            log_probs = log_probs[:rollout_len]
            rewards = rewards[:rollout_len]
            costs = costs[:rollout_len]
            dones = dones[:rollout_len]
            reward_values = reward_values[:rollout_len]
            cost_values = cost_values[:rollout_len]

            with torch.no_grad():
                last_obs = torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0)
                last_reward_value = float(model.reward_critic(last_obs).item())
                last_cost_value = float(model.cost_critic(last_obs).item())

            reward_adv, reward_returns = compute_gae(
                rewards, reward_values, dones, last_reward_value, args.gamma, args.gae_lambda)
            cost_adv, cost_returns = compute_gae(
                costs, cost_values, dones, last_cost_value, args.cost_gamma, args.gae_lambda)

            if completed_episode_costs:
                mean_episode_cost = float(np.mean(completed_episode_costs[-20:]))
            else:
                mean_episode_cost = float(np.sum(costs))
            lagrange_multiplier = max(
                0.0,
                lagrange_multiplier + args.lagrange_lr * (mean_episode_cost - args.cost_limit),
            )

            lagrangian_adv = reward_adv - lagrange_multiplier * cost_adv
            lagrangian_adv = (
                (lagrangian_adv - lagrangian_adv.mean())
                / (lagrangian_adv.std() + 1e-8)
            )

            batch_obs = torch.as_tensor(observations, dtype=torch.float32)
            batch_actions = torch.as_tensor(actions, dtype=torch.float32)
            batch_log_probs = torch.as_tensor(log_probs, dtype=torch.float32)
            batch_adv = torch.as_tensor(lagrangian_adv, dtype=torch.float32)
            batch_reward_returns = torch.as_tensor(reward_returns, dtype=torch.float32)
            batch_cost_returns = torch.as_tensor(cost_returns, dtype=torch.float32)

            indices = np.arange(rollout_len)
            last_policy_loss = 0.0
            last_value_loss = 0.0
            for _epoch in range(args.update_epochs):
                np.random.shuffle(indices)
                for start in range(0, rollout_len, args.minibatch_size):
                    mb = indices[start:start + args.minibatch_size]
                    new_log_prob, entropy, new_reward_value, new_cost_value = model.evaluate(
                        batch_obs[mb], batch_actions[mb])
                    ratio = torch.exp(new_log_prob - batch_log_probs[mb])
                    unclipped = -batch_adv[mb] * ratio
                    clipped = -batch_adv[mb] * torch.clamp(
                        ratio, 1.0 - args.clip_coef, 1.0 + args.clip_coef)
                    policy_loss = torch.max(unclipped, clipped).mean()
                    reward_value_loss = ((new_reward_value - batch_reward_returns[mb]) ** 2).mean()
                    cost_value_loss = ((new_cost_value - batch_cost_returns[mb]) ** 2).mean()
                    value_loss = reward_value_loss + cost_value_loss
                    entropy_loss = entropy.mean()
                    loss = policy_loss + args.vf_coef * value_loss - args.ent_coef * entropy_loss

                    optimizer.zero_grad()
                    loss.backward()
                    torch.nn.utils.clip_grad_norm_(model.parameters(), 0.5)
                    optimizer.step()
                    last_policy_loss = float(policy_loss.item())
                    last_value_loss = float(value_loss.item())

            row = {
                "global_step": global_step,
                "mean_episode_cost": mean_episode_cost,
                "lagrange_multiplier": lagrange_multiplier,
                "mean_reward": float(np.mean(rewards)),
                "mean_cost": float(np.mean(costs)),
                "policy_loss": last_policy_loss,
                "value_loss": last_value_loss,
            }
            writer.writerow(row)
            log_file.flush()
            print(
                f"update step={global_step}, mean_reward={row['mean_reward']:.3f}, "
                f"mean_cost={row['mean_cost']:.3f}, episode_cost={mean_episode_cost:.3f}, "
                f"lambda={lagrange_multiplier:.3f}"
            )

    policy_path = output_dir / "policy.pt"
    torch.save(
        {
            "obs_dim": obs_dim,
            "act_dim": act_dim,
            "hidden_sizes": hidden_sizes,
            "actor_state_dict": model.actor.state_dict(),
            "log_std": model.log_std.detach().cpu().numpy(),
            "obs_mean": np.zeros(obs_dim, dtype=np.float32),
            "obs_std": np.ones(obs_dim, dtype=np.float32),
            "config": vars(args),
        },
        policy_path,
    )
    print(f"saved policy: {policy_path}")
    print(f"saved training log: {log_path}")


if __name__ == "__main__":
    main()

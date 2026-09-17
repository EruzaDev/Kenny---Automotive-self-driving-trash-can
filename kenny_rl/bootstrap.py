"""Behavioral cloning from the non-privileged route follower for PPO startup."""
import numpy as np

from .demo import route_action
from .env import KennyEnv


def collect_demonstrations(robot, config, episodes, seed):
    if episodes < 1:
        raise ValueError("behavior cloning episodes must be positive")
    observations, actions, successes, attempts = [], [], 0, 0
    env = KennyEnv(robot, config)
    try:
        while successes < episodes and attempts < episodes*5:
            episode_observations, episode_actions = [], []
            obs, _ = env.reset(seed=seed+attempts)
            attempts += 1
            while True:
                action = route_action(env)
                episode_observations.append(obs)
                episode_actions.append(action)
                obs, _, terminated, truncated, info = env.step(action)
                if terminated or truncated:
                    if info["is_success"]:
                        successes += 1
                        observations.extend(episode_observations)
                        actions.extend(episode_actions)
                    break
    finally:
        env.close()
    if successes != episodes:
        raise RuntimeError(f"Only collected {successes}/{episodes} successful bootstrap episodes")
    return np.asarray(observations, np.float32), np.asarray(actions, np.float32)


def clone_policy(model, observations, actions, epochs, batch_size, learning_rate, log_std, seed):
    import torch
    import torch.nn.functional as F
    if epochs < 1 or batch_size < 1 or learning_rate <= 0:
        raise ValueError("Invalid behavior cloning optimizer settings")
    rng = np.random.default_rng(seed)
    device = model.device
    obs = torch.as_tensor(observations, device=device)
    targets = torch.as_tensor(actions, device=device)
    optimizer = torch.optim.Adam(model.policy.parameters(), lr=learning_rate)
    losses = []
    model.policy.set_training_mode(True)
    for _ in range(epochs):
        indices = rng.permutation(len(observations))
        total = 0.
        for start in range(0, len(indices), batch_size):
            batch = torch.as_tensor(indices[start:start+batch_size], device=device)
            predicted = model.policy.get_distribution(obs[batch]).distribution.mean
            loss = F.mse_loss(predicted, targets[batch])
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.policy.parameters(), .5)
            optimizer.step()
            total += loss.item()*len(batch)
        losses.append(total/len(indices))
    with torch.no_grad():
        model.policy.log_std.fill_(float(log_std))
    model.policy.set_training_mode(False)
    return losses

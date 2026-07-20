"""Gaussian-policy Actor-Critic (NumPy) + PPO / MAPPO update."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from drone_rl_planner.mlp import MLP


@dataclass
class PPOConfig:
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_eps: float = 0.2
    lr: float = 3e-4
    epochs: int = 4
    ent_coef: float = 0.01
    vf_coef: float = 0.5
    max_grad_norm: float = 1.0
    log_std_init: float = -0.5
    hidden: Tuple[int, ...] = (128, 128)


class ActorCritic:
    def __init__(
        self,
        obs_dim: int,
        act_dim: int,
        cfg: Optional[PPOConfig] = None,
        seed: int = 0,
        critic_obs_dim: Optional[int] = None,
    ) -> None:
        self.cfg = cfg or PPOConfig()
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.critic_obs_dim = critic_obs_dim or obs_dim
        rng = np.random.default_rng(seed)
        hid = list(self.cfg.hidden)
        self.actor = MLP([obs_dim, *hid, act_dim], rng=rng)
        self.critic = MLP([self.critic_obs_dim, *hid, 1], rng=np.random.default_rng(seed + 1))
        self.log_std = np.full(act_dim, self.cfg.log_std_init, dtype=np.float64)

    def act(
        self, obs: np.ndarray, deterministic: bool = False
    ) -> Tuple[np.ndarray, float, float]:
        obs = np.asarray(obs, dtype=np.float64).ravel()
        mean, _ = self.actor.forward(obs)
        mean = np.tanh(mean)  # bound mean to [-1,1]
        std = np.exp(self.log_std)
        if deterministic:
            action = mean
        else:
            action = mean + std * np.random.randn(self.act_dim)
        action = np.clip(action, -1.0, 1.0)
        logp = self.log_prob(mean, action)
        if self.critic_obs_dim == self.obs_dim:
            value, _ = self.critic.forward(obs)
            value = float(value.ravel()[0])
        else:
            value = 0.0
        return action, float(logp), value

    def value_of(self, critic_obs: np.ndarray) -> float:
        v, _ = self.critic.forward(np.asarray(critic_obs, dtype=np.float64).ravel())
        return float(v.ravel()[0])

    def mean_of(self, obs: np.ndarray) -> np.ndarray:
        mean, _ = self.actor.forward(np.asarray(obs, dtype=np.float64).ravel())
        return np.tanh(mean)

    def log_prob(self, mean: np.ndarray, action: np.ndarray) -> float:
        std = np.exp(self.log_std)
        # Gaussian on pre-clip action ≈ use clipped action for stability
        var = std ** 2
        logp = -0.5 * (
            np.sum(((action - mean) ** 2) / (var + 1e-8) + 2 * self.log_std + np.log(2 * np.pi))
        )
        return float(logp)

    def evaluate(
        self, obs: np.ndarray, actions: np.ndarray, critic_obs: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Batch evaluate logp, entropy, values. obs (N,d), actions (N,a)."""
        means = []
        for o in obs:
            m, _ = self.actor.forward(o)
            means.append(np.tanh(m))
        mean = np.stack(means, axis=0)
        std = np.exp(self.log_std)[None, :]
        var = std ** 2
        logp = -0.5 * (
            np.sum(((actions - mean) ** 2) / (var + 1e-8) + 2 * self.log_std + np.log(2 * np.pi), axis=1)
        )
        ent = np.sum(self.log_std + 0.5 * np.log(2 * np.pi * np.e), axis=0)
        ent = np.full(obs.shape[0], ent, dtype=np.float64)
        cobs = critic_obs if critic_obs is not None else obs
        values = np.array([self.value_of(c) for c in cobs], dtype=np.float64)
        return logp, ent, values

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            'obs_dim': self.obs_dim,
            'act_dim': self.act_dim,
            'critic_obs_dim': self.critic_obs_dim,
            'log_std': self.log_std,
            'actor_sizes': np.asarray(self.actor.state_dict()['sizes'], dtype=np.int32),
            'critic_sizes': np.asarray(self.critic.state_dict()['sizes'], dtype=np.int32),
        }
        for i, p in enumerate(self.actor.parameters()):
            payload[f'actor_{i}'] = p
        for i, p in enumerate(self.critic.parameters()):
            payload[f'critic_{i}'] = p
        np.savez_compressed(path, **payload)

    @classmethod
    def load(cls, path: str | Path) -> 'ActorCritic':
        data = np.load(path, allow_pickle=False)
        ac = cls(
            int(data['obs_dim']),
            int(data['act_dim']),
            critic_obs_dim=int(data['critic_obs_dim']),
        )
        actor_params = []
        i = 0
        while f'actor_{i}' in data:
            actor_params.append(data[f'actor_{i}'].astype(np.float64))
            i += 1
        critic_params = []
        i = 0
        while f'critic_{i}' in data:
            critic_params.append(data[f'critic_{i}'].astype(np.float64))
            i += 1
        ac.actor = MLP.from_state_dict({
            'sizes': data['actor_sizes'].tolist(),
            'params': actor_params,
        })
        ac.critic = MLP.from_state_dict({
            'sizes': data['critic_sizes'].tolist(),
            'params': critic_params,
        })
        ac.log_std = data['log_std'].astype(np.float64)
        return ac


def compute_gae(
    rewards: Sequence[float],
    values: Sequence[float],
    dones: Sequence[bool],
    gamma: float,
    lam: float,
) -> Tuple[np.ndarray, np.ndarray]:
    n = len(rewards)
    adv = np.zeros(n, dtype=np.float64)
    last = 0.0
    for t in reversed(range(n)):
        next_nonterminal = 1.0 - float(dones[t])
        next_v = values[t + 1] if t + 1 < n else 0.0
        delta = rewards[t] + gamma * next_v * next_nonterminal - values[t]
        last = delta + gamma * lam * next_nonterminal * last
        adv[t] = last
    ret = adv + np.asarray(values[:n], dtype=np.float64)
    return adv, ret


def ppo_update(
    ac: ActorCritic,
    obs: np.ndarray,
    actions: np.ndarray,
    old_logp: np.ndarray,
    advantages: np.ndarray,
    returns: np.ndarray,
    critic_obs: Optional[np.ndarray] = None,
) -> Dict[str, float]:
    """Finite-difference / analytic hybrid: analytic for critic, REINFORCE-style actor."""
    cfg = ac.cfg
    adv = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
    cobs = critic_obs if critic_obs is not None else obs

    # Critic MSE grads
    for _ in range(cfg.epochs):
        # Actor: clipped surrogate via score-function on mean network
        means = []
        acts_cache = []
        for o in obs:
            m, acts = ac.actor.forward(o)
            means.append(np.tanh(m))
            acts_cache.append(acts)
        mean = np.stack(means, axis=0)
        std = np.exp(ac.log_std)
        # ∂logπ/∂mean ≈ (a-mean)/std^2 ; chain through tanh: ∂tanh/∂z = 1-tanh^2
        ratio_logp = []
        for i in range(len(obs)):
            lp = ac.log_prob(mean[i], actions[i])
            ratio_logp.append(lp)
        new_logp = np.asarray(ratio_logp)
        ratio = np.exp(new_logp - old_logp)
        clipped = np.clip(ratio, 1.0 - cfg.clip_eps, 1.0 + cfg.clip_eps)
        # Prefer direction that increases surrogate
        surr1 = ratio * adv
        surr2 = clipped * adv
        # Gradient of min(surr1,surr2) w.r.t logp ≈ chosen branch * adv, then score
        use1 = surr1 <= surr2
        chosen = np.where(use1, ratio, clipped)
        # score ∂logπ/∂z where mean=tanh(z)
        actor_grads = ac.actor.zero_grad_like()
        for i in range(len(obs)):
            # weight for this sample
            w = chosen[i] * adv[i]
            # ∂logπ/∂mean
            dmean = (actions[i] - mean[i]) / (std ** 2 + 1e-8)
            # ∂mean/∂z = 1 - mean^2
            dz = dmean * (1.0 - mean[i] ** 2) * w
            # maximize objective → ascend; apply_grads descends so pass -grads
            g = ac.actor.backward(acts_cache[i], -dz)
            for k in range(len(actor_grads)):
                actor_grads[k] += g[k] / len(obs)
        # entropy bonus on log_std (ascend)
        ac.log_std += cfg.lr * cfg.ent_coef * 0.01
        ac.log_std = np.clip(ac.log_std, -2.0, 0.5)
        ac.actor.apply_grads(actor_grads, cfg.lr, cfg.max_grad_norm)

        # Critic
        critic_grads = ac.critic.zero_grad_like()
        losses = []
        for i in range(len(cobs)):
            v, acts = ac.critic.forward(cobs[i])
            err = float(v.ravel()[0] - returns[i])
            losses.append(err * err)
            g = ac.critic.backward(acts, np.array([2.0 * err * cfg.vf_coef]))
            for k in range(len(critic_grads)):
                critic_grads[k] += g[k] / len(cobs)
        ac.critic.apply_grads(critic_grads, cfg.lr, cfg.max_grad_norm)

    return {
        'policy_loss': float(-np.mean(np.minimum(surr1, surr2))),
        'value_loss': float(np.mean(losses)),
        'entropy': float(np.sum(ac.log_std + 0.5 * np.log(2 * np.pi * np.e))),
    }

"""Soft Actor-Critic with DrQ-lite random-shift augmentation (Path H).

Observation: dict(image=(2,R,S), vector=(V,))
Action: continuous [-1,1]^3
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal


LOG_STD_MIN = -5.0
LOG_STD_MAX = 2.0


def _mlp(sizes, act=nn.ReLU, out_act=None) -> nn.Sequential:
    layers = []
    for i in range(len(sizes) - 1):
        layers.append(nn.Linear(sizes[i], sizes[i + 1]))
        if i < len(sizes) - 2:
            layers.append(act())
        elif out_act is not None:
            layers.append(out_act())
    return nn.Sequential(*layers)


class PolarEncoder(nn.Module):
    def __init__(self, in_ch: int = 2, feat_dim: int = 128) -> None:
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(in_ch, 32, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, 3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 64, 3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((2, 3)),
        )
        self.fc = nn.Linear(64 * 2 * 3, feat_dim)

    def forward(self, img: torch.Tensor) -> torch.Tensor:
        x = self.conv(img)
        x = x.flatten(1)
        return F.relu(self.fc(x))


class Actor(nn.Module):
    def __init__(self, img_ch: int, vec_dim: int, act_dim: int, feat_dim: int = 128) -> None:
        super().__init__()
        self.enc = PolarEncoder(img_ch, feat_dim)
        self.body = _mlp([feat_dim + vec_dim, 256, 256])
        self.mean = nn.Linear(256, act_dim)
        self.log_std = nn.Linear(256, act_dim)

    def forward(self, img: torch.Tensor, vec: torch.Tensor):
        h = torch.cat([self.enc(img), vec], dim=-1)
        h = self.body(h)
        mean = self.mean(h)
        log_std = torch.clamp(self.log_std(h), LOG_STD_MIN, LOG_STD_MAX)
        return mean, log_std

    def sample(self, img: torch.Tensor, vec: torch.Tensor, deterministic: bool = False):
        mean, log_std = self.forward(img, vec)
        if deterministic:
            action = torch.tanh(mean)
            log_prob = None
        else:
            std = log_std.exp()
            dist = Normal(mean, std)
            x = dist.rsample()
            action = torch.tanh(x)
            # tanh correction
            log_prob = dist.log_prob(x) - torch.log(1.0 - action.pow(2) + 1e-6)
            log_prob = log_prob.sum(-1, keepdim=True)
        return action, log_prob, mean


class Critic(nn.Module):
    def __init__(self, img_ch: int, vec_dim: int, act_dim: int, feat_dim: int = 128) -> None:
        super().__init__()
        self.enc = PolarEncoder(img_ch, feat_dim)
        self.q1 = _mlp([feat_dim + vec_dim + act_dim, 256, 256, 1])
        self.q2 = _mlp([feat_dim + vec_dim + act_dim, 256, 256, 1])

    def forward(self, img: torch.Tensor, vec: torch.Tensor, act: torch.Tensor):
        f = self.enc(img)
        x = torch.cat([f, vec, act], dim=-1)
        return self.q1(x), self.q2(x)


@dataclass
class SACConfig:
    gamma: float = 0.99
    tau: float = 0.005
    lr: float = 3e-4
    alpha_lr: float = 3e-4
    # Throughput-first defaults: env steps/hour matter more than peak GPU %.
    # (updates_per_step=12 + batch=256 made training ~8× slower in env-steps.)
    batch_size: int = 128
    buffer_size: int = 200_000
    start_steps: int = 1_000
    updates_per_step: int = 2
    update_every: int = 1
    target_entropy: Optional[float] = None
    drq_pad: int = 4
    device: str = 'cpu'


class ReplayBuffer:
    def __init__(
        self,
        capacity: int,
        img_shape: Tuple[int, int, int],
        vec_dim: int,
        act_dim: int,
    ) -> None:
        self.capacity = capacity
        self.ptr = 0
        self.size = 0
        c, h, w = img_shape
        self.img = np.zeros((capacity, c, h, w), dtype=np.float32)
        self.vec = np.zeros((capacity, vec_dim), dtype=np.float32)
        self.act = np.zeros((capacity, act_dim), dtype=np.float32)
        self.rew = np.zeros((capacity, 1), dtype=np.float32)
        self.next_img = np.zeros((capacity, c, h, w), dtype=np.float32)
        self.next_vec = np.zeros((capacity, vec_dim), dtype=np.float32)
        self.done = np.zeros((capacity, 1), dtype=np.float32)

    def add(self, obs, act, rew, next_obs, done) -> None:
        i = self.ptr
        self.img[i] = obs['image']
        self.vec[i] = obs['vector']
        self.act[i] = act
        self.rew[i] = rew
        self.next_img[i] = next_obs['image']
        self.next_vec[i] = next_obs['vector']
        self.done[i] = float(done)
        self.ptr = (self.ptr + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size: int, rng: np.random.Generator):
        idx = rng.integers(0, self.size, size=batch_size)
        return (
            self.img[idx],
            self.vec[idx],
            self.act[idx],
            self.rew[idx],
            self.next_img[idx],
            self.next_vec[idx],
            self.done[idx],
        )


class SACAgent:
    def __init__(
        self,
        img_shape: Tuple[int, int, int],
        vec_dim: int,
        act_dim: int,
        cfg: Optional[SACConfig] = None,
    ) -> None:
        self.cfg = cfg or SACConfig()
        self.device = torch.device(self.cfg.device)
        self.act_dim = act_dim
        ch = img_shape[0]
        self.actor = Actor(ch, vec_dim, act_dim).to(self.device)
        self.critic = Critic(ch, vec_dim, act_dim).to(self.device)
        self.critic_tgt = copy.deepcopy(self.critic).to(self.device)
        for p in self.critic_tgt.parameters():
            p.requires_grad = False

        self.actor_opt = torch.optim.Adam(self.actor.parameters(), lr=self.cfg.lr)
        self.critic_opt = torch.optim.Adam(self.critic.parameters(), lr=self.cfg.lr)

        self.log_alpha = torch.zeros(1, requires_grad=True, device=self.device)
        self.alpha_opt = torch.optim.Adam([self.log_alpha], lr=self.cfg.alpha_lr)
        self.target_entropy = (
            self.cfg.target_entropy
            if self.cfg.target_entropy is not None
            else -float(act_dim)
        )
        self.rng = np.random.default_rng(0)

    @property
    def alpha(self) -> torch.Tensor:
        return self.log_alpha.exp()

    def act(self, obs: Dict[str, np.ndarray], deterministic: bool = False) -> np.ndarray:
        img = torch.as_tensor(obs['image'][None], device=self.device)
        vec = torch.as_tensor(obs['vector'][None], device=self.device)
        with torch.no_grad():
            action, _, _ = self.actor.sample(img, vec, deterministic=deterministic)
        return action.cpu().numpy()[0].astype(np.float32)

    def _augment_batch(self, img: np.ndarray) -> torch.Tensor:
        """DrQ random-shift on GPU (batch-shared shift) — keeps the GPU busy."""
        t = torch.as_tensor(img, device=self.device, dtype=torch.float32)
        pad = int(self.cfg.drq_pad)
        if pad <= 0:
            return t
        # Pad rings (H) by replication; wrap sectors (W).
        t = F.pad(t, (0, 0, pad, pad), mode='replicate')
        t = torch.cat([t[..., -pad:], t, t[..., :pad]], dim=-1)
        h, w = img.shape[-2], img.shape[-1]
        dy = int(self.rng.integers(0, 2 * pad + 1))
        dx = int(self.rng.integers(0, 2 * pad + 1))
        return t[:, :, dy:dy + h, dx:dx + w].contiguous()

    def update(self, buf: ReplayBuffer) -> Dict[str, float]:
        if buf.size < self.cfg.batch_size:
            return {}
        img, vec, act, rew, nimg, nvec, done = buf.sample(self.cfg.batch_size, self.rng)
        img_t = self._augment_batch(img)
        nimg_t = self._augment_batch(nimg)
        vec_t = torch.as_tensor(vec, device=self.device, dtype=torch.float32)
        act_t = torch.as_tensor(act, device=self.device, dtype=torch.float32)
        rew_t = torch.as_tensor(rew, device=self.device, dtype=torch.float32)
        nvec_t = torch.as_tensor(nvec, device=self.device, dtype=torch.float32)
        done_t = torch.as_tensor(done, device=self.device, dtype=torch.float32)

        with torch.no_grad():
            next_a, next_logp, _ = self.actor.sample(nimg_t, nvec_t)
            q1_t, q2_t = self.critic_tgt(nimg_t, nvec_t, next_a)
            q_t = torch.min(q1_t, q2_t) - self.alpha.detach() * next_logp
            backup = rew_t + (1.0 - done_t) * self.cfg.gamma * q_t

        q1, q2 = self.critic(img_t, vec_t, act_t)
        critic_loss = F.mse_loss(q1, backup) + F.mse_loss(q2, backup)
        self.critic_opt.zero_grad()
        critic_loss.backward()
        self.critic_opt.step()

        a_pi, logp_pi, _ = self.actor.sample(img_t, vec_t)
        q1_pi, q2_pi = self.critic(img_t, vec_t, a_pi)
        q_pi = torch.min(q1_pi, q2_pi)
        actor_loss = (self.alpha.detach() * logp_pi - q_pi).mean()
        self.actor_opt.zero_grad()
        actor_loss.backward()
        self.actor_opt.step()

        alpha_loss = -(self.log_alpha * (logp_pi + self.target_entropy).detach()).mean()
        self.alpha_opt.zero_grad()
        alpha_loss.backward()
        self.alpha_opt.step()

        with torch.no_grad():
            for p, pt in zip(self.critic.parameters(), self.critic_tgt.parameters()):
                pt.data.mul_(1.0 - self.cfg.tau)
                pt.data.add_(self.cfg.tau * p.data)

        return {
            'critic_loss': float(critic_loss.item()),
            'actor_loss': float(actor_loss.item()),
            'alpha': float(self.alpha.item()),
        }

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        torch.save({
            'actor': self.actor.state_dict(),
            'critic': self.critic.state_dict(),
            'critic_tgt': self.critic_tgt.state_dict(),
            'log_alpha': self.log_alpha.detach().cpu(),
            'cfg': self.cfg.__dict__,
        }, path)

    def load(self, path: str | Path, map_location: Optional[str] = None) -> None:
        ckpt = torch.load(path, map_location=map_location or self.device, weights_only=False)
        self.actor.load_state_dict(ckpt['actor'])
        self.critic.load_state_dict(ckpt['critic'])
        self.critic_tgt.load_state_dict(ckpt['critic_tgt'])
        if 'log_alpha' in ckpt:
            self.log_alpha = ckpt['log_alpha'].to(self.device).clone().detach().requires_grad_(True)
            self.alpha_opt = torch.optim.Adam([self.log_alpha], lr=self.cfg.alpha_lr)

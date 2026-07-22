"""Soft Actor-Critic with DrQ (Path H) — aligned with denisyarats/drq + facebookresearch/drqv2.

Key design choices from open-source code (not SACPlanner — paper only):
  - Shared convolutional encoder trained only via critic loss (DrQ / DrQ-v2)
  - Actor uses detached encoder features (DrQ)
  - K random-shift augmentations averaged into Q targets (DrQ)
  - n-step returns (DrQ-v2 default nstep=3)
  - Wider MLP heads (512) + LayerNorm+tanh encoder trunk (DrQ-v2-style)

Observation: dict(image=(2,R,S), vector=(V,))
Action: continuous [-1,1]^3
"""

from __future__ import annotations

import copy
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Dict, Optional, Tuple

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


class SharedEncoder(nn.Module):
    """Polar CNN → compact feature (LayerNorm + tanh), SACPlanner/DrQ style."""

    def __init__(self, in_ch: int = 2, feat_dim: int = 64) -> None:
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(in_ch, 32, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 32, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 32, 3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 32, 3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((2, 3)),
        )
        self.fc = nn.Linear(32 * 2 * 3, feat_dim)
        self.ln = nn.LayerNorm(feat_dim)

    def forward(self, img: torch.Tensor) -> torch.Tensor:
        x = self.conv(img).flatten(1)
        return torch.tanh(self.ln(self.fc(x)))


class ActorHead(nn.Module):
    def __init__(self, feat_dim: int, vec_dim: int, act_dim: int, hidden: int = 512) -> None:
        super().__init__()
        self.body = _mlp([feat_dim + vec_dim, hidden, hidden])
        self.mean = nn.Linear(hidden, act_dim)
        self.log_std = nn.Linear(hidden, act_dim)

    def forward(self, feat: torch.Tensor, vec: torch.Tensor):
        h = self.body(torch.cat([feat, vec], dim=-1))
        mean = self.mean(h)
        log_std = torch.clamp(self.log_std(h), LOG_STD_MIN, LOG_STD_MAX)
        return mean, log_std

    def sample(self, feat: torch.Tensor, vec: torch.Tensor, deterministic: bool = False):
        mean, log_std = self.forward(feat, vec)
        if deterministic:
            return torch.tanh(mean), None, mean
        std = log_std.exp()
        dist = Normal(mean, std)
        x = dist.rsample()
        action = torch.tanh(x)
        log_prob = dist.log_prob(x) - torch.log(1.0 - action.pow(2) + 1e-6)
        log_prob = log_prob.sum(-1, keepdim=True)
        return action, log_prob, mean


class CriticHead(nn.Module):
    def __init__(self, feat_dim: int, vec_dim: int, act_dim: int, hidden: int = 512) -> None:
        super().__init__()
        self.q1 = _mlp([feat_dim + vec_dim + act_dim, hidden, hidden, 1])
        self.q2 = _mlp([feat_dim + vec_dim + act_dim, hidden, hidden, 1])

    def forward(self, feat: torch.Tensor, vec: torch.Tensor, act: torch.Tensor):
        x = torch.cat([feat, vec, act], dim=-1)
        return self.q1(x), self.q2(x)


# Backward-compatible aliases used by older imports / monitor.
class PolarEncoder(SharedEncoder):
    def __init__(self, in_ch: int = 2, feat_dim: int = 128) -> None:
        super().__init__(in_ch=in_ch, feat_dim=feat_dim)


class Actor(nn.Module):
    """Legacy wrapper: private encoder + head (old checkpoints)."""

    def __init__(self, img_ch: int, vec_dim: int, act_dim: int, feat_dim: int = 128) -> None:
        super().__init__()
        self.enc = SharedEncoder(img_ch, feat_dim)
        self.head = ActorHead(feat_dim, vec_dim, act_dim, hidden=256)

    def forward(self, img: torch.Tensor, vec: torch.Tensor):
        return self.head.forward(self.enc(img), vec)

    def sample(self, img: torch.Tensor, vec: torch.Tensor, deterministic: bool = False):
        return self.head.sample(self.enc(img), vec, deterministic=deterministic)


class Critic(nn.Module):
    def __init__(self, img_ch: int, vec_dim: int, act_dim: int, feat_dim: int = 128) -> None:
        super().__init__()
        self.enc = SharedEncoder(img_ch, feat_dim)
        self.head = CriticHead(feat_dim, vec_dim, act_dim, hidden=256)

    def forward(self, img: torch.Tensor, vec: torch.Tensor, act: torch.Tensor):
        return self.head(self.enc(img), vec, act)


@dataclass
class SACConfig:
    gamma: float = 0.99
    tau: float = 0.01
    lr: float = 1e-3
    alpha_lr: float = 1e-3
    batch_size: int = 256
    buffer_size: int = 300_000
    start_steps: int = 1_000
    updates_per_step: int = 4
    update_every: int = 1
    target_entropy: Optional[float] = None
    drq_pad: int = 4
    drq_k: int = 2
    nstep: int = 3  # DrQ-v2 default
    feat_dim: int = 64
    hidden: int = 512
    device: str = 'cpu'


class ReplayBuffer:
    """CPU replay. Stores n-step (reward, bootstrap_discount) tuples.

    ``discount`` is the DrQ-v2 bootstrap multiplier: target = rew + discount * Q(next).
    For 1-step this equals (1 - done) * gamma.
    """

    def __init__(
        self,
        capacity: int,
        img_shape: Tuple[int, int, int],
        vec_dim: int,
        act_dim: int,
        *,
        mmap_dir: Optional[Path] = None,
    ) -> None:
        self.capacity = int(capacity)
        self.ptr = 0
        self.size = 0
        self.mmap_dir = Path(mmap_dir) if mmap_dir else None
        self._meta_path = self.mmap_dir / 'meta.json' if self.mmap_dir else None
        c, h, w = img_shape
        if self.mmap_dir is not None:
            self.mmap_dir.mkdir(parents=True, exist_ok=True)
            self.img = self._mmap('img', (self.capacity, c, h, w))
            self.vec = self._mmap('vec', (self.capacity, vec_dim))
            self.act = self._mmap('act', (self.capacity, act_dim))
            self.rew = self._mmap('rew', (self.capacity, 1))
            self.next_img = self._mmap('next_img', (self.capacity, c, h, w))
            self.next_vec = self._mmap('next_vec', (self.capacity, vec_dim))
            # Prefer discount.dat; fall back to legacy done.dat name for fresh alloc.
            disc_path = self.mmap_dir / 'discount.dat'
            legacy = self.mmap_dir / 'done.dat'
            if not disc_path.is_file() and legacy.is_file():
                legacy.rename(disc_path)
            self.discount = self._mmap('discount', (self.capacity, 1))
            self._load_meta()
        else:
            self.img = np.zeros((self.capacity, c, h, w), dtype=np.float32)
            self.vec = np.zeros((self.capacity, vec_dim), dtype=np.float32)
            self.act = np.zeros((self.capacity, act_dim), dtype=np.float32)
            self.rew = np.zeros((self.capacity, 1), dtype=np.float32)
            self.next_img = np.zeros((self.capacity, c, h, w), dtype=np.float32)
            self.next_vec = np.zeros((self.capacity, vec_dim), dtype=np.float32)
            self.discount = np.zeros((self.capacity, 1), dtype=np.float32)

    def _mmap(self, name: str, shape: Tuple[int, ...]) -> np.ndarray:
        path = self.mmap_dir / f'{name}.dat'
        mode = 'r+' if path.is_file() else 'w+'
        return np.memmap(path, dtype=np.float32, mode=mode, shape=shape)

    def _load_meta(self) -> None:
        if self._meta_path is None or not self._meta_path.is_file():
            return
        try:
            import json
            meta = json.loads(self._meta_path.read_text())
            self.ptr = int(meta.get('ptr', 0)) % self.capacity
            self.size = min(int(meta.get('size', 0)), self.capacity)
        except Exception:
            self.ptr, self.size = 0, 0

    def flush_meta(self) -> None:
        if self._meta_path is None:
            return
        import json
        tmp = self._meta_path.with_suffix('.tmp')
        tmp.write_text(json.dumps({'ptr': self.ptr, 'size': self.size}))
        tmp.replace(self._meta_path)
        for arr in (self.img, self.vec, self.act, self.rew,
                    self.next_img, self.next_vec, self.discount):
            if isinstance(arr, np.memmap):
                arr.flush()

    def add(self, obs, act, rew, next_obs, discount) -> None:
        i = self.ptr
        self.img[i] = obs['image']
        self.vec[i] = obs['vector']
        self.act[i] = act
        self.rew[i] = rew
        self.next_img[i] = next_obs['image']
        self.next_vec[i] = next_obs['vector']
        self.discount[i] = float(discount)
        self.ptr = (self.ptr + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size: int, rng: np.random.Generator):
        idx = rng.integers(0, self.size, size=batch_size)
        return (
            np.asarray(self.img[idx]),
            np.asarray(self.vec[idx]),
            np.asarray(self.act[idx]),
            np.asarray(self.rew[idx]),
            np.asarray(self.next_img[idx]),
            np.asarray(self.next_vec[idx]),
            np.asarray(self.discount[idx]),
        )


class NStepAdder:
    """Online n-step accumulator (DrQ-v2 math) writing into ReplayBuffer.

    One instance per parallel env. On episode end, flushes truncated horizons.
    """

    def __init__(self, nstep: int, gamma: float) -> None:
        self.nstep = max(1, int(nstep))
        self.gamma = float(gamma)
        self._pending: Deque[dict] = deque()

    def reset(self) -> None:
        self._pending.clear()

    def add(
        self,
        buf: ReplayBuffer,
        obs,
        act,
        rew: float,
        next_obs,
        done: bool,
        episode_end: bool,
    ) -> int:
        self._pending.append({
            'obs': obs,
            'act': np.asarray(act, dtype=np.float32).copy(),
            'rew': float(rew),
            'next_obs': next_obs,
            'done': bool(done),
        })
        n_written = 0
        while len(self._pending) >= self.nstep:
            self._emit(buf, self.nstep)
            n_written += 1
        if episode_end:
            while self._pending:
                self._emit(buf, len(self._pending))
                n_written += 1
        return n_written

    def _emit(self, buf: ReplayBuffer, horizon: int) -> None:
        assert horizon >= 1 and len(self._pending) >= horizon
        first = self._pending[0]
        R = 0.0
        disc = 1.0
        next_obs = first['next_obs']
        for i in range(horizon):
            tr = self._pending[i]
            R += disc * tr['rew']
            next_obs = tr['next_obs']
            if tr['done']:
                disc = 0.0
                break
            disc *= self.gamma
        buf.add(first['obs'], first['act'], R, next_obs, disc)
        self._pending.popleft()


class SACAgent:
    """DrQ-SAC with shared encoder (new default)."""

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
        self.vec_dim = vec_dim
        ch = img_shape[0]
        fd = int(self.cfg.feat_dim)
        hid = int(self.cfg.hidden)

        self.encoder = SharedEncoder(ch, fd).to(self.device)
        self.actor = ActorHead(fd, vec_dim, act_dim, hidden=hid).to(self.device)
        self.critic = CriticHead(fd, vec_dim, act_dim, hidden=hid).to(self.device)
        self.encoder_tgt = copy.deepcopy(self.encoder).to(self.device)
        self.critic_tgt = copy.deepcopy(self.critic).to(self.device)
        for p in list(self.encoder_tgt.parameters()) + list(self.critic_tgt.parameters()):
            p.requires_grad = False

        # Encoder + critic optimized together; actor head separate.
        self.critic_opt = torch.optim.Adam(
            list(self.encoder.parameters()) + list(self.critic.parameters()),
            lr=self.cfg.lr,
        )
        self.actor_opt = torch.optim.Adam(self.actor.parameters(), lr=self.cfg.lr)
        self.log_alpha = torch.zeros(1, requires_grad=True, device=self.device)
        self.alpha_opt = torch.optim.Adam([self.log_alpha], lr=self.cfg.alpha_lr)
        self.target_entropy = (
            self.cfg.target_entropy
            if self.cfg.target_entropy is not None
            else -float(act_dim)
        )
        self.rng = np.random.default_rng(0)
        self.arch = 'shared_drq_v2'

    @property
    def alpha(self) -> torch.Tensor:
        return self.log_alpha.exp()

    def _encode(self, img: torch.Tensor, detach: bool = False) -> torch.Tensor:
        feat = self.encoder(img)
        return feat.detach() if detach else feat

    def act(self, obs: Dict[str, np.ndarray], deterministic: bool = False) -> np.ndarray:
        img = torch.as_tensor(obs['image'][None], device=self.device)
        vec = torch.as_tensor(obs['vector'][None], device=self.device)
        with torch.no_grad():
            feat = self.encoder(img)
            action, _, _ = self.actor.sample(feat, vec, deterministic=deterministic)
        return action.cpu().numpy()[0].astype(np.float32)

    def _augment_batch(self, img: np.ndarray) -> torch.Tensor:
        t = torch.as_tensor(img, device=self.device, dtype=torch.float32)
        pad = int(self.cfg.drq_pad)
        if pad <= 0:
            return t
        t = F.pad(t, (0, 0, pad, pad), mode='replicate')
        t = torch.cat([t[..., -pad:], t, t[..., :pad]], dim=-1)
        h, w = img.shape[-2], img.shape[-1]
        dy = int(self.rng.integers(0, 2 * pad + 1))
        dx = int(self.rng.integers(0, 2 * pad + 1))
        return t[:, :, dy:dy + h, dx:dx + w].contiguous()

    def update(self, buf: ReplayBuffer) -> Dict[str, float]:
        if buf.size < self.cfg.batch_size:
            return {}
        img, vec, act, rew, nimg, nvec, discount = buf.sample(
            self.cfg.batch_size, self.rng)
        vec_t = torch.as_tensor(vec, device=self.device, dtype=torch.float32)
        act_t = torch.as_tensor(act, device=self.device, dtype=torch.float32)
        rew_t = torch.as_tensor(rew, device=self.device, dtype=torch.float32)
        nvec_t = torch.as_tensor(nvec, device=self.device, dtype=torch.float32)
        # DrQ-v2: discount already folds gamma^n and mid-horizon terminals.
        disc_t = torch.as_tensor(discount, device=self.device, dtype=torch.float32)

        k = max(1, int(self.cfg.drq_k))
        # DrQ: average Q-targets over K augmentations of next observation.
        with torch.no_grad():
            q_backs = []
            for _ in range(k):
                nimg_k = self._augment_batch(nimg)
                feat_n = self.encoder_tgt(nimg_k)
                next_a, next_logp, _ = self.actor.sample(feat_n, nvec_t)
                q1_t, q2_t = self.critic_tgt(feat_n, nvec_t, next_a)
                q_t = torch.min(q1_t, q2_t) - self.alpha.detach() * next_logp
                q_backs.append(rew_t + disc_t * q_t)
            backup = torch.stack(q_backs, dim=0).mean(dim=0)

        img_t = self._augment_batch(img)
        feat = self.encoder(img_t)
        q1, q2 = self.critic(feat, vec_t, act_t)
        critic_loss = F.mse_loss(q1, backup) + F.mse_loss(q2, backup)
        self.critic_opt.zero_grad()
        critic_loss.backward()
        nn.utils.clip_grad_norm_(
            list(self.encoder.parameters()) + list(self.critic.parameters()), 10.0)
        self.critic_opt.step()

        # Actor uses detached encoder features (DrQ / SACPlanner).
        feat_det = self.encoder(img_t).detach()
        a_pi, logp_pi, _ = self.actor.sample(feat_det, vec_t)
        q1_pi, q2_pi = self.critic(feat_det, vec_t, a_pi)
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
            for p, pt in zip(self.encoder.parameters(), self.encoder_tgt.parameters()):
                pt.data.mul_(1.0 - self.cfg.tau)
                pt.data.add_(self.cfg.tau * p.data)
            for p, pt in zip(self.critic.parameters(), self.critic_tgt.parameters()):
                pt.data.mul_(1.0 - self.cfg.tau)
                pt.data.add_(self.cfg.tau * p.data)

        return {
            'critic_loss': float(critic_loss.item()),
            'actor_loss': float(actor_loss.item()),
            'alpha': float(self.alpha.item()),
        }

    def set_lr(self, lr: float) -> None:
        for opt in (self.actor_opt, self.critic_opt, self.alpha_opt):
            for g in opt.param_groups:
                g['lr'] = float(lr)

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        torch.save({
            'arch': self.arch,
            'encoder': self.encoder.state_dict(),
            'encoder_tgt': self.encoder_tgt.state_dict(),
            'actor': self.actor.state_dict(),
            'critic': self.critic.state_dict(),
            'critic_tgt': self.critic_tgt.state_dict(),
            'log_alpha': self.log_alpha.detach().cpu(),
            'actor_opt': self.actor_opt.state_dict(),
            'critic_opt': self.critic_opt.state_dict(),
            'alpha_opt': self.alpha_opt.state_dict(),
            'cfg': self.cfg.__dict__,
        }, path)

    def load(
        self,
        path: str | Path,
        map_location: Optional[str] = None,
        *,
        load_optim: bool = True,
    ) -> None:
        ckpt = torch.load(path, map_location=map_location or self.device, weights_only=False)
        if ckpt.get('arch') == self.arch and 'encoder' in ckpt:
            self.encoder.load_state_dict(ckpt['encoder'])
            self.encoder_tgt.load_state_dict(ckpt.get('encoder_tgt', ckpt['encoder']))
            self.actor.load_state_dict(ckpt['actor'])
            self.critic.load_state_dict(ckpt['critic'])
            self.critic_tgt.load_state_dict(ckpt['critic_tgt'])
        else:
            raise RuntimeError(
                f'Checkpoint arch mismatch (got {ckpt.get("arch")!r}). '
                'Train a fresh --fast run; old dual-encoder .pt is incompatible.'
            )
        if 'log_alpha' in ckpt:
            self.log_alpha = ckpt['log_alpha'].to(self.device).clone().detach().requires_grad_(True)
            self.alpha_opt = torch.optim.Adam([self.log_alpha], lr=self.cfg.alpha_lr)
        if not load_optim:
            # Fresh Adam moments — critical after domain-shift curriculum resume.
            self.actor_opt = torch.optim.Adam(self.actor.parameters(), lr=self.cfg.lr)
            self.critic_opt = torch.optim.Adam(
                list(self.encoder.parameters()) + list(self.critic.parameters()),
                lr=self.cfg.lr,
            )
            self.alpha_opt = torch.optim.Adam([self.log_alpha], lr=self.cfg.alpha_lr)
            return
        for key, opt in (
            ('actor_opt', self.actor_opt),
            ('critic_opt', self.critic_opt),
            ('alpha_opt', self.alpha_opt),
        ):
            if key in ckpt:
                try:
                    opt.load_state_dict(ckpt[key])
                except Exception:
                    pass

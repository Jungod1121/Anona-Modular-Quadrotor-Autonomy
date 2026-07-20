"""Train single-agent PPO local planner (offline NumPy env)."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from drone_rl_planner.local_nav_env import EnvConfig, LocalNavEnv
from drone_rl_planner.ppo import ActorCritic, PPOConfig, compute_gae, ppo_update


def _bc_warmstart(ac: ActorCritic, env: LocalNavEnv, steps: int = 2000) -> None:
    """Fit actor mean toward a reactive heuristic so PPO starts near a working policy."""
    obs = env.reset()
    for _ in range(steps):
        o = obs
        n_rays = env.cfg.n_rays
        gx, gy = float(o[n_rays]), float(o[n_rays + 1])
        rays = o[:n_rays]
        angles = np.linspace(-np.pi, np.pi, n_rays, endpoint=False)
        i = int(np.argmin(rays))
        if rays[i] < 0.4:
            ang = angles[i] + np.pi
            target = np.array([np.cos(ang), np.sin(ang)]) * 0.9
        else:
            target = np.array([gx, gy])
        mean, acts = ac.actor.forward(o)
        pred = np.tanh(mean)
        err = pred - target
        # ∂tanh/∂z = 1-tanh^2; descend on MSE
        dz = err * (1.0 - pred ** 2)
        grads = ac.actor.backward(acts, dz)
        ac.actor.apply_grads(grads, lr=1e-2, max_norm=2.0)
        action = target
        obs, _, done, _ = env.step(action)
        if done:
            obs = env.reset()


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description='Train PPO local nav policy')
    p.add_argument('--steps', type=int, default=40_000)
    p.add_argument('--rollout', type=int, default=1024)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--out', type=str, default='')
    p.add_argument('--bc_steps', type=int, default=3000)
    args = p.parse_args(argv)

    env = LocalNavEnv(EnvConfig(), seed=args.seed)
    ac = ActorCritic(env.obs_dim, env.act_dim, PPOConfig(), seed=args.seed)
    if args.bc_steps > 0:
        print(f'BC warm-start ({args.bc_steps} steps)…', flush=True)
        _bc_warmstart(ac, env, steps=args.bc_steps)

    obs = env.reset()
    ep_ret = 0.0
    successes = 0
    episodes = 0
    total = 0
    best_succ = -1.0

    out = Path(args.out) if args.out else (
        Path(__file__).resolve().parents[1] / 'checkpoints' / 'ppo_local.npz')
    out.parent.mkdir(parents=True, exist_ok=True)

    while total < args.steps:
        buf_o, buf_a, buf_lp, buf_r, buf_v, buf_d = [], [], [], [], [], []
        for _ in range(args.rollout):
            action, logp, val = ac.act(obs, deterministic=False)
            nobs, reward, done, info = env.step(action)
            buf_o.append(obs)
            buf_a.append(action)
            buf_lp.append(logp)
            buf_r.append(reward)
            buf_v.append(val)
            buf_d.append(done)
            ep_ret += reward
            obs = nobs
            total += 1
            if done:
                episodes += 1
                if info.get('success'):
                    successes += 1
                obs = env.reset()
                ep_ret = 0.0
            if total >= args.steps:
                break

        obs_arr = np.stack(buf_o)
        act_arr = np.stack(buf_a)
        old_lp = np.asarray(buf_lp)
        adv, ret = compute_gae(buf_r, buf_v, buf_d, ac.cfg.gamma, ac.cfg.gae_lambda)
        stats = ppo_update(ac, obs_arr, act_arr, old_lp, adv, ret)
        rate = successes / max(1, episodes)
        print(
            f'steps={total} episodes={episodes} success_rate={rate:.2f} '
            f'pi_loss={stats["policy_loss"]:.3f} v_loss={stats["value_loss"]:.3f}',
            flush=True,
        )
        if rate >= best_succ:
            best_succ = rate
            ac.save(out)
            print(f'  saved {out} (success_rate={rate:.2f})', flush=True)

    ac.save(out)
    print(f'Done. checkpoint={out} success_rate={successes / max(1, episodes):.2f}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

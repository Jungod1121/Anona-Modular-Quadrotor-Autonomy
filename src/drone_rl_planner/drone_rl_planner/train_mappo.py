"""Train MAPPO (shared actor, centralized critic) on multi-agent local nav."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from drone_rl_planner.local_nav_env import EnvConfig, MultiLocalNavEnv
from drone_rl_planner.ppo import ActorCritic, PPOConfig, compute_gae, ppo_update


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description='Train MAPPO local nav policy')
    p.add_argument('--agents', type=int, default=2)
    p.add_argument('--steps', type=int, default=50_000)
    p.add_argument('--rollout', type=int, default=1024)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--out', type=str, default='')
    args = p.parse_args(argv)

    env = MultiLocalNavEnv(n_agents=args.agents, cfg=EnvConfig(), seed=args.seed)
    critic_dim = env.obs_dim * args.agents
    ac = ActorCritic(
        env.obs_dim, env.act_dim, PPOConfig(), seed=args.seed, critic_obs_dim=critic_dim)

    obs_list = env.reset()
    successes = 0
    episodes = 0
    total = 0
    best = -1.0

    out = Path(args.out) if args.out else (
        Path(__file__).resolve().parents[1] / 'checkpoints' / 'mappo_local.npz')
    out.parent.mkdir(parents=True, exist_ok=True)

    # Per-agent buffers within a rollout window (flattened for shared update)
    while total < args.steps:
        buf_o, buf_a, buf_lp, buf_r, buf_v, buf_d, buf_c = [], [], [], [], [], [], []
        for _ in range(args.rollout // args.agents):
            gstate = env.global_state()
            actions = []
            for oi in obs_list:
                a, lp, _ = ac.act(oi, deterministic=False)
                # Centralized value
                v = ac.value_of(gstate)
                actions.append(a)
                buf_o.append(oi)
                buf_a.append(a)
                buf_lp.append(lp)
                buf_v.append(v)
                buf_c.append(gstate)
            nobs, rewards, dones, infos = env.step(actions)
            for r, d, info in zip(rewards, dones, infos):
                buf_r.append(r)
                buf_d.append(d)
                total += 1
                if d:
                    episodes += 1
                    if info.get('success'):
                        successes += 1
            obs_list = nobs
            if any(dones):
                obs_list = env.reset()
            if total >= args.steps:
                break

        if not buf_o:
            break
        obs_arr = np.stack(buf_o)
        act_arr = np.stack(buf_a)
        old_lp = np.asarray(buf_lp)
        c_arr = np.stack(buf_c)
        adv, ret = compute_gae(buf_r, buf_v, buf_d, ac.cfg.gamma, ac.cfg.gae_lambda)
        stats = ppo_update(ac, obs_arr, act_arr, old_lp, adv, ret, critic_obs=c_arr)
        rate = successes / max(1, episodes)
        print(
            f'steps={total} episodes={episodes} success_rate={rate:.2f} '
            f'pi_loss={stats["policy_loss"]:.3f} v_loss={stats["value_loss"]:.3f}',
            flush=True,
        )
        if rate >= best:
            best = rate
            ac.save(out)
            print(f'  saved {out}', flush=True)

    ac.save(out)
    print(f'Done. checkpoint={out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

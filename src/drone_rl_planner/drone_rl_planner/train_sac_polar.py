"""Train Path H: Polar DrQ-SAC local planner."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import torch

from drone_rl_planner.polar_nav_env import PolarEnvConfig, PolarNavEnv
from drone_rl_planner.sac_drq import ReplayBuffer, SACAgent, SACConfig


def _default_ckpt_dir() -> Path:
    return Path(__file__).resolve().parents[1] / 'checkpoints'


def evaluate(agent: SACAgent, env: PolarNavEnv, n_eps: int = 60) -> dict:
    """Honest eval: enough episodes + timeout/collision split (not just success%)."""
    successes = 0
    collisions = 0
    timeouts = 0
    returns = []
    min_clears = []
    for _ in range(n_eps):
        obs = env.reset()
        done = False
        ep_r = 0.0
        info = {}
        ep_clear = []
        while not done:
            act = agent.act(obs, deterministic=True)
            obs, r, done, info = env.step(act)
            ep_r += r
            if 'min_clear' in info:
                ep_clear.append(float(info['min_clear']))
        returns.append(ep_r)
        if info.get('success'):
            successes += 1
        elif info.get('collision'):
            collisions += 1
        else:
            timeouts += 1
        if ep_clear:
            min_clears.append(min(ep_clear))
    n = max(n_eps, 1)
    sr = successes / n
    cr = collisions / n
    # Score used for checkpoint selection: penalize collisions harder than
    # raw success (a lucky 96% on 25 eps was hiding 60% catalog-density crash).
    score = sr - 0.75 * cr - 0.25 * (timeouts / n)
    return {
        'success_rate': sr,
        'collision_rate': cr,
        'timeout_rate': timeouts / n,
        'mean_return': float(np.mean(returns)) if returns else 0.0,
        'mean_min_clear': float(np.mean(min_clears)) if min_clears else 0.0,
        'n_episodes': n_eps,
        'score': float(score),
    }


def write_status(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(payload, indent=2))
    tmp.replace(path)


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description='Path H Polar DrQ-SAC trainer')
    ap.add_argument('--steps', type=int, default=80_000)
    ap.add_argument('--eval-every', type=int, default=5_000)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--device', type=str, default='',
                    help='cuda | cpu | empty=auto (prefer GPU)')
    ap.add_argument('--ckpt-dir', type=str, default='')
    ap.add_argument('--name', type=str, default='sac_polar_local')
    ap.add_argument('--resume', type=str, default='',
                    help='Load existing .pt before continuing')
    ap.add_argument('--forest-heavy', action='store_true',
                    help='Bias domain mix toward official_forest density')
    ap.add_argument('--dense-heavy', action='store_true',
                    help='Bias domain mix toward dense_field pillar clutter')
    ap.add_argument('--eval-episodes', type=int, default=60,
                    help='Episodes per eval (25 was too noisy / optimistic)')
    ap.add_argument('--target', type=float, default=0.0,
                    help='Early-stop when best success_rate >= target (0=disabled)')
    ap.add_argument('--batch-size', type=int, default=0,
                    help='SAC batch (0=auto: 256 cuda / 64 cpu)')
    ap.add_argument('--updates-per-step', type=int, default=0,
                    help='Gradient updates per env step (0=auto: 12 cuda / 1 cpu)')
    ap.add_argument('--update-every', type=int, default=0,
                    help='Env steps between update bursts (0=auto: 1)')
    ap.add_argument('--n-envs', type=int, default=0,
                    help='Parallel CPU envs feeding the buffer (0=auto: 4 cuda / 1 cpu)')
    args = ap.parse_args(argv)

    if args.forest_heavy and args.dense_heavy:
        raise SystemExit('Choose only one of --forest-heavy / --dense-heavy')

    device = args.device or ('cuda' if torch.cuda.is_available() else 'cpu')
    if device.startswith('cuda') and torch.cuda.is_available():
        torch.backends.cudnn.benchmark = True
        try:
            torch.set_float32_matmul_precision('high')
        except Exception:
            pass

    # Throughput defaults: prefer env-steps/hour over saturating a laptop GPU.
    # Old fast run ≈ 200k steps/h with updates_per_step≈0.5–1; 12×updates crushed that.
    use_cuda = str(device).startswith('cuda')
    batch_size = args.batch_size or (128 if use_cuda else 64)
    updates_per_step = args.updates_per_step or (2 if use_cuda else 1)
    update_every = args.update_every or 1
    n_envs = max(1, args.n_envs or (2 if use_cuda else 1))

    ckpt_dir = Path(args.ckpt_dir) if args.ckpt_dir else _default_ckpt_dir()
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    status_path = ckpt_dir / 'sac_training_status.json'
    best_path = ckpt_dir / f'{args.name}_best.pt'
    last_path = ckpt_dir / f'{args.name}.pt'

    envs = [
        PolarNavEnv(
            PolarEnvConfig(), seed=args.seed + i,
            forest_heavy=args.forest_heavy, dense_heavy=args.dense_heavy)
        for i in range(n_envs)
    ]
    env0 = envs[0]
    eval_env = PolarNavEnv(
        PolarEnvConfig(), seed=args.seed + 1000,
        forest_heavy=args.forest_heavy, dense_heavy=args.dense_heavy)
    cfg = SACConfig(
        device=device,
        batch_size=batch_size,
        updates_per_step=updates_per_step,
        update_every=update_every,
        start_steps=min(3000, max(500, args.steps // 20)),
    )
    if args.resume:
        cfg.start_steps = 0  # already trained
    agent = SACAgent(env0.image_shape, env0.vec_dim, env0.act_dim, cfg)
    resume_path = Path(args.resume) if args.resume else None
    if resume_path and resume_path.is_file():
        agent.load(resume_path, map_location=device)
        print(f'[Path H] resumed from {resume_path}', flush=True)
    elif best_path.is_file() and args.resume == 'auto':
        agent.load(best_path, map_location=device)
        cfg.start_steps = 0
        print(f'[Path H] resumed from {best_path}', flush=True)
    buf = ReplayBuffer(cfg.buffer_size, env0.image_shape, env0.vec_dim, env0.act_dim)

    obss = [e.reset() for e in envs]
    best_score = -1e9
    best_sr = -1.0
    # When resuming, keep the historical best so a worse mid-chunk eval cannot
    # overwrite sac_polar_local_best.pt (that used to wipe a 78% ckpt with ~40%).
    if args.resume or (best_path.is_file() and resume_path and resume_path.is_file()):
        if status_path.is_file():
            try:
                prev = json.loads(status_path.read_text())
                prev_sr = float(prev.get('best_success') or 0.0)
                prev_sc = float(prev.get('best_score') or -1e9)
                if prev_sr > 0:
                    best_sr = prev_sr
                if prev_sc > best_score:
                    best_score = prev_sc
                # If only success is known, synthesize a conservative score floor.
                if best_score <= -1e8 and best_sr > 0:
                    best_score = best_sr - 0.75 * (1.0 - best_sr)
                print(
                    f'[Path H] preserve prior best_success={best_sr:.1%} '
                    f'score={best_score:.3f}',
                    flush=True,
                )
            except Exception:
                pass
    t0 = time.time()
    dens = env0._base.cfg.n_obstacles if hasattr(env0, '_base') else '?'
    print(
        f'[Path H] training on {device} for {args.steps} steps'
        f'{" (forest-heavy)" if args.forest_heavy else ""}'
        f'{" (dense-heavy)" if args.dense_heavy else ""}'
        f'  obstacles≈{dens}  eval_eps={args.eval_episodes}'
        f'  n_envs={n_envs} batch={batch_size} updates/step={updates_per_step}',
        flush=True,
    )

    write_status(status_path, {
        'state': 'running',
        'algorithm': 'DrQ-SAC',
        'steps': 0,
        'total_steps': args.steps,
        'best_success': best_sr if best_sr > 0 else 0.0,
        'best_score': best_score if best_score > -1e8 else 0.0,
        'device': device,
        'n_envs': n_envs,
        'batch_size': batch_size,
        'updates_per_step': updates_per_step,
    })

    env_steps = 0
    while env_steps < args.steps:
        # Collect one step from each parallel env (CPU), then burst GPU updates.
        for i, env in enumerate(envs):
            if env_steps >= args.steps:
                break
            if env_steps < cfg.start_steps:
                action = np.random.uniform(-1.0, 1.0, size=env.act_dim).astype(np.float32)
            else:
                action = agent.act(obss[i], deterministic=False)
            next_obs, reward, done, info = env.step(action)
            buf.add(obss[i], action, reward, next_obs, done)
            obss[i] = env.reset() if done else next_obs
            env_steps += 1

            if env_steps >= cfg.start_steps and env_steps % max(1, cfg.update_every) == 0:
                for _ in range(cfg.updates_per_step):
                    agent.update(buf)

            if env_steps % 500 == 0:
                elapsed = time.time() - t0
                write_status(status_path, {
                    'state': 'running',
                    'algorithm': 'DrQ-SAC',
                    'steps': env_steps,
                    'total_steps': args.steps,
                    'best_success': best_sr if best_sr >= 0 else 0.0,
                    'best_score': best_score if best_score > -1e8 else 0.0,
                    'device': device,
                    'elapsed_s': elapsed,
                    'n_envs': n_envs,
                    'batch_size': batch_size,
                    'updates_per_step': updates_per_step,
                    'buf': buf.size,
                })
                print(
                    f'  … step {env_steps}/{args.steps}  t={elapsed:.0f}s  '
                    f'buf={buf.size}  gpu_updates~{updates_per_step}/env',
                    flush=True,
                )

            if env_steps % args.eval_every == 0 or env_steps >= args.steps:
                metrics = evaluate(agent, eval_env, n_eps=args.eval_episodes)
                sr = metrics['success_rate']
                score = metrics['score']
                agent.save(last_path)
                improved = ''
                if score >= best_score:
                    best_score = score
                    best_sr = sr
                    agent.save(best_path)
                    improved = ' *BEST*'
                elapsed = time.time() - t0
                print(
                    f'step={env_steps}/{args.steps}  success={sr:.1%}  '
                    f'collision={metrics["collision_rate"]:.1%}  '
                    f'timeout={metrics["timeout_rate"]:.1%}  '
                    f'clear={metrics["mean_min_clear"]:.2f}  '
                    f'score={score:.3f}  '
                    f'return={metrics["mean_return"]:.1f}  '
                    f't={elapsed:.0f}s{improved}',
                    flush=True,
                )
                done_run = env_steps >= args.steps
                early = args.target > 0 and best_sr + 1e-9 >= float(args.target)
                write_status(status_path, {
                    'state': 'done' if (done_run or early) else 'running',
                    'algorithm': 'DrQ-SAC',
                    'steps': env_steps,
                    'total_steps': args.steps,
                    'success_rate': sr,
                    'collision_rate': metrics['collision_rate'],
                    'timeout_rate': metrics['timeout_rate'],
                    'mean_min_clear': metrics['mean_min_clear'],
                    'mean_return': metrics['mean_return'],
                    'score': score,
                    'best_success': best_sr,
                    'best_score': best_score,
                    'n_episodes': metrics['n_episodes'],
                    'device': device,
                    'checkpoint': str(best_path if best_sr >= 0 else last_path),
                    'target': float(args.target) if args.target > 0 else None,
                    'n_envs': n_envs,
                    'batch_size': batch_size,
                    'updates_per_step': updates_per_step,
                    'early_stop': bool(early),
                })
                if early:
                    print(
                        f'[Path H] early-stop: best_success={best_sr:.1%} '
                        f'>= target={args.target:.0%}',
                        flush=True)
                    env_steps = args.steps
                    break

    # Prefer best weights
    if best_path.is_file():
        agent.load(best_path)
        agent.save(last_path)
    print(f'[Path H] done. best_success={best_sr:.1%}  best_score={best_score:.3f}  '
          f'ckpt={last_path}', flush=True)


if __name__ == '__main__':
    main()

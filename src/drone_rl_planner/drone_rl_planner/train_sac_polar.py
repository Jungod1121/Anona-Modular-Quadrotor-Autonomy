"""Train Path H: Polar DrQ-SAC local planner."""

from __future__ import annotations

import argparse
import json
import signal
import time
from pathlib import Path

import numpy as np
import torch

from drone_rl_planner.polar_nav_env import PolarEnvConfig, PolarNavEnv
from drone_rl_planner.sac_drq import NStepAdder, ReplayBuffer, SACAgent, SACConfig


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
        ep_r = 0.0
        info = {}
        ep_clear = []
        while True:
            act = agent.act(obs, deterministic=True)
            obs, r, done, info = env.step(act)
            ep_r += r
            if 'min_clear' in info:
                ep_clear.append(float(info['min_clear']))
            if done or info.get('episode_end') or info.get('truncated'):
                break
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


def write_status(path: Path, payload: dict, base: dict | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    out = dict(base or {})
    out.update(payload)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(out, indent=2))
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
    ap.add_argument('--easy', action='store_true',
                    help='Sparse short-horizon scenes for fast high success')
    ap.add_argument('--medium', action='store_true',
                    help='Mid difficulty bridge (easy → dense)')
    ap.add_argument('--mix-mid-dense', action='store_true',
                    help='Per-episode mix of medium + dense (see --mix-dense-p)')
    ap.add_argument('--mix-dense-p', type=float, default=-1.0,
                    help='P(dense episode) under --mix-mid-dense (-1=auto)')
    ap.add_argument('--fast', action='store_true',
                    help='Preset: easy + shared DrQ + aggressive LR (quick win)')
    ap.add_argument('--stage2', action='store_true',
                    help='Preset: resume fast best → medium, target 85%%')
    ap.add_argument('--stage3', action='store_true',
                    help='Preset: resume mid/fast best → dense-heavy, target 80%%+')
    ap.add_argument('--stage3b', action='store_true',
                    help='Preset: polish dense best — tighter clearance, target 85–90%%')
    ap.add_argument('--stage4', action='store_true',
                    help='Preset: mid_best → light mid/dense mix + n-step')
    ap.add_argument('--stage5', action='store_true',
                    help='Preset: mixb_best → denser mix (p_dense≈30%%)')
    ap.add_argument('--stage6', action='store_true',
                    help='Preset: mixc_best → denser mix (p_dense≈50%%)')
    ap.add_argument('--eval-episodes', type=int, default=60,
                    help='Episodes per eval (25 was too noisy / optimistic)')
    ap.add_argument('--target', type=float, default=0.0,
                    help='Early-stop when best success_rate >= target (0=disabled)')
    ap.add_argument('--batch-size', type=int, default=0,
                    help='SAC batch (0=auto)')
    ap.add_argument('--updates-per-step', type=int, default=0,
                    help='Gradient updates per env step (0=auto)')
    ap.add_argument('--update-every', type=int, default=0,
                    help='Env steps between update bursts (0=auto: 1)')
    ap.add_argument('--n-envs', type=int, default=0,
                    help='Parallel CPU envs (0=auto)')
    ap.add_argument('--buffer-size', type=int, default=0,
                    help='Replay capacity (0=auto)')
    ap.add_argument('--nstep', type=int, default=0,
                    help='DrQ-v2 n-step returns (0=auto: 3)')
    ap.add_argument('--persist-buffer', action='store_true', default=True,
                    help='Keep replay on disk across restarts (default on)')
    ap.add_argument('--no-persist-buffer', action='store_false', dest='persist_buffer')
    ap.add_argument('--finetune-lr', type=float, default=0.0,
                    help='Override LR when --resume (0=keep profile default)')
    ap.add_argument('--reset-buffer', action='store_true',
                    help='Ignore existing mmap replay and start empty')
    args = ap.parse_args(argv)

    ckpt_root = Path(args.ckpt_dir) if args.ckpt_dir else _default_ckpt_dir()
    fast_best = ckpt_root / 'sac_polar_fast_best.pt'
    mid_best = ckpt_root / 'sac_polar_mid_best.pt'
    dense_best = ckpt_root / 'sac_polar_dense_best.pt'
    mixb_best = ckpt_root / 'sac_polar_mixb_best.pt'
    mixc_best = ckpt_root / 'sac_polar_mixc_best.pt'

    if args.fast:
        args.easy = True
        args.medium = False
        args.dense_heavy = False
        args.forest_heavy = False
        args.mix_mid_dense = False
        if args.name == 'sac_polar_local':
            args.name = 'sac_polar_fast'
        if args.target <= 0:
            args.target = 0.90
        if args.steps == 80_000:
            args.steps = 300_000
        if args.eval_episodes == 60:
            args.eval_episodes = 40
        args.reset_buffer = True
        args.resume = ''

    if args.stage2:
        args.medium = True
        args.easy = False
        args.dense_heavy = False
        args.forest_heavy = False
        args.mix_mid_dense = False
        args.name = 'sac_polar_mid'
        if args.target <= 0:
            args.target = 0.85
        if args.steps == 80_000:
            args.steps = 250_000
        if args.eval_episodes == 60:
            args.eval_episodes = 50
        args.resume = str(fast_best if fast_best.is_file() else '')
        if args.finetune_lr <= 0:
            args.finetune_lr = 3e-4
        args.reset_buffer = True

    if args.stage3:
        args.dense_heavy = True
        args.medium = False
        args.easy = False
        args.forest_heavy = False
        args.mix_mid_dense = False
        args.name = 'sac_polar_dense'
        if args.target <= 0:
            args.target = 0.80
        if args.steps == 80_000:
            args.steps = 400_000
        if args.eval_episodes < 60:
            args.eval_episodes = 60
        src = mid_best if mid_best.is_file() else fast_best
        args.resume = str(src if src.is_file() else '')
        if args.finetune_lr <= 0:
            args.finetune_lr = 1e-4
        args.reset_buffer = True

    if args.stage3b:
        # Re-climb dense from mid with learnable dense_heavy (see polar_nav_env).
        args.dense_heavy = True
        args.medium = False
        args.easy = False
        args.forest_heavy = False
        args.mix_mid_dense = False
        args.name = 'sac_polar_dense'
        if args.target <= 0:
            args.target = 0.80
        if args.steps == 80_000:
            args.steps = 400_000
        if args.eval_episodes < 60:
            args.eval_episodes = 60
        src = mid_best if mid_best.is_file() else (
            dense_best if dense_best.is_file() else fast_best)
        args.resume = str(src if src.is_file() else '')
        if args.finetune_lr <= 0:
            args.finetune_lr = 1e-4
        args.reset_buffer = True
        args.best_save_floor = 0.0

    if args.stage4:
        # Light dense drip on top of strong medium prior (fast~92%, mid~96%).
        # Prior 40% dense + dense-only eval destroyed the policy; keep dense rare
        # and eval on the same mix so Live tracks the learnable distribution.
        args.mix_mid_dense = True
        args.dense_heavy = False
        args.medium = False
        args.easy = False
        args.forest_heavy = False
        args.name = 'sac_polar_mixb'
        if args.mix_dense_p < 0:
            args.mix_dense_p = 0.15
        if args.target <= 0:
            args.target = 0.85
        if args.steps == 80_000:
            args.steps = 300_000
        if args.eval_episodes < 60:
            args.eval_episodes = 50
        src = mid_best if mid_best.is_file() else fast_best
        args.resume = str(src if src.is_file() else '')
        if args.finetune_lr <= 0:
            args.finetune_lr = 3e-4
        args.reset_buffer = True
        args.best_save_floor = 0.0

    if args.stage5:
        # Ramp dense from 15% → 30% after mixb hit 85%.
        args.mix_mid_dense = True
        args.dense_heavy = False
        args.medium = False
        args.easy = False
        args.forest_heavy = False
        args.name = 'sac_polar_mixc'
        if args.mix_dense_p < 0:
            args.mix_dense_p = 0.30
        if args.target <= 0:
            args.target = 0.80
        if args.steps == 80_000:
            args.steps = 350_000
        if args.eval_episodes < 50:
            args.eval_episodes = 50
        src = mixb_best if mixb_best.is_file() else (
            mid_best if mid_best.is_file() else fast_best)
        args.resume = str(src if src.is_file() else '')
        if args.finetune_lr <= 0:
            args.finetune_lr = 2e-4
        args.reset_buffer = True
        args.best_save_floor = 0.0

    if args.stage6:
        # Ramp dense from 30% → 50% after mixc is stable.
        args.mix_mid_dense = True
        args.dense_heavy = False
        args.medium = False
        args.easy = False
        args.forest_heavy = False
        args.name = 'sac_polar_mixd'
        if args.mix_dense_p < 0:
            args.mix_dense_p = 0.50
        if args.target <= 0:
            args.target = 0.75
        if args.steps == 80_000:
            args.steps = 400_000
        if args.eval_episodes < 50:
            args.eval_episodes = 60
        src = mixc_best if mixc_best.is_file() else (
            mixb_best if mixb_best.is_file() else mid_best)
        args.resume = str(src if src.is_file() else '')
        if args.finetune_lr <= 0:
            args.finetune_lr = 1.5e-4
        args.reset_buffer = True
        args.best_save_floor = 0.0
    else:
        args.best_save_floor = getattr(args, 'best_save_floor', 0.0)

    if not hasattr(args, 'best_save_floor'):
        args.best_save_floor = 0.0

    if args.mix_dense_p < 0:
        args.mix_dense_p = 0.15 if args.mix_mid_dense else 0.40

    mode_flags = (
        args.forest_heavy, args.dense_heavy, args.easy, args.medium,
        args.mix_mid_dense)
    if sum(bool(x) for x in mode_flags) > 1:
        raise SystemExit(
            'Choose only one of --forest-heavy / --dense-heavy / --easy / '
            '--medium / --mix-mid-dense')

    device = args.device or ('cuda' if torch.cuda.is_available() else 'cpu')
    if device.startswith('cuda') and torch.cuda.is_available():
        torch.backends.cudnn.benchmark = True
        try:
            torch.set_float32_matmul_precision('high')
        except Exception:
            pass

    use_cuda = str(device).startswith('cuda')
    nstep = args.nstep if args.nstep > 0 else 3
    if args.fast or args.easy:
        batch_size = args.batch_size or (256 if use_cuda else 128)
        updates_per_step = args.updates_per_step or (4 if use_cuda else 2)
        n_envs = max(1, args.n_envs or (4 if use_cuda else 2))
        lr = 1e-3
        buffer_size = args.buffer_size or 300_000
    elif args.medium or args.stage2 or args.mix_mid_dense or args.stage4:
        batch_size = args.batch_size or (256 if use_cuda else 128)
        updates_per_step = args.updates_per_step or (3 if use_cuda else 2)
        n_envs = max(1, args.n_envs or (4 if use_cuda else 2))
        lr = 3e-4
        buffer_size = args.buffer_size or 350_000
    elif args.stage3b:
        # Same compute profile as the stage3 run that reached 75%.
        batch_size = args.batch_size or (128 if use_cuda else 64)
        updates_per_step = args.updates_per_step or (2 if use_cuda else 1)
        n_envs = max(1, args.n_envs or (4 if use_cuda else 1))
        lr = 1e-4
        buffer_size = args.buffer_size or 250_000
    else:
        batch_size = args.batch_size or (128 if use_cuda else 64)
        updates_per_step = args.updates_per_step or (2 if use_cuda else 1)
        n_envs = max(1, args.n_envs or (4 if use_cuda else 1))
        lr = 3e-4
        buffer_size = args.buffer_size or 250_000
    update_every = args.update_every or 1
    if args.resume and args.finetune_lr > 0:
        lr = float(args.finetune_lr)

    ckpt_dir = Path(args.ckpt_dir) if args.ckpt_dir else _default_ckpt_dir()
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    status_path = ckpt_dir / 'sac_training_status.json'
    best_path = ckpt_dir / f'{args.name}_best.pt'
    last_path = ckpt_dir / f'{args.name}.pt'
    replay_dir = ckpt_dir / f'{args.name}_replay'

    stage_needs_resume = (
        args.stage2 or args.stage3 or args.stage3b
        or args.stage4 or args.stage5 or args.stage6
    )
    if stage_needs_resume and not args.resume:
        raise SystemExit(
            f'[Path H] curriculum stage needs a prior checkpoint '
            f'(looked for mid/fast best under {ckpt_dir}).')

    mix_p = float(args.mix_dense_p)
    envs = [
        PolarNavEnv(
            PolarEnvConfig(), seed=args.seed + i,
            forest_heavy=args.forest_heavy, dense_heavy=args.dense_heavy,
            easy=args.easy, medium=args.medium,
            mix_mid_dense=args.mix_mid_dense, mix_dense_p=mix_p)
        for i in range(n_envs)
    ]
    env0 = envs[0]
    # Eval matches train mix (not pure dense) so Live/Best track the recipe.
    eval_env = PolarNavEnv(
        PolarEnvConfig(), seed=args.seed + 1000,
        forest_heavy=args.forest_heavy,
        dense_heavy=args.dense_heavy,
        easy=args.easy,
        medium=args.medium,
        mix_mid_dense=args.mix_mid_dense,
        mix_dense_p=mix_p)

    # Keep default target entropy for dense climb (stage3 recipe that worked).
    te = None
    cfg = SACConfig(
        device=device,
        batch_size=batch_size,
        updates_per_step=updates_per_step,
        update_every=update_every,
        start_steps=min(2000, max(500, args.steps // 30)),
        buffer_size=buffer_size,
        lr=lr,
        alpha_lr=lr,
        tau=0.01 if (
            args.fast or args.easy or args.medium or args.mix_mid_dense
        ) else 0.005,
        drq_k=2,
        nstep=nstep,
        feat_dim=64,
        hidden=512,
        target_entropy=te,
    )
    if args.resume:
        # Domain-shift curriculum: collect before updating (empty buffer),
        # otherwise first 128 collision transitions destroy the policy.
        if args.reset_buffer or stage_needs_resume:
            cfg.start_steps = max(12_000, int(cfg.batch_size) * 40)
        else:
            cfg.start_steps = 0

    agent = SACAgent(env0.image_shape, env0.vec_dim, env0.act_dim, cfg)
    resume_path = Path(args.resume) if args.resume else None
    curriculum_resume = bool(stage_needs_resume or args.reset_buffer)
    if resume_path and resume_path.is_file():
        agent.load(resume_path, map_location=device, load_optim=not curriculum_resume)
        if args.finetune_lr > 0:
            agent.set_lr(args.finetune_lr)
        print(
            f'[Path H] resumed from {resume_path}  '
            f'load_optim={not curriculum_resume}  '
            f'start_steps={cfg.start_steps}',
            flush=True,
        )
    elif best_path.is_file() and args.resume == 'auto':
        agent.load(best_path, map_location=device, load_optim=not curriculum_resume)
        if curriculum_resume:
            cfg.start_steps = max(cfg.start_steps, 12_000)
        else:
            cfg.start_steps = 0
        if args.finetune_lr > 0:
            agent.set_lr(args.finetune_lr)
        print(f'[Path H] resumed from {best_path}', flush=True)

    if args.reset_buffer and replay_dir.is_dir():
        import shutil
        shutil.rmtree(replay_dir)
        print(f'[Path H] cleared replay dir {replay_dir}', flush=True)

    buf = ReplayBuffer(
        cfg.buffer_size, env0.image_shape, env0.vec_dim, env0.act_dim,
        mmap_dir=replay_dir if args.persist_buffer else None,
    )
    if buf.size > 0:
        print(f'[Path H] restored replay buffer size={buf.size}/{buf.capacity}', flush=True)
    nstep_adders = [NStepAdder(cfg.nstep, cfg.gamma) for _ in envs]

    stop = {'flag': False}

    def _on_signal(signum, _frame):
        stop['flag'] = True
        print(f'[Path H] signal {signum} — finishing after flush…', flush=True)

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    obss = [e.reset() for e in envs]
    best_score = -1e9
    best_sr = -1.0
    # Curriculum stages change difficulty — never inherit easy Best into medium/dense
    # or early-stop fires immediately (easy 92% >= medium target 85%).
    keep_prior_best = (
        (args.resume or (resume_path and resume_path.is_file()))
        and not (stage_needs_resume or args.reset_buffer)
    )
    if keep_prior_best and status_path.is_file():
        try:
            prev = json.loads(status_path.read_text())
            prev_sr = float(prev.get('best_success') or 0.0)
            prev_sc = float(prev.get('best_score') or -1e9)
            if prev_sr > 0:
                best_sr = prev_sr
            if prev_sc > best_score:
                best_score = prev_sc
            if best_score <= -1e8 and best_sr > 0:
                best_score = best_sr - 0.75 * (1.0 - best_sr)
            print(
                f'[Path H] preserve prior best_success={best_sr:.1%} '
                f'score={best_score:.3f}',
                flush=True,
            )
        except Exception:
            pass
    elif stage_needs_resume:
        print(
            '[Path H] curriculum stage: reset Best metrics for new difficulty '
            '(weights still loaded from resume)',
            flush=True,
        )

    t0 = time.time()
    dens = env0._base.cfg.n_obstacles if hasattr(env0, '_base') else '?'
    print(
        f'[Path H] training on {device} for {args.steps} steps'
        f'{" (forest-heavy)" if args.forest_heavy else ""}'
        f'{" (dense-heavy)" if args.dense_heavy else ""}'
        f'{" (easy/fast)" if args.easy else ""}'
        f'{" (medium)" if args.medium else ""}'
        f'{" (mix mid/dense)" if args.mix_mid_dense else ""}'
        f'{f" p_dense={mix_p:.0%}" if args.mix_mid_dense else ""}'
        f'  obstacles≈{dens}  eval_eps={args.eval_episodes}'
        f'  n_envs={n_envs} batch={batch_size} updates/step={updates_per_step}'
        f'  nstep={cfg.nstep}  lr={lr:g}  arch={agent.arch}'
        f'  persist_buffer={args.persist_buffer}',
        flush=True,
    )

    if args.easy:
        difficulty = 'easy'
    elif args.mix_mid_dense:
        difficulty = f'mix_mid_dense_p{mix_p:.2f}'
    elif args.medium:
        difficulty = 'medium'
    elif args.dense_heavy:
        difficulty = 'dense'
    elif args.forest_heavy:
        difficulty = 'forest'
    else:
        difficulty = 'default'
    status_base = {
        'algorithm': 'DrQ-SAC',
        'run_name': args.name,
        'difficulty': difficulty,
        'mix_dense_p': mix_p if args.mix_mid_dense else None,
        'device': device,
        'n_envs': n_envs,
        'batch_size': batch_size,
        'updates_per_step': updates_per_step,
        'nstep': int(cfg.nstep),
        'target': float(args.target) if args.target > 0 else None,
        'persist_buffer': bool(args.persist_buffer),
        'lr': lr,
        'last_checkpoint': str(last_path),
        'checkpoint': str(best_path if best_path.is_file() else last_path),
        'arch': getattr(agent, 'arch', 'shared_drq_v2'),
        'log_file': 'train_sac_curriculum.log' if (
            stage_needs_resume or args.fast or args.mix_mid_dense
        ) else 'train_sac_overnight.log',
        'stage': (
            '6' if args.stage6 else
            '5' if args.stage5 else
            '4' if args.stage4 else
            '3b' if args.stage3b else
            '3' if args.stage3 else
            '2' if args.stage2 else
            'fast' if args.fast else
            'custom'
        ),
    }

    write_status(status_path, {
        'state': 'running',
        'steps': 0,
        'total_steps': args.steps,
        'best_success': best_sr if best_sr > 0 else 0.0,
        'best_score': best_score if best_score > -1e8 else 0.0,
        'buf': buf.size,
    }, base=status_base)

    env_steps = 0
    while env_steps < args.steps and not stop['flag']:
        for i, env in enumerate(envs):
            if env_steps >= args.steps or stop['flag']:
                break
            if env_steps < cfg.start_steps:
                # Mix: mostly prior policy, some noise — fill buffer before SGD.
                if env_steps % 5 == 0:
                    action = np.random.uniform(-1.0, 1.0, size=env.act_dim).astype(np.float32)
                else:
                    action = agent.act(obss[i], deterministic=False)
            else:
                action = agent.act(obss[i], deterministic=False)
            next_obs, reward, done, info = env.step(action)
            # Only true terminals (collision/success) cut bootstrap; timeouts don't.
            terminal = bool(info.get('collision') or info.get('success'))
            episode_end = bool(info.get('episode_end') or done or info.get('truncated'))
            nstep_adders[i].add(
                buf, obss[i], action, reward, next_obs, terminal, episode_end)
            if episode_end:
                nstep_adders[i].reset()  # already flushed; keep empty
                obss[i] = env.reset()
            else:
                obss[i] = next_obs
            env_steps += 1

            min_buf = max(int(cfg.batch_size) * 20, 5_000)
            if (env_steps >= cfg.start_steps
                    and buf.size >= min_buf
                    and env_steps % max(1, cfg.update_every) == 0):
                for _ in range(cfg.updates_per_step):
                    agent.update(buf)

            if env_steps % 500 == 0:
                elapsed = time.time() - t0
                write_status(status_path, {
                    'state': 'running',
                    'steps': env_steps,
                    'total_steps': args.steps,
                    'best_success': best_sr if best_sr >= 0 else 0.0,
                    'best_score': best_score if best_score > -1e8 else 0.0,
                    'elapsed_s': elapsed,
                    'buf': buf.size,
                    'last_checkpoint': str(last_path),
                }, base=status_base)
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
                buf.flush_meta()
                improved = ''
                if score >= best_score:
                    best_score = score
                    if best_sr < 0 or sr >= best_sr:
                        best_sr = sr
                    floor = float(getattr(args, 'best_save_floor', 0.0) or 0.0)
                    # Protect on-disk Best from curriculum-reset first evals that
                    # are still far below a usable policy (stage3b clobber bug).
                    if sr + 1e-9 >= floor:
                        agent.save(best_path)
                        improved = ' *BEST*'
                    else:
                        improved = f' *BEST(metrics only, floor={floor:.0%})*'
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
                    'state': 'done' if (done_run or early or stop['flag']) else 'running',
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
                    'checkpoint': str(best_path if best_sr >= 0 else last_path),
                    'last_checkpoint': str(last_path),
                    'early_stop': bool(early),
                    'buf': buf.size,
                    'elapsed_s': elapsed,
                }, base=status_base)
                if early:
                    print(
                        f'[Path H] early-stop: best_success={best_sr:.1%} '
                        f'>= target={args.target:.0%}',
                        flush=True)
                    env_steps = args.steps
                    break

    # Always flush foundation before exit.
    agent.save(last_path)
    buf.flush_meta()
    if best_path.is_file():
        # Keep last as best when early-stopped / interrupted at peak.
        pass
    print(f'[Path H] done. best_success={best_sr:.1%}  best_score={best_score:.3f}  '
          f'ckpt={last_path}  buf={buf.size}', flush=True)
    write_status(status_path, {
        'state': 'done',
        'steps': env_steps,
        'total_steps': args.steps,
        'best_success': best_sr if best_sr >= 0 else 0.0,
        'best_score': best_score if best_score > -1e8 else 0.0,
        'checkpoint': str(best_path),
        'last_checkpoint': str(last_path),
        'buf': buf.size,
        'early_stop': bool(args.target > 0 and best_sr + 1e-9 >= float(args.target)),
    }, base=status_base)


if __name__ == '__main__':
    main()

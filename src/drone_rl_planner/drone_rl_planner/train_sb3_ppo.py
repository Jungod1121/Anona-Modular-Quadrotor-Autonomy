"""Train Path G local planner with Stable-Baselines3 PPO (PyTorch).

Default: catalog-scale domain mix (dense / forest lane / gate / maze)
with velocity actions matching the ROS plant — target ≥95% success.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
import torch

from drone_rl_planner.gym_env import LocalNavGymEnv, make_env
from drone_rl_planner.local_nav_env import EnvConfig
from drone_rl_planner.sb3_callbacks import SuccessRateCallback
from drone_rl_planner.sensing import OBS_DEFAULTS


def _pick_device(requested: str) -> str:
    req = (requested or 'auto').strip().lower()
    if req == 'auto':
        if torch.cuda.is_available():
            name = torch.cuda.get_device_name(0)
            print(f'Using GPU: {name}', flush=True)
            return 'cuda'
        print('CUDA not available — training on CPU '
              '(run scripts/setup_rl_gpu.sh + reboot to enable GPU)', flush=True)
        return 'cpu'
    if req.startswith('cuda') and not torch.cuda.is_available():
        print(f'Requested {req} but CUDA unavailable — falling back to CPU', flush=True)
        return 'cpu'
    return req


def train_cfg(easy: bool = False) -> EnvConfig:
    """Env for map transfer. easy=True is curriculum bootstrap only."""
    if easy:
        return EnvConfig(
            n_rays=OBS_DEFAULTS['n_rays'],
            ray_max=OBS_DEFAULTS['ray_max'],
            dt=0.1,
            max_speed=OBS_DEFAULTS['max_speed'],
            max_steps=350,
            world_size=16.0,
            world_scale=OBS_DEFAULTS['world_scale'],
            n_obstacles=12,
            obstacle_r=(0.20, 0.40),
            robot_r=OBS_DEFAULTS['robot_r'],
            goal_tol=0.75,
            collide_penalty=-6.0,
            step_penalty=-0.003,
            progress_scale=4.0,
            goal_bonus=25.0,
            p_dense=0.35,
            p_forest=0.20,
            p_corridor=0.15,
            p_gate=0.15,
            p_maze=0.10,
            cruise_z_jitter=0.15,
        )
    # Hard: catalog-like density (60–80 obstacles on dense/forest)
    return EnvConfig(
        n_rays=OBS_DEFAULTS['n_rays'],
        ray_max=OBS_DEFAULTS['ray_max'],
        dt=0.1,
        max_speed=OBS_DEFAULTS['max_speed'],
        max_steps=600,
        world_size=30.0,
        world_scale=OBS_DEFAULTS['world_scale'],
        n_obstacles=55,
        obstacle_r=(0.18, 0.48),
        robot_r=OBS_DEFAULTS['robot_r'],
        goal_tol=0.70,
        collide_penalty=-8.0,
        step_penalty=-0.004,
        progress_scale=3.5,
        goal_bonus=25.0,
        p_dense=0.28,
        p_forest=0.22,
        p_corridor=0.12,
        p_gate=0.20,
        p_maze=0.15,
        cruise_z_jitter=0.30,
        use_voxel=True,
    )


def _resolve_resume(path: str) -> Path | None:
    p = Path(path)
    if p.is_file():
        return p
    z = p.with_suffix('.zip')
    if z.is_file():
        return z
    return None


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description='SB3 PPO local nav (catalog-scale domain mix)')
    p.add_argument('--steps', type=int, default=2_500_000)
    p.add_argument('--n-envs', type=int, default=8)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--target', type=float, default=0.95)
    p.add_argument('--eval-freq', type=int, default=25_000)
    p.add_argument('--eval-episodes', type=int, default=150)
    p.add_argument('--out', type=str, default='')
    p.add_argument('--tensorboard', type=str, default='')
    p.add_argument('--resume', type=str, default='')
    p.add_argument('--easy', action='store_true', help='Easier curriculum (bootstrap)')
    p.add_argument('--fresh', action='store_true', help='Ignore old checkpoint')
    p.add_argument('--device', type=str, default='auto',
                   help='auto | cpu | cuda | cuda:0')
    args = p.parse_args(argv)

    device = _pick_device(args.device)
    cfg = train_cfg(easy=args.easy)
    pkg = Path(__file__).resolve().parents[1]
    out = Path(args.out) if args.out else pkg / 'checkpoints' / 'sb3_ppo_local'
    out.parent.mkdir(parents=True, exist_ok=True)
    status_path = out.parent / 'training_status.json'
    meta_path = out.parent / 'obs_meta.json'
    tb = args.tensorboard or str(pkg / 'runs' / 'sb3_ppo')

    meta = {
        **OBS_DEFAULTS,
        'world_scale': cfg.world_scale,
        'max_speed': cfg.max_speed,
        'goal_tol': cfg.goal_tol,
        'action': 'desired_velocity',
        'horizon_s': 0.45,
        'easy': args.easy,
        'n_obstacles': cfg.n_obstacles,
        'world_size': cfg.world_size,
        'domain': 'dense+forest+gate+corridor+maze',
    }
    meta_path.write_text(json.dumps(meta, indent=2))

    status_path.write_text(json.dumps({
        'running': True,
        'timesteps': 0,
        'success_rate': 0.0,
        'best_success_rate': 0.0,
        'target': args.target,
        'started_at': time.time(),
        'phase': 'starting',
        'easy': args.easy,
        'domain': meta['domain'],
    }, indent=2))

    n_envs = max(1, args.n_envs)
    if n_envs > 1:
        vec = make_vec_env(
            make_env(cfg),
            n_envs=n_envs,
            vec_env_cls=SubprocVecEnv,
            seed=args.seed,
        )
    else:
        vec = DummyVecEnv([make_env(cfg, 0)])

    eval_env = LocalNavGymEnv(cfg=cfg, seed=args.seed + 999)

    resume = None
    if not args.fresh:
        if args.resume:
            resume = _resolve_resume(args.resume)
        else:
            resume = _resolve_resume(str(out))

    initial_best = 0.0
    model = None
    if resume is not None:
        print(f'Resuming from {resume}', flush=True)
        try:
            model = PPO.load(str(resume), env=vec, device=device)
            model.tensorboard_log = tb
            probe = SuccessRateCallback(
                eval_env, n_episodes=min(40, args.eval_episodes), eval_freq=1, verbose=0,
            )
            probe.model = model
            initial_best = probe._evaluate()
            print(f'Resume baseline eval success={initial_best:.1%}', flush=True)
        except Exception as exc:
            print(f'Resume failed ({exc}) — training from scratch', flush=True)
            resume = None
            model = None

    if model is None:
        model = PPO(
            'MlpPolicy',
            vec,
            learning_rate=2.5e-4,
            n_steps=2048,
            batch_size=256,
            n_epochs=12,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.18,
            ent_coef=0.01,
            vf_coef=0.5,
            max_grad_norm=0.5,
            policy_kwargs=dict(net_arch=dict(pi=[256, 256], vf=[256, 256])),
            verbose=1,
            seed=args.seed,
            tensorboard_log=tb,
            device=device,
        )

    cb = SuccessRateCallback(
        eval_env,
        n_episodes=args.eval_episodes,
        eval_freq=args.eval_freq,
        success_threshold=args.target,
        save_path=str(out),
        status_path=str(status_path),
        initial_best=initial_best,
        verbose=1,
    )

    print(
        f'Training PPO → {out} (target ≥ {args.target:.0%}, '
        f'easy={args.easy}, world={cfg.world_size}m, n_obs≈{cfg.n_obstacles})',
        flush=True,
    )
    model.learn(
        total_timesteps=args.steps,
        callback=cb,
        progress_bar=True,
        reset_num_timesteps=resume is None,
    )
    # Keep best checkpoint — never overwrite with a weaker final policy
    best_zip = Path(str(out) + '.zip')
    if best_zip.is_file():
        try:
            model = PPO.load(str(out), env=vec, device=device)
            cb.model = model
        except Exception:
            pass
    elif cb.best_rate < 0:
        model.save(str(out))

    rate = cb._evaluate()
    if rate > cb.best_rate:
        cb.best_rate = rate
        model.save(str(out))
        print(f'  final_eval improved best → {rate:.1%}', flush=True)
    cb._write_status(False, {'phase': 'finished', 'final_eval': rate})
    print(
        f'Done. final_eval={rate:.1%} best={cb.best_rate:.1%} checkpoint={out}',
        flush=True,
    )
    vec.close()
    eval_env.close()
    return 0 if cb.best_rate >= args.target else 1


if __name__ == '__main__':
    raise SystemExit(main())

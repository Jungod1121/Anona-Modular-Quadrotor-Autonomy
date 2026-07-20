"""SB3 training callbacks — success-rate eval for local nav."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Optional

from stable_baselines3.common.callbacks import BaseCallback


class SuccessRateCallback(BaseCallback):
    """Roll out N episodes on eval env; log success rate to TensorBoard."""

    def __init__(
        self,
        eval_env,
        n_episodes: int = 50,
        eval_freq: int = 10_000,
        success_threshold: float = 0.80,
        save_path: Optional[str] = None,
        status_path: Optional[str] = None,
        initial_best: Optional[float] = None,
        verbose: int = 1,
    ) -> None:
        super().__init__(verbose)
        self.eval_env = eval_env
        self.n_episodes = n_episodes
        self.eval_freq = eval_freq
        self.success_threshold = success_threshold
        self.save_path = save_path
        self.status_path = Path(status_path) if status_path else None
        self.best_rate = initial_best if initial_best is not None else -1.0
        self.last_rate = 0.0
        # Only stop when an eval *in this run* hits the target (ignore resume flukes)
        self._hits_this_run = 0
        self._hits_needed = 1

    def _write_status(self, running: bool, extra: Optional[dict] = None) -> None:
        if self.status_path is None:
            return
        payload = {
            'running': running,
            'timesteps': int(self.num_timesteps),
            'success_rate': float(self.last_rate),
            'best_success_rate': float(self.best_rate),
            'target': float(self.success_threshold),
            'updated_at': time.time(),
        }
        if extra:
            payload.update(extra)
        self.status_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.status_path.with_suffix('.tmp')
        tmp.write_text(json.dumps(payload, indent=2))
        tmp.replace(self.status_path)

    def _evaluate(self) -> float:
        successes = 0
        for _ in range(self.n_episodes):
            obs, _ = self.eval_env.reset()
            done = False
            info: dict = {}
            while not done:
                action, _ = self.model.predict(obs, deterministic=True)
                obs, _, terminated, truncated, info = self.eval_env.step(action)
                done = terminated or truncated
            if info.get('success'):
                successes += 1
        return successes / self.n_episodes

    def _on_training_start(self) -> None:
        self._write_status(True, {'phase': 'training'})

    def _on_training_end(self) -> None:
        self._write_status(False, {'phase': 'finished'})

    def _on_step(self) -> bool:
        if self.num_timesteps == 0 or self.num_timesteps % self.eval_freq != 0:
            return True
        rate = self._evaluate()
        self.last_rate = rate
        self.logger.record('eval/success_rate', rate)
        self.logger.record('eval/best_success_rate', max(self.best_rate, rate))
        if rate > self.best_rate and self.save_path:
            self.best_rate = rate
            self.model.save(self.save_path)
            if self.verbose:
                print(
                    f'[eval] step={self.num_timesteps} success_rate={rate:.1%} '
                    f'best={self.best_rate:.1%}',
                    flush=True,
                )
                print(f'  saved best → {self.save_path}', flush=True)
        elif self.verbose:
            print(
                f'[eval] step={self.num_timesteps} success_rate={rate:.1%} '
                f'best={self.best_rate:.1%}',
                flush=True,
            )
        self._write_status(True)
        if rate >= self.success_threshold:
            self._hits_this_run += 1
            if self._hits_this_run >= self._hits_needed:
                if self.verbose:
                    print(
                        f'  target {self.success_threshold:.0%} reached '
                        f'(best={self.best_rate:.1%}) — stopping.',
                        flush=True,
                    )
                return False
        else:
            self._hits_this_run = 0
        return True

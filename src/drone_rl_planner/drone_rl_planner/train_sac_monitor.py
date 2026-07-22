"""Live Path H training monitor — popup GUI (Tk + Matplotlib).

Usage:
  cd ~/drone_ws
  export PYTHONPATH=src/drone_rl_planner:$PYTHONPATH
  python3 -m drone_rl_planner.train_sac_monitor
"""

from __future__ import annotations

import json
import re
import threading
import time
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import ttk

import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


def _ckpt_dir() -> Path:
    return Path(__file__).resolve().parents[1] / 'checkpoints'


EVAL_RE = re.compile(
    r'step=(\d+)/(\d+)\s+success=([\d.]+)%\s+collision=([\d.]+)%'
)

# Prefer current curriculum logs before the old overnight file.
LOG_CANDIDATES = (
    'train_sac_curriculum.log',
    'train_sac_fast.log',
    'train_sac_overnight.log',
    'train_sac_fresh.log',
)


class TrainMonitorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title('Anona Path H — SAC Training Monitor')
        self.root.geometry('1100x720')
        self.root.minsize(900, 600)

        self.ckpt = _ckpt_dir()
        self.status_path = self.ckpt / 'sac_training_status.json'
        self.log_path: Path | None = None
        self._log_pos = 0

        self.hist_steps: deque[float] = deque(maxlen=400)
        self.hist_sr: deque[float] = deque(maxlen=400)
        self.hist_cr: deque[float] = deque(maxlen=400)
        self.hist_best: deque[float] = deque(maxlen=400)
        self._seen_evals: set[tuple] = set()
        self._last_status_eval: tuple | None = None
        self._run_fp: tuple | None = None
        self._section_idx: int = -1
        self._rollout_busy = False
        self._agent = None
        self._env = None
        self._env_diff: str | None = None
        self._ckpt_loaded: Path | None = None
        self._ckpt_mtime = 0.0
        self._target_line = None
        self._prev_steps = None
        self._prev_t = None
        self._last_rollout_kick = 0.0

        self._build_ui()
        self._resolve_log(force=True)
        self._load_log_history()
        self.root.after(200, self._tick)

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill=tk.X)

        self.lbl_state = ttk.Label(top, text='state: —', font=('Segoe UI', 12, 'bold'))
        self.lbl_state.pack(anchor=tk.W)
        self.lbl_log = ttk.Label(top, text='log: —', font=('Segoe UI', 9))
        self.lbl_log.pack(anchor=tk.W)

        metrics = ttk.Frame(top)
        metrics.pack(fill=tk.X, pady=(8, 0))

        self.var_best = tk.StringVar(value='Best: —')
        self.var_live = tk.StringVar(value='Live: —')
        self.var_steps = tk.StringVar(value='Steps: —')
        self.var_buf = tk.StringVar(value='Buffer: —')
        self.var_speed = tk.StringVar(value='Speed: —')
        self.var_eta = tk.StringVar(value='ETA: —')
        self.var_target = tk.StringVar(value='Target: —')
        self.var_extras = tk.StringVar(value='n_envs / batch / updates / lr: —')

        for i, var in enumerate(
            (self.var_best, self.var_live, self.var_steps, self.var_buf,
             self.var_speed, self.var_eta, self.var_target, self.var_extras)
        ):
            ttk.Label(metrics, textvariable=var, font=('Segoe UI', 11)).grid(
                row=i // 4, column=i % 4, sticky=tk.W, padx=(0, 24), pady=2)

        bars = ttk.Frame(top)
        bars.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(bars, text='Progress / target').pack(anchor=tk.W)
        self.bar_steps = ttk.Progressbar(bars, maximum=100, mode='determinate')
        self.bar_steps.pack(fill=tk.X, pady=2)
        self.bar_best = ttk.Progressbar(bars, maximum=100, mode='determinate')
        self.bar_best.pack(fill=tk.X, pady=2)

        legend = ttk.LabelFrame(top, text='Parameters · 参数说明', padding=8)
        legend.pack(fill=tk.X, pady=(10, 0))
        legend_text = (
            'Best — best checkpoint success so far / 历史最佳成功率\n'
            'Live — latest eval success & collision / 最近一次评测\n'
            'Steps / Buffer / Speed / ETA / Target — run progress\n'
            'Green = Live success · Red = collision · Blue dashed = Best so far\n'
            'Curves reset automatically when curriculum stage changes (2→3→3b)\n'
            'Right panel — live policy top-down rollout (active ckpt + difficulty)'
        )
        ttk.Label(legend, text=legend_text, justify=tk.LEFT, font=('Segoe UI', 9)).pack(
            anchor=tk.W)

        body = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        left = ttk.Frame(body)
        right = ttk.Frame(body)
        body.add(left, weight=3)
        body.add(right, weight=2)

        self.fig, self.ax = plt.subplots(figsize=(6, 4), dpi=100)
        self.fig.patch.set_facecolor('#f7f7f8')
        self.ax.set_facecolor('#ffffff')
        self.line_sr, = self.ax.plot([], [], color='#1f7a4c', lw=2, label='live success')
        self.line_cr, = self.ax.plot([], [], color='#c0392b', lw=1.5, alpha=0.85,
                                      label='collision')
        self.line_best, = self.ax.plot([], [], color='#2563eb', lw=1.6, ls='--',
                                        label='best')
        self._target_line = self.ax.axhline(
            0.85, color='#888', ls=':', lw=1, label='target')
        self.ax.set_ylim(0, 1.05)
        self.ax.set_xlabel('env steps (this stage)')
        self.ax.set_ylabel('rate')
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc='upper right', fontsize=8)
        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        ttk.Label(right, text='Live policy rollout (top-down)',
                  font=('Segoe UI', 10, 'bold')).pack(anchor=tk.W)
        self.roll_canvas = tk.Canvas(right, bg='#111318', highlightthickness=0)
        self.roll_canvas.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
        btns = ttk.Frame(right)
        btns.pack(fill=tk.X, pady=6)
        ttk.Button(btns, text='Refresh rollout', command=self._kick_rollout).pack(
            side=tk.LEFT)
        ttk.Button(btns, text='Reload log history', command=self._reload_history).pack(
            side=tk.LEFT, padx=8)
        self.lbl_roll = ttk.Label(right, text='Rollout: idle')
        self.lbl_roll.pack(anchor=tk.W)

    def _clear_curves(self) -> None:
        self.hist_steps.clear()
        self.hist_sr.clear()
        self.hist_cr.clear()
        self.hist_best.clear()
        self._seen_evals.clear()
        self._last_status_eval = None
        self._log_pos = 0

    def _run_fingerprint(self, st: dict) -> tuple:
        return (
            str(st.get('run_name') or ''),
            str(st.get('difficulty') or ''),
            str(st.get('stage') or ''),
            int(st.get('total_steps') or 0),
            round(float(st.get('target') or 0.0), 4),
            round(float(st.get('lr') or 0.0), 8),
        )

    def _latest_section_idx(self, text: str) -> int:
        markers = (
            '===== CURRICULUM',
            '===== FAST',
            '===== continuous',
            '===== START',
        )
        idx = -1
        for m in markers:
            j = text.rfind(m)
            if j > idx:
                idx = j
        return idx

    def _resolve_log(self, force: bool = False, st: dict | None = None) -> Path | None:
        """Pick active training log (status hint → newest curriculum/fast/…)."""
        preferred = None
        if st and st.get('log_file'):
            preferred = self.ckpt / str(st['log_file'])
            if not preferred.is_file():
                preferred = None
        candidates: list[Path] = []
        if preferred is not None:
            candidates.append(preferred)
        for name in LOG_CANDIDATES:
            p = self.ckpt / name
            if p.is_file() and p not in candidates:
                candidates.append(p)
        for p in self.ckpt.glob('train_sac_*.log'):
            if p not in candidates:
                candidates.append(p)
        if not candidates:
            self.log_path = None
            return None
        # Prefer status hint; otherwise newest mtime.
        newest = preferred if preferred is not None else max(
            candidates, key=lambda p: p.stat().st_mtime)
        if force or self.log_path is None or newest != self.log_path:
            switched = self.log_path is not None and newest != self.log_path
            self.log_path = newest
            if switched or force:
                self._clear_curves()
                self._section_idx = -1
                self._load_log_history()
        return self.log_path

    def _maybe_reset_for_new_run(self, st: dict) -> None:
        """Same log file can hold stage2/3/3b — reset curves on stage change."""
        fp = self._run_fingerprint(st)
        section_changed = False
        if self.log_path is not None and self.log_path.is_file():
            try:
                # Cheap: only read tail for marker; full read on change.
                text = self.log_path.read_text(errors='ignore')
                idx = self._latest_section_idx(text)
                if self._section_idx >= 0 and idx > self._section_idx:
                    section_changed = True
                if self._section_idx < 0:
                    self._section_idx = idx
            except Exception:
                pass
        if self._run_fp is None:
            self._run_fp = fp
            return
        if fp != self._run_fp or section_changed:
            self._run_fp = fp
            self._clear_curves()
            self._section_idx = -1
            self._load_log_history()

    def _reload_history(self) -> None:
        self._clear_curves()
        self._section_idx = -1
        self._run_fp = None
        self._resolve_log(force=True)
        self._load_log_history()
        self._redraw_curves()

    def _append_eval(self, step: float, sr: float, cr: float) -> None:
        key = (int(step), round(sr, 4), round(cr, 4))
        if key in self._seen_evals:
            return
        self._seen_evals.add(key)
        self.hist_steps.append(step)
        self.hist_sr.append(sr)
        self.hist_cr.append(cr)
        prev_best = self.hist_best[-1] if self.hist_best else 0.0
        self.hist_best.append(max(prev_best, sr))

    def _load_log_history(self) -> None:
        if self.log_path is None or not self.log_path.is_file():
            return
        try:
            text = self.log_path.read_text(errors='ignore')
            self._log_pos = len(text)
            idx = self._latest_section_idx(text)
            self._section_idx = idx
            chunk = text[idx:] if idx >= 0 else text[-200000:]
            for m in EVAL_RE.finditer(chunk):
                self._append_eval(
                    float(m.group(1)),
                    float(m.group(3)) / 100.0,
                    float(m.group(4)) / 100.0,
                )
        except Exception:
            pass

    def _tail_log(self) -> None:
        if self.log_path is None or not self.log_path.is_file():
            return
        try:
            size = self.log_path.stat().st_size
            if size < self._log_pos:
                self._log_pos = 0
            with self.log_path.open('r', errors='ignore') as f:
                f.seek(self._log_pos)
                data = f.read()
                self._log_pos = f.tell()
            # New curriculum banner in the tail → wipe prior stage points.
            if '===== CURRICULUM' in data or '===== FAST' in data:
                self._clear_curves()
                self._load_log_history()
                return
            for line in data.splitlines():
                m = EVAL_RE.search(line)
                if not m:
                    continue
                self._append_eval(
                    float(m.group(1)),
                    float(m.group(3)) / 100.0,
                    float(m.group(4)) / 100.0,
                )
        except Exception:
            pass

    def _ingest_status_eval(self, st: dict) -> None:
        """Curves should update even if the wrong log was open."""
        if st.get('success_rate') is None:
            return
        try:
            step = float(st.get('steps') or 0)
            sr = float(st['success_rate'])
            cr = float(st.get('collision_rate') or 0.0)
        except (TypeError, ValueError):
            return
        key = (int(step), round(sr, 4), round(cr, 4))
        if key == self._last_status_eval:
            return
        self._last_status_eval = key
        self._append_eval(step, sr, cr)

    def _read_status(self) -> dict:
        if not self.status_path.is_file():
            return {}
        try:
            return json.loads(self.status_path.read_text())
        except Exception:
            return {}

    def _resolve_checkpoint(self, st: dict) -> Path | None:
        for key in ('last_checkpoint', 'checkpoint'):
            raw = st.get(key)
            if raw:
                p = Path(str(raw))
                if p.is_file():
                    return p
        name = str(st.get('run_name') or '')
        if name:
            for cand in (self.ckpt / f'{name}.pt', self.ckpt / f'{name}_best.pt'):
                if cand.is_file():
                    return cand
        # Newest shared-arch curriculum weight
        prefs = (
            'sac_polar_mid.pt', 'sac_polar_mid_best.pt',
            'sac_polar_fast.pt', 'sac_polar_fast_best.pt',
            'sac_polar_dense.pt', 'sac_polar_dense_best.pt',
            'sac_polar_local.pt', 'sac_polar_local_best.pt',
        )
        existing = [self.ckpt / n for n in prefs if (self.ckpt / n).is_file()]
        if not existing:
            return None
        return max(existing, key=lambda p: p.stat().st_mtime)

    def _redraw_curves(self, target: float | None = None) -> None:
        if target is not None and self._target_line is not None:
            self._target_line.set_ydata([target, target])
            self._target_line.set_label(f'target {target:.0%}')
            self.ax.legend(loc='upper right', fontsize=8)
        if not self.hist_steps:
            self.line_sr.set_data([], [])
            self.line_cr.set_data([], [])
            self.line_best.set_data([], [])
            self.canvas.draw_idle()
            return
        xs = list(self.hist_steps)
        self.line_sr.set_data(xs, list(self.hist_sr))
        self.line_cr.set_data(xs, list(self.hist_cr))
        self.line_best.set_data(xs, list(self.hist_best))
        self.ax.set_xlim(max(0, xs[0] - 1000), max(xs[-1] * 1.05, 10_000))
        self.ax.relim()
        self.ax.autoscale_view(scalex=False, scaley=False)
        self.canvas.draw_idle()

    def _tick(self) -> None:
        st = self._read_status()
        self._resolve_log(st=st)
        self._maybe_reset_for_new_run(st)
        self._tail_log()
        self._ingest_status_eval(st)

        state = str(st.get('state') or 'unknown')
        steps = int(st.get('steps') or 0)
        total = max(int(st.get('total_steps') or 1), 1)
        best = float(st.get('best_success') or 0.0)
        score = float(st.get('best_score') or 0.0)
        live = st.get('success_rate')
        col = st.get('collision_rate')
        buf = int(st.get('buf') or 0)
        target = float(st.get('target') or 0.85)
        elapsed = float(st.get('elapsed_s') or 0.0)
        run_name = st.get('run_name') or '—'
        difficulty = st.get('difficulty') or '—'
        stage = st.get('stage') or '—'

        log_name = self.log_path.name if self.log_path else '—'
        self.lbl_state.configure(
            text=f'state: {state}   device={st.get("device", "?")}   '
                 f'run={run_name}   stage={stage}   difficulty={difficulty}')
        self.lbl_log.configure(
            text=f'log: {log_name}   points={len(self.hist_steps)} '
                 f'(this stage only)')

        self.var_best.set(f'Best: {best:.1%}  (score {score:.3f})')
        if live is not None:
            self.var_live.set(
                f'Live: {float(live):.1%}'
                + (f'  col {float(col):.1%}' if col is not None else '')
            )
        else:
            self.var_live.set('Live: (waiting for eval)')
        self.var_steps.set(f'Steps: {steps:,} / {total:,}')
        self.var_buf.set(f'Buffer: {buf:,}')
        self.var_target.set(f'Target: {target:.0%}')
        extras = []
        for key, label in (
            ('n_envs', 'n_envs'), ('batch_size', 'batch'),
            ('updates_per_step', 'updates/step'), ('lr', 'lr'),
        ):
            if st.get(key) is not None:
                extras.append(f'{label}={st[key]}')
        self.var_extras.set(' · '.join(extras) if extras else 'n_envs / batch / updates / lr: —')

        now = time.time()
        speed = None
        if self._prev_steps is not None and self._prev_t is not None:
            dt = max(now - self._prev_t, 1e-3)
            ds = steps - self._prev_steps
            if ds >= 0:
                speed = ds / dt * 3600.0
        self._prev_steps = steps
        self._prev_t = now
        if speed is not None and speed > 0:
            self.var_speed.set(f'Speed: {speed:,.0f} steps/h')
            remain = max(total - steps, 0)
            self.var_eta.set(f'ETA (budget): {remain / max(speed, 1.0):.1f} h')
        elif elapsed > 1 and steps > 0:
            speed = steps / elapsed * 3600.0
            self.var_speed.set(f'Speed: ~{speed:,.0f} steps/h')
            remain = max(total - steps, 0)
            self.var_eta.set(f'ETA (budget): {remain / max(speed, 1):.1f} h')
        else:
            self.var_speed.set('Speed: —')
            self.var_eta.set('ETA: —')

        self.bar_steps['value'] = 100.0 * steps / total
        self.bar_best['value'] = 100.0 * min(best / max(target, 1e-6), 1.0)

        self._redraw_curves(target=target)

        # Periodic rollout (~every 25s), once canvas has a real size.
        if (not self._rollout_busy
                and now - self._last_rollout_kick > 25.0
                and self.roll_canvas.winfo_width() > 50):
            self._last_rollout_kick = now
            self._kick_rollout()

        self.root.after(1000, self._tick)

    def _kick_rollout(self) -> None:
        if self._rollout_busy:
            return
        self._rollout_busy = True
        self.lbl_roll.configure(text='Rollout: running…')
        st = self._read_status()
        threading.Thread(
            target=self._rollout_worker, args=(st,), daemon=True).start()

    def _rollout_worker(self, st: dict) -> None:
        try:
            frames = self._simulate_episode(st)
            self.root.after(0, lambda: self._draw_rollout(frames))
        except Exception as exc:
            msg = f'Rollout error: {exc}'
            self.root.after(0, lambda m=msg: self.lbl_roll.configure(text=m))
        finally:
            self.root.after(0, self._clear_rollout_busy)

    def _clear_rollout_busy(self) -> None:
        self._rollout_busy = False

    def _simulate_episode(self, st: dict):
        from drone_rl_planner.polar_nav_env import PolarEnvConfig, PolarNavEnv
        from drone_rl_planner.sac_drq import SACAgent, SACConfig

        ckpt = self._resolve_checkpoint(st)
        if ckpt is None or not ckpt.is_file():
            raise RuntimeError('no checkpoint')

        difficulty = str(st.get('difficulty') or 'medium')
        need_new_env = (
            self._env is None
            or self._env_diff != difficulty
        )
        if need_new_env:
            kwargs = {
                'easy': difficulty == 'easy',
                'medium': difficulty == 'medium',
                'dense_heavy': difficulty in ('dense', 'default'),
                'forest_heavy': difficulty == 'forest',
            }
            # Only one difficulty flag may be true.
            if difficulty == 'default':
                kwargs = {'dense_heavy': True}
            self._env = PolarNavEnv(PolarEnvConfig(), seed=7, **kwargs)
            self._env_diff = difficulty
            self._agent = None

        env = self._env
        mtime = ckpt.stat().st_mtime
        if (self._agent is None
                or self._ckpt_loaded != ckpt
                or mtime != self._ckpt_mtime):
            device = 'cpu'
            agent = SACAgent(
                env.image_shape, env.vec_dim, env.act_dim, SACConfig(device=device))
            agent.load(ckpt, map_location=device)
            self._agent = agent
            self._ckpt_loaded = ckpt
            self._ckpt_mtime = mtime

        agent = self._agent
        obs = env.reset()
        frames = []
        done = False
        steps = 0
        info = {}
        while not done and steps < 400:
            act = agent.act(obs, deterministic=True)
            obs, _r, done, info = env.step(act)
            base = env._base
            frames.append({
                'pos': np.asarray(base.pos[:2], dtype=float),
                'goal': np.asarray(base.goal[:2], dtype=float),
                'obstacles': np.asarray(base.obs_xy, dtype=float)
                if getattr(base, 'obs_xy', None) is not None else np.zeros((0, 2)),
                'radii': np.asarray(base.obs_r, dtype=float)
                if getattr(base, 'obs_r', None) is not None else None,
                'success': bool(info.get('success')),
                'collision': bool(info.get('collision')),
                't': steps,
            })
            steps += 1
        frames.append({
            'done': True,
            'info': info,
            'n': steps,
            'ckpt': str(ckpt.name),
            'difficulty': difficulty,
        })
        return frames

    def _draw_rollout(self, frames) -> None:
        if not frames:
            self.lbl_roll.configure(text='Rollout: empty')
            return
        meta = frames[-1]
        path_frames = frames[:-1]
        info = meta.get('info') or {}
        outcome = 'success' if info.get('success') else (
            'collision' if info.get('collision') else 'timeout')
        self.lbl_roll.configure(
            text=f'Rollout: {outcome} in {meta.get("n", 0)} steps  '
                 f'[{meta.get("ckpt", "?")} · {meta.get("difficulty", "?")}]')

        c = self.roll_canvas
        c.update_idletasks()
        c.delete('all')
        w = max(c.winfo_width(), 200)
        h = max(c.winfo_height(), 200)

        if not path_frames:
            c.create_text(w // 2, h // 2, text='no path', fill='#889',
                          font=('Segoe UI', 12))
            return

        pts = []
        for fr in path_frames:
            pts.append(fr['pos'])
            pts.append(fr['goal'])
            if len(fr['obstacles']):
                pts.append(fr['obstacles'])
        all_xy = np.vstack([p if p.ndim == 2 else p[None, :] for p in pts])
        xmin, ymin = all_xy.min(axis=0) - 1.0
        xmax, ymax = all_xy.max(axis=0) + 1.0
        span = max(xmax - xmin, ymax - ymin, 1.0)

        def to_px(xy):
            x = (xy[0] - xmin) / span * (w - 20) + 10
            y = h - ((xy[1] - ymin) / span * (h - 20) + 10)
            return x, y

        last = path_frames[-1]
        obs = last['obstacles']
        radii = last.get('radii')
        for i, p in enumerate(obs):
            x, y = to_px(p)
            r = 4.0
            if radii is not None and i < len(radii):
                r = max(2.0, float(radii[i]) / span * (w - 20))
            c.create_oval(x - r, y - r, x + r, y + r, fill='#5a6472', outline='')

        if len(path_frames) >= 2:
            coords = []
            for fr in path_frames:
                coords.extend(to_px(fr['pos']))
            c.create_line(*coords, fill='#6ee7a8', width=2)

        gx, gy = to_px(last['goal'])
        c.create_oval(gx - 6, gy - 6, gx + 6, gy + 6, outline='#f5c542', width=2)
        px, py = to_px(last['pos'])
        color = '#6ee7a8' if info.get('success') else (
            '#ff6b6b' if info.get('collision') else '#9ec1ff')
        c.create_oval(px - 7, py - 7, px + 7, py + 7, fill=color, outline='white')


def main(argv=None) -> None:
    root = tk.Tk()
    try:
        style = ttk.Style()
        if 'clam' in style.theme_names():
            style.theme_use('clam')
    except Exception:
        pass
    TrainMonitorApp(root)
    root.mainloop()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Standalone real-time interference / disturbance monitor for scenario 6.

Auto-launched by ``stability_demo.launch.py``. Subscribes to:
  - /drone/disturbance_status  (wind + IMU noise)
  - /drone/odom                (hover position error)

Usage (manual):
  ros2 run drone_bringup interference_monitor
"""

from __future__ import annotations

import math
import os
import threading
from collections import deque
from typing import Deque, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

try:
    from drone_msgs.msg import DisturbanceStatus
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        'drone_msgs.DisturbanceStatus missing — rebuild: '
        'colcon build --packages-up-to drone_bringup'
    ) from exc

import tkinter as tk
from tkinter import ttk

try:
    import matplotlib

    matplotlib.use('TkAgg')
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    _HAVE_MPL = True
except Exception:  # pragma: no cover
    _HAVE_MPL = False


class InterferenceMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('interference_monitor')
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_z', 1.5)
        self.declare_parameter('hover_limit_m', 0.3)

        self.goal = (
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
            float(self.get_parameter('goal_z').value),
        )
        self.hover_limit = float(self.get_parameter('hover_limit_m').value)

        self.lock = threading.Lock()
        self.disturb: Optional[DisturbanceStatus] = None
        self.pos: Optional[Tuple[float, float, float]] = None
        self.err: Optional[float] = None
        self.max_err = 0.0
        self.t0: Optional[float] = None

        self.hist_t: Deque[float] = deque(maxlen=400)
        self.hist_wn: Deque[float] = deque(maxlen=400)
        self.hist_err: Deque[float] = deque(maxlen=400)

        self.create_subscription(
            DisturbanceStatus, '/drone/disturbance_status', self._on_disturb, 20)
        self.create_subscription(Odometry, '/drone/odom', self._on_odom, 50)
        self.get_logger().info(
            f'Interference monitor ready (goal={self.goal}, limit={self.hover_limit} m)')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_disturb(self, msg: DisturbanceStatus) -> None:
        with self.lock:
            self.disturb = msg
            if self.t0 is None:
                self.t0 = self._now()
            t = self._now() - self.t0
            self.hist_t.append(t)
            self.hist_wn.append(float(msg.wind_force_norm))
            self.hist_err.append(float(self.err) if self.err is not None else float('nan'))

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        pos = (float(p.x), float(p.y), float(p.z))
        err = math.sqrt(
            (pos[0] - self.goal[0]) ** 2
            + (pos[1] - self.goal[1]) ** 2
            + (pos[2] - self.goal[2]) ** 2
        )
        with self.lock:
            self.pos = pos
            self.err = err
            if err > self.max_err:
                self.max_err = err


class InterferenceMonitorApp:
    def __init__(self, root: tk.Tk, node: InterferenceMonitorNode) -> None:
        self.root = root
        self.node = node
        self.root.title('Anona — Interference Monitor (Scenario 6)')
        self.root.geometry('720x560')
        self.root.minsize(560, 420)
        self.root.configure(bg='#1a1d23')

        style = ttk.Style()
        try:
            style.theme_use('clam')
        except tk.TclError:
            pass
        style.configure('TFrame', background='#1a1d23')
        style.configure('TLabel', background='#1a1d23', foreground='#e8eaed', font=('Segoe UI', 11))
        style.configure('Title.TLabel', font=('Segoe UI', 14, 'bold'), foreground='#f5f7fa')
        style.configure('ChipOn.TLabel', foreground='#3dd68c', font=('Segoe UI', 12, 'bold'))
        style.configure('ChipOff.TLabel', foreground='#8b919a', font=('Segoe UI', 12, 'bold'))
        style.configure('Warn.TLabel', foreground='#f5a524', font=('Segoe UI', 12, 'bold'))
        style.configure('Fail.TLabel', foreground='#f07178', font=('Segoe UI', 12, 'bold'))
        style.configure('Ok.TLabel', foreground='#3dd68c', font=('Segoe UI', 12, 'bold'))
        style.configure('Mono.TLabel', font=('Consolas', 11), foreground='#c5c8ce')

        outer = ttk.Frame(root, padding=12)
        outer.pack(fill=tk.BOTH, expand=True)

        ttk.Label(outer, text='Stability / Interference', style='Title.TLabel').pack(anchor='w')
        ttk.Label(
            outer,
            text='Wind + IMU noise · live plant disturbance (RViz = geometry only)',
            style='TLabel',
        ).pack(anchor='w', pady=(0, 10))

        chips = ttk.Frame(outer)
        chips.pack(fill=tk.X, pady=(0, 8))
        self.lbl_wind = ttk.Label(chips, text='WIND —', style='ChipOff.TLabel')
        self.lbl_wind.pack(side=tk.LEFT, padx=(0, 16))
        self.lbl_imu = ttk.Label(chips, text='IMU NOISE —', style='ChipOff.TLabel')
        self.lbl_imu.pack(side=tk.LEFT)

        grid = ttk.Frame(outer)
        grid.pack(fill=tk.X, pady=4)

        self.vars = {
            'sim': tk.StringVar(value='sim_time: —'),
            'force': tk.StringVar(value='wind force: —'),
            'cfg': tk.StringVar(value='wind cfg: —'),
            'imu_cfg': tk.StringVar(value='IMU σ: —'),
            'imu_a': tk.StringVar(value='accel: —'),
            'imu_g': tk.StringVar(value='gyro: —'),
            'pos': tk.StringVar(value='position: —'),
            'err': tk.StringVar(value='hover err: —'),
            'maxe': tk.StringVar(value='max err: —'),
            'pass': tk.StringVar(value='hover ≤0.3 m: —'),
        }
        for key in ('sim', 'force', 'cfg', 'imu_cfg', 'imu_a', 'imu_g', 'pos', 'err', 'maxe'):
            ttk.Label(grid, textvariable=self.vars[key], style='Mono.TLabel').pack(anchor='w')

        self.lbl_pass = ttk.Label(grid, textvariable=self.vars['pass'], style='Warn.TLabel')
        self.lbl_pass.pack(anchor='w', pady=(6, 0))

        if _HAVE_MPL:
            fig = Figure(figsize=(6.4, 2.6), dpi=100, facecolor='#1a1d23')
            self.ax = fig.add_subplot(111)
            self.ax.set_facecolor('#12141a')
            self.ax.tick_params(colors='#9aa0a6')
            for spine in self.ax.spines.values():
                spine.set_color('#3c4048')
            # NOTE: hist_t is wall-clock time (node clock at message arrival), NOT
            # sim_time shown in the HUD readout above; axis label kept explicit so
            # the two time bases are not confused.
            self.ax.set_xlabel('t [s] (wall clock since start)', color='#9aa0a6')
            self.ax.set_ylabel('wind |F| [N] / err [m]', color='#9aa0a6')
            self.line_w, = self.ax.plot([], [], color='#4cc9f0', lw=1.6, label='|F_wind|')
            self.line_e, = self.ax.plot([], [], color='#f5a524', lw=1.6, label='hover err')
            self.ax.axhline(node.hover_limit, color='#f07178', ls='--', lw=1.0, alpha=0.8)
            self.ax.legend(loc='upper right', facecolor='#1a1d23', edgecolor='#3c4048',
                           labelcolor='#e8eaed', fontsize=8)
            fig.tight_layout()
            self.canvas = FigureCanvasTkAgg(fig, master=outer)
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        else:
            self.ax = None
            ttk.Label(outer, text='(matplotlib unavailable — numbers only)', style='TLabel').pack(
                anchor='w', pady=8)

        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.after(100, self._tick)

    def _on_close(self) -> None:
        self.root.destroy()

    def _tick(self) -> None:
        with self.node.lock:
            d = self.node.disturb
            pos = self.node.pos
            err = self.node.err
            max_err = self.node.max_err
            ht = list(self.node.hist_t)
            hw = list(self.node.hist_wn)
            he = list(self.node.hist_err)

        if d is not None:
            self.lbl_wind.configure(
                text='WIND ON' if d.wind_enable else 'WIND OFF',
                style='ChipOn.TLabel' if d.wind_enable else 'ChipOff.TLabel',
            )
            self.lbl_imu.configure(
                text='IMU NOISE ON' if d.imu_noise_enable else 'IMU NOISE OFF',
                style='ChipOn.TLabel' if d.imu_noise_enable else 'ChipOff.TLabel',
            )
            self.vars['sim'].set(f'sim_time: {d.sim_time:6.1f} s')
            self.vars['force'].set(
                f'wind force: ({d.wind_force_x:+.3f}, {d.wind_force_y:+.3f}, '
                f'{d.wind_force_z:+.3f}) N   |F|={d.wind_force_norm:.3f}'
            )
            self.vars['cfg'].set(
                f'wind cfg: const=({d.wind_const_x:+.2f},{d.wind_const_y:+.2f},'
                f'{d.wind_const_z:+.2f})  sin_amp={d.wind_sin_amp:.2f} N  '
                f'f={d.wind_sin_freq:.2f} Hz'
            )
            self.vars['imu_cfg'].set(
                f'IMU σ: accel={d.imu_accel_noise_std:.4f} m/s²  '
                f'gyro={d.imu_gyro_noise_std:.4f} rad/s'
            )
            self.vars['imu_a'].set(
                f'accel (noisy): ({d.imu_accel_x:+.3f}, {d.imu_accel_y:+.3f}, '
                f'{d.imu_accel_z:+.3f}) m/s²'
            )
            self.vars['imu_g'].set(
                f'gyro  (noisy): ({d.imu_gyro_x:+.3f}, {d.imu_gyro_y:+.3f}, '
                f'{d.imu_gyro_z:+.3f}) rad/s'
            )

        if pos is not None:
            self.vars['pos'].set(f'position: ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}) m')
        if err is not None:
            self.vars['err'].set(f'hover err: {err:.4f} m')
            self.vars['maxe'].set(f'max err:  {max_err:.4f} m')
            ok = err <= self.node.hover_limit
            self.vars['pass'].set(
                f'hover ≤{self.node.hover_limit:.1f} m: '
                + ('PASS (now)' if ok else 'ABOVE LIMIT')
            )
            self.lbl_pass.configure(style='Ok.TLabel' if ok else 'Fail.TLabel')

        if self.ax is not None and ht:
            self.line_w.set_data(ht, hw)
            self.line_e.set_data(ht, [v if math.isfinite(v) else float('nan') for v in he])
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw_idle()

        self.root.after(100, self._tick)


def main(argv=None) -> int:
    if not os.environ.get('DISPLAY'):
        print('interference_monitor: no DISPLAY — skipping GUI', flush=True)
        return 0

    rclpy.init(args=argv)
    node = InterferenceMonitorNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    InterferenceMonitorApp(root, node)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

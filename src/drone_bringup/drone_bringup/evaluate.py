#!/usr/bin/env python3
"""
Scripted evaluation for acceptance scenarios.
Subscribes odom / motor RPM / planner status / obstacle cloud,
optionally planned path / trajectory_cmd / planner diagnostics,
exports CSV + matplotlib plots to output directory.
"""

import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from drone_msgs.msg import MotorCommand, PlannerDiagnostics, PlannerStatus, TrajectoryCommand
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool


@dataclass
class Sample:
    t: float
    px: float
    py: float
    pz: float
    err: float
    rpm0: float
    rpm1: float
    rpm2: float
    rpm3: float
    min_obs: float
    planner_state: str = ''


@dataclass
class FallbackEvent:
    t: float
    active: bool
    reason: str = ''
    source: str = ''


@dataclass
class TrajCmdSample:
    t: float
    px: float
    py: float
    pz: float
    ready: bool


@dataclass
class EvalState:
    goal: Tuple[float, float, float]
    safety_distance: float = 0.35
    samples: List[Sample] = field(default_factory=list)
    obstacles: List[Tuple[float, float, float]] = field(default_factory=list)
    last_rpm: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    last_planner: str = ''
    planner_success_ever: bool = False
    t0: Optional[float] = None
    planned_path: List[Tuple[float, float, float]] = field(default_factory=list)
    traj_cmd_samples: List[TrajCmdSample] = field(default_factory=list)
    fallback_events: List[FallbackEvent] = field(default_factory=list)
    last_fallback_active: bool = False
    hold_at_goal_samples: int = 30


def min_distance_to_obstacles(pos: np.ndarray, obstacles: List[Tuple[float, float, float]]) -> float:
    if not obstacles:
        return float('inf')
    obs = np.asarray(obstacles)
    # Dense maps have ~1e5 points; subsample for real-time min-distance.
    if obs.shape[0] > 4000:
        step = max(1, obs.shape[0] // 4000)
        obs = obs[::step]
    d = np.linalg.norm(obs - pos[None, :], axis=1)
    return float(np.min(d))


def path_length_xyz(pts: np.ndarray) -> float:
    if pts.shape[0] < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def mean_jerk_from_positions(ts: np.ndarray, pts: np.ndarray) -> float:
    """Finite-difference jerk magnitude (m/s³) averaged over interior samples."""
    if pts.shape[0] < 5:
        return float('nan')
    dt = np.diff(ts)
    if np.any(dt <= 1e-6):
        return float('nan')
    vel = np.diff(pts, axis=0) / dt[:, None]
    dt2 = dt[1:]
    acc = np.diff(vel, axis=0) / dt2[:, None]
    dt3 = dt2[1:]
    jerk = np.diff(acc, axis=0) / dt3[:, None]
    return float(np.mean(np.linalg.norm(jerk, axis=1)))


def mean_tracking_error_to_path(
    flown: np.ndarray,
    planned: List[Tuple[float, float, float]],
) -> float:
    if not planned or flown.shape[0] == 0:
        return float('nan')
    plan = np.asarray(planned)
    errs = []
    for p in flown:
        errs.append(float(np.min(np.linalg.norm(plan - p[None, :], axis=1))))
    return float(np.mean(errs))


class EvaluateNode(Node):
    def __init__(self, state: EvalState, duration: float, output_dir: str):
        super().__init__('evaluate_drone')
        self.state = state
        self.duration = duration
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        self.create_subscription(Odometry, '/drone/odom', self._on_odom, 50)
        self.create_subscription(MotorCommand, '/drone/motor_rpm_cmd', self._on_motor, 50)
        self.create_subscription(PlannerStatus, '/planner/status', self._on_status, 10)
        self.create_subscription(
            PointCloud2, '/map/obstacles',
            self._on_map, rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        )
        # Optional planner topics — metrics computed only when messages arrive.
        path_qos = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/planner/trajectory', self._on_planned_path, path_qos)
        self.create_subscription(TrajectoryCommand, '/planner/trajectory_cmd', self._on_traj_cmd, 10)
        self.create_subscription(PlannerDiagnostics, '/planner/diagnostics', self._on_diagnostics, 10)
        self.create_subscription(Bool, '/planner/fallback_active', self._on_fallback_bool, 10)

        self._latest_odom: Optional[Odometry] = None
        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(f'Evaluation started: goal={state.goal}, duration={duration}s')

    def _elapsed(self) -> float:
        if self.state.t0 is None:
            return 0.0
        return self.get_clock().now().nanoseconds * 1e-9 - self.state.t0

    def _record_fallback(self, active: bool, reason: str, source: str) -> None:
        if active == self.state.last_fallback_active:
            return
        self.state.last_fallback_active = active
        self.state.fallback_events.append(FallbackEvent(
            t=self._elapsed(), active=active, reason=reason, source=source,
        ))

    def _on_map(self, msg: PointCloud2):
        if self.state.obstacles:
            return
        pts = []
        for x, y, z in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            pts.append((float(x), float(y), float(z)))
        self.state.obstacles = pts
        self.get_logger().info(f'Loaded {len(pts)} obstacle points')

    def _on_motor(self, msg: MotorCommand):
        self.state.last_rpm = tuple(msg.rpm)

    def _on_status(self, msg: PlannerStatus):
        self.state.last_planner = msg.state
        if msg.success:
            self.state.planner_success_ever = True

    def _on_planned_path(self, msg: Path):
        self.state.planned_path = [
            (float(p.pose.position.x), float(p.pose.position.y), float(p.pose.position.z))
            for p in msg.poses
        ]

    def _on_traj_cmd(self, msg: TrajectoryCommand):
        ready = bool(msg.trajectory_ready)
        # Path F / EGO bridges publish TrajectoryCommand but not always PlannerStatus.
        if ready:
            self.state.planner_success_ever = True
            if not self.state.last_planner:
                self.state.last_planner = 'TRAJ_CMD'
        if self.state.t0 is None:
            return
        self.state.traj_cmd_samples.append(TrajCmdSample(
            t=self._elapsed(),
            px=float(msg.position.x),
            py=float(msg.position.y),
            pz=float(msg.position.z),
            ready=ready,
        ))

    def _on_diagnostics(self, msg: PlannerDiagnostics):
        self._record_fallback(
            bool(msg.fallback_active),
            msg.fallback_reason or '',
            'diagnostics',
        )

    def _on_fallback_bool(self, msg: Bool):
        self._record_fallback(bool(msg.data), '', 'fallback_active')

    def _on_odom(self, msg: Odometry):
        self._latest_odom = msg

    def _tick(self):
        if self.state.t0 is None:
            if self._latest_odom is None:
                return
            self.state.t0 = self.get_clock().now().nanoseconds * 1e-9

        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self.state.t0
        if self._latest_odom is not None:
            p = np.array([
                self._latest_odom.pose.pose.position.x,
                self._latest_odom.pose.pose.position.y,
                self._latest_odom.pose.pose.position.z,
            ])
            g = np.array(self.state.goal)
            err = float(np.linalg.norm(p - g))
            mind = min_distance_to_obstacles(p, self.state.obstacles)
            self.state.samples.append(Sample(
                t=elapsed, px=p[0], py=p[1], pz=p[2], err=err,
                rpm0=self.state.last_rpm[0], rpm1=self.state.last_rpm[1],
                rpm2=self.state.last_rpm[2], rpm3=self.state.last_rpm[3],
                min_obs=mind, planner_state=self.state.last_planner,
            ))

        if elapsed >= self.duration:
            self._export()
            self.timer.cancel()
            rclpy.shutdown()

    def _export(self):
        if not self.state.samples:
            self.get_logger().warn('No samples recorded')
            return

        csv_path = os.path.join(self.output_dir, 'metrics.csv')
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                't', 'px', 'py', 'pz', 'pos_err', 'rpm0', 'rpm1', 'rpm2', 'rpm3',
                'min_obstacle_dist', 'planner_state',
            ])
            for s in self.state.samples:
                w.writerow([
                    f'{s.t:.3f}', f'{s.px:.4f}', f'{s.py:.4f}', f'{s.pz:.4f}',
                    f'{s.err:.4f}', f'{s.rpm0:.1f}', f'{s.rpm1:.1f}',
                    f'{s.rpm2:.1f}', f'{s.rpm3:.1f}',
                    f'{s.min_obs:.4f}' if math.isfinite(s.min_obs) else 'inf',
                    s.planner_state,
                ])

        if self.state.planned_path:
            with open(os.path.join(self.output_dir, 'planned_path.csv'), 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['px', 'py', 'pz'])
                for x, y, z in self.state.planned_path:
                    w.writerow([f'{x:.4f}', f'{y:.4f}', f'{z:.4f}'])

        if self.state.traj_cmd_samples:
            with open(os.path.join(self.output_dir, 'trajectory_cmd.csv'), 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'px', 'py', 'pz', 'trajectory_ready'])
                for s in self.state.traj_cmd_samples:
                    w.writerow([
                        f'{s.t:.3f}', f'{s.px:.4f}', f'{s.py:.4f}', f'{s.pz:.4f}',
                        str(s.ready),
                    ])

        if self.state.fallback_events:
            with open(os.path.join(self.output_dir, 'fallback_events.csv'), 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'active', 'reason', 'source'])
                for e in self.state.fallback_events:
                    w.writerow([f'{e.t:.3f}', str(e.active), e.reason, e.source])

        if self.state.obstacles:
            obs_path = os.path.join(self.output_dir, 'obstacles.csv')
            pts = self.state.obstacles
            # Cap file size for dense clouds while keeping plot coverage.
            max_pts = 50000
            if len(pts) > max_pts:
                step = max(1, len(pts) // max_pts)
                pts = pts[::step]
            with open(obs_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['x', 'y', 'z'])
                for x, y, z in pts:
                    w.writerow([f'{x:.4f}', f'{y:.4f}', f'{z:.4f}'])

        errs = [s.err for s in self.state.samples]
        mins = [s.min_obs for s in self.state.samples if math.isfinite(s.min_obs)]
        ts = np.array([s.t for s in self.state.samples])
        flown = np.array([[s.px, s.py, s.pz] for s in self.state.samples])

        hold_n = min(self.state.hold_at_goal_samples, len(errs))
        hold_tail = errs[-hold_n:] if hold_n > 0 else []
        hold_pass = bool(hold_tail) and all(e <= 0.3 for e in hold_tail)

        flown_len = path_length_xyz(flown)
        start = flown[0]
        straight = float(np.linalg.norm(np.array(self.state.goal) - start))
        detour_ratio = flown_len / straight if straight > 1e-3 else float('nan')
        mean_jerk = mean_jerk_from_positions(ts, flown)
        plan_track_err = mean_tracking_error_to_path(flown, self.state.planned_path)

        duration_s = float(ts[-1] - ts[0]) if len(ts) > 1 else 0.0
        # Count rising edges into fallback-active.
        fb_rising = 0
        prev = False
        for e in self.state.fallback_events:
            if e.active and not prev:
                fb_rising += 1
            prev = e.active
        fallback_rate = fb_rising / duration_s if duration_s > 1e-3 else float('nan')

        summary_path = os.path.join(self.output_dir, 'summary.txt')
        with open(summary_path, 'w') as f:
            f.write(f'samples: {len(self.state.samples)}\n')
            f.write(f'mean_pos_err: {np.mean(errs):.4f} m\n')
            f.write(f'max_pos_err: {np.max(errs):.4f} m\n')
            f.write(f'final_pos_err: {errs[-1]:.4f} m\n')
            f.write(f'hover_pass_0.3m: {np.mean(errs[-int(len(errs)*0.3):]) <= 0.3}\n')
            f.write(f'goal_pass_0.3m: {errs[-1] <= 0.3}\n')
            f.write(f'hold_at_goal_pass_0.3m: {hold_pass}\n')
            f.write(f'hold_at_goal_samples: {hold_n}\n')
            f.write(f'planner_success_ever: {self.state.planner_success_ever}\n')
            f.write(f'final_planner_state: {self.state.last_planner}\n')
            if mins:
                min_obs = float(np.min(mins))
                f.write(f'min_obstacle_distance: {min_obs:.4f} m\n')
                f.write(f'avoidance_safety_distance: {self.state.safety_distance:.4f} m\n')
                f.write(f'avoidance_pass: {min_obs > self.state.safety_distance}\n')
                # Retain the legacy field for older reports and scripts.
                f.write(f'avoidance_pass_0.35m: {min_obs > 0.35}\n')
            if math.isfinite(flown_len) and flown_len > 0:
                f.write(f'flown_path_length: {flown_len:.4f} m\n')
            if math.isfinite(detour_ratio):
                f.write(f'detour_ratio: {detour_ratio:.4f}\n')
            if math.isfinite(mean_jerk):
                f.write(f'mean_jerk: {mean_jerk:.4f} m/s^3\n')
            if self.state.planned_path and math.isfinite(plan_track_err):
                f.write(f'planned_tracking_error_mean: {plan_track_err:.4f} m\n')
            if self.state.fallback_events:
                f.write(f'fallback_trigger_count: {fb_rising}\n')
                if math.isfinite(fallback_rate):
                    f.write(f'fallback_trigger_rate_hz: {fallback_rate:.4f}\n')

        try:
            from drone_bringup.eval_figures import save_evaluation_figure

            rpm = np.array(
                [[s.rpm0, s.rpm1, s.rpm2, s.rpm3] for s in self.state.samples],
                dtype=float,
            )
            planned_xy = None
            if self.state.planned_path:
                planned_xy = np.array(
                    [[p[0], p[1]] for p in self.state.planned_path], dtype=float)
            save_evaluation_figure(
                output_path=os.path.join(self.output_dir, 'evaluation.png'),
                ts=ts,
                errs=errs,
                px=flown[:, 0],
                py=flown[:, 1],
                rpm=rpm,
                min_obs=[s.min_obs for s in self.state.samples],
                goal_xy=(self.state.goal[0], self.state.goal[1]),
                planned_xy=planned_xy,
                obs_limit=self.state.safety_distance,
            )
        except ImportError:
            self.get_logger().warn('matplotlib / eval_figures not available; skipped plots')
        except Exception as exc:
            self.get_logger().warn(f'evaluation figure failed: {exc}')

        self.get_logger().info(f'Exported metrics to {self.output_dir}')
        self.get_logger().info(
            f'mean_err={np.mean(errs):.3f} max_err={np.max(errs):.3f} final_err={errs[-1]:.3f}'
        )
        raise SystemExit(0)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Evaluate drone simulation metrics')
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--output-dir', type=str, default='')
    parser.add_argument('--goal-x', type=float, default=0.0)
    parser.add_argument('--goal-y', type=float, default=0.0)
    parser.add_argument('--goal-z', type=float, default=1.5)
    parser.add_argument('--safety-distance', type=float, default=0.35,
                        help='Minimum obstacle clearance used for pass/fail and plots')
    parser.add_argument('--hold-samples', type=int, default=30,
                        help='Last N odom samples for hold-at-goal check (default 30)')
    args = parser.parse_args(argv)

    out = args.output_dir or os.path.expanduser('~/drone_ws/scripts/output')
    state = EvalState(goal=(args.goal_x, args.goal_y, args.goal_z),
                      safety_distance=args.safety_distance,
                      hold_at_goal_samples=args.hold_samples)

    rclpy.init(args=argv)
    node = EvaluateNode(state, args.duration, out)
    rclpy.spin(node)
    return 0


if __name__ == '__main__':
    sys.exit(main())

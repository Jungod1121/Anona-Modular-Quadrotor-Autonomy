#!/usr/bin/env python3
"""
Scripted evaluation for acceptance scenarios.
Subscribes odom / motor RPM / planner status / obstacle cloud,
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
from drone_msgs.msg import MotorCommand, PlannerStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


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
class EvalState:
    goal: Tuple[float, float, float]
    samples: List[Sample] = field(default_factory=list)
    obstacles: List[Tuple[float, float, float]] = field(default_factory=list)
    last_rpm: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    last_planner: str = ''
    planner_success_ever: bool = False
    t0: Optional[float] = None


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
        self._latest_odom: Optional[Odometry] = None
        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(f'Evaluation started: goal={state.goal}, duration={duration}s')

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

        errs = [s.err for s in self.state.samples]
        mins = [s.min_obs for s in self.state.samples if math.isfinite(s.min_obs)]
        summary_path = os.path.join(self.output_dir, 'summary.txt')
        with open(summary_path, 'w') as f:
            f.write(f'samples: {len(self.state.samples)}\n')
            f.write(f'mean_pos_err: {np.mean(errs):.4f} m\n')
            f.write(f'max_pos_err: {np.max(errs):.4f} m\n')
            f.write(f'final_pos_err: {errs[-1]:.4f} m\n')
            f.write(f'hover_pass_0.3m: {np.mean(errs[-int(len(errs)*0.3):]) <= 0.3}\n')
            f.write(f'goal_pass_0.3m: {errs[-1] <= 0.3}\n')
            f.write(f'planner_success_ever: {self.state.planner_success_ever}\n')
            f.write(f'final_planner_state: {self.state.last_planner}\n')
            if mins:
                f.write(f'min_obstacle_distance: {np.min(mins):.4f} m\n')
                f.write(f'avoidance_pass_0.35m: {np.min(mins) > 0.35}\n')

        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            ts = [s.t for s in self.state.samples]
            fig, axes = plt.subplots(2, 2, figsize=(10, 8))
            axes[0, 0].plot(ts, errs)
            axes[0, 0].axhline(0.3, color='r', ls='--', label='0.3m limit')
            axes[0, 0].set_title('Position error')
            axes[0, 0].set_xlabel('t [s]')
            axes[0, 0].legend()
            axes[0, 0].grid(True)

            for i, lbl in enumerate(['rpm0', 'rpm1', 'rpm2', 'rpm3']):
                axes[0, 1].plot(ts, [getattr(s, lbl) for s in self.state.samples], label=lbl)
            axes[0, 1].set_title('Motor RPM')
            axes[0, 1].legend()
            axes[0, 1].grid(True)

            axes[1, 0].plot([s.px for s in self.state.samples], [s.py for s in self.state.samples], 'b-')
            axes[1, 0].plot(self.state.goal[0], self.state.goal[1], 'r*', ms=12)
            axes[1, 0].set_title('XY trajectory')
            axes[1, 0].set_aspect('equal')
            axes[1, 0].grid(True)

            if mins:
                axes[1, 1].plot(ts, [s.min_obs for s in self.state.samples])
                axes[1, 1].axhline(0.35, color='r', ls='--', label='safety 0.35m')
                axes[1, 1].set_title('Min obstacle distance')
                axes[1, 1].legend()
                axes[1, 1].grid(True)
            else:
                axes[1, 1].text(0.5, 0.5, 'No obstacle cloud', ha='center', va='center')
                axes[1, 1].set_axis_off()

            fig.tight_layout()
            fig.savefig(os.path.join(self.output_dir, 'evaluation.png'), dpi=120)
            plt.close(fig)
        except ImportError:
            self.get_logger().warn('matplotlib not available; skipped plots')

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
    args = parser.parse_args(argv)

    out = args.output_dir or os.path.expanduser('~/drone_ws/scripts/output')
    state = EvalState(goal=(args.goal_x, args.goal_y, args.goal_z))

    rclpy.init(args=argv)
    node = EvaluateNode(state, args.duration, out)
    rclpy.spin(node)
    return 0


if __name__ == '__main__':
    sys.exit(main())

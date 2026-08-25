#!/usr/bin/env python3
"""Planner contract-conformance checker.

Subscribes the planner-facing side of the plant contract and verifies that a
backend actually honours what `planner_registry` declares about it:

Required channel (all backends):
  /planner/local_goal   PoseStamped frame=map, sustained rate while tracking,
                        monotonic stamps, and the vehicle makes measurable
                        progress toward the goal.

Declared optional channel (`traj_ff`):
  /planner/trajectory_cmd with trajectory_ready=true at a sane rate.

Plus /planner/status liveness.

Run headless against a live stack (see scripts/run_conformance.py):
    ros2 run drone_bringup planner_conformance --planner ego \
        --goal-x 8 --goal-y 6 --window 45
Exit code 0 = PASS, 1 = FAIL. Full JSON verdict on stdout.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from typing import Dict, List

import rclpy
from drone_msgs.msg import PlannerStatus, TrajectoryCommand
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

KNOWN_STATUS_STATES = {
    'INIT', 'WAIT_TARGET', 'GEN_NEW_TRAJ', 'EXEC_TRAJ', 'REPLAN_TRAJ',
    'EMERGENCY_STOP', 'FAIL', 'TRAJ_CMD', 'HOLD', 'EXPLORE', 'FINISHED',
}


class ConformanceChecker(Node):

    def __init__(self) -> None:
        super().__init__('planner_conformance')
        self.declare_parameter('planner_id', 'ego')
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 6.0)
        self.declare_parameter('goal_z', 1.0)
        self.declare_parameter('window_s', 60.0)
        self.declare_parameter('min_local_goal_hz', 3.0)
        self.declare_parameter('min_traj_ff_hz', 5.0)
        self.declare_parameter('min_progress_m', 1.5)

        # Resolve declared channels from the registry (single source of truth).
        try:
            from drone_bringup.planner_registry import PLANNERS
            meta = PLANNERS.get(self.get_parameter('planner_id').value, {})
            self.channels: List[str] = list(meta.get('command_channels', ['local_goal']))
        except Exception:
            self.channels = ['local_goal']

        self.lock = threading.Lock()
        self.local_goals: List[PoseStamped] = []
        self.traj_cmds: List[TrajectoryCommand] = []
        self.statuses: List[PlannerStatus] = []
        self.odom_pos = None
        self.first_odom_pos = None

        self.create_subscription(
            PoseStamped, '/planner/local_goal', self._on_local_goal, 20)
        self.create_subscription(
            TrajectoryCommand, '/planner/trajectory_cmd', self._on_traj_cmd, 20)
        self.create_subscription(
            PlannerStatus, '/planner/status', self._on_status, 20)
        self.create_subscription(Odometry, '/drone/odom', self._on_odom, 20)

    # ------------------------------------------------------------------ callbacks
    def _on_local_goal(self, msg: PoseStamped) -> None:
        with self.lock:
            self.local_goals.append(msg)

    def _on_traj_cmd(self, msg: TrajectoryCommand) -> None:
        with self.lock:
            if msg.trajectory_ready:
                self.traj_cmds.append(msg)

    def _on_status(self, msg: PlannerStatus) -> None:
        with self.lock:
            self.statuses.append(msg)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        with self.lock:
            if self.odom_pos is None:
                self.first_odom_pos = (p.x, p.y)
            self.odom_pos = (p.x, p.y, msg.header.stamp.sec +
                             msg.header.stamp.nanosec * 1e-9)

    # ------------------------------------------------------------------ checks
    def _send_goal(self) -> None:
        pub = self.create_publisher(PoseStamped, '/drone/goal', 10)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = float(self.get_parameter('goal_x').value)
        goal.pose.position.y = float(self.get_parameter('goal_y').value)
        goal.pose.position.z = float(self.get_parameter('goal_z').value)
        goal.pose.orientation.w = 1.0
        # A few repeats so one-shot QoS drops cannot fail the run.
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            pub.publish(goal)
            time.sleep(0.25)

    def run(self) -> Dict:
        self._send_goal()
        window = float(self.get_parameter('window_s').value)
        t_end = time.monotonic() + window
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.2)
            with self.lock:
                pos = self.odom_pos
            if pos is not None:
                d = math.dist(pos[:2], (
                    float(self.get_parameter('goal_x').value),
                    float(self.get_parameter('goal_y').value)))
                if d < 0.35:      # arrived early — no need to burn the window
                    break

        return self._verdict(window)

    def _verdict(self, window: float) -> Dict:
        min_lg = float(self.get_parameter('min_local_goal_hz').value)
        min_ff = float(self.get_parameter('min_traj_ff_hz').value)
        min_prog = float(self.get_parameter('min_progress_m').value)
        gx = float(self.get_parameter('goal_x').value)
        gy = float(self.get_parameter('goal_y').value)

        checks: Dict[str, bool] = {}
        detail: Dict[str, object] = {}

        with self.lock:
            lg = list(self.local_goals)
            ff = list(self.traj_cmds)
            st = list(self.statuses)
            pos = self.odom_pos
            start = self.first_odom_pos

        # 1) local_goal channel (contract minimum)
        n_lg = len(lg)
        rate_ok = n_lg >= max(10, min_lg * window * 0.5)
        checks['local_goal_rate'] = rate_ok
        detail['local_goal_count'] = n_lg
        if lg:
            stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                      for m in lg]
            checks['local_goal_stamps_monotonic'] = all(
                b >= a - 1e-6 for a, b in zip(stamps, stamps[1:]))
            checks['local_goal_frame_map'] = all(
                m.header.frame_id == 'map' for m in lg)
        else:
            checks['local_goal_stamps_monotonic'] = False
            checks['local_goal_frame_map'] = False

        # 2) feedforward channel (declared only)
        if 'traj_ff' in self.channels:
            n_ff = len(ff)
            checks['traj_ff_rate'] = n_ff >= max(10, min_ff * window * 0.3)
            detail['traj_ff_ready_count'] = n_ff
        else:
            checks['traj_ff_rate'] = True       # not declared → not required
            detail['traj_ff_ready_count'] = 0

        # 3) status liveness
        checks['status_liveness'] = len(st) > 0
        bad_states = [m.state for m in st if m.state not in KNOWN_STATUS_STATES]
        checks['status_states_known'] = not bad_states
        detail['status_last'] = st[-1].state if st else ''

        # 4) progress toward goal
        progress_ok = False
        if pos is not None and start is not None:
            d_start = math.dist(start, (gx, gy))
            d_now = math.dist(pos[:2], (gx, gy))
            detail['dist_start'] = round(d_start, 2)
            detail['dist_final'] = round(d_now, 2)
            progress_ok = (d_now <= d_start - min_prog) or d_now < 0.35
        checks['progress_toward_goal'] = progress_ok

        passed = all(checks.values())
        return {'pass': passed, 'checks': checks, 'detail': detail,
                'channels_declared': self.channels}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--planner', default='ego')
    parser.add_argument('--goal-x', type=float, default=8.0)
    parser.add_argument('--goal-y', type=float, default=6.0)
    parser.add_argument('--goal-z', type=float, default=1.0)
    parser.add_argument('--window', type=float, default=60.0)
    ns = parser.parse_args(argv)

    # Standard ROS parameter passthrough: declare_parameter() picks these up.
    ros_args: List[str] = []
    for k, v in (
        ('planner_id', ns.planner),
        ('goal_x', ns.goal_x),
        ('goal_y', ns.goal_y),
        ('goal_z', ns.goal_z),
        ('window_s', ns.window),
    ):
        ros_args += ['--ros-args', '-p', f'{k}:={v}']

    rclpy.init(args=ros_args)
    node = ConformanceChecker()
    verdict = node.run()
    node.destroy_node()
    rclpy.shutdown()

    print(json.dumps(verdict, indent=2))
    print(f"conformance [{'PASS' if verdict['pass'] else 'FAIL'}] "
          f"planner={ns.planner}")
    return 0 if verdict['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

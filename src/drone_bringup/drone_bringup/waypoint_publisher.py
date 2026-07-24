#!/usr/bin/env python3
"""Trajectory input: publish sequential /drone/goal waypoints (square/circle/eight/list).

Supports multi-cycle looping and optional odom-based arrival before advancing.
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def square_waypoints(side: float, z: float) -> List[Tuple[float, float, float]]:
    h = side * 0.5
    return [
        (h, h, z),
        (-h, h, z),
        (-h, -h, z),
        (h, -h, z),
        (h, h, z),
    ]


def circle_waypoints(radius: float, z: float, n: int = 16) -> List[Tuple[float, float, float]]:
    pts = []
    for i in range(n + 1):
        t = 2.0 * math.pi * i / n
        pts.append((radius * math.cos(t), radius * math.sin(t), z))
    return pts


def eight_waypoints(scale: float, z: float, n: int = 32) -> List[Tuple[float, float, float]]:
    pts = []
    for i in range(n + 1):
        t = 2.0 * math.pi * i / n
        x = scale * math.sin(t)
        y = scale * math.sin(t) * math.cos(t)
        pts.append((x, y, z))
    return pts


class WaypointPublisher(Node):
    def __init__(
        self,
        waypoints: List[Tuple[float, float, float]],
        hold_sec: float,
        topic: str,
        cycles: int = 1,
        wait_arrival: bool = False,
        arrival_tol: float = 1.0,
        max_hold: float = 90.0,
        odom_topic: str = '/drone/odom',
    ):
        super().__init__('waypoint_publisher')
        if not waypoints:
            raise ValueError('waypoints must be non-empty')
        self.waypoints = list(waypoints)
        self.hold_sec = float(hold_sec)
        self.cycles = int(cycles)  # 0 = infinite
        self.wait_arrival = bool(wait_arrival)
        self.arrival_tol = float(arrival_tol)
        self.max_hold = float(max_hold)
        self.idx = 0
        self.cycle_i = 0
        self._goal_t0: Optional[float] = None
        self._pos: Optional[Tuple[float, float, float]] = None
        self._finished = False
        self.pub = self.create_publisher(PoseStamped, topic, 10)
        self._repub_period = 0.5
        self._repub_budget = 6  # bursts per new WP (covers DDS late-join)
        self._repubs_left = 0
        self._last_repub_t: Optional[float] = None
        if self.wait_arrival:
            self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
            # Short poll; advance on arrival or max_hold.
            self.create_timer(0.2, self._tick)
            # First goal after brief warm-up (matches plant / FP bring-up).
            self.create_timer(0.5, self._start_once)
        else:
            # Legacy: first goal after hold_sec, then every hold_sec.
            self.create_timer(self.hold_sec, self._tick)
        self._started = False
        cyc_s = '∞' if self.cycles <= 0 else str(self.cycles)
        mode = 'arrival' if self.wait_arrival else f'fixed_hold={self.hold_sec}s'
        self.get_logger().info(
            f'Waypoint publisher: {len(self.waypoints)} pts/cycle × {cyc_s} cycles, '
            f'mode={mode}, topic={topic}'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _make_msg(self, x: float, y: float, z: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        return msg

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pos = (float(p.x), float(p.y), float(p.z))

    def _start_once(self) -> None:
        if self._started or self._finished:
            return
        self._started = True
        self._publish_current()

    def _mission_complete_after_current(self) -> bool:
        """True if finishing the current WP ends the mission."""
        if self.cycles <= 0:
            return False
        at_last = self.idx >= len(self.waypoints) - 1
        on_last_cycle = self.cycle_i >= self.cycles - 1
        return at_last and on_last_cycle

    def _publish_current(self, *, log: bool = True, reset_timer: bool = True) -> None:
        x, y, z = self.waypoints[self.idx]
        total = len(self.waypoints)
        cyc = self.cycle_i + 1
        cyc_s = f'{cyc}' if self.cycles <= 0 else f'{cyc}/{self.cycles}'
        self.pub.publish(self._make_msg(x, y, z))
        now = self._now()
        if reset_timer or self._goal_t0 is None:
            self._goal_t0 = now
            # Fresh WP: allow a short burst of re-publishes, then stop so EGO
            # is not forced into continuous REPLAN_TRAJ.
            self._repubs_left = max(0, self._repub_budget - 1)
        self._last_repub_t = now
        if log:
            self.get_logger().info(
                f'Waypoint cycle {cyc_s} {self.idx + 1}/{total}: ({x:.2f}, {y:.2f}, {z:.2f})'
            )

    def _finish(self) -> None:
        if self._finished:
            return
        self._finished = True
        self.get_logger().info('Waypoint sequence complete')
        self.create_timer(0.2, lambda: rclpy.shutdown())

    def _advance_or_finish(self) -> None:
        if self._mission_complete_after_current():
            self._finish()
            return
        self.idx += 1
        if self.idx >= len(self.waypoints):
            self.idx = 0
            self.cycle_i += 1
            self.get_logger().info(
                f'Starting cycle {self.cycle_i + 1}'
                + ('' if self.cycles <= 0 else f'/{self.cycles}')
            )
        self._publish_current()

    def _tick(self) -> None:
        if self._finished:
            return
        if not self.wait_arrival:
            # Fixed hold: each timer fire publishes next (or first).
            if not self._started:
                self._started = True
                self._publish_current()
                return
            self._advance_or_finish()
            return

        # Arrival mode: wait until near WP or max_hold.
        if not self._started or self._goal_t0 is None:
            return
        x, y, _z = self.waypoints[self.idx]
        arrived = False
        if self._pos is not None:
            arrived = math.hypot(self._pos[0] - x, self._pos[1] - y) <= self.arrival_tol
        now = self._now()
        elapsed = now - self._goal_t0
        # Short burst re-publish so a single lost DDS sample cannot stall EGO.
        if (
            not arrived
            and self._repubs_left > 0
            and self._repub_period > 0.0
            and (self._last_repub_t is None or (now - self._last_repub_t) >= self._repub_period)
        ):
            self._publish_current(log=False, reset_timer=False)
            self._repubs_left -= 1
        timed_out = elapsed >= self.max_hold
        if not (arrived or timed_out):
            return
        if timed_out and not arrived:
            self.get_logger().warn(
                f'Timeout ({self.max_hold:.0f}s) advancing past '
                f'({x:.1f},{y:.1f}) without arrival'
            )
        self._advance_or_finish()


def parse_list(s: str) -> List[Tuple[float, float, float]]:
    pts = []
    for token in s.split(';'):
        token = token.strip()
        if not token:
            continue
        parts = [float(v) for v in token.split(',')]
        if len(parts) != 3:
            raise ValueError(f'Expected x,y,z got: {token}')
        pts.append((parts[0], parts[1], parts[2]))
    return pts


def _normalize_argv(argv: Optional[List[str]]) -> Optional[List[str]]:
    """Rewrite `--list -8,...` → `--list=-8,...` so argparse keeps the value."""
    if argv is None:
        return None
    out: List[str] = []
    i = 0
    while i < len(argv):
        if argv[i] == '--list' and i + 1 < len(argv) and not argv[i + 1].startswith('--'):
            out.append(f'--list={argv[i + 1]}')
            i += 2
            continue
        out.append(argv[i])
        i += 1
    return out


def main(argv=None):
    cli = _normalize_argv(argv if argv is not None else sys.argv[1:])
    parser = argparse.ArgumentParser(description='Publish waypoint sequence to /drone/goal')
    parser.add_argument('--pattern', choices=['square', 'circle', 'eight', 'list'], default='square')
    parser.add_argument('--z', type=float, default=1.5)
    parser.add_argument('--side', type=float, default=2.0, help='Square side length [m]')
    parser.add_argument('--radius', type=float, default=1.5, help='Circle radius [m]')
    parser.add_argument('--scale', type=float, default=1.5, help='Figure-8 scale [m]')
    parser.add_argument('--hold', type=float, default=8.0,
                        help='Hold time per waypoint [s] (fixed mode)')
    parser.add_argument('--list', dest='waypoint_list', type=str, default='',
                        help='Semicolon-separated x,y,z triples (prefer --list=x,y,z;...)')
    parser.add_argument('--topic', type=str, default='/drone/goal')
    parser.add_argument('--cycles', type=int, default=1,
                        help='Number of full loops (0 = infinite)')
    parser.add_argument('--wait-arrival', action='store_true',
                        help='Advance when within --arrival-tol of the current WP')
    parser.add_argument('--arrival-tol', type=float, default=1.0,
                        help='XY arrival tolerance [m] when --wait-arrival')
    parser.add_argument('--max-hold', type=float, default=90.0,
                        help='Max seconds per WP before forcing advance (--wait-arrival)')
    parser.add_argument('--odom-topic', type=str, default='/drone/odom')
    args = parser.parse_args(cli)

    if args.pattern == 'square':
        wps = square_waypoints(args.side, args.z)
    elif args.pattern == 'circle':
        wps = circle_waypoints(args.radius, args.z)
    elif args.pattern == 'eight':
        wps = eight_waypoints(args.scale, args.z)
    else:
        wps = parse_list(args.waypoint_list)
        if not wps:
            print('Error: --pattern list requires --list "x,y,z;..."', file=sys.stderr)
            return 1

    rclpy.init(args=None)
    node = WaypointPublisher(
        wps, args.hold, args.topic,
        cycles=args.cycles,
        wait_arrival=args.wait_arrival,
        arrival_tol=args.arrival_tol,
        max_hold=args.max_hold,
        odom_topic=args.odom_topic,
    )
    rclpy.spin(node)
    return 0


if __name__ == '__main__':
    sys.exit(main())

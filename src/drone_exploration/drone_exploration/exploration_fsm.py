"""Exploration FSM: RViz /drone/goal triggers; publish sequential /exploration/nav_goal."""

from __future__ import annotations

from enum import Enum, auto
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class State(Enum):
    IDLE = auto()
    EXPLORE = auto()
    FINISHED = auto()


class ExplorationFsm(Node):
    def __init__(self) -> None:
        super().__init__('exploration_fsm')
        self.declare_parameter('cruise_z', 1.5)
        self.declare_parameter('arrive_tol', 0.70)
        self.declare_parameter('replan_period', 1.0)
        self.declare_parameter('min_goal_sep', 2.5)
        self.declare_parameter('max_goals', 40)
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter('trigger_topic', '/drone/goal')
        self.declare_parameter('nav_goal_topic', '/exploration/nav_goal')
        self.declare_parameter('frontiers_topic', '/exploration/frontiers')
        self.declare_parameter('status_topic', '/exploration/status')
        self.declare_parameter('frame_id', 'map')
        # Bootstrap only along explore_dir when fog has not yet found frontiers.
        self.declare_parameter('seed_viewpoints', True)
        self.declare_parameter('explore_dir_x', 1.0)
        self.declare_parameter('explore_dir_y', 0.0)
        self.declare_parameter('min_travel_m', 1.5)
        self.declare_parameter('goal_cooldown', 4.0)
        self.declare_parameter('prefer_forward_weight', 2.5)
        self.declare_parameter('prefer_size_weight', 0.12)
        self.declare_parameter('box_min_x', -1.0)
        self.declare_parameter('box_min_y', -1.0)
        self.declare_parameter('box_min_z', 0.0)
        self.declare_parameter('box_max_x', 21.0)
        self.declare_parameter('box_max_y', 11.0)
        self.declare_parameter('box_max_z', 3.0)

        self.cruise_z = float(self.get_parameter('cruise_z').value)
        self.arrive_tol = float(self.get_parameter('arrive_tol').value)
        self.min_goal_sep = float(self.get_parameter('min_goal_sep').value)
        self.max_goals = int(self.get_parameter('max_goals').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.seed_viewpoints = bool(self.get_parameter('seed_viewpoints').value)
        self.min_travel_m = float(self.get_parameter('min_travel_m').value)
        self.goal_cooldown = float(self.get_parameter('goal_cooldown').value)
        self.prefer_forward_weight = float(
            self.get_parameter('prefer_forward_weight').value)
        self.prefer_size_weight = float(
            self.get_parameter('prefer_size_weight').value)
        edir = np.array([
            float(self.get_parameter('explore_dir_x').value),
            float(self.get_parameter('explore_dir_y').value),
        ], dtype=np.float64)
        n = float(np.linalg.norm(edir))
        self._explore_dir = edir / n if n > 1e-6 else np.array([1.0, 0.0])
        self._box: Tuple[float, float, float, float] = (
            float(self.get_parameter('box_min_x').value),
            float(self.get_parameter('box_min_y').value),
            float(self.get_parameter('box_max_x').value),
            float(self.get_parameter('box_max_y').value),
        )

        self.state = State.IDLE
        self._odom: Optional[np.ndarray] = None
        self._frontiers: List[Tuple[np.ndarray, float]] = []
        self._current: Optional[np.ndarray] = None
        self._goal_origin: Optional[np.ndarray] = None
        self._goal_sent_at: Optional[float] = None
        self._visited: List[np.ndarray] = []
        self._goal_count = 0
        self._idle_ticks = 0

        nav_topic = str(self.get_parameter('nav_goal_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        self._goal_pub = self.create_publisher(PoseStamped, nav_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)

        odom_topic = str(self.get_parameter('odom_topic').value)
        trigger_topic = str(self.get_parameter('trigger_topic').value)
        frontiers_topic = str(self.get_parameter('frontiers_topic').value)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.create_subscription(PoseStamped, trigger_topic, self._on_trigger, 10)
        self.create_subscription(PoseArray, frontiers_topic, self._on_frontiers, 10)

        period = float(self.get_parameter('replan_period').value)
        self.create_timer(max(0.2, period), self._tick)
        self._publish_status('IDLE waiting for /drone/goal trigger')
        self.get_logger().info(
            f'exploration_fsm: trigger={trigger_topic} → nav={nav_topic} '
            f'dir=({self._explore_dir[0]:.2f},{self._explore_dir[1]:.2f})')

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._odom = np.array([p.x, p.y, p.z], dtype=np.float64)

    def _on_frontiers(self, msg: PoseArray) -> None:
        out: List[Tuple[np.ndarray, float]] = []
        for p in msg.poses:
            xyz = np.array([p.position.x, p.position.y, p.position.z], dtype=np.float64)
            size = float(p.orientation.x) if abs(p.orientation.x) > 1e-6 else 1.0
            out.append((xyz, size))
        self._frontiers = out

    def _on_trigger(self, _msg: PoseStamped) -> None:
        if self.state == State.EXPLORE:
            self.get_logger().info('Ignore explore trigger (already running)')
            return
        if self.state == State.FINISHED:
            self._visited.clear()
            self._goal_count = 0
        self.state = State.EXPLORE
        self._current = None
        self._goal_origin = None
        self._goal_sent_at = None
        self._idle_ticks = 0
        self._publish_status('EXPLORE triggered')
        self.get_logger().info('Exploration started (RViz goal = trigger only)')
        self._pick_and_send(force=True)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _in_box(self, xyz: np.ndarray, margin: float = 0.5) -> bool:
        xmin, ymin, xmax, ymax = self._box
        return (
            xmin + margin <= float(xyz[0]) <= xmax - margin
            and ymin + margin <= float(xyz[1]) <= ymax - margin
        )

    def _clamp_to_box(self, xyz: np.ndarray, margin: float = 0.8) -> np.ndarray:
        xmin, ymin, xmax, ymax = self._box
        out = xyz.copy()
        out[0] = float(np.clip(out[0], xmin + margin, xmax - margin))
        out[1] = float(np.clip(out[1], ymin + margin, ymax - margin))
        out[2] = self.cruise_z
        return out

    def _send_goal(self, xyz: np.ndarray) -> None:
        xyz = self._clamp_to_box(xyz)
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = self.frame_id
        g.pose.position.x = float(xyz[0])
        g.pose.position.y = float(xyz[1])
        g.pose.position.z = float(self.cruise_z)
        g.pose.orientation.w = 1.0
        self._goal_pub.publish(g)
        self._current = xyz.copy()
        self._goal_origin = self._odom.copy() if self._odom is not None else None
        self._goal_sent_at = self.get_clock().now().nanoseconds * 1e-9
        self._goal_count += 1
        self.get_logger().info(
            f'Nav goal #{self._goal_count}: '
            f'({g.pose.position.x:.1f}, {g.pose.position.y:.1f}, {g.pose.position.z:.1f})')

    def _too_close_visited(self, xyz: np.ndarray) -> bool:
        for v in self._visited:
            if np.hypot(xyz[0] - v[0], xyz[1] - v[1]) < self.min_goal_sep:
                return True
        return False

    def _forward_seeds(self) -> List[np.ndarray]:
        """Line-of-march seeds only — radial rings caused orbit / scatter."""
        if self._odom is None:
            return []
        ox, oy = float(self._odom[0]), float(self._odom[1])
        dx, dy = float(self._explore_dir[0]), float(self._explore_dir[1])
        cands: List[np.ndarray] = []
        for radius in (5.0, 7.0, 9.0):
            cands.append(self._clamp_to_box(np.array([
                ox + dx * radius, oy + dy * radius, self.cruise_z,
            ])))
        return [c for c in cands if self._in_box(c)]

    def _score(self, xyz: np.ndarray, cluster_size: float, dist: float) -> float:
        if self._odom is None:
            return dist
        delta = xyz[:2] - self._odom[:2]
        dn = float(np.linalg.norm(delta))
        forward = 0.0
        if dn > 1e-3:
            forward = float(np.dot(delta / dn, self._explore_dir))
        # Lower is better: medium range, large frontier, forward along corridor.
        return (
            abs(dist - 5.5)
            + 0.04 * dist
            - self.prefer_size_weight * cluster_size
            - self.prefer_forward_weight * max(0.0, forward)
            + 2.0 * max(0.0, -forward)
        )

    def _candidates(self) -> List[Tuple[np.ndarray, float]]:
        cands: List[Tuple[np.ndarray, float]] = [
            (c, size) for c, size in self._frontiers if self._in_box(c)
        ]
        if not cands and self.seed_viewpoints:
            for seed in self._forward_seeds():
                cands.append((seed, 1.0))
        return cands

    def _pick_and_send(self, force: bool = False) -> bool:
        if self._odom is None:
            return False
        if self._goal_count >= self.max_goals:
            self.state = State.FINISHED
            self._publish_status('FINISHED max_goals')
            return False

        min_range = max(2.5, 0.6 * self.min_goal_sep)
        scored: List[Tuple[float, np.ndarray]] = []
        for c, size in self._candidates():
            if self._too_close_visited(c):
                continue
            dist = float(np.hypot(c[0] - self._odom[0], c[1] - self._odom[1]))
            if dist < min_range:
                continue
            if self._current is not None and not force:
                if np.hypot(c[0] - self._current[0], c[1] - self._current[1]) < 1.0:
                    continue
            scored.append((self._score(c, size, dist), c))
        if not scored:
            return False
        scored.sort(key=lambda t: t[0])
        choice = scored[0][1]
        if self._current is not None:
            self._visited.append(self._current.copy())
        self._send_goal(choice)
        self._publish_status(
            f'EXPLORE goal={self._goal_count} frontiers={len(self._frontiers)}')
        return True

    def _goal_elapsed(self) -> float:
        if self._goal_sent_at is None:
            return 0.0
        now = self.get_clock().now().nanoseconds * 1e-9
        return max(0.0, now - self._goal_sent_at)

    def _traveled_since_goal(self) -> float:
        if self._goal_origin is None or self._odom is None:
            return 0.0
        return float(np.hypot(
            self._odom[0] - self._goal_origin[0],
            self._odom[1] - self._goal_origin[1]))

    def _tick(self) -> None:
        if self.state != State.EXPLORE or self._odom is None:
            return

        if self._current is not None:
            d = float(np.hypot(
                self._odom[0] - self._current[0],
                self._odom[1] - self._current[1]))
            if d > self.arrive_tol:
                return
            if self._goal_elapsed() < self.goal_cooldown:
                return
            if self._traveled_since_goal() < self.min_travel_m:
                return
            self._visited.append(self._current.copy())
            self._current = None

        if not self._pick_and_send():
            self._idle_ticks += 1
            if self._idle_ticks >= 5 and not self._frontiers:
                self.state = State.FINISHED
                self._publish_status('FINISHED no frontiers')
                self.get_logger().info('Exploration finished (no frontiers left)')
            elif self._idle_ticks >= 15:
                self.state = State.FINISHED
                self._publish_status('FINISHED stalled')
        else:
            self._idle_ticks = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorationFsm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

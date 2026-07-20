#!/usr/bin/env python3
"""
Formation coordinator: followers track leader pose + formation offsets.

Kept for fixed 3-UAV demos (1 leader + 2 followers):
  line   — side-by-side left / right of leader
  column — followers trail in a single file behind leader
  v      — rear wedge (classic V)

Triangle / diamond removed: with only two followers they duplicate V or
need a fourth aircraft.
"""

from __future__ import annotations

import argparse
import math
from typing import Dict, List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

# Valid shapes for the 3-UAV stack.
FORMATIONS = ('line', 'column', 'v')


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def offsets_for(formation: str, spacing: float) -> List[Tuple[float, float]]:
    """Body-frame offsets (x forward, y left) for followers [1], [2]."""
    s = spacing
    f = formation.lower()
    # Legacy aliases → V (same footprint as old triangle).
    if f in ('triangle', 'wedge'):
        f = 'v'
    if f == 'diamond':
        # Incomplete with 2 followers; fall back to column-ish trail + side.
        f = 'column'
    if f == 'line':
        return [(0.0, s), (0.0, -s)]
    if f == 'column':
        return [(-s, 0.0), (-2.0 * s, 0.0)]
    if f == 'v':
        return [(-s, 0.75 * s), (-s, -0.75 * s)]
    return [(-s, 0.75 * s), (-s, -0.75 * s)]


class FormationCoordinator(Node):
    def __init__(
        self,
        leader: str,
        followers: List[str],
        formation: str,
        spacing: float,
        z: float,
        rate_hz: float,
    ):
        super().__init__('formation_coordinator')
        self.leader = leader
        self.followers = followers
        self.formation = formation
        self.spacing = spacing
        self.z = z
        self.leader_odom: Odometry | None = None

        self.create_subscription(
            Odometry, f'/{leader}/drone/odom', self._on_leader, 20)

        self.pubs: Dict[str, any] = {}
        for ns in followers:
            self.pubs[ns] = self.create_publisher(
                PoseStamped, f'/{ns}/drone/goal', 10)

        self.offs = offsets_for(formation, spacing)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'Formation "{formation}" leader=/{leader} followers={followers} '
            f'spacing={spacing:.2f}m'
        )

    def _on_leader(self, msg: Odometry):
        self.leader_odom = msg

    def _tick(self):
        if self.leader_odom is None:
            return
        p = self.leader_odom.pose.pose.position
        yaw = yaw_from_quat(self.leader_odom.pose.pose.orientation)
        c, s = math.cos(yaw), math.sin(yaw)

        for i, ns in enumerate(self.followers):
            if i >= len(self.offs):
                break
            ox, oy = self.offs[i]
            # Rotate body offset into map frame (x forward, y left).
            dx = c * ox - s * oy
            dy = s * ox + c * oy
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = p.x + dx
            msg.pose.position.y = p.y + dy
            msg.pose.position.z = self.z
            msg.pose.orientation = self.leader_odom.pose.pose.orientation
            self.pubs[ns].publish(msg)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Multi-drone formation coordinator')
    parser.add_argument('--leader', default='uav0')
    parser.add_argument('--followers', default='uav1,uav2',
                        help='Comma-separated follower namespaces')
    parser.add_argument('--formation', default='v', choices=list(FORMATIONS))
    parser.add_argument('--spacing', type=float, default=1.5)
    parser.add_argument('--z', type=float, default=1.5)
    parser.add_argument('--rate', type=float, default=10.0)
    args = parser.parse_args(argv)

    followers = [x.strip() for x in args.followers.split(',') if x.strip()]
    rclpy.init(args=None)
    node = FormationCoordinator(
        args.leader, followers, args.formation, args.spacing, args.z, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

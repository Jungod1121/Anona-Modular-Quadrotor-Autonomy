#!/usr/bin/env python3
"""
Formation coordinator: followers track leader pose + formation offsets.

formation types:
  line     — side-by-side left/right of leader
  column   — followers trail behind leader
  v        — V / wedge behind leader
  triangle — same footprint as a closed triangle (3 drones)
  diamond  — diamond corners (needs 3 followers; with 2 followers uses front+side)
"""

from __future__ import annotations

import argparse
import math
from typing import Dict, List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def offsets_for(formation: str, spacing: float) -> List[Tuple[float, float]]:
    """Body-frame offsets (x forward, y left) for followers [1], [2], ..."""
    s = spacing
    f = formation.lower()
    if f == 'line':
        return [(0.0, s), (0.0, -s)]
    if f == 'column':
        return [(-s, 0.0), (-2.0 * s, 0.0)]
    if f in ('v', 'wedge'):
        return [(-s, 0.75 * s), (-s, -0.75 * s)]
    if f == 'triangle':
        return [(-s, 0.9 * s), (-s, -0.9 * s)]
    if f == 'diamond':
        return [(-s, 0.0), (0.0, s), (0.0, -s)]
    # default: shallow V
    return [(-s, 0.7 * s), (-s, -0.7 * s)]


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
    parser.add_argument('--formation', default='v',
                        choices=['line', 'column', 'v', 'triangle', 'diamond'])
    parser.add_argument('--spacing', type=float, default=1.5)
    parser.add_argument('--z', type=float, default=1.5)
    parser.add_argument('--rate', type=float, default=10.0)
    args = parser.parse_args(argv)

    followers = [x.strip() for x in args.followers.split(',') if x.strip()]
    rclpy.init(args=argv)
    node = FormationCoordinator(
        args.leader, followers, args.formation, args.spacing, args.z, args.rate)
    rclpy.spin(node)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

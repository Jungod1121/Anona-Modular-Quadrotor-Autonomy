#!/usr/bin/env python3
"""Trajectory input: publish sequential /drone/goal waypoints (square/circle/eight/list)."""

import argparse
import math
import sys
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
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
    def __init__(self, waypoints: List[Tuple[float, float, float]], hold_sec: float, topic: str):
        super().__init__('waypoint_publisher')
        self.waypoints = waypoints
        self.hold_sec = hold_sec
        self.idx = 0
        self.pub = self.create_publisher(PoseStamped, topic, 10)
        self.pub.publish(self._make_msg(0.0, 0.0, 1.5))  # warm-up
        self.timer = self.create_timer(hold_sec, self._step)
        self.get_logger().info(
            f'Waypoint publisher: {len(waypoints)} points, hold={hold_sec}s, topic={topic}'
        )

    def _make_msg(self, x: float, y: float, z: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        return msg

    def _step(self):
        if self.idx >= len(self.waypoints):
            self.get_logger().info('Waypoint sequence complete')
            self.timer.cancel()
            self.create_timer(0.2, lambda: rclpy.shutdown())
            return
        x, y, z = self.waypoints[self.idx]
        self.pub.publish(self._make_msg(x, y, z))
        self.get_logger().info(f'Waypoint {self.idx + 1}/{len(self.waypoints)}: ({x:.2f}, {y:.2f}, {z:.2f})')
        self.idx += 1


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


def main(argv=None):
    parser = argparse.ArgumentParser(description='Publish waypoint sequence to /drone/goal')
    parser.add_argument('--pattern', choices=['square', 'circle', 'eight', 'list'], default='square')
    parser.add_argument('--z', type=float, default=1.5)
    parser.add_argument('--side', type=float, default=2.0, help='Square side length [m]')
    parser.add_argument('--radius', type=float, default=1.5, help='Circle radius [m]')
    parser.add_argument('--scale', type=float, default=1.5, help='Figure-8 scale [m]')
    parser.add_argument('--hold', type=float, default=8.0, help='Hold time per waypoint [s]')
    parser.add_argument('--list', dest='waypoint_list', type=str, default='',
                         help='Semicolon-separated x,y,z triples')
    parser.add_argument('--topic', type=str, default='/drone/goal')
    args = parser.parse_args(argv)

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

    rclpy.init(args=argv)
    node = WaypointPublisher(wps, args.hold, args.topic)
    rclpy.spin(node)
    return 0


if __name__ == '__main__':
    sys.exit(main())

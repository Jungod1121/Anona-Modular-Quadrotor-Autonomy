#!/usr/bin/env python3
"""Smoke Path D: local cloud + exploration trigger → nav goals."""

from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def main() -> int:
    rclpy.init()
    node = rclpy.create_node('fuel_smoke')
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )
    got = {'local': 0, 'nav': 0, 'status': ''}

    def on_local(msg: PointCloud2) -> None:
        got['local'] = int(msg.width)

    def on_nav(msg: PoseStamped) -> None:
        got['nav'] += 1
        print(
            f"NAV_GOAL {msg.pose.position.x:.2f},"
            f"{msg.pose.position.y:.2f},{msg.pose.position.z:.2f}",
            flush=True,
        )

    def on_status(msg: String) -> None:
        got['status'] = msg.data

    node.create_subscription(PointCloud2, '/map/obstacles_local', on_local, qos)
    node.create_subscription(PoseStamped, '/exploration/nav_goal', on_nav, 10)
    node.create_subscription(String, '/exploration/status', on_status, 10)
    trig = node.create_publisher(PoseStamped, '/drone/goal', 10)

    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.2)

    g = PoseStamped()
    g.header.frame_id = 'map'
    g.pose.position.x = 5.0
    g.pose.position.y = 5.0
    g.pose.position.z = 1.5
    g.pose.orientation.w = 1.0
    for _ in range(1):
        g.header.stamp = node.get_clock().now().to_msg()
        trig.publish(g)
        rclpy.spin_once(node, timeout_sec=0.1)
    print('TRIGGERED', flush=True)

    end = time.time() + 20.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.25)

    print(
        f"RESULT local_width={got['local']} nav_goals={got['nav']} "
        f"status={got['status']!r}",
        flush=True,
    )
    ok = got['local'] > 0 and got['nav'] >= 1
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

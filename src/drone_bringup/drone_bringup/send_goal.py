#!/usr/bin/env python3
"""Publish PoseStamped goal (RViz-compatible volatile/reliable QoS)."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class SendGoal(Node):
    def __init__(self, x: float, y: float, z: float, yaw: float, topic: str, repeats: int):
        super().__init__('send_goal')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(PoseStamped, topic, qos)
        self.x, self.y, self.z, self.yaw = x, y, z, yaw
        self.left = max(1, repeats)
        # Wait briefly for DDS discovery so the first publish is not lost.
        deadline = time.time() + 3.0
        while time.time() < deadline and self.pub.get_subscription_count() < 1:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.timer = self.create_timer(0.35, self._publish)

    def _make_msg(self) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _publish(self):
        self.pub.publish(self._make_msg())
        self.get_logger().info(
            f'Published goal ({self.x:.2f}, {self.y:.2f}, {self.z:.2f}) '
            f'yaw={self.yaw:.2f} remaining={self.left - 1} '
            f'subs={self.pub.get_subscription_count()}'
        )
        self.left -= 1
        if self.left <= 0:
            self.timer.cancel()
            self.create_timer(0.2, lambda: rclpy.shutdown())


def main(argv=None):
    parser = argparse.ArgumentParser(description='Send PoseStamped goal')
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=1.5)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--topic', type=str, default='/drone/goal')
    parser.add_argument('--repeats', type=int, default=1,
                        help='Republish count (default 1)')
    args = parser.parse_args(argv)

    rclpy.init(args=argv)
    node = SendGoal(args.x, args.y, args.z, args.yaw, args.topic, args.repeats)
    rclpy.spin(node)
    return 0


if __name__ == '__main__':
    sys.exit(main())

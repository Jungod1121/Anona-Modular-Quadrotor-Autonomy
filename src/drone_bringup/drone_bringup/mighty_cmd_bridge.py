#!/usr/bin/env python3
"""Bridge MIGHTY dynus Goal / Trajectory → plant /planner/* contract."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from drone_msgs.msg import TrajectoryCommand

try:
    from dynus_interfaces.msg import Goal, Trajectory
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        'dynus_interfaces not found — build mighty_vendor first: '
        'colcon build --packages-up-to mighty'
    ) from exc


PATH_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class MightyCmdBridge(Node):
    def __init__(self) -> None:
        super().__init__('mighty_cmd_bridge')
        self.declare_parameter('goal_cmd_topic', 'goal')
        self.declare_parameter('trajectory_topic', 'trajectory')
        self.declare_parameter('cruise_height', 1.0)

        goal_cmd = self.get_parameter('goal_cmd_topic').value
        traj_topic = self.get_parameter('trajectory_topic').value

        self.traj_pub = self.create_publisher(TrajectoryCommand, '/planner/trajectory_cmd', 10)
        self.local_pub = self.create_publisher(PoseStamped, '/planner/local_goal', 10)
        self.path_pub = self.create_publisher(Path, '/planner/trajectory', PATH_QOS)

        self.create_subscription(Goal, goal_cmd, self.on_goal_cmd, 50)
        self.create_subscription(Trajectory, traj_topic, self.on_trajectory, 10)

        self.get_logger().info(
            f'MIGHTY bridge: {goal_cmd}/{traj_topic} → /planner/local_goal + trajectory_cmd')

    def on_goal_cmd(self, msg: Goal) -> None:
        stamp = msg.header.stamp if msg.header.stamp.sec or msg.header.stamp.nanosec else (
            self.get_clock().now().to_msg())
        local = PoseStamped()
        local.header.stamp = stamp
        local.header.frame_id = msg.header.frame_id or 'map'
        local.pose.position.x = float(msg.p.x)
        local.pose.position.y = float(msg.p.y)
        local.pose.position.z = float(msg.p.z)
        if local.pose.position.z < 0.5:
            local.pose.position.z = float(self.get_parameter('cruise_height').value)
        local.pose.orientation.w = 1.0
        self.local_pub.publish(local)

        tc = TrajectoryCommand()
        tc.header = local.header
        tc.position.x = local.pose.position.x
        tc.position.y = local.pose.position.y
        tc.position.z = local.pose.position.z
        tc.velocity.x = float(msg.v.x)
        tc.velocity.y = float(msg.v.y)
        tc.velocity.z = float(msg.v.z)
        tc.acceleration.x = float(msg.a.x)
        tc.acceleration.y = float(msg.a.y)
        tc.acceleration.z = float(msg.a.z)
        tc.yaw = float(msg.yaw)
        tc.yaw_dot = float(msg.dyaw)
        tc.trajectory_ready = True
        self.traj_pub.publish(tc)

    def on_trajectory(self, msg: Trajectory) -> None:
        if not msg.goals:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = msg.header.frame_id or 'map'
        for g in msg.goals:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(g.p.x)
            ps.pose.position.y = float(g.p.y)
            ps.pose.position.z = float(g.p.z)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)


def main() -> None:
    rclpy.init()
    node = MightyCmdBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

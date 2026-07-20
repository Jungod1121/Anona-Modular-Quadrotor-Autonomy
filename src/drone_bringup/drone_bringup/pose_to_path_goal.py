#!/usr/bin/env python3
"""Convert PoseStamped goals to Path for Fast-Planner kino_replan FSM.

Fast-Planner listens on /waypoint_generator/waypoints (nav_msgs/Path).
Our plant / dashboard publishes geometry_msgs/PoseStamped on /drone/goal.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class PoseToPathGoal(Node):
    def __init__(self) -> None:
        super().__init__('pose_to_path_goal')
        self.declare_parameter('pose_topic', '/drone/goal')
        self.declare_parameter('path_topic', '/waypoint_generator/waypoints')
        self.declare_parameter('force_z', -1.0)  # <0 → keep pose z

        pose_topic = self.get_parameter('pose_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.force_z = float(self.get_parameter('force_z').value)

        self.pub = self.create_publisher(Path, path_topic, 10)
        self.create_subscription(PoseStamped, pose_topic, self.on_pose, 10)
        self.get_logger().info(f'Bridging {pose_topic} → {path_topic}')

    def on_pose(self, msg: PoseStamped) -> None:
        path = Path()
        path.header = msg.header
        if not path.header.frame_id:
            path.header.frame_id = 'world'
        pose = PoseStamped()
        pose.header = path.header
        pose.pose = msg.pose
        if self.force_z >= 0.0:
            pose.pose.position.z = self.force_z
        path.poses = [pose]
        self.pub.publish(path)


def main() -> None:
    rclpy.init()
    node = PoseToPathGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

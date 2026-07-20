#!/usr/bin/env python3
"""Crop EGO occupancy_inflate for RViz only — does not affect planning."""

from __future__ import annotations

from typing import Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class InflateVisCrop(Node):
    def __init__(self) -> None:
        super().__init__('inflate_vis_crop')
        self.declare_parameter('input_topic', '/drone_0_grid/grid_map/occupancy_inflate')
        self.declare_parameter('output_topic', '/drone_0_grid/grid_map/occupancy_inflate_vis')
        self.declare_parameter('odom_topic', '/drone/odom')
        # Path B local_update_range_x was 20 m → 1/10 → 2.0 m half-radius.
        self.declare_parameter('vis_radius', 2.0)
        self.declare_parameter('frame_id', 'map')

        self._radius = float(self.get_parameter('vis_radius').value)
        self._frame = str(self.get_parameter('frame_id').value)
        self._odom: Optional[Tuple[float, float, float]] = None

        in_topic = str(self.get_parameter('input_topic').value)
        out_topic = str(self.get_parameter('output_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)

        self._pub = self.create_publisher(PointCloud2, out_topic, 10)
        self.create_subscription(PointCloud2, in_topic, self._on_cloud, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.get_logger().info(
            f'inflate_vis_crop: {in_topic} → {out_topic} radius={self._radius:.2f} m')

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._odom = (float(p.x), float(p.y), float(p.z))

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self._odom is None:
            return
        ox, oy, oz = self._odom
        r2 = self._radius * self._radius
        pts = []
        for x, y, z in point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True):
            dx = float(x) - ox
            dy = float(y) - oy
            dz = float(z) - oz
            if dx * dx + dy * dy + dz * dz <= r2:
                pts.append((float(x), float(y), float(z)))
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame
        self._pub.publish(point_cloud2.create_cloud_xyz32(header, pts))


def main() -> None:
    rclpy.init()
    node = InflateVisCrop()
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

#!/usr/bin/env python3
"""Official-style local sensing for Path B (EGO).

Crops /map_generator/global_cloud to sensing_horizon around the drone
(like pcl_render_node). Publishes with TRANSIENT_LOCAL so EGO's
grid_map/cloud subscription accepts the stream.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def _latched_qos() -> QoSProfile:
    # Must match GridMap::indep_cloud_sub_ (transient_local + reliable).
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


class LocalSenseCloud(Node):
    def __init__(self) -> None:
        super().__init__('local_sense_cloud')
        self.declare_parameter('global_cloud_topic', '/map_generator/global_cloud')
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter('local_cloud_topic', '/drone_0_pcl_render_node/cloud')
        self.declare_parameter('sensing_horizon', 5.0)
        self.declare_parameter('sensing_rate', 10.0)
        self.declare_parameter('frame_id', 'map')

        self._horizon = float(self.get_parameter('sensing_horizon').value)
        self._frame = str(self.get_parameter('frame_id').value)
        self._pts: Optional[np.ndarray] = None
        self._odom: Optional[np.ndarray] = None
        self._tick_count = 0

        global_topic = str(self.get_parameter('global_cloud_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        local_topic = str(self.get_parameter('local_cloud_topic').value)

        self._pub = self.create_publisher(PointCloud2, local_topic, _latched_qos())
        self.create_subscription(PointCloud2, global_topic, self._on_cloud, _latched_qos())
        self.create_subscription(Odometry, odom_topic, self._on_odom, 50)

        rate = max(1.0, float(self.get_parameter('sensing_rate').value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'local_sense: {global_topic} → {local_topic} '
            f'horizon={self._horizon:.1f} m @ {rate:.0f} Hz (TRANSIENT_LOCAL)')

    def _on_cloud(self, msg: PointCloud2) -> None:
        try:
            rows = []
            for p in point_cloud2.read_points(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True):
                rows.append((float(p[0]), float(p[1]), float(p[2])))
            if not rows:
                self._pts = np.zeros((0, 3), dtype=np.float64)
                return
            self._pts = np.asarray(rows, dtype=np.float64)
            self.get_logger().info(
                f'global cloud cached: {self._pts.shape[0]} points',
                throttle_duration_sec=5.0)
        except Exception as exc:  # noqa: BLE001 — keep node alive
            self.get_logger().error(f'cloud parse failed: {exc}')

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._odom = np.array([p.x, p.y, p.z], dtype=np.float64)

    def _tick(self) -> None:
        try:
            if self._pts is None or self._odom is None or self._pts.shape[0] == 0:
                return
            d = self._pts - self._odom
            mask = (d[:, 0] * d[:, 0] + d[:, 1] * d[:, 1] + d[:, 2] * d[:, 2]) <= (
                self._horizon * self._horizon)
            local = self._pts[mask]
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self._frame
            xyz = local.astype(np.float32)
            # create_cloud_xyz32 wants list/iterable of (x,y,z)
            pts = [(float(x), float(y), float(z)) for x, y, z in xyz]
            self._pub.publish(point_cloud2.create_cloud_xyz32(header, pts))
            self._tick_count += 1
            if self._tick_count == 1 or self._tick_count % 50 == 0:
                self.get_logger().info(
                    f'local cloud #{self._tick_count}: {len(pts)} pts '
                    f'@ ({self._odom[0]:.1f},{self._odom[1]:.1f},{self._odom[2]:.1f})')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'tick failed: {exc}')


def main() -> None:
    rclpy.init()
    node = LocalSenseCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

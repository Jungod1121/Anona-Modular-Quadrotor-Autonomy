#!/usr/bin/env python3
"""Republish a PointCloud2 onto one or more topics with a fixed frame_id.

Used so Path A (/map/obstacles) and Path B/C (/map_generator/global_cloud)
can share the same map generator regardless of backend.

Input QoS is flexible (volatile or latched). Outputs are always transient-local
so late joiners (RViz / planners) still see the map.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class CloudBridge(Node):
    def __init__(self) -> None:
        super().__init__('cloud_bridge')
        self.declare_parameter('input_topic', '/map/obstacles')
        self.declare_parameter('output_topics', [
            '/map_generator/global_cloud',
            '/map/obstacles',
        ])
        self.declare_parameter('frame_id', 'map')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        outputs = list(self.get_parameter('output_topics').value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Drop self-echo if input is also listed as an output.
        self.outputs = [t for t in outputs if t and t != input_topic]

        # mockamap publishes VOLATILE; random_forest / drone_map are TRANSIENT_LOCAL.
        # A TRANSIENT_LOCAL *subscriber* cannot receive VOLATILE publishers — so keep
        # the input side VOLATILE (compatible with both: offered durability ≥ requested).
        sub_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        pub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pubs = [
            self.create_publisher(PointCloud2, topic, pub_qos)
            for topic in self.outputs
        ]
        self._count = 0
        self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, sub_qos)
        self.get_logger().info(
            f'bridging {input_topic} → {self.outputs} (frame_id={self.frame_id})')

    def _on_cloud(self, msg: PointCloud2) -> None:
        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = self.frame_id or msg.header.frame_id
        # Keep stamp fresh so RViz Time panel / TF lookups stay valid.
        out.header.stamp = self.get_clock().now().to_msg()
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = msg.data
        out.is_dense = msg.is_dense
        for pub in self._pubs:
            pub.publish(out)
        self._count += 1
        if self._count == 1 or self._count % 20 == 0:
            self.get_logger().info(
                f'relayed cloud #{self._count} width={out.width} → {self.outputs}')


def main() -> None:
    rclpy.init()
    node = CloudBridge()
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

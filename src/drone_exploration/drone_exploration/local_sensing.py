"""FUEL-style fog-of-war sensing: global cloud + odom → local obstacles + frontiers."""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from drone_exploration.occupancy import OccupancyGrid3D


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


class LocalSensingNode(Node):
    def __init__(self) -> None:
        super().__init__('local_sensing')
        self.declare_parameter('box_min_x', -1.0)
        self.declare_parameter('box_min_y', -1.0)
        self.declare_parameter('box_min_z', 0.0)
        self.declare_parameter('box_max_x', 21.0)
        self.declare_parameter('box_max_y', 11.0)
        self.declare_parameter('box_max_z', 3.0)
        self.declare_parameter('resolution', 0.4)
        self.declare_parameter('sensing_radius', 4.0)
        self.declare_parameter('sensing_z_half', 1.2)
        self.declare_parameter('obstacle_inflate', 0.35)
        self.declare_parameter('cruise_z', 1.5)
        self.declare_parameter('global_cloud_topic', '/map/obstacles')
        self.declare_parameter('local_cloud_topic', '/map/obstacles_local')
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter('frontiers_topic', '/exploration/frontiers')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period', 0.25)
        self.declare_parameter('frontier_min_size', 3)

        box_min = [
            self.get_parameter('box_min_x').value,
            self.get_parameter('box_min_y').value,
            self.get_parameter('box_min_z').value,
        ]
        box_max = [
            self.get_parameter('box_max_x').value,
            self.get_parameter('box_max_y').value,
            self.get_parameter('box_max_z').value,
        ]
        self.grid = OccupancyGrid3D(
            box_min, box_max, float(self.get_parameter('resolution').value))
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.sensing_radius = float(self.get_parameter('sensing_radius').value)
        self.sensing_z_half = float(self.get_parameter('sensing_z_half').value)
        self.inflate = float(self.get_parameter('obstacle_inflate').value)
        self.cruise_z = float(self.get_parameter('cruise_z').value)
        self.frontier_min_size = int(self.get_parameter('frontier_min_size').value)

        self._cloud: Optional[np.ndarray] = None
        self._odom: Optional[np.ndarray] = None

        local_topic = str(self.get_parameter('local_cloud_topic').value)
        frontiers_topic = str(self.get_parameter('frontiers_topic').value)
        self._local_pub = self.create_publisher(PointCloud2, local_topic, _latched_qos())
        self._frontier_pub = self.create_publisher(PoseArray, frontiers_topic, 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/exploration/frontier_markers', 10)

        global_topic = str(self.get_parameter('global_cloud_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        self.create_subscription(
            PointCloud2, global_topic, self._on_cloud, _latched_qos())
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)

        period = float(self.get_parameter('publish_period').value)
        self.create_timer(max(0.1, period), self._tick)
        self.get_logger().info(
            f'local_sensing: {global_topic} → {local_topic} '
            f'radius={self.sensing_radius:.1f}m box={box_min}..{box_max}')

    def _on_cloud(self, msg: PointCloud2) -> None:
        pts = list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not pts:
            # An empty latched publish must not wipe stored knowledge — that
            # painted phantom FREE space across unobserved area.
            self.get_logger().warn('empty obstacle cloud — keeping previous map',
                                   throttle_duration_sec=5.0)
            return
        arr = np.asarray([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
        # Light downsample for realtime.
        if arr.shape[0] > 80000:
            step = int(np.ceil(arr.shape[0] / 80000))
            arr = arr[::step]
        self._cloud = arr

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._odom = np.array([p.x, p.y, p.z], dtype=np.float64)

    def _tick(self) -> None:
        if self._odom is None:
            return
        self.grid.reveal(
            self._odom, self._cloud,
            self.sensing_radius, self.sensing_z_half, self.inflate)
        occ = self.grid.observed_obstacle_points()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        cloud = point_cloud2.create_cloud_xyz32(header, occ.tolist() if occ.size else [])
        self._local_pub.publish(cloud)

        clusters = self.grid.frontier_clusters(
            self.cruise_z, min_size=max(2, self.frontier_min_size))
        pa = PoseArray()
        pa.header = header
        centroids = []
        for c, size in clusters:
            pose = Pose()
            pose.position.x = float(c[0])
            pose.position.y = float(c[1])
            pose.position.z = float(c[2])
            # Internal: orientation.x carries cluster size for the FSM scorer.
            pose.orientation.x = float(size)
            pose.orientation.w = 1.0
            pa.poses.append(pose)
            centroids.append(c)
        self._frontier_pub.publish(pa)
        self._publish_markers(header, centroids)

    def _publish_markers(self, header: Header, clusters) -> None:
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for i, c in enumerate(clusters):
            m = Marker()
            m.header = header
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(c[0])
            m.pose.position.y = float(c[1])
            m.pose.position.z = float(c[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.45
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.55, 0.1, 0.85
            ma.markers.append(m)
        self._marker_pub.publish(ma)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalSensingNode()
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

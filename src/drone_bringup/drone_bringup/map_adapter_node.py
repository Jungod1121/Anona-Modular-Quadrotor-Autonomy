#!/usr/bin/env python3
"""Bridge map clouds and publish occupancy + JSON metadata.

Subscribes to a generator topic (drone_map / random_forest / mockamap), optionally
stamps boundary wall points for official maps, republishes onto planner topics,
and exposes /map/occupancy (2D cruise-height slice) plus /map/metadata.
"""

from __future__ import annotations

import json
import math
from typing import Any, Dict, Iterable, List, Sequence, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String

from drone_bringup.maps_catalog import catalog_metadata, normalize_map_id


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


def _walk_edge(
    p0: Tuple[float, float, float],
    p1: Tuple[float, float, float],
    step: float,
) -> Iterable[Tuple[float, float, float]]:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    span = math.sqrt(dx * dx + dy * dy + dz * dz)
    if span < 1e-6:
        yield p0
        return
    n = max(1, int(math.ceil(span / step)))
    for i in range(n + 1):
        t = i / n
        yield (p0[0] + t * dx, p0[1] + t * dy, p0[2] + t * dz)


def aabb_boundary_points(
    bounds: Dict[str, float],
    resolution: float = 0.15,
    wall_height: float | None = None,
) -> List[Tuple[float, float, float]]:
    """Sample points along the 12 edges of an axis-aligned box."""
    xmin = float(bounds['xmin'])
    xmax = float(bounds['xmax'])
    ymin = float(bounds['ymin'])
    ymax = float(bounds['ymax'])
    zmin = float(bounds['zmin'])
    zmax = float(bounds['zmax'])
    if wall_height is not None:
        zmax = min(zmax, zmin + wall_height)
    step = max(0.08, float(resolution))
    corners = [
        (xmin, ymin, zmin), (xmax, ymin, zmin), (xmax, ymax, zmin), (xmin, ymax, zmin),
        (xmin, ymin, zmax), (xmax, ymin, zmax), (xmax, ymax, zmax), (xmin, ymax, zmax),
    ]
    edges = (
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    )
    pts: List[Tuple[float, float, float]] = []
    seen: set[Tuple[int, int, int]] = set()
    for i, j in edges:
        for x, y, z in _walk_edge(corners[i], corners[j], step):
            key = (int(round(x * 100)), int(round(y * 100)), int(round(z * 100)))
            if key in seen:
                continue
            seen.add(key)
            pts.append((x, y, z))
    return pts


def merge_boundary_cloud(
    msg: PointCloud2,
    bounds: Dict[str, float],
    resolution: float,
) -> PointCloud2:
    pts = [
        (float(x), float(y), float(z))
        for x, y, z in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    ]
    pts.extend(aabb_boundary_points(bounds, resolution=resolution))
    out = PointCloud2()
    out.header = msg.header
    return pc2.create_cloud_xyz32(out.header, pts)


def occupancy_grid_from_cloud(
    msg: PointCloud2,
    bounds: Dict[str, float],
    cruise_z: float,
    z_band: float,
    resolution: float,
    frame_id: str,
    stamp,
    project_all_z: bool = False,
) -> OccupancyGrid:
    res = max(0.12, float(resolution))
    xmin, xmax = float(bounds['xmin']), float(bounds['xmax'])
    ymin, ymax = float(bounds['ymin']), float(bounds['ymax'])
    width = max(1, int(math.ceil((xmax - xmin) / res)))
    height = max(1, int(math.ceil((ymax - ymin) / res)))
    cells = [0] * (width * height)
    z_lo = cruise_z - z_band
    z_hi = cruise_z + z_band

    for x, y, z in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
        if not project_all_z:
            zf = float(z)
            if zf < z_lo or zf > z_hi:
                continue
        ix = int(math.floor((float(x) - xmin) / res))
        iy = int(math.floor((float(y) - ymin) / res))
        if 0 <= ix < width and 0 <= iy < height:
            cells[iy * width + ix] = 100

    grid = OccupancyGrid()
    grid.header.frame_id = frame_id
    grid.header.stamp = stamp
    grid.info = MapMetaData()
    grid.info.resolution = res
    grid.info.width = width
    grid.info.height = height
    grid.info.origin.position.x = xmin
    grid.info.origin.position.y = ymin
    grid.info.origin.position.z = 0.0
    grid.info.origin.orientation.w = 1.0
    grid.data = cells
    return grid


class MapAdapter(Node):
    def __init__(self) -> None:
        super().__init__('map_adapter')
        self.declare_parameter('input_topic', '/map/obstacles')
        self.declare_parameter('output_topics', [
            '/map_generator/global_cloud',
            '/map/obstacles',
        ])
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('map_id', 'official_forest')
        self.declare_parameter('seed', 1)
        self.declare_parameter('cruise_z', 1.0)
        self.declare_parameter('z_band', 0.55)
        self.declare_parameter('grid_resolution', 0.25)
        self.declare_parameter('ensure_boundary', False)
        self.declare_parameter('boundary_resolution', 0.15)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        outputs = list(self.get_parameter('output_topics').value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.map_id = normalize_map_id(
            self.get_parameter('map_id').get_parameter_value().string_value)
        self.seed = int(self.get_parameter('seed').value)
        self.cruise_z = float(self.get_parameter('cruise_z').value)
        self.z_band = float(self.get_parameter('z_band').value)
        self.grid_resolution = float(self.get_parameter('grid_resolution').value)
        self.ensure_boundary = bool(self.get_parameter('ensure_boundary').value)
        self.boundary_resolution = float(self.get_parameter('boundary_resolution').value)

        meta = catalog_metadata(self.map_id, seed=self.seed)
        self._bounds = dict(meta['bounds'])
        self._metadata = meta

        self.outputs = [t for t in outputs if t and t != input_topic]
        pub_qos = _latched_qos()
        self._cloud_pubs = [
            self.create_publisher(PointCloud2, topic, pub_qos)
            for topic in self.outputs
        ]
        self._occ_pub = self.create_publisher(OccupancyGrid, '/map/occupancy', pub_qos)
        # Full XY projection (ignore Z) — same silhouette as RViz top-down cloud.
        self._occ_top_pub = self.create_publisher(
            OccupancyGrid, '/map/occupancy_topdown', pub_qos)
        self._meta_pub = self.create_publisher(String, '/map/metadata', pub_qos)
        self._last_occ: OccupancyGrid | None = None
        self._last_occ_top: OccupancyGrid | None = None
        self._last_meta = ''
        self._last_cloud: PointCloud2 | None = None
        # Reinforce latched map + cloud for late subscribers (FP / dashboard / RViz).
        self.create_timer(2.0, self._republish_latched)

        self._count = 0
        self.create_subscription(PointCloud2, input_topic, self._on_cloud, _latched_qos())
        self.get_logger().info(
            f'map_adapter {input_topic} → clouds {self.outputs}, '
            f'occupancy + metadata for map={self.map_id}')

    def _restamp_cloud(self, msg: PointCloud2) -> PointCloud2:
        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = self.frame_id or msg.header.frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = msg.data
        out.is_dense = msg.is_dense
        return out

    def _on_cloud(self, msg: PointCloud2) -> None:
        raw = (
            merge_boundary_cloud(msg, self._bounds, self.boundary_resolution)
            if self.ensure_boundary else msg
        )
        stamped = self._restamp_cloud(raw)
        self._last_cloud = stamped

        for pub in self._cloud_pubs:
            pub.publish(stamped)

        frame = self.frame_id or stamped.header.frame_id
        stamp = stamped.header.stamp
        occ = occupancy_grid_from_cloud(
            stamped,
            self._bounds,
            self.cruise_z,
            self.z_band,
            self.grid_resolution,
            frame,
            stamp,
        )
        occ_top = occupancy_grid_from_cloud(
            stamped,
            self._bounds,
            self.cruise_z,
            self.z_band,
            self.grid_resolution,
            frame,
            stamp,
            project_all_z=True,
        )
        self._occ_pub.publish(occ)
        self._occ_top_pub.publish(occ_top)
        self._last_occ = occ
        self._last_occ_top = occ_top

        meta_msg = String()
        meta_msg.data = json.dumps(self._metadata, sort_keys=True)
        self._meta_pub.publish(meta_msg)
        self._last_meta = meta_msg.data

        self._count += 1
        if self._count == 1 or self._count % 20 == 0:
            n_top = sum(1 for v in occ_top.data if int(v) >= 50)
            self.get_logger().info(
                f'adapted cloud #{self._count} width={stamped.width} '
                f'occ={occ.info.width}x{occ.info.height} '
                f'topdown_cells={n_top}')

    def _republish_latched(self) -> None:
        if self._last_cloud is not None:
            stamped = self._restamp_cloud(self._last_cloud)
            self._last_cloud = stamped
            for pub in self._cloud_pubs:
                pub.publish(stamped)
        if self._last_occ is not None:
            self._last_occ.header.stamp = self.get_clock().now().to_msg()
            self._occ_pub.publish(self._last_occ)
        if self._last_occ_top is not None:
            self._last_occ_top.header.stamp = self.get_clock().now().to_msg()
            self._occ_top_pub.publish(self._last_occ_top)
        if self._last_meta:
            meta_msg = String()
            meta_msg.data = self._last_meta
            self._meta_pub.publish(meta_msg)

def main() -> None:
    rclpy.init()
    node = MapAdapter()
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

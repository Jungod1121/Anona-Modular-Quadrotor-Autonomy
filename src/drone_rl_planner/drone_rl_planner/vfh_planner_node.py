"""PX4-Avoidance–style VFH+ local planner for Path G.

Replaces the brittle PPO→PID stack with a classical polar histogram
local avoider that:
  1) builds a 2D polar cost histogram from /map/obstacles
  2) picks a clear heading biased toward the goal (VFH+)
  3) publishes a smooth multi-waypoint /planner/trajectory (yellow path)
  4) tracks it with /planner/local_goal (same plant interface as Path A)

This is NOT learning. It is deterministic and works on all catalog maps.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker

from drone_msgs.msg import PlannerStatus
from drone_rl_planner.vfh_core import polar_clearance, sector_clearance, select_heading


def _latched() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


class VfhPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('vfh_planner_node')
        self.declare_parameter('n_sectors', 72)
        self.declare_parameter('ray_max', 6.0)
        self.declare_parameter('robot_r', 0.28)
        self.declare_parameter('safety', 0.35)
        self.declare_parameter('cruise_z', 1.5)
        self.declare_parameter('lookahead_m', 1.6)
        self.declare_parameter('path_step_m', 0.45)
        self.declare_parameter('path_horizon_m', 7.0)
        self.declare_parameter('goal_tol', 0.70)
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('hist_smooth', 3)
        self.declare_parameter('goal_weight', 1.2)
        self.declare_parameter('clear_threshold', 0.42)  # fraction of ray_max
        self.declare_parameter('map_topic', '/map/obstacles')
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter('goal_topic', '/drone/goal')

        self.n_sectors = int(self.get_parameter('n_sectors').value)
        self.ray_max = float(self.get_parameter('ray_max').value)
        self.robot_r = float(self.get_parameter('robot_r').value)
        self.safety = float(self.get_parameter('safety').value)
        self.cruise_z = float(self.get_parameter('cruise_z').value)
        self.lookahead_m = float(self.get_parameter('lookahead_m').value)
        self.path_step_m = float(self.get_parameter('path_step_m').value)
        self.path_horizon_m = float(self.get_parameter('path_horizon_m').value)
        self.goal_tol = float(self.get_parameter('goal_tol').value)
        self.hist_smooth = int(self.get_parameter('hist_smooth').value)
        self.goal_weight = float(self.get_parameter('goal_weight').value)
        self.clear_threshold = float(self.get_parameter('clear_threshold').value)

        self._odom: Optional[Odometry] = None
        self._goal: Optional[PoseStamped] = None
        self._cloud: Optional[np.ndarray] = None
        self._heading = 0.0  # filtered command heading

        self._local_pub = self.create_publisher(PoseStamped, '/planner/local_goal', 10)
        self._path_pub = self.create_publisher(NavPath, '/planner/trajectory', _latched())
        self._status_pub = self.create_publisher(PlannerStatus, '/planner/status', 10)
        self._marker_pub = self.create_publisher(Marker, '/planner/local_goal_marker', 10)

        map_topic = str(self.get_parameter('map_topic').value)
        self.create_subscription(PointCloud2, map_topic, self._on_cloud, _latched())
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self._on_odom, 20)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('goal_topic').value), self._on_goal, 10)

        hz = float(self.get_parameter('control_hz').value)
        self.create_timer(max(0.04, 1.0 / hz), self._tick)
        self.get_logger().info(
            f'vfh_planner_node ready (sectors={self.n_sectors}, '
            f'ray_max={self.ray_max}, lookahead={self.lookahead_m})')

    def _on_cloud(self, msg: PointCloud2) -> None:
        pts = list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not pts:
            self._cloud = np.zeros((0, 3))
            return
        arr = np.asarray([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
        # Keep band around cruise altitude
        band = np.abs(arr[:, 2] - self.cruise_z) <= 1.4
        self._cloud = arr[band] if np.any(band) else arr

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_goal(self, msg: PoseStamped) -> None:
        if msg.pose.position.z < 0.3:
            msg.pose.position.z = self.cruise_z
        self._goal = msg

    def _polar_clearance(self, pos_xy: np.ndarray) -> np.ndarray:
        cloud_xy = None
        if self._cloud is not None and self._cloud.size:
            cloud_xy = self._cloud[:, :2]
        return polar_clearance(
            cloud_xy, pos_xy,
            n_sectors=self.n_sectors,
            ray_max=self.ray_max,
            robot_r=self.robot_r,
            safety=self.safety,
            hist_smooth=self.hist_smooth,
        )

    def _select_heading(
        self, clear: np.ndarray, goal_ang: float, cur_heading: float,
    ) -> float:
        return select_heading(
            clear, goal_ang, cur_heading,
            ray_max=self.ray_max,
            clear_threshold=self.clear_threshold,
            goal_weight=self.goal_weight,
        )

    def _sector_clearance(self, clear: np.ndarray, heading: float) -> float:
        return sector_clearance(clear, heading)

    def _build_path(
        self,
        pos: np.ndarray,
        heading: float,
        goal_xy: np.ndarray,
        clear: np.ndarray,
        stamp,
    ) -> Tuple[NavPath, np.ndarray]:
        """Roll a short polyline along VFH headings; re-query histogram along the way."""
        path = NavPath()
        path.header.stamp = stamp
        path.header.frame_id = 'map'

        def add(xy: np.ndarray) -> None:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(xy[0])
            ps.pose.position.y = float(xy[1])
            ps.pose.position.z = float(self.cruise_z)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        cur = pos[:2].copy()
        add(cur)
        h = heading
        traveled = 0.0
        local_goal = cur + np.array([np.cos(h), np.sin(h)]) * self.lookahead_m

        while traveled < self.path_horizon_m:
            to_goal = goal_xy - cur
            dist_g = float(np.linalg.norm(to_goal))
            if dist_g < self.goal_tol:
                add(goal_xy)
                local_goal = goal_xy
                break
            # Refresh local clearance at current virtual pose (cheap: use global cloud)
            clear_i = self._polar_clearance(cur)
            goal_ang = float(np.arctan2(to_goal[1], to_goal[0]))
            h = self._select_heading(clear_i, goal_ang, h)
            # Step length limited by forward clearance
            fwd = self._sector_clearance(clear_i, h)
            step = min(self.path_step_m, max(0.25, fwd - 0.4), dist_g)
            cur = cur + np.array([np.cos(h), np.sin(h)]) * step
            traveled += step
            add(cur)
            if traveled >= self.lookahead_m * 0.9 and path.poses.__len__() >= 3:
                local_goal = cur.copy()
        else:
            add(goal_xy)

        if float(np.linalg.norm(local_goal - pos[:2])) < 0.8:
            local_goal = pos[:2] + np.array([np.cos(h), np.sin(h)]) * max(
                1.0, min(self.lookahead_m, float(np.linalg.norm(goal_xy - pos[:2]))))
        return path, local_goal

    def _publish_marker(self, target: np.ndarray, stamp) -> None:
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = 'map'
        m.ns = 'vfh_local_goal'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(target[0])
        m.pose.position.y = float(target[1])
        m.pose.position.z = float(self.cruise_z)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.35
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.1, 0.95
        self._marker_pub.publish(m)

    def _tick(self) -> None:
        if self._odom is None or self._goal is None:
            return
        p = self._odom.pose.pose.position
        g = self._goal.pose.position
        pos = np.array([p.x, p.y, p.z], dtype=np.float64)
        goal = np.array([g.x, g.y, g.z], dtype=np.float64)
        to_goal = goal[:2] - pos[:2]
        dist_goal = float(np.linalg.norm(to_goal))
        stamp = self.get_clock().now().to_msg()

        if dist_goal < self.goal_tol:
            target = goal[:2]
            path = NavPath()
            path.header.stamp = stamp
            path.header.frame_id = 'map'
            for xy in (pos[:2], goal[:2]):
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x, ps.pose.position.y = float(xy[0]), float(xy[1])
                ps.pose.position.z = float(self.cruise_z)
                ps.pose.orientation.w = 1.0
                path.poses.append(ps)
            state, success = 'REACHED', True
            yaw = float(np.arctan2(to_goal[1], to_goal[0])) if dist_goal > 1e-3 else self._heading
        else:
            clear = self._polar_clearance(pos[:2])
            goal_ang = float(np.arctan2(to_goal[1], to_goal[0]))
            raw_h = self._select_heading(clear, goal_ang, self._heading)
            # Mild heading LPF (avoid chatter, not RL dither)
            alpha = 0.35
            # unwrap blend
            d = ((raw_h - self._heading + np.pi) % (2 * np.pi)) - np.pi
            self._heading = self._heading + (1.0 - alpha) * d
            self._heading = (self._heading + np.pi) % (2 * np.pi) - np.pi
            path, target = self._build_path(pos, self._heading, goal[:2], clear, stamp)
            # Cap local_goal distance
            vec = target - pos[:2]
            d = float(np.linalg.norm(vec))
            if d > self.lookahead_m:
                target = pos[:2] + vec / d * self.lookahead_m
            yaw = self._heading
            state, success = 'EXEC_TRAJ', False

        lg = PoseStamped()
        lg.header.stamp = stamp
        lg.header.frame_id = 'map'
        lg.pose.position.x = float(target[0])
        lg.pose.position.y = float(target[1])
        lg.pose.position.z = float(self.cruise_z)
        lg.pose.orientation.z = float(np.sin(yaw * 0.5))
        lg.pose.orientation.w = float(np.cos(yaw * 0.5))
        self._local_pub.publish(lg)
        self._path_pub.publish(path)
        self._publish_marker(target, stamp)

        st = PlannerStatus()
        st.header = lg.header
        st.state = state
        st.success = success
        st.message = 'vfh_plus'
        st.path_length = dist_goal
        if self._cloud is not None and self._cloud.size:
            clear = self._polar_clearance(pos[:2])
            st.min_obstacle_distance = float(np.min(clear))
        self._status_pub.publish(st)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VfhPlannerNode()
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

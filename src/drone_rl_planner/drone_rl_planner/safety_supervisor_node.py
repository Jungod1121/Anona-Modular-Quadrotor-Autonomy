"""Adapter-level VFH safety supervisor for Path H (and similar).

Subscribes to SAC local_goal / trajectory and can override with VFH when
clearance is unsafe. Publishes /planner/diagnostics + fallback flag.

Path H remains a pure SAC solver; this node is the abnormal-condition switch.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool

from drone_msgs.msg import PlannerDiagnostics, PlannerStatus
from drone_rl_planner.bezier_path import build_rolled_path, corridor_clearance
from drone_rl_planner.vfh_core import vfh_heading


def _latched() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


class SafetySupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__('safety_supervisor_node')
        self.declare_parameter('primary_local_goal_topic', '/planner/sac_local_goal')
        self.declare_parameter('primary_trajectory_topic', '/planner/sac_trajectory')
        self.declare_parameter('map_topic', '/map/obstacles')
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter('goal_topic', '/drone/goal')
        self.declare_parameter('fallback_clear_m', 0.40)
        self.declare_parameter('blend_clear_m', 1.4)
        self.declare_parameter('cruise_z', 1.5)
        self.declare_parameter('path_horizon_m', 8.0)
        self.declare_parameter('path_step_m', 0.45)
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('enable_fallback', True)
        self.declare_parameter('planner_id', 'sac')
        self.declare_parameter('robot_r', 0.28)
        self.declare_parameter('safety', 0.30)
        # Take over when VFH clearance beats SAC by this much (dense: lower).
        self.declare_parameter('vfh_prefer_delta_m', 0.15)
        # Always prefer VFH if SAC corridor is this tight (emergency).
        self.declare_parameter('emergency_clear_m', 0.22)
        # If true: any time SAC is inside blend band and VFH is not worse, use VFH.
        self.declare_parameter('prefer_vfh', False)
        # Keep VFH engaged for N ticks after a takeover (stops chatter / divergence).
        self.declare_parameter('fallback_hold_ticks', 0)

        self.fallback_clear_m = float(self.get_parameter('fallback_clear_m').value)
        self.blend_clear_m = float(self.get_parameter('blend_clear_m').value)
        self.cruise_z = float(self.get_parameter('cruise_z').value)
        self.path_horizon_m = float(self.get_parameter('path_horizon_m').value)
        self.path_step_m = float(self.get_parameter('path_step_m').value)
        self.enable_fallback = bool(self.get_parameter('enable_fallback').value)
        self.planner_id = str(self.get_parameter('planner_id').value)
        self.robot_r = float(self.get_parameter('robot_r').value)
        self.safety = float(self.get_parameter('safety').value)
        self.vfh_prefer_delta_m = float(self.get_parameter('vfh_prefer_delta_m').value)
        self.emergency_clear_m = float(self.get_parameter('emergency_clear_m').value)
        self.prefer_vfh = bool(self.get_parameter('prefer_vfh').value)
        self.fallback_hold_ticks = int(self.get_parameter('fallback_hold_ticks').value)
        self._fb_hold = 0

        self._odom: Optional[Odometry] = None
        self._goal: Optional[PoseStamped] = None
        self._cloud: Optional[np.ndarray] = None
        self._primary_goal: Optional[PoseStamped] = None
        self._primary_path: Optional[NavPath] = None
        self._vfh_h = 0.0

        self._local_pub = self.create_publisher(PoseStamped, '/planner/local_goal', 10)
        self._path_pub = self.create_publisher(NavPath, '/planner/trajectory', _latched())
        self._status_pub = self.create_publisher(PlannerStatus, '/planner/status', 10)
        self._diag_pub = self.create_publisher(PlannerDiagnostics, '/planner/diagnostics', 10)
        self._fb_pub = self.create_publisher(Bool, '/planner/fallback_active', 10)

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('primary_local_goal_topic').value),
            self._on_primary_goal, 10)
        self.create_subscription(
            NavPath,
            str(self.get_parameter('primary_trajectory_topic').value),
            self._on_primary_path, _latched())
        self.create_subscription(
            PointCloud2, str(self.get_parameter('map_topic').value), self._on_cloud, _latched())
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self._on_odom, 20)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('goal_topic').value), self._on_goal, 10)

        hz = float(self.get_parameter('control_hz').value)
        self.create_timer(max(0.04, 1.0 / hz), self._tick)
        self.get_logger().info(
            f'safety_supervisor ready (fallback_clear={self.fallback_clear_m}, '
            f'enable={self.enable_fallback})')

    def _on_cloud(self, msg: PointCloud2) -> None:
        pts = list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not pts:
            self._cloud = np.zeros((0, 3))
            return
        arr = np.asarray([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
        band = np.abs(arr[:, 2] - self.cruise_z) <= 1.4
        self._cloud = arr[band] if np.any(band) else arr

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_goal(self, msg: PoseStamped) -> None:
        if msg.pose.position.z < 0.3:
            msg.pose.position.z = self.cruise_z
        self._goal = msg

    def _on_primary_goal(self, msg: PoseStamped) -> None:
        self._primary_goal = msg

    def _on_primary_path(self, msg: NavPath) -> None:
        self._primary_path = msg

    def _tick(self) -> None:
        if self._odom is None or self._goal is None or self._primary_goal is None:
            return
        p = self._odom.pose.pose.position
        g = self._goal.pose.position
        pos = np.array([p.x, p.y], dtype=np.float64)
        goal = np.array([g.x, g.y], dtype=np.float64)
        stamp = self.get_clock().now().to_msg()

        cloud_xy = None if self._cloud is None else self._cloud[:, :2]
        primary_xy = np.array([
            self._primary_goal.pose.position.x,
            self._primary_goal.pose.position.y,
        ], dtype=np.float64)

        probe = np.stack([pos, primary_xy], axis=0)
        c_sac = corridor_clearance(
            probe, cloud_xy, half_width=self.robot_r + 0.25,
        ) if cloud_xy is not None else 99.0

        reason = ''
        out_goal = self._primary_goal
        out_path = self._primary_path
        fallback = False

        if self.enable_fallback and cloud_xy is not None and c_sac <= self.blend_clear_m:
            self._vfh_h, _ = vfh_heading(cloud_xy, pos, goal, self._vfh_h)

            def heading_fn(cur_xy, goal_ang, cur_h):
                h, _ = vfh_heading(cloud_xy, cur_xy, goal, cur_h)
                return h

            path_xy, _h, c_vfh = build_rolled_path(
                pos, self._vfh_h, goal, heading_fn, cloud_xy,
                step_m=self.path_step_m,
                horizon_m=self.path_horizon_m,
                robot_r=self.robot_r,
                safety=self.safety,
            )
            if c_sac <= self.fallback_clear_m and c_vfh > c_sac + self.vfh_prefer_delta_m:
                fallback = True
                reason = 'clearance_below_fallback'
            elif c_sac <= self.emergency_clear_m and c_vfh > c_sac:
                fallback = True
                reason = 'emergency_clearance'
            elif self.prefer_vfh and c_vfh >= c_sac - 0.02:
                fallback = True
                reason = 'prefer_vfh'
            # Sticky hold: avoid SAC↔VFH chatter that destabilizes tracking.
            if fallback:
                self._fb_hold = max(self._fb_hold, self.fallback_hold_ticks)
            elif self._fb_hold > 0:
                self._fb_hold -= 1
                fallback = True
                reason = reason or 'fallback_hold'
            if fallback:
                look = min(1.6, max(0.8, float(np.linalg.norm(goal - pos))))
                local = pos + np.array([np.cos(self._vfh_h), np.sin(self._vfh_h)]) * look
                if len(path_xy) > 2:
                    local = path_xy[min(3, len(path_xy) - 1)]
                out_goal = PoseStamped()
                out_goal.header.stamp = stamp
                out_goal.header.frame_id = 'map'
                out_goal.pose.position.x = float(local[0])
                out_goal.pose.position.y = float(local[1])
                out_goal.pose.position.z = self.cruise_z
                out_goal.pose.orientation.w = 1.0
                out_path = NavPath()
                out_path.header = out_goal.header
                for xy in path_xy:
                    ps = PoseStamped()
                    ps.header = out_goal.header
                    ps.pose.position.x = float(xy[0])
                    ps.pose.position.y = float(xy[1])
                    ps.pose.position.z = self.cruise_z
                    ps.pose.orientation.w = 1.0
                    out_path.poses.append(ps)

        self._local_pub.publish(out_goal)
        if out_path is not None:
            self._path_pub.publish(out_path)

        st = PlannerStatus()
        st.header.stamp = stamp
        st.state = 'FALLBACK' if fallback else 'OK'
        st.success = not fallback
        st.message = reason or 'primary'
        st.min_obstacle_distance = float(c_sac)
        self._status_pub.publish(st)

        diag = PlannerDiagnostics()
        diag.header.stamp = stamp
        diag.planner_id = self.planner_id
        diag.state = st.state
        diag.fallback_active = fallback
        diag.fallback_reason = reason
        diag.clearance_m = float(c_sac)
        self._diag_pub.publish(diag)

        fb = Bool()
        fb.data = fallback
        self._fb_pub.publish(fb)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetySupervisorNode()
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

"""Path H inference: Polar DrQ-SAC → Bézier yellow path + VFH fallback.

Plant contract:
  in:  /drone/odom, /drone/goal, /map/obstacles
  out: /planner/local_goal, /planner/trajectory, /planner/status
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_rl_planner.odom_util import world_vel_2d
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker

from drone_msgs.msg import PlannerStatus
from drone_rl_planner.bezier_path import build_rolled_path, corridor_clearance
from drone_rl_planner.polar_sensing import (
    POLAR_DEFAULTS,
    build_polar_image,
    build_polar_vector,
    decode_action,
    downsample_cloud,
)


def _latched() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


class SacPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('sac_planner_node')
        self.declare_parameter('checkpoint', '')
        self.declare_parameter('n_rings', POLAR_DEFAULTS['n_rings'])
        self.declare_parameter('n_sectors', POLAR_DEFAULTS['n_sectors'])
        self.declare_parameter('ray_max', POLAR_DEFAULTS['ray_max'])
        self.declare_parameter('robot_r', POLAR_DEFAULTS['robot_r'])
        self.declare_parameter('safety', POLAR_DEFAULTS['safety'])
        self.declare_parameter('max_speed', POLAR_DEFAULTS['max_speed'])
        self.declare_parameter('world_scale', POLAR_DEFAULTS['world_scale'])
        self.declare_parameter('cruise_z', 1.5)
        self.declare_parameter('goal_tol', 0.70)
        self.declare_parameter('hold_exit_m', 1.6)
        self.declare_parameter('approach_m', 2.5)
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('action_ema', 0.35)
        self.declare_parameter('fallback_clear_m', 0.40)
        self.declare_parameter('path_horizon_m', 8.0)
        self.declare_parameter('path_step_m', 0.45)
        self.declare_parameter('blend_clear_m', 1.4)
        self.declare_parameter('map_topic', '/map/obstacles')
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter('goal_topic', '/drone/goal')
        # Pure SAC outputs — supervisor remaps these to /planner/* when enabled.
        self.declare_parameter('local_goal_topic', '/planner/sac_local_goal')
        self.declare_parameter('trajectory_topic', '/planner/sac_trajectory')
        self.declare_parameter('status_topic', '/planner/sac_status')
        self.declare_parameter('direct_plant', False)  # True: publish on /planner/* (no supervisor)

        self.n_rings = int(self.get_parameter('n_rings').value)
        self.n_sectors = int(self.get_parameter('n_sectors').value)
        self.ray_max = float(self.get_parameter('ray_max').value)
        self.robot_r = float(self.get_parameter('robot_r').value)
        self.safety = float(self.get_parameter('safety').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.world_scale = float(self.get_parameter('world_scale').value)
        self.cruise_z = float(self.get_parameter('cruise_z').value)
        self.goal_tol = float(self.get_parameter('goal_tol').value)
        self.hold_exit_m = float(self.get_parameter('hold_exit_m').value)
        self.approach_m = float(self.get_parameter('approach_m').value)
        self.action_ema = float(np.clip(self.get_parameter('action_ema').value, 0.0, 0.95))
        self.fallback_clear_m = float(self.get_parameter('fallback_clear_m').value)
        self.path_horizon_m = float(self.get_parameter('path_horizon_m').value)
        self.path_step_m = float(self.get_parameter('path_step_m').value)
        self.blend_clear_m = float(self.get_parameter('blend_clear_m').value)

        self._agent = None
        self._have_policy = False
        ckpt = str(self.get_parameter('checkpoint').value).strip()
        if not ckpt:
            ckpt = self._find_default_checkpoint()
        if ckpt:
            self._load_policy(ckpt)

        self._odom: Optional[Odometry] = None
        self._goal: Optional[PoseStamped] = None
        self._cloud: Optional[np.ndarray] = None
        self._prev_action = np.zeros(3, dtype=np.float64)
        self._filt_action = np.zeros(3, dtype=np.float64)
        self._have_filt = False
        self._heading = 0.0
        self._holding = False  # latch hover after first reach
        self._hold_yaw = 0.0
        self._goal_xy: Optional[np.ndarray] = None

        direct = bool(self.get_parameter('direct_plant').value)
        lg_topic = '/planner/local_goal' if direct else str(
            self.get_parameter('local_goal_topic').value)
        tr_topic = '/planner/trajectory' if direct else str(
            self.get_parameter('trajectory_topic').value)
        st_topic = '/planner/status' if direct else str(
            self.get_parameter('status_topic').value)

        self._local_pub = self.create_publisher(PoseStamped, lg_topic, 10)
        self._path_pub = self.create_publisher(NavPath, tr_topic, _latched())
        self._status_pub = self.create_publisher(PlannerStatus, st_topic, 10)
        self._marker_pub = self.create_publisher(Marker, '/planner/local_goal_marker', 10)

        self.create_subscription(
            PointCloud2, str(self.get_parameter('map_topic').value),
            self._on_cloud, _latched())
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self._on_odom, 20)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('goal_topic').value), self._on_goal, 10)

        hz = float(self.get_parameter('control_hz').value)
        self.create_timer(max(0.04, 1.0 / hz), self._tick)
        mode = 'SAC' if self._have_policy else 'heuristic-heading'
        self.get_logger().info(
            f'sac_planner_node ready ({mode}, out={lg_topic}, direct_plant={direct})')

    def _workspace_roots(self) -> List[Path]:
        roots = []
        env_ws = os.environ.get('DRONE_WS', '').strip()
        if env_ws:
            roots.append(Path(env_ws))
        roots.append(Path(__file__).resolve().parents[3])
        roots.append(Path.home() / 'drone_ws')
        out, seen = [], set()
        for r in roots:
            try:
                r = r.resolve()
            except Exception:
                continue
            if r not in seen:
                seen.add(r)
                out.append(r)
        return out

    def _find_default_checkpoint(self) -> str:
        names = ('sac_polar_local_best.pt', 'sac_polar_local.pt')
        for ws in self._workspace_roots():
            base = ws / 'src' / 'drone_rl_planner' / 'checkpoints'
            for n in names:
                p = base / n
                if p.is_file():
                    return str(p)
        return ''

    def _load_policy(self, ckpt: str) -> None:
        try:
            import torch
            from drone_rl_planner.sac_drq import SACAgent, SACConfig
            device = 'cpu'
            agent = SACAgent(
                (2, self.n_rings, self.n_sectors),
                POLAR_DEFAULTS['vec_dim'],
                3,
                SACConfig(device=device),
            )
            agent.load(ckpt, map_location=device)
            self._agent = agent
            self._have_policy = True
            self.get_logger().info(f'Loaded Path H SAC {ckpt}')
        except Exception as exc:
            self.get_logger().error(f'SAC load failed: {exc} — using VFH fallback')
            self._have_policy = False

    def _on_cloud(self, msg: PointCloud2) -> None:
        pts = list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not pts:
            self._cloud = np.zeros((0, 3))
            return
        arr = np.asarray([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
        self._cloud = downsample_cloud(arr)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_goal(self, msg: PoseStamped) -> None:
        if msg.pose.position.z < 0.3:
            msg.pose.position.z = self.cruise_z
        # New goal → release hold latch
        new_xy = np.array([msg.pose.position.x, msg.pose.position.y], dtype=np.float64)
        if self._goal_xy is None or float(np.linalg.norm(new_xy - self._goal_xy)) > 0.35:
            self._holding = False
            self._have_filt = False
            self._prev_action[:] = 0.0
        self._goal_xy = new_xy
        self._goal = msg

    def _ema(self, action: np.ndarray) -> np.ndarray:
        a = np.clip(action, -1.0, 1.0)
        if not self._have_filt or self.action_ema <= 0.0:
            self._filt_action = a.copy()
            self._have_filt = True
            return self._filt_action
        α = self.action_ema
        self._filt_action = (1.0 - α) * a + α * self._filt_action
        return self._filt_action

    def _goal_heading(self, pos_xy: np.ndarray, goal_xy: np.ndarray) -> float:
        d = goal_xy - pos_xy
        if float(np.linalg.norm(d)) < 1e-6:
            return self._heading
        return float(np.arctan2(d[1], d[0]))

    def _publish_marker(self, target: np.ndarray, stamp) -> None:
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = 'map'
        m.ns = 'sac_local_goal'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(target[0])
        m.pose.position.y = float(target[1])
        m.pose.position.z = float(self.cruise_z)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.35
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.85, 1.0, 0.95
        self._marker_pub.publish(m)

    def _publish_cmd(
        self,
        target: np.ndarray,
        path_xy: np.ndarray,
        yaw: float,
        stamp,
        dist_goal: float,
        min_clear: float,
        mode: str,
        success: bool,
        append_goal_tip: bool = False,
    ) -> None:
        lg = PoseStamped()
        lg.header.stamp = stamp
        lg.header.frame_id = 'map'
        lg.pose.position.x = float(target[0])
        lg.pose.position.y = float(target[1])
        lg.pose.position.z = float(self.cruise_z)
        lg.pose.orientation.z = float(np.sin(yaw * 0.5))
        lg.pose.orientation.w = float(np.cos(yaw * 0.5))
        self._local_pub.publish(lg)
        self._publish_marker(target, stamp)

        path = NavPath()
        path.header.stamp = stamp
        path.header.frame_id = 'map'
        for xy in path_xy:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(xy[0])
            ps.pose.position.y = float(xy[1])
            ps.pose.position.z = float(self.cruise_z)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        if append_goal_tip and self._goal is not None:
            tip = PoseStamped()
            tip.header = path.header
            tip.pose.position.x = float(self._goal.pose.position.x)
            tip.pose.position.y = float(self._goal.pose.position.y)
            tip.pose.position.z = float(self.cruise_z)
            tip.pose.orientation.w = 1.0
            path.poses.append(tip)
        self._path_pub.publish(path)

        st = PlannerStatus()
        st.header = lg.header
        st.state = 'REACHED' if success else 'EXEC_TRAJ'
        st.success = success
        st.message = f'path_h_{mode}'
        st.path_length = dist_goal
        st.min_obstacle_distance = float(min_clear)
        self._status_pub.publish(st)

    def _tick(self) -> None:
        if self._odom is None or self._goal is None:
            return
        p = self._odom.pose.pose.position
        g = self._goal.pose.position
        pos = np.array([p.x, p.y], dtype=np.float64)
        vel = world_vel_2d(self._odom)
        goal = np.array([g.x, g.y], dtype=np.float64)
        to_goal = goal - pos
        dist_goal = float(np.linalg.norm(to_goal))
        stamp = self.get_clock().now().to_msg()
        goal_yaw = (
            float(np.arctan2(to_goal[1], to_goal[0])) if dist_goal > 1e-3 else self._heading
        )

        cloud_xy = None
        if self._cloud is not None and self._cloud.size:
            band = np.abs(self._cloud[:, 2] - self.cruise_z) <= 1.4
            cloud_xy = self._cloud[band][:, :2] if np.any(band) else self._cloud[:, :2]

        # Hold latch: after first reach, pin local_goal at goal (stops orbiting)
        if self._holding:
            if dist_goal > self.hold_exit_m:
                self._holding = False
            else:
                self._heading = self._hold_yaw
                self._publish_cmd(
                    goal, np.stack([pos, goal], axis=0), self._hold_yaw, stamp,
                    dist_goal, self.ray_max, 'HOLD', True)
                return

        if dist_goal < self.goal_tol:
            self._holding = True
            self._hold_yaw = goal_yaw if dist_goal > 0.05 else self._heading
            self._heading = self._hold_yaw
            self._have_filt = False
            self._prev_action[:] = 0.0
            self._publish_cmd(
                goal, np.stack([pos, goal], axis=0), self._hold_yaw, stamp,
                dist_goal, self.ray_max, 'REACHED', True)
            return

        # Final approach: straight to goal
        if dist_goal < self.approach_m:
            d = ((goal_yaw - self._heading + np.pi) % (2 * np.pi)) - np.pi
            self._heading = self._heading + 0.75 * d
            target = goal.copy()
            if dist_goal > 1.2:
                target = pos + to_goal / dist_goal * max(0.55, dist_goal * 0.5)
            min_clear = self.ray_max
            if cloud_xy is not None:
                tip = pos + np.array([np.cos(self._heading), np.sin(self._heading)]) * min(
                    1.0, dist_goal)
                min_clear = corridor_clearance(
                    np.stack([pos, tip], axis=0), cloud_xy,
                    half_width=self.robot_r + 0.2)
            self._publish_cmd(
                target, np.stack([pos, goal], axis=0), self._heading, stamp,
                dist_goal, min_clear, 'APPROACH', False)
            return

        look = 1.6
        if self._have_policy and self._agent is not None:
            img = build_polar_image(
                pos, goal, self._cloud,
                n_rings=self.n_rings, n_sectors=self.n_sectors,
                ray_max=self.ray_max, robot_r=self.robot_r,
                safety=self.safety, cruise_z=self.cruise_z,
            )
            vec = build_polar_vector(
                pos, vel, goal, self._prev_action,
                max_speed=self.max_speed, world_scale=self.world_scale,
            )
            raw = self._agent.act({'image': img, 'vector': vec}, deterministic=True)
            action = self._ema(raw)
            heading, look, _speed = decode_action(action, goal, pos)
            self._prev_action = action.copy()
            mode = 'SAC'
        else:
            heading = self._goal_heading(pos, goal)
            mode = 'GOAL'

        d = ((heading - self._heading + np.pi) % (2 * np.pi)) - np.pi
        self._heading = self._heading + 0.55 * d
        self._heading = (self._heading + np.pi) % (2 * np.pi) - np.pi

        def heading_fn(cur_xy, goal_ang, cur_h):
            # Pure SAC path roll: bias toward filtered policy heading + goal.
            d2 = ((self._heading - goal_ang + np.pi) % (2 * np.pi)) - np.pi
            return goal_ang + 0.55 * d2

        path_xy, _, min_clear = build_rolled_path(
            pos, self._heading, goal, heading_fn, cloud_xy,
            step_m=self.path_step_m, horizon_m=self.path_horizon_m,
            robot_r=self.robot_r, safety=self.safety, goal_tol=self.goal_tol,
        )
        look_cap = min(1.7, max(1.0, look), max(0.8, dist_goal - 0.35))
        cum = 0.0
        target = path_xy[min(3, len(path_xy) - 1)].copy()
        for i in range(1, len(path_xy)):
            cum += float(np.linalg.norm(path_xy[i] - path_xy[i - 1]))
            if cum >= look_cap:
                target = path_xy[i].copy()
                break
        if float(np.linalg.norm(target - goal)) > dist_goal:
            target = goal.copy()
        vec_t = target - pos
        dlen = float(np.linalg.norm(vec_t))
        if dlen > 2.0:
            target = pos + vec_t / dlen * 2.0

        self._publish_cmd(
            target, path_xy, self._heading, stamp, dist_goal, min_clear, mode, False,
            append_goal_tip=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SacPlannerNode()
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

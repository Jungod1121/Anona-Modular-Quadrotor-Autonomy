"""ROS2 inference node: PPO policy → /planner/local_goal (+ path).

Patterns from open-source UAV RL (ntnu-arl/rl_nav):
  - EMA filter + heading rate-limit (inference-only smoothing)
  - local_goal ahead of PID settling radius, but short enough to track
  - Transient-local Path QoS for RViz PlannedTrajectory

Action contract (matches training):
  a ∈ [-1,1]² = desired XY velocity / max_speed
  Flight speed is scaled by cmd_speed_scale (obs still uses max_speed).
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import List, Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_rl_planner.odom_util import world_vel_2d
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker

from drone_msgs.msg import PlannerStatus
from drone_rl_planner.sensing import OBS_DEFAULTS, build_observation, voxel_downsample


def _latched() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


class RlPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('rl_planner_node')
        self.declare_parameter('checkpoint', '')
        self.declare_parameter('n_rays', OBS_DEFAULTS['n_rays'])
        self.declare_parameter('ray_max', OBS_DEFAULTS['ray_max'])
        self.declare_parameter('max_speed', OBS_DEFAULTS['max_speed'])
        self.declare_parameter('world_scale', OBS_DEFAULTS['world_scale'])
        self.declare_parameter('robot_r', OBS_DEFAULTS['robot_r'])
        self.declare_parameter('cruise_z', 1.5)
        self.declare_parameter('lookahead_m', 1.4)
        self.declare_parameter('horizon_s', 0.45)
        self.declare_parameter('lookahead', 1.4)
        self.declare_parameter('goal_tol', 0.70)
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('action_ema', 0.55)
        self.declare_parameter('cmd_speed_scale', 0.65)
        self.declare_parameter('pred_horizon_m', 4.5)
        self.declare_parameter('pred_step_m', 0.35)
        self.declare_parameter('dir_rate_limit', 1.8)
        self.declare_parameter('map_topic', '/map/obstacles')
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter('goal_topic', '/drone/goal')
        self.declare_parameter('peer_namespaces', '')

        self.n_rays = int(self.get_parameter('n_rays').value)
        self.ray_max = float(self.get_parameter('ray_max').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.world_scale = float(self.get_parameter('world_scale').value)
        self.robot_r = float(self.get_parameter('robot_r').value)
        self.cruise_z = float(self.get_parameter('cruise_z').value)
        look = float(self.get_parameter('lookahead_m').value)
        if look <= 0.0:
            look = float(self.get_parameter('lookahead').value)
        self.lookahead_m = float(np.clip(look, 1.0, 3.0))
        self.goal_tol = float(self.get_parameter('goal_tol').value)
        self.action_ema = float(np.clip(self.get_parameter('action_ema').value, 0.0, 0.95))
        self.cmd_speed_scale = float(np.clip(
            self.get_parameter('cmd_speed_scale').value, 0.25, 1.0))
        self.pred_horizon_m = float(self.get_parameter('pred_horizon_m').value)
        self.pred_step_m = float(self.get_parameter('pred_step_m').value)
        self.dir_rate_limit = float(self.get_parameter('dir_rate_limit').value)
        self._cmd_speed = self.max_speed * self.cmd_speed_scale

        self._filt_action = np.zeros(2, dtype=np.float64)
        self._have_filt = False
        self._filt_dir = np.array([1.0, 0.0], dtype=np.float64)
        self._have_dir = False
        self._last_tick = None

        ckpt = str(self.get_parameter('checkpoint').value).strip()
        self.sb3_model = None
        self.ac = None
        if not ckpt:
            ckpt = self._find_default_checkpoint()
        if ckpt:
            self._load_policy(ckpt)
        else:
            self.get_logger().error(
                'No checkpoint found — heuristic fallback only. '
                'Train: python3 -m drone_rl_planner.train_sb3_ppo')

        self._odom: Optional[Odometry] = None
        self._goal: Optional[PoseStamped] = None
        self._cloud: Optional[np.ndarray] = None
        self._peer_pos: dict = {}

        self._local_pub = self.create_publisher(PoseStamped, '/planner/local_goal', 10)
        self._path_pub = self.create_publisher(NavPath, '/planner/trajectory', _latched())
        self._status_pub = self.create_publisher(PlannerStatus, '/planner/status', 10)
        self._marker_pub = self.create_publisher(Marker, '/planner/local_goal_marker', 10)

        map_topic = str(self.get_parameter('map_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        goal_topic = str(self.get_parameter('goal_topic').value)
        self.create_subscription(PointCloud2, map_topic, self._on_cloud, _latched())
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)

        peers = [
            s.strip()
            for s in str(self.get_parameter('peer_namespaces').value).split(',')
            if s.strip()
        ]
        for ns in peers:
            self.create_subscription(
                Odometry, f'/{ns}/drone/odom',
                lambda msg, n=ns: self._on_peer(n, msg), 10)

        hz = float(self.get_parameter('control_hz').value)
        self.create_timer(max(0.04, 1.0 / hz), self._tick)
        self.get_logger().info(
            f'rl_planner_node ready '
            f'(lookahead={self.lookahead_m:.2f}m, ema={self.action_ema:.2f}, '
            f'cmd_speed={self._cmd_speed:.2f} m/s)')

    def _workspace_candidates(self) -> List[Path]:
        roots: List[Path] = []
        env_ws = os.environ.get('DRONE_WS', '').strip()
        if env_ws:
            roots.append(Path(env_ws))
        roots.append(Path(__file__).resolve().parents[3])
        roots.append(Path.home() / 'drone_ws')
        out: List[Path] = []
        seen = set()
        for r in roots:
            try:
                r = r.resolve()
            except Exception:
                continue
            if r in seen:
                continue
            seen.add(r)
            out.append(r)
        return out

    def _find_default_checkpoint(self) -> str:
        candidates: List[Path] = []
        for ws in self._workspace_candidates():
            base = ws / 'src' / 'drone_rl_planner' / 'checkpoints'
            candidates.append(base / 'sb3_ppo_local.zip')
            candidates.append(base / 'sb3_ppo_local')
        candidates.append(
            Path(__file__).resolve().parents[1] / 'checkpoints' / 'sb3_ppo_local.zip')
        try:
            share = Path(get_package_share_directory('drone_rl_planner')) / 'checkpoints'
            candidates.append(share / 'sb3_ppo_local.zip')
            candidates.append(share / 'sb3_ppo_local')
        except Exception:
            pass
        for cand in candidates:
            if cand.is_file():
                return str(cand.with_suffix('') if cand.suffix == '.zip' else cand)
            if cand.with_suffix('.zip').is_file():
                return str(cand)
        return ''

    def _load_policy(self, ckpt: str) -> None:
        path = Path(ckpt)
        zip_path = path if path.suffix == '.zip' else path.with_suffix('.zip')
        if zip_path.is_file() or (path.is_file() and path.suffix == '.zip'):
            try:
                from stable_baselines3 import PPO
                load_p = str(zip_path if zip_path.is_file() else path)
                self.sb3_model = PPO.load(load_p, device='cpu')
                self.get_logger().info(f'Loaded SB3 PPO {load_p}')
                return
            except Exception as exc:
                self.get_logger().error(f'SB3 load failed: {exc}')
        if path.is_file() and path.suffix == '.npz':
            from drone_rl_planner.ppo import ActorCritic
            self.ac = ActorCritic.load(path)
            self.get_logger().info(f'Loaded legacy npz {path}')

    def _on_cloud(self, msg: PointCloud2) -> None:
        pts = list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not pts:
            self._cloud = np.zeros((0, 3))
            return
        arr = np.asarray([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
        self._cloud = voxel_downsample(arr, voxel=0.25, max_pts=80000)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_goal(self, msg: PoseStamped) -> None:
        if msg.pose.position.z < 0.3:
            msg.pose.position.z = self.cruise_z
        self._goal = msg
        self._have_filt = False
        self._filt_action[:] = 0.0
        self._have_dir = False

    def _on_peer(self, ns: str, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._peer_pos[ns] = np.array([p.x, p.y, p.z], dtype=np.float64)

    def _heuristic_action(self, obs: np.ndarray) -> np.ndarray:
        gx, gy = float(obs[self.n_rays]), float(obs[self.n_rays + 1])
        rays = obs[: self.n_rays]
        angles = np.linspace(-np.pi, np.pi, self.n_rays, endpoint=False)
        i = int(np.argmin(rays))
        if rays[i] < 0.35:
            ang = angles[i] + np.pi
            return np.array([np.cos(ang), np.sin(ang)]) * 0.8
        return np.array([gx, gy])

    def _reactive_avoid(self, obs: np.ndarray, action: np.ndarray, to_goal: np.ndarray) -> np.ndarray:
        """VFH-style early dodge (PX4-Avoidance idea) — react before RL panics.

        Starts steering away ~2.5 m out so we do not hug obstacles then loop.
        """
        rays = obs[: self.n_rays]
        angles = np.linspace(-np.pi, np.pi, self.n_rays, endpoint=False)
        goal_ang = float(np.arctan2(to_goal[1], to_goal[0]))
        # Forward-weighted nearest hit
        best_i = 0
        best_cost = 1e9
        for i, (ang, r) in enumerate(zip(angles, rays)):
            d_head = abs(((ang - goal_ang + np.pi) % (2 * np.pi)) - np.pi)
            # Prefer obstacles near the goal heading
            cost = float(r) + 0.15 * d_head
            if cost < best_cost:
                best_cost = cost
                best_i = i
        clear_m = float(rays[best_i]) * self.ray_max
        if clear_m >= 2.6:
            return action

        # Strength ramps from 2.6 m → 0.9 m
        strength = float(np.clip((2.6 - clear_m) / 1.7, 0.0, 1.0))
        obs_ang = float(angles[best_i])
        # Two escape headings: ±90° from obstacle bearing; pick closer to goal
        cand = [obs_ang + 0.5 * np.pi, obs_ang - 0.5 * np.pi]
        escape = cand[0]
        best_align = -1e9
        for a in cand:
            align = np.cos(a - goal_ang)
            # Also prefer the side that is clearer
            # map angle to ray index
            idx = int(np.argmin(np.abs(((angles - a + np.pi) % (2 * np.pi)) - np.pi)))
            score = align + 0.5 * float(rays[idx])
            if score > best_align:
                best_align = score
                escape = a
        avoid = np.array([np.cos(escape), np.sin(escape)], dtype=np.float64)
        mixed = (1.0 - strength) * action + strength * avoid
        n = float(np.linalg.norm(mixed))
        if n < 1e-6:
            return avoid
        # Keep some magnitude so we do not stall while dodging
        mag = max(float(np.linalg.norm(action)), 0.55)
        return mixed / n * min(1.0, mag + 0.15 * strength)

    def _ema(self, action: np.ndarray) -> np.ndarray:
        a = np.clip(action, -1.0, 1.0)
        if not self._have_filt or self.action_ema <= 0.0:
            self._filt_action = a.copy()
            self._have_filt = True
            return self._filt_action
        α = self.action_ema
        self._filt_action = (1.0 - α) * a + α * self._filt_action
        return self._filt_action

    def _smooth_direction(self, direction: np.ndarray, dt: float) -> np.ndarray:
        d = direction / max(float(np.linalg.norm(direction)), 1e-6)
        if not self._have_dir:
            self._filt_dir = d.copy()
            self._have_dir = True
            return self._filt_dir
        cross = self._filt_dir[0] * d[1] - self._filt_dir[1] * d[0]
        dot = float(np.clip(np.dot(self._filt_dir, d), -1.0, 1.0))
        ang = float(np.arctan2(cross, dot))
        max_step = self.dir_rate_limit * max(dt, 1e-3)
        if abs(ang) > max_step:
            ca = np.cos(np.sign(ang) * max_step)
            sa = np.sin(np.sign(ang) * max_step)
            fx, fy = self._filt_dir
            self._filt_dir = np.array([fx * ca - fy * sa, fx * sa + fy * ca])
            n = float(np.linalg.norm(self._filt_dir))
            if n > 1e-6:
                self._filt_dir /= n
        else:
            mixed = 0.7 * self._filt_dir + 0.3 * d
            n = float(np.linalg.norm(mixed))
            self._filt_dir = mixed / max(n, 1e-6)
        return self._filt_dir

    def _clearance_ahead(self, obs: np.ndarray, direction: np.ndarray) -> float:
        rays = obs[: self.n_rays]
        angles = np.linspace(-np.pi, np.pi, self.n_rays, endpoint=False)
        heading = float(np.arctan2(direction[1], direction[0]))
        best = self.ray_max
        for ang, r in zip(angles, rays):
            d = abs(((ang - heading + np.pi) % (2 * np.pi)) - np.pi)
            if d < np.deg2rad(35.0):
                best = min(best, float(r) * self.ray_max)
        return best

    def _predict_path(
        self,
        pos: np.ndarray,
        direction: np.ndarray,
        goal_xy: np.ndarray,
        clearance_m: float,
        stamp,
    ) -> NavPath:
        path = NavPath()
        path.header.stamp = stamp
        path.header.frame_id = 'map'

        def add_pt(xy: np.ndarray) -> None:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(xy[0])
            ps.pose.position.y = float(xy[1])
            ps.pose.position.z = float(self.cruise_z)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        add_pt(pos[:2])
        horizon = min(self.pred_horizon_m, max(1.0, clearance_m - 0.3))
        n = max(2, int(horizon / max(self.pred_step_m, 0.2)))
        for i in range(1, n + 1):
            t = i / n
            xy = pos[:2] + direction * (horizon * t)
            if clearance_m > 2.5:
                to_g = goal_xy - xy
                dg = float(np.linalg.norm(to_g))
                if dg > 1e-3:
                    xy = xy + 0.12 * t * (to_g / dg) * self.pred_step_m
            add_pt(xy)
        add_pt(goal_xy)
        return path

    def _publish_goal_marker(self, target: np.ndarray, stamp) -> None:
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = 'map'
        m.ns = 'rl_local_goal'
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
        now = self.get_clock().now()
        if self._last_tick is None:
            dt = 0.05
        else:
            dt = max(0.02, min(0.2, (now - self._last_tick).nanoseconds * 1e-9))
        self._last_tick = now

        p = self._odom.pose.pose.position
        g = self._goal.pose.position
        pos = np.array([p.x, p.y, p.z], dtype=np.float64)
        vel = world_vel_2d(self._odom)
        goal = np.array([g.x, g.y, g.z], dtype=np.float64)
        to_goal = goal[:2] - pos[:2]
        dist_goal = float(np.linalg.norm(to_goal))
        stamp = now.to_msg()

        cloud = self._cloud
        if self._peer_pos and cloud is not None:
            peers = np.asarray(list(self._peer_pos.values()), dtype=np.float64)
            if peers.size:
                cloud = np.concatenate([cloud, peers], axis=0)

        obs = build_observation(
            pos, vel, goal, cloud,
            n_rays=self.n_rays, ray_max=self.ray_max,
            max_speed=self.max_speed, world_scale=self.world_scale,
            cruise_z=self.cruise_z, robot_r=self.robot_r,
        )

        if self.sb3_model is not None:
            raw, _ = self.sb3_model.predict(obs, deterministic=True)
            raw = np.asarray(raw, dtype=np.float64).ravel()
            if not np.all(np.isfinite(raw)):
                self.get_logger().error(
                    'policy produced non-finite action — zeroing',
                    throttle_duration_sec=2.0)
                raw = np.zeros_like(raw)
            mode = 'SB3-PPO'
        elif self.ac is not None and obs.shape[0] == self.ac.obs_dim:
            raw = self.ac.mean_of(obs)
            mode = 'legacy-PPO'
        else:
            raw = self._heuristic_action(obs)
            mode = 'heuristic'

        action = self._ema(raw)
        action = self._reactive_avoid(obs, action, to_goal)

        if dist_goal < self.goal_tol:
            target = goal[:2].copy()
            direction = to_goal / max(dist_goal, 1e-6)
            direction = self._smooth_direction(direction, dt)
            state = 'REACHED'
            success = True
            clearance = self.ray_max
        else:
            desired = action * self._cmd_speed
            speed = float(np.linalg.norm(desired))
            if speed < 0.05:
                direction = to_goal / max(dist_goal, 1e-6)
            else:
                direction = desired / speed
                clear = self._clearance_ahead(obs, direction)
                if clear > 2.0:
                    goal_dir = to_goal / max(dist_goal, 1e-6)
                    mixed = 0.80 * direction + 0.20 * goal_dir
                    nrm = float(np.linalg.norm(mixed))
                    if nrm > 1e-6:
                        direction = mixed / nrm

            direction = self._smooth_direction(direction, dt)
            clearance = self._clearance_ahead(obs, direction)
            look = min(self.lookahead_m, max(1.05, clearance - 0.4))
            look = min(look, max(0.9, dist_goal - 0.15))
            target = pos[:2] + direction * look
            state = 'EXEC_TRAJ'
            success = False

        for peer in self._peer_pos.values():
            d = target - peer[:2]
            dist = float(np.linalg.norm(d))
            if dist < 1.2 and dist > 1e-3:
                target = target + (d / dist) * (1.2 - dist)

        lg = PoseStamped()
        lg.header.stamp = stamp
        lg.header.frame_id = 'map'
        lg.pose.position.x = float(target[0])
        lg.pose.position.y = float(target[1])
        lg.pose.position.z = float(self.cruise_z)
        yaw = float(np.arctan2(direction[1], direction[0]))
        lg.pose.orientation.z = float(np.sin(yaw * 0.5))
        lg.pose.orientation.w = float(np.cos(yaw * 0.5))
        self._local_pub.publish(lg)
        self._publish_goal_marker(target, stamp)

        path = self._predict_path(pos, direction, goal[:2], clearance, stamp)
        self._path_pub.publish(path)

        st = PlannerStatus()
        st.header = lg.header
        st.state = state
        st.success = success
        st.message = f'rl_{mode}'
        st.path_length = dist_goal
        st.min_obstacle_distance = float(np.min(obs[: self.n_rays]) * self.ray_max)
        self._status_pub.publish(st)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RlPlannerNode()
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

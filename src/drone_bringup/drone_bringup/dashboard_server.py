#!/usr/bin/env python3
"""Local web control panel for Path A/B/C/D — replaces typing ros2 launch by hand.

Serves a single-page UI and manages one ros2 launch process at a time.
"""

from __future__ import annotations

import base64
import json
import math
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from urllib.parse import unquote, urlparse

# Optional ROS graph helpers (status / goal). Dashboard still works if ROS is only used via CLI.
try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2
    from std_msgs.msg import Bool, String
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    _HAVE_RCLPY = True
    _HAVE_PC2 = True
except ImportError:  # pragma: no cover
    _HAVE_RCLPY = False
    _HAVE_PC2 = False
    String = None  # type: ignore
    Bool = None  # type: ignore
    OccupancyGrid = None  # type: ignore
    NavPath = None  # type: ignore
    PointCloud2 = None  # type: ignore
    pc2 = None  # type: ignore
    MultiThreadedExecutor = None  # type: ignore
    DurabilityPolicy = None  # type: ignore
    HistoryPolicy = None  # type: ignore
    QoSProfile = None  # type: ignore
    ReliabilityPolicy = None  # type: ignore

try:
    from drone_msgs.msg import PlannerDiagnostics, PlannerStatus
    _HAVE_PLANNER_MSGS = True
except ImportError:  # pragma: no cover
    _HAVE_PLANNER_MSGS = False
    PlannerDiagnostics = None  # type: ignore
    PlannerStatus = None  # type: ignore


def _signal_process_group(proc: subprocess.Popen, sig: signal.Signals) -> None:
    """Send signal to a session leader; fall back to the Popen handle if needed."""
    try:
        os.killpg(proc.pid, sig)
        return
    except ProcessLookupError:
        return
    except PermissionError:
        pass
    try:
        proc.send_signal(sig)
    except ProcessLookupError:
        pass
    except Exception:
        try:
            if sig in (signal.SIGKILL, signal.SIGTERM):
                proc.kill()
            else:
                proc.terminate()
        except ProcessLookupError:
            pass

from drone_bringup.maps_catalog import (
    DEFAULT_MAP_BY_PLANNER,
    MAPS,
    map_public_info,
    normalize_map_id,
)
from drone_bringup.planner_registry import (
    MULTI_MODES,
    RATES,
    dashboard_planners_legacy,
    normalize_planner_id,
    planner_public_info,
)

ROOT = Path(__file__).resolve().parent
STATIC_DIR = ROOT / 'dashboard_static'
BG_USER_DIR = Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'drone-ws' / 'backgrounds'
# No practical size cap — wallpapers can be large; keep type/name checks only.
BG_MAX_BYTES = None
BG_ALLOWED_EXT = {'.png', '.jpg', '.jpeg', '.webp', '.gif', '.svg'}
_BG_NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,120}$')
_STATE = {
    'ws_root': Path(os.environ.get('DRONE_WS', Path.home() / 'drone_ws')).resolve(),
}


def ws_root() -> Path:
    return _STATE['ws_root']


def _bg_user_dir() -> Path:
    BG_USER_DIR.mkdir(parents=True, exist_ok=True)
    return BG_USER_DIR


def _safe_bg_name(name: str) -> Optional[str]:
    name = Path(str(name or '')).name
    if not name or not _BG_NAME_RE.match(name):
        return None
    if Path(name).suffix.lower() not in BG_ALLOWED_EXT:
        return None
    return name


def _list_backgrounds() -> List[Dict[str, Any]]:
    items: List[Dict[str, Any]] = []
    # Packaged defaults living in dashboard_static/
    for path in sorted(STATIC_DIR.glob('bg-*.png')) + sorted((STATIC_DIR / 'backgrounds').glob('*')):
        if not path.is_file():
            continue
        if path.suffix.lower() not in BG_ALLOWED_EXT:
            continue
        rel = path.relative_to(STATIC_DIR).as_posix()
        items.append({
            'id': f'builtin:{path.name}',
            'name': path.stem.replace('-', ' ').replace('_', ' '),
            'url': f'/{rel}',
            'builtin': True,
            'deletable': False,
        })
    # User uploads
    for path in sorted(_bg_user_dir().iterdir()):
        if not path.is_file() or path.suffix.lower() not in BG_ALLOWED_EXT:
            continue
        items.append({
            'id': f'user:{path.name}',
            'name': path.stem,
            'url': f'/user-bg/{path.name}',
            'builtin': False,
            'deletable': True,
        })
    # Stable unique by url
    seen = set()
    out = []
    for it in items:
        if it['url'] in seen:
            continue
        seen.add(it['url'])
        out.append(it)
    return out


def _save_background_upload(filename: str, data_b64: str) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
    raw_name = Path(str(filename or 'upload.png')).name
    ext = Path(raw_name).suffix.lower()
    if ext not in BG_ALLOWED_EXT:
        return None, f'unsupported type (use {", ".join(sorted(BG_ALLOWED_EXT))})'
    stem = re.sub(r'[^A-Za-z0-9._-]+', '-', Path(raw_name).stem).strip('.-_') or 'bg'
    stem = stem[:80]
    safe = _safe_bg_name(f'{stem}-{uuid.uuid4().hex[:8]}{ext}')
    if not safe:
        return None, 'invalid filename'
    try:
        payload = data_b64.split(',', 1)[-1] if ',' in data_b64 else data_b64
        raw = base64.b64decode(payload, validate=False)
    except Exception:
        return None, 'invalid base64 image data'
    if not raw:
        return None, 'empty image'
    if BG_MAX_BYTES is not None and len(raw) > BG_MAX_BYTES:
        return None, f'image too large (max {BG_MAX_BYTES // (1024 * 1024)}MB)'
    path = _bg_user_dir() / safe
    path.write_bytes(raw)
    return {
        'id': f'user:{safe}',
        'name': Path(safe).stem,
        'url': f'/user-bg/{safe}',
        'builtin': False,
        'deletable': True,
    }, None


def _delete_background(bg_id: str) -> Tuple[bool, str]:
    raw = str(bg_id or '')
    if not raw.startswith('user:'):
        return False, 'only uploaded backgrounds can be deleted'
    name = _safe_bg_name(raw.split(':', 1)[1])
    if not name:
        return False, 'invalid id'
    path = (_bg_user_dir() / name).resolve()
    if not str(path).startswith(str(_bg_user_dir().resolve())) or not path.is_file():
        return False, 'not found'
    path.unlink()
    return True, 'ok'

# Canonical registry (single source of truth). Keep PLANNERS name for older code.
PLANNERS = dashboard_planners_legacy()



class RosStatus:
    """Background rclpy node for live odom + goal publish."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.odom: Optional[Dict[str, float]] = None
        self.swarm_odom: Dict[str, Dict[str, float]] = {}
        self.planned_path: Optional[List[Dict[str, float]]] = None
        self.swarm_planned: Dict[str, List[Dict[str, float]]] = {}
        self.exploration_status: str = ''
        self.planner_status: Optional[Dict[str, Any]] = None
        self.planner_diagnostics: Optional[Dict[str, Any]] = None
        self.fallback_active: bool = False
        self.occupancy: Optional[Dict[str, Any]] = None
        self._occupancy_from_cloud = False
        self._node: Any = None
        self._goal_pub: Any = None
        self._goal_pubs: Dict[str, Any] = {}
        self._running = False

    def start(self) -> None:
        if not _HAVE_RCLPY or self._running:
            return
        self._running = True
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self) -> None:
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            node = rclpy.create_node('drone_dashboard')
            self._node = node
            self._goal_pub = node.create_publisher(PoseStamped, '/drone/goal', 10)
            # Match ego_swarm UI max (20). Previously only uav0..2 → 4th drone missing on map.
            for i in range(20):
                ns = f'uav{i}'
                self._goal_pubs[ns] = node.create_publisher(
                    PoseStamped, f'/{ns}/drone/goal', 10)
                node.create_subscription(
                    Odometry, f'/{ns}/drone/odom',
                    lambda msg, n=ns: self._on_swarm_odom(n, msg), 50)
            node.create_subscription(Odometry, '/drone/odom', self._on_odom, 50)
            if String is not None:
                node.create_subscription(
                    String, '/exploration/status', self._on_explore_status, 10)
            if _HAVE_PLANNER_MSGS and PlannerStatus is not None:
                node.create_subscription(
                    PlannerStatus, '/planner/status', self._on_planner_status, 10)
            if _HAVE_PLANNER_MSGS and PlannerDiagnostics is not None:
                node.create_subscription(
                    PlannerDiagnostics, '/planner/diagnostics',
                    self._on_planner_diagnostics, 10)
            if Bool is not None:
                node.create_subscription(
                    Bool, '/planner/fallback_active', self._on_fallback_bool, 10)
            if QoSProfile is not None:
                map_qos = QoSProfile(
                    depth=1,
                    history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                path_qos = QoSProfile(
                    depth=1,
                    history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                # Same cloud RViz shows — project all Z into XY for the UI map.
                if _HAVE_PC2 and PointCloud2 is not None:
                    for topic in (
                        '/map_generator/global_cloud',
                        '/map/obstacles',
                    ):
                        node.create_subscription(
                            PointCloud2, topic, self._on_obstacle_cloud, map_qos)
                if OccupancyGrid is not None:
                    node.create_subscription(
                        OccupancyGrid, '/map/occupancy_topdown',
                        self._on_occupancy_topdown, map_qos)
                    node.create_subscription(
                        OccupancyGrid, '/map/occupancy',
                        self._on_occupancy, map_qos)
                if NavPath is not None:
                    node.create_subscription(
                        NavPath, '/planner/trajectory',
                        self._on_planned_path, path_qos)
                    for i in range(20):
                        ns = f'uav{i}'
                        node.create_subscription(
                            NavPath, f'/{ns}/planner/trajectory',
                            lambda msg, n=ns: self._on_swarm_planned(n, msg),
                            path_qos)
            # MultiThreadedExecutor: a lone spin_once(~20 ms) only services ~50
            # callbacks/s across ALL topics. With N×100 Hz odom the map starves
            # and only one UAV pose updates in the dashboard.
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(node)
            try:
                while self._running and rclpy.ok():
                    executor.spin_once(timeout_sec=0.05)
            finally:
                try:
                    executor.remove_node(node)
                    executor.shutdown()
                except Exception:
                    pass
        except Exception as exc:
            print(f'[dashboard ROS] spin failed: {exc}', file=sys.stderr, flush=True)
        finally:
            try:
                if self._node is not None:
                    self._node.destroy_node()
            except Exception:
                pass

    def _pack_odom(self, msg: Any) -> Dict[str, float]:
        q = msg.pose.pose.orientation
        # Yaw from quaternion (ENU, yaw=0 faces +X).
        siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
        cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return {
            'x': float(msg.pose.pose.position.x),
            'y': float(msg.pose.pose.position.y),
            'z': float(msg.pose.pose.position.z),
            'vx': float(msg.twist.twist.linear.x),
            'vy': float(msg.twist.twist.linear.y),
            'vz': float(msg.twist.twist.linear.z),
            'yaw': float(yaw),
        }

    def _pack_path(self, msg: Any, max_pts: int = 400) -> List[Dict[str, float]]:
        poses = list(getattr(msg, 'poses', None) or [])
        if not poses:
            return []
        if len(poses) > max_pts:
            step = max(1, len(poses) // max_pts)
            sampled = poses[::step]
            if sampled[-1] is not poses[-1]:
                sampled.append(poses[-1])
            poses = sampled
        out: List[Dict[str, float]] = []
        for ps in poses:
            p = ps.pose.position
            out.append({'x': float(p.x), 'y': float(p.y)})
        return out

    def _on_odom(self, msg: Any) -> None:
        with self.lock:
            self.odom = self._pack_odom(msg)

    def _on_swarm_odom(self, ns: str, msg: Any) -> None:
        with self.lock:
            self.swarm_odom[ns] = self._pack_odom(msg)

    def _on_planned_path(self, msg: Any) -> None:
        packed = self._pack_path(msg)
        with self.lock:
            self.planned_path = packed or None

    def _on_swarm_planned(self, ns: str, msg: Any) -> None:
        packed = self._pack_path(msg)
        with self.lock:
            if packed:
                self.swarm_planned[ns] = packed
            else:
                self.swarm_planned.pop(ns, None)

    def live_map_snapshot(self) -> Dict[str, Any]:
        """Lightweight pose + planned path for high-rate map polling."""
        with self.lock:
            swarm = dict(self.swarm_odom)
            planned = dict(self.swarm_planned)
            odom = self.odom
            path = self.planned_path
        # Only expose the configured swarm size (avoid stale uavN ghosts).
        try:
            mode = str(MANAGER.cfg.get('mode', 'single')).lower()
            multi = str(MANAGER.cfg.get('multi_mode', 'ego_swarm'))
            if multi == 'formation':
                n = 3
            elif multi == 'shared_field':
                n = 2
            else:
                n = max(2, min(int(MANAGER.cfg.get('num_drones', 2)), 20))
        except Exception:
            mode = 'single'
            n = 20
        if mode == 'multi':
            keep = {f'uav{i}' for i in range(n)}
            # Keep any live uav* that fall in the expected set; never shrink below
            # what the mode promises when odom is already streaming.
            live = {k for k in swarm if k.startswith('uav')}
            if live:
                keep = keep | (live & {f'uav{i}' for i in range(max(n, 3))})
            swarm = {k: v for k, v in swarm.items() if k in keep}
            planned = {k: v for k, v in planned.items() if k in keep}
        return {
            'ok': True,
            'odom': odom,
            'swarm_odom': swarm,
            'planned_path': path,
            'swarm_planned': planned,
        }

    def reset_live_tracks(self) -> None:
        with self.lock:
            self.odom = None
            self.swarm_odom.clear()
            self.planned_path = None
            self.swarm_planned.clear()

    def _pack_occupancy_grid(self, msg: Any) -> Dict[str, Any]:
        info = msg.info
        occupied = [i for i, v in enumerate(msg.data) if int(v) >= 50]
        return {
            'resolution': float(info.resolution),
            'width': int(info.width),
            'height': int(info.height),
            'origin': {
                'x': float(info.origin.position.x),
                'y': float(info.origin.position.y),
            },
            'occupied': occupied,
            'source': 'grid',
        }

    def _on_occupancy_topdown(self, msg: Any) -> None:
        packed = self._pack_occupancy_grid(msg)
        packed['source'] = 'topdown'
        with self.lock:
            # Cloud projection wins when present (matches RViz PointCloud view).
            if self._occupancy_from_cloud and self.occupancy is not None:
                return
            self.occupancy = packed

    def _on_occupancy(self, msg: Any) -> None:
        packed = self._pack_occupancy_grid(msg)
        packed['source'] = 'cruise'
        with self.lock:
            # Only use thin cruise slice if nothing better arrived yet.
            if self.occupancy is not None and (
                self._occupancy_from_cloud
                or self.occupancy.get('source') in ('topdown', 'cloud')
            ):
                return
            self.occupancy = packed

    def _on_obstacle_cloud(self, msg: Any) -> None:
        """Project 3D obstacle cloud to XY — same top-down silhouette as RViz."""
        if pc2 is None:
            return
        try:
            raw_pts = list(pc2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True))
        except Exception as exc:
            print(f'[dashboard ROS] cloud read failed: {exc}', file=sys.stderr, flush=True)
            return
        if not raw_pts:
            return
        # Cap work for dense forests while keeping silhouette.
        max_pts = 60000
        if len(raw_pts) > max_pts:
            step = max(1, len(raw_pts) // max_pts)
            raw_pts = raw_pts[::step]
        pts: List[tuple] = []
        for p in raw_pts:
            try:
                pts.append((float(p[0]), float(p[1]), float(p[2])))
            except Exception:
                continue
        if not pts:
            return
        res = 0.25
        xmin = min(p[0] for p in pts)
        xmax = max(p[0] for p in pts)
        ymin = min(p[1] for p in pts)
        ymax = max(p[1] for p in pts)
        pad = res
        xmin -= pad
        ymin -= pad
        xmax += pad
        ymax += pad
        width = max(1, int(math.ceil((xmax - xmin) / res)))
        height = max(1, int(math.ceil((ymax - ymin) / res)))
        # Guard against pathological extents
        if width * height > 400_000:
            scale = math.sqrt((width * height) / 400_000.0)
            res *= scale
            width = max(1, int(math.ceil((xmax - xmin) / res)))
            height = max(1, int(math.ceil((ymax - ymin) / res)))
        occupied_set = set()
        for x, y, _z in pts:
            ix = int(math.floor((x - xmin) / res))
            iy = int(math.floor((y - ymin) / res))
            if 0 <= ix < width and 0 <= iy < height:
                occupied_set.add(iy * width + ix)
        packed = {
            'resolution': float(res),
            'width': int(width),
            'height': int(height),
            'origin': {'x': float(xmin), 'y': float(ymin)},
            'occupied': sorted(occupied_set),
            'source': 'cloud',
        }
        with self.lock:
            self.occupancy = packed
            self._occupancy_from_cloud = True

    def _on_explore_status(self, msg: Any) -> None:
        with self.lock:
            self.exploration_status = str(msg.data or '')

    def _on_planner_status(self, msg: Any) -> None:
        with self.lock:
            self.planner_status = {
                'state': str(msg.state or ''),
                'success': bool(msg.success),
                'message': str(msg.message or ''),
                'path_length': float(msg.path_length),
                'min_obstacle_distance': float(msg.min_obstacle_distance),
            }

    def _on_planner_diagnostics(self, msg: Any) -> None:
        with self.lock:
            self.planner_diagnostics = {
                'planner_id': str(msg.planner_id or ''),
                'state': str(msg.state or ''),
                'fallback_active': bool(msg.fallback_active),
                'fallback_reason': str(msg.fallback_reason or ''),
                'solve_time_ms': float(msg.solve_time_ms),
                'clearance_m': float(msg.clearance_m),
                'tracking_error_m': float(msg.tracking_error_m),
            }
            self.fallback_active = bool(msg.fallback_active)

    def _on_fallback_bool(self, msg: Any) -> None:
        with self.lock:
            self.fallback_active = bool(msg.data)

    def publish_goal(
        self, x: float, y: float, z: float, yaw: float = 0.0,
        namespace: str = '',
    ) -> bool:
        msg = PoseStamped()
        if self._node is None:
            return False
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        pub = self._goal_pubs.get(namespace) if namespace else self._goal_pub
        if pub is None:
            return False
        pub.publish(msg)
        return True

    def stop(self) -> None:
        self._running = False


class LaunchManager:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.proc: Optional[subprocess.Popen] = None
        self.cfg: Dict[str, Any] = {
            'mode': 'single',  # single | multi
            'planner': 'gcopter',
            'multi_mode': 'ego_swarm',
            'num_drones': 2,
            'formation': 'v',
            'map': 'official_forest',
            'seed': 1,
            'use_rviz': True,
            'max_vel': 1.2,
            'goal_x': 15.0,
            'goal_y': 0.0,
            'goal_z': 1.0,
        }
        self.log_lines: List[str] = []
        self.started_at: Optional[float] = None
        self._reader: Optional[threading.Thread] = None

    def build_cmd(self, cfg: Optional[Dict[str, Any]] = None) -> List[str]:
        incoming = dict(cfg or {})
        c = {**self.cfg, **incoming}
        # Prefer explicit mode from THIS request (UI always sends it). Do not
        # promote to multi just because self.cfg still holds a stale multi_mode.
        mode = str(incoming.get('mode', c.get('mode', 'single'))).lower()
        multi = str(
            incoming.get('multi_mode', c.get('multi_mode', 'ego_swarm')) or ''
        ).strip().lower()
        if mode != 'multi' and 'mode' not in incoming and multi in MULTI_MODES:
            mode = 'multi'
            c['mode'] = 'multi'
        use_rviz = 'true' if c.get('use_rviz', True) else 'false'
        seed = int(c.get('seed', 1))

        if mode == 'multi':
            if multi not in MULTI_MODES:
                multi = 'ego_swarm'
            if multi not in MULTI_MODES:
                raise ValueError(f'Unknown multi_mode: {multi}')
            launch = MULTI_MODES[multi]['launch']
            cmd = [
                'ros2', 'launch', 'drone_bringup', launch,
                f'seed:={seed}',
                f'use_rviz:={use_rviz}',
            ]
            if multi == 'ego_swarm':
                map_id = normalize_map_id(
                    str(c.get('map', 'official_forest')), planner='ego')
                cmd.append(f'map:={map_id}')
                n = max(2, min(int(c.get('num_drones', 2)), 20))
                cmd.append(f'num_drones:={n}')
            elif multi == 'shared_field':
                # Launch hard-locks dense_field; keep dashboard cfg consistent.
                c['map'] = 'dense_field'
                c['num_drones'] = 2
                self.cfg['map'] = 'dense_field'
                self.cfg['num_drones'] = 2
            elif multi == 'formation':
                form = str(c.get('formation', 'v')).lower()
                if form in ('triangle', 'wedge'):
                    form = 'v'
                elif form not in ('line', 'column', 'v'):
                    form = 'v'
                cmd.append(f'formation:={form}')
                c['formation'] = form
                c['map'] = 'dense_field'
                c['num_drones'] = 3
                self.cfg['map'] = 'dense_field'
                self.cfg['num_drones'] = 3
                self.cfg['formation'] = form
            return cmd

        planner = normalize_planner_id(str(c.get('planner', 'gcopter')))
        launch_planner = 'rl' if planner == 'vfh' else planner
        if launch_planner not in PLANNERS and planner not in PLANNERS:
            raise ValueError(f'Unknown planner: {planner}')
        map_key = launch_planner if launch_planner in DEFAULT_MAP_BY_PLANNER else planner
        map_id = normalize_map_id(str(c.get('map', 'auto')), planner=map_key)
        return [
            'ros2', 'launch', 'drone_bringup', 'planner_sim.launch.py',
            f'planner:={launch_planner}',
            f'map:={map_id}',
            f'seed:={seed}',
            f'use_rviz:={use_rviz}',
        ]

    def preview(self, cfg: Optional[Dict[str, Any]] = None) -> str:
        return ' '.join(self.build_cmd(cfg))

    def _append_log(self, line: str) -> None:
        with self.lock:
            self.log_lines.append(line.rstrip('\n'))
            if len(self.log_lines) > 400:
                self.log_lines = self.log_lines[-400:]

    def _read_stdout(self, proc: subprocess.Popen) -> None:
        assert proc.stdout is not None
        for raw in proc.stdout:
            self._append_log(raw)
        code = proc.poll()
        self._append_log(f'[dashboard] process exited code={code}')

    def is_running(self) -> bool:
        with self.lock:
            return self.proc is not None and self.proc.poll() is None

    def start(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        with self.lock:
            if self.proc is not None and self.proc.poll() is None:
                return {'ok': False, 'error': 'Already running — stop first'}
            # Multi homemade modes: force drone count + dense map before merge.
            # Only when THIS request explicitly asks for multi. The UI always
            # sends a leftover multi_mode (default ego_swarm) even on single
            # Path G — never promote single → multi from that field alone.
            if str(cfg.get('mode', '')).lower() == 'multi':
                mm = str(cfg.get('multi_mode', self.cfg.get('multi_mode', '')))
                if mm == 'shared_field':
                    cfg['map'] = 'dense_field'
                    cfg['num_drones'] = 2
                elif mm == 'formation':
                    cfg['map'] = 'dense_field'
                    cfg['num_drones'] = 3
                elif mm not in MULTI_MODES:
                    cfg['multi_mode'] = 'ego_swarm'
            # Sync default goal from map if caller did not override.
            if 'map' in cfg or 'planner' in cfg:
                planner = cfg.get('planner', self.cfg.get('planner', 'gcopter'))
                try:
                    mid = normalize_map_id(
                        str(cfg.get('map', self.cfg.get('map', 'auto'))),
                        planner=planner)
                except ValueError as exc:
                    return {'ok': False, 'error': str(exc)}
                cfg['map'] = mid
                if 'goal_x' not in cfg and mid in MAPS:
                    pose = MAPS[mid]['pose']
                    cfg.setdefault('goal_x', pose['goal_x'])
                    cfg.setdefault('goal_y', pose['goal_y'])
                    cfg.setdefault('goal_z', pose['goal_z'])
            self.cfg.update(cfg)
            cmd = self.build_cmd(self.cfg)
            env = os.environ.copy()
            # Ensure overlay is present if user only sourced humble.
            install = ws_root() / 'install' / 'setup.bash'
            self.log_lines = [f'[dashboard] starting: {" ".join(cmd)}']
            # Launch via bash -lc so workspace overlay is sourced when available.
            bash_cmd = (
                'source /opt/ros/humble/setup.bash 2>/dev/null; '
                f'[ -f "{install}" ] && source "{install}"; '
                f'exec {shlex.join(cmd)}'
            )
            self.proc = subprocess.Popen(
                ['bash', '-lc', bash_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
                env=env,
                cwd=str(ws_root()),
            )
            self.started_at = time.time()
            self._reader = threading.Thread(
                target=self._read_stdout, args=(self.proc,), daemon=True)
            self._reader.start()

        # Soft-apply controller max_vel after launch settles (best-effort).
        max_vel = float(self.cfg.get('max_vel', 1.2))
        threading.Thread(
            target=self._deferred_param, args=(max_vel,), daemon=True).start()
        return {'ok': True, 'cmd': ' '.join(cmd), 'pid': self.proc.pid}

    def _deferred_param(self, max_vel: float) -> None:
        time.sleep(6.0)
        mode = str(self.cfg.get('mode', 'single'))
        targets = ['/drone_controller']
        if mode == 'multi':
            n = int(self.cfg.get('num_drones', 2))
            if self.cfg.get('multi_mode') == 'formation':
                n = 3
            targets = [f'/drone_controller_uav{i}' for i in range(n)]
            # nodes may be named drone_controller_uav0 depending on launch_utils
        try:
            install = ws_root() / 'install' / 'setup.bash'
            for node in targets:
                subprocess.run(
                    ['bash', '-lc',
                     f'source /opt/ros/humble/setup.bash; '
                     f'[ -f "{install}" ] && source "{install}"; '
                     f'ros2 param set {node} max_vel {max_vel}'],
                    check=False, capture_output=True, text=True, timeout=8)
            self._append_log(f'[dashboard] requested max_vel={max_vel} on {targets}')
        except Exception as exc:
            self._append_log(f'[dashboard] param set skipped: {exc}')

    def stop(self) -> Dict[str, Any]:
        with self.lock:
            proc = self.proc
            self.proc = None
        if proc is None:
            self._kill_sim_nodes()
            return {'ok': True, 'message': 'Nothing to stop'}
        _signal_process_group(proc, signal.SIGINT)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            _signal_process_group(proc, signal.SIGKILL)
        self._kill_sim_nodes()
        self._append_log('[dashboard] stopped')
        return {'ok': True}

    def _kill_sim_nodes(self) -> None:
        patterns = [
            'planner_sim.launch.py', 'ego_avoidance.launch.py',
            'gcopter_avoidance.launch.py', 'avoidance.launch.py',
            'fuel_explore.launch.py', 'rl_avoidance.launch.py', 'sac_avoidance.launch.py',
            'ego_swarm.launch.py', 'shared_field.launch.py', 'formation.launch.py',
            'ego_planner_node', 'global_planning_node', 'random_forest',
            'dynamics_node', 'controller_node', 'planner_node', 'map_node',
            'mockamap_node', 'cloud_bridge', 'formation_coordinator',
            'traj_server', 'ego_cmd_bridge', 'viz_node', 'rl_planner_node',
            'vfh_planner_node', 'sac_planner_node', 'safety_supervisor_node',
            'map_adapter_node',
            'lib/drone_exploration/exploration_fsm',
        ]
        for pat in patterns:
            subprocess.run(
                ['pkill', '-9', '-f', pat], check=False,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def status(self) -> Dict[str, Any]:
        running = self.is_running()
        with self.lock:
            logs = list(self.log_lines[-80:])
            cfg = dict(self.cfg)
            started = self.started_at
        return {
            'running': running,
            'pid': self.proc.pid if running and self.proc else None,
            'uptime_s': (time.time() - started) if running and started else 0.0,
            'config': cfg,
            'cmd': self.preview(cfg),
            'planners': PLANNERS,
            'planner_registry': planner_public_info('en'),
            'planner_registry_zh': planner_public_info('zh'),
            'rates': RATES,
            'multi_modes': MULTI_MODES,
            'maps': map_public_info(),
            'map_defaults': DEFAULT_MAP_BY_PLANNER,
            'logs': logs,
            'have_rclpy': _HAVE_RCLPY,
            'rl_train': RL_TRAIN.status(),
            'sac_train': SAC_TRAIN.status(),
            'acceptance': ACCEPTANCE.status(),
        }


MANAGER = LaunchManager()


ACCEPTANCE_SCENARIOS = [
    {'id': 1, 'name_zh': '悬停', 'name_en': 'Hover', 'launch': 'hover.launch.py'},
    {'id': 2, 'name_zh': '单目标点', 'name_en': 'Single goal', 'launch': 'single_goal.launch.py'},
    {'id': 3, 'name_zh': '多目标点', 'name_en': 'Multi waypoint', 'launch': 'multi_goal.launch.py'},
    {'id': 4, 'name_zh': '静态避障', 'name_en': 'Avoidance', 'launch': 'avoidance.launch.py'},
    {'id': 5, 'name_zh': '狭窄通道', 'name_en': 'Narrow passage', 'launch': 'narrow_passage.launch.py'},
    {'id': 6, 'name_zh': '稳定性展示', 'name_en': 'Stability', 'launch': 'stability_demo.launch.py'},
]


class AcceptanceManager:
    """Background runner for scripts/run_acceptance.py (all six or selected ids)."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.proc: Optional[subprocess.Popen] = None
        self.log_lines: List[str] = []
        self.started_at: Optional[float] = None
        self.cfg: Dict[str, Any] = {'only': '', 'mode': 'all', 'use_rviz': False}
        self.last_exit: Optional[int] = None

    def is_running(self) -> bool:
        with self.lock:
            return self.proc is not None and self.proc.poll() is None

    def _append(self, line: str) -> None:
        with self.lock:
            self.log_lines.append(line.rstrip('\n'))
            self.log_lines = self.log_lines[-200:]

    def _reader(self, proc: subprocess.Popen) -> None:
        assert proc.stdout is not None
        for raw in proc.stdout:
            with self.lock:
                # Drop late output from a superseded process.
                if self.proc is not None and self.proc is not proc:
                    continue
            self._append(raw)
        code = proc.wait()
        with self.lock:
            if self.proc is proc:
                self.proc = None
                self.last_exit = code
            elif self.proc is None and self.last_exit is None:
                self.last_exit = code
        if code == -2 or code == 130:
            self._append('[acceptance] cancelled')
        else:
            self._append(f'[acceptance] exited code={code}')

    def start(self, cfg: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        if self.is_running():
            # Switching scenarios (or re-run) should not hard-fail the UI.
            self.stop()
            time.sleep(0.8)
        if MANAGER.is_running():
            MANAGER.stop()
            time.sleep(1.0)
        if RL_TRAIN.is_running():
            return {'ok': False, 'error': 'stop RL training first'}
        if SAC_TRAIN.is_running():
            return {'ok': False, 'error': 'stop Path H training first'}

        data = dict(cfg or {})
        only_raw = str(data.get('only', '') or '').strip()
        mode = str(data.get('mode', 'all') or 'all').lower()
        use_rviz = bool(data.get('use_rviz', False))
        only_ids: List[int] = []
        if mode == 'single' or only_raw:
            tokens = only_raw.replace(' ', '').split(',') if only_raw else []
            for tok in tokens:
                if not tok:
                    continue
                try:
                    n = int(tok)
                except ValueError:
                    return {'ok': False, 'error': f'bad scenario id: {tok}'}
                if n < 1 or n > 6:
                    return {'ok': False, 'error': f'scenario id out of range: {n}'}
                only_ids.append(n)
            if mode == 'single' and len(only_ids) != 1:
                return {'ok': False, 'error': 'pick exactly one scenario for single mode'}
            only_ids = sorted(set(only_ids))

        script = ws_root() / 'scripts' / 'run_acceptance.py'
        if not script.is_file():
            return {'ok': False, 'error': f'missing {script}'}

        cmd = ['python3', '-u', str(script)]
        if only_ids:
            cmd.extend(['--only', ','.join(str(i) for i in only_ids)])
        if use_rviz:
            cmd.append('--use-rviz')

        env = os.environ.copy()
        env['PYTHONUNBUFFERED'] = '1'
        env.setdefault('PYTHONNOUSERSITE', '1')
        # Prefer system setuptools for any nested ament tooling.
        env['PYTHONPATH'] = '/usr/lib/python3/dist-packages:' + env.get('PYTHONPATH', '')
        if use_rviz and not env.get('DISPLAY'):
            env['DISPLAY'] = ':0'
        install = ws_root() / 'install' / 'setup.bash'
        # Source ROS via bash wrapper so child inherits workspace.
        bash_cmd = (
            'source /opt/ros/humble/setup.bash; '
            f'[ -f "{install}" ] && source "{install}"; '
            + ' '.join(shlex.quote(c) for c in cmd)
        )

        with self.lock:
            self.cfg = {
                'mode': 'single' if only_ids and len(only_ids) == 1 else ('subset' if only_ids else 'all'),
                'only': ','.join(str(i) for i in only_ids),
                'use_rviz': use_rviz,
            }
            self.log_lines = [f'[acceptance] starting: {" ".join(cmd)}']
            self.started_at = time.time()
            self.last_exit = None
            self.proc = subprocess.Popen(
                ['bash', '-lc', bash_cmd],
                cwd=str(ws_root()),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
                env=env,
            )
            proc = self.proc
        threading.Thread(target=self._reader, args=(proc,), daemon=True).start()
        return {'ok': True, 'pid': proc.pid, 'cmd': cmd, 'config': dict(self.cfg)}

    def stop(self) -> Dict[str, Any]:
        with self.lock:
            proc = self.proc
            self.proc = None
        if proc is None:
            cleanup = ws_root() / 'scripts' / 'cleanup_sim.sh'
            if cleanup.is_file():
                subprocess.run(['bash', str(cleanup)], check=False)
            return {'ok': True, 'stopped': False}
        _signal_process_group(proc, signal.SIGINT)
        try:
            proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            _signal_process_group(proc, signal.SIGKILL)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                pass
        cleanup = ws_root() / 'scripts' / 'cleanup_sim.sh'
        if cleanup.is_file():
            subprocess.run(['bash', str(cleanup)], check=False)
        self._append('[acceptance] stopped')
        return {'ok': True, 'stopped': True}

    def status(self) -> Dict[str, Any]:
        running = self.is_running()
        with self.lock:
            logs = list(self.log_lines[-120:])
            cfg = dict(self.cfg)
            started = self.started_at
            last_exit = self.last_exit
        summary: Dict[str, Any] = {}
        results_path = ws_root() / 'report' / 'acceptance_results.json'
        if results_path.is_file():
            try:
                payload = json.loads(results_path.read_text())
                summary = {
                    'passed': payload.get('passed'),
                    'total': payload.get('total'),
                    'timestamp': payload.get('timestamp'),
                    'results': [
                        {
                            'id': r.get('id'),
                            'name': r.get('name'),
                            'pass': r.get('pass'),
                            'status': r.get('status'),
                        }
                        for r in payload.get('results', [])
                    ],
                }
            except (json.JSONDecodeError, OSError):
                pass
        return {
            'running': running,
            'pid': self.proc.pid if running and self.proc else None,
            'uptime_s': (time.time() - started) if running and started else 0.0,
            'config': cfg,
            'last_exit': last_exit,
            'logs': logs,
            'scenarios': ACCEPTANCE_SCENARIOS,
            'summary': summary,
            'report_md': 'report/acceptance_report.md',
            'report_json': 'report/acceptance_results.json',
        }


ACCEPTANCE = AcceptanceManager()


def list_reports() -> Dict[str, Any]:
    """Recent acceptance / batch artifacts under workspace report/."""
    root = ws_root() / 'report'
    entries: List[Dict[str, Any]] = []
    if not root.is_dir():
        return {'report_dir': str(root), 'files': entries}

    def _add(path: Path, kind: str) -> None:
        if not path.is_file():
            return
        st = path.stat()
        entries.append({
            'name': path.name,
            'path': str(path.relative_to(ws_root())),
            'kind': kind,
            'size': st.st_size,
            'mtime': st.st_mtime,
        })

    _add(root / 'acceptance_results.json', 'acceptance')
    _add(root / 'acceptance_report.md', 'acceptance')
    _add(root / 'batch_matrix' / 'manifest.json', 'batch')
    for md in sorted(root.glob('*.md'), key=lambda p: p.stat().st_mtime, reverse=True):
        if md.name == 'acceptance_report.md':
            continue
        _add(md, 'markdown')
    entries.sort(key=lambda e: e['mtime'], reverse=True)
    return {'report_dir': str(root), 'files': entries[:40]}


class RlTrainManager:
    """Background SB3 PPO training for Path G (separate from sim launch)."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.proc: Optional[subprocess.Popen] = None
        self.log_lines: List[str] = []
        self.cfg: Dict[str, Any] = {
            'target': 0.95,
            'steps': 2_000_000,
            'easy': False,  # domain-randomized hard maps by default
            'n_envs': 8,
            'fresh': False,
        }

    def _pkg_root(self) -> Path:
        return ws_root() / 'src' / 'drone_rl_planner'

    def _status_file(self) -> Path:
        return self._pkg_root() / 'checkpoints' / 'training_status.json'

    def _checkpoint(self) -> Path:
        return self._pkg_root() / 'checkpoints' / 'sb3_ppo_local.zip'

    def is_running(self) -> bool:
        with self.lock:
            return self.proc is not None and self.proc.poll() is None

    def _append_log(self, line: str) -> None:
        with self.lock:
            self.log_lines.append(line.rstrip('\n'))
            if len(self.log_lines) > 300:
                self.log_lines = self.log_lines[-300:]

    def _read_stdout(self, proc: subprocess.Popen) -> None:
        assert proc.stdout is not None
        for raw in proc.stdout:
            self._append_log(raw)
        code = proc.poll()
        self._append_log(f'[dashboard] RL train exited code={code}')

    def status(self) -> Dict[str, Any]:
        st: Dict[str, Any] = {
            'running': self.is_running(),
            'target': self.cfg.get('target', 0.95),
            'checkpoint': str(self._checkpoint()),
            'checkpoint_exists': self._checkpoint().is_file(),
        }
        sf = self._status_file()
        if sf.is_file():
            try:
                st.update(json.loads(sf.read_text()))
            except json.JSONDecodeError:
                pass
        with self.lock:
            st['logs'] = list(self.log_lines[-40:])
        return st

    def start(self, cfg: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        with self.lock:
            if self.proc is not None and self.proc.poll() is None:
                return {'ok': False, 'error': 'RL training already running'}
            if SAC_TRAIN.is_running():
                return {'ok': False, 'error': 'stop Path H training first'}
            if cfg:
                self.cfg.update(cfg)
            target = float(self.cfg.get('target', 0.95))
            steps = int(self.cfg.get('steps', 2_000_000))
            n_envs = int(self.cfg.get('n_envs', 8))
            easy = bool(self.cfg.get('easy', False))
            fresh = bool(self.cfg.get('fresh', False))
            pkg = self._pkg_root()
            out = pkg / 'checkpoints' / 'sb3_ppo_local'
            resume = out if out.with_suffix('.zip').is_file() else ''
            cmd = [
                'python3', '-m', 'drone_rl_planner.train_sb3_ppo',
                f'--steps={steps}',
                f'--target={target}',
                f'--n-envs={n_envs}',
                f'--out={out}',
                '--eval-freq=20000',
                '--eval-episodes=120',
            ]
            if easy:
                cmd.append('--easy')
            if fresh:
                cmd.append('--fresh')
            elif resume:
                cmd.append(f'--resume={resume}')
            env = os.environ.copy()
            env['PYTHONPATH'] = f'{pkg}:{env.get("PYTHONPATH", "")}'
            self.log_lines = [f'[dashboard] RL train: {" ".join(cmd)}']
            bash = (
                'source /opt/ros/humble/setup.bash 2>/dev/null; '
                f'cd "{ws_root()}"; '
                f'export PYTHONPATH="{pkg}:$PYTHONPATH"; '
                f'exec {" ".join(shlex.quote(c) for c in cmd)}'
            )
            self.proc = subprocess.Popen(
                ['bash', '-lc', bash],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            threading.Thread(target=self._read_stdout, args=(self.proc,), daemon=True).start()
            return {'ok': True, 'cmd': ' '.join(cmd), 'target': target}

    def stop(self) -> Dict[str, Any]:
        with self.lock:
            proc = self.proc
            self.proc = None
        if proc is None or proc.poll() is not None:
            return {'ok': True, 'stopped': False}
        _signal_process_group(proc, signal.SIGTERM)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            _signal_process_group(proc, signal.SIGKILL)
        self._append_log('[dashboard] RL train stopped')
        return {'ok': True, 'stopped': True}


RL_TRAIN = RlTrainManager()


class SacTrainManager:
    """Background Polar DrQ-SAC training for Path H (separate from sim launch)."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.proc: Optional[subprocess.Popen] = None
        self.log_lines: List[str] = []
        self.cfg: Dict[str, Any] = {
            'target': 0.95,
            'steps': 1_000_000,
            'dense_heavy': True,
            'eval_every': 5_000,
            'eval_episodes': 60,
            'resume': 'none',
            'fresh': True,
        }

    def _pkg_root(self) -> Path:
        return ws_root() / 'src' / 'drone_rl_planner'

    def _status_file(self) -> Path:
        return self._pkg_root() / 'checkpoints' / 'sac_training_status.json'

    def _checkpoint(self) -> Path:
        best = self._pkg_root() / 'checkpoints' / 'sac_polar_local_best.pt'
        if best.is_file():
            return best
        return self._pkg_root() / 'checkpoints' / 'sac_polar_local.pt'

    def is_running(self) -> bool:
        with self.lock:
            return self.proc is not None and self.proc.poll() is None

    def _append_log(self, line: str) -> None:
        with self.lock:
            self.log_lines.append(line.rstrip('\n'))
            if len(self.log_lines) > 300:
                self.log_lines = self.log_lines[-300:]

    def _read_stdout(self, proc: subprocess.Popen) -> None:
        assert proc.stdout is not None
        for raw in proc.stdout:
            self._append_log(raw)
        code = proc.poll()
        self._append_log(f'[dashboard] SAC train exited code={code}')

    def status(self) -> Dict[str, Any]:
        st: Dict[str, Any] = {
            'running': self.is_running(),
            'target': self.cfg.get('target', 0.90),
            'checkpoint': str(self._checkpoint()),
            'checkpoint_exists': self._checkpoint().is_file(),
            'algorithm': 'DrQ-SAC',
        }
        sf = self._status_file()
        if sf.is_file():
            try:
                file_st = json.loads(sf.read_text())
                st.update(file_st)
            except json.JSONDecodeError:
                pass
        # Normalize field names so the Path G train card can reuse them.
        if 'timesteps' not in st and 'steps' in st:
            st['timesteps'] = st['steps']
        if 'best_success_rate' not in st and 'best_success' in st:
            st['best_success_rate'] = st['best_success']
        if 'success_rate' not in st and 'best_success' in st and not st.get('running'):
            st['success_rate'] = st.get('best_success')
        # External / resumed jobs write state=running to the status file.
        if str(st.get('state', '')).lower() == 'running':
            st['running'] = True
        with self.lock:
            st['logs'] = list(self.log_lines[-40:])
        return st

    def start(self, cfg: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        with self.lock:
            if self.proc is not None and self.proc.poll() is None:
                return {'ok': False, 'error': 'SAC training already running'}
            if RL_TRAIN.is_running():
                return {'ok': False, 'error': 'stop Path G training first'}
            if cfg:
                self.cfg.update(cfg)
            steps = int(self.cfg.get('steps', 1_000_000))
            eval_every = int(self.cfg.get('eval_every', 5_000))
            eval_episodes = int(self.cfg.get('eval_episodes', 60))
            target = float(self.cfg.get('target', 0.95))
            dense_heavy = bool(self.cfg.get('dense_heavy', True))
            resume = str(self.cfg.get('resume', 'none') or '').strip()
            fresh = bool(self.cfg.get('fresh', False))
            pkg = self._pkg_root()
            best = pkg / 'checkpoints' / 'sac_polar_local_best.pt'
            last = pkg / 'checkpoints' / 'sac_polar_local.pt'
            if fresh:
                for p in (best, last):
                    try:
                        if p.is_file():
                            p.unlink()
                    except OSError:
                        pass
                resume = 'none'
            cmd = [
                'python3', '-m', 'drone_rl_planner.train_sac_polar',
                f'--steps={steps}',
                f'--eval-every={eval_every}',
                f'--eval-episodes={eval_episodes}',
                f'--target={target}',
                '--device=cuda',
                '--batch-size=128',
                '--updates-per-step=2',
                '--n-envs=2',
            ]
            if dense_heavy:
                cmd.append('--dense-heavy')
            if resume == 'auto':
                if best.is_file():
                    cmd.append(f'--resume={best}')
                elif last.is_file():
                    cmd.append(f'--resume={last}')
            elif resume and resume.lower() not in ('', 'none', 'false', '0'):
                cmd.append(f'--resume={resume}')
            self.log_lines = [f'[dashboard] SAC train: {" ".join(cmd)}']
            bash = (
                'source /opt/ros/humble/setup.bash 2>/dev/null; '
                f'cd "{ws_root()}"; '
                f'export PYTHONPATH="{pkg}:$PYTHONPATH"; '
                f'exec {" ".join(shlex.quote(c) for c in cmd)}'
            )
            self.proc = subprocess.Popen(
                ['bash', '-lc', bash],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            threading.Thread(target=self._read_stdout, args=(self.proc,), daemon=True).start()
            return {'ok': True, 'cmd': ' '.join(cmd), 'target': self.cfg.get('target', 0.90)}

    def stop(self) -> Dict[str, Any]:
        with self.lock:
            proc = self.proc
            self.proc = None
        stopped = False
        if proc is not None and proc.poll() is None:
            _signal_process_group(proc, signal.SIGTERM)
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                _signal_process_group(proc, signal.SIGKILL)
            stopped = True
        # Also stop externally started fine-tunes (e.g. CLI / prior session).
        try:
            subprocess.run(
                ['pkill', '-f', 'drone_rl_planner.train_sac_polar'],
                check=False, timeout=5,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            stopped = True
        except Exception:
            pass
        # Mark status file idle so the UI unlocks.
        sf = self._status_file()
        if sf.is_file():
            try:
                data = json.loads(sf.read_text())
                data['state'] = 'idle'
                sf.write_text(json.dumps(data, indent=2))
            except Exception:
                pass
        self._append_log('[dashboard] SAC train stopped')
        return {'ok': True, 'stopped': stopped}


SAC_TRAIN = SacTrainManager()
ROS = RosStatus()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args: Any) -> None:
        # Keep server quiet; UI shows process logs.
        return

    def _json(self, code: int, payload: Any) -> None:
        body = json.dumps(payload).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> Dict[str, Any]:
        n = int(self.headers.get('Content-Length', '0') or 0)
        if n <= 0:
            return {}
        raw = self.rfile.read(n)
        try:
            return json.loads(raw.decode('utf-8'))
        except json.JSONDecodeError:
            return {}

    def _serve_static(self, rel: str) -> None:
        if rel in ('', '/'):
            rel = '/index.html'
        path = (STATIC_DIR / rel.lstrip('/')).resolve()
        if not str(path).startswith(str(STATIC_DIR.resolve())) or not path.is_file():
            self.send_error(404)
            return
        data = path.read_bytes()
        ctype = 'text/plain'
        if path.suffix == '.html':
            ctype = 'text/html; charset=utf-8'
        elif path.suffix == '.css':
            ctype = 'text/css; charset=utf-8'
        elif path.suffix == '.js':
            ctype = 'application/javascript; charset=utf-8'
        elif path.suffix == '.svg':
            ctype = 'image/svg+xml; charset=utf-8'
        elif path.suffix == '.png':
            ctype = 'image/png'
        elif path.suffix in ('.jpg', '.jpeg'):
            ctype = 'image/jpeg'
        elif path.suffix == '.webp':
            ctype = 'image/webp'
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self) -> None:
        u = urlparse(self.path)
        if u.path == '/api/status':
            st = MANAGER.status()
            live = ROS.live_map_snapshot()
            st['odom'] = live.get('odom')
            st['swarm_odom'] = live.get('swarm_odom') or {}
            st['planned_path'] = live.get('planned_path')
            st['swarm_planned'] = live.get('swarm_planned') or {}
            with ROS.lock:
                st['exploration_status'] = ROS.exploration_status
                st['planner_status'] = ROS.planner_status
                st['planner_diagnostics'] = ROS.planner_diagnostics
                st['fallback_active'] = ROS.fallback_active
            self._json(200, st)
            return
        if u.path == '/api/map/live':
            self._json(200, ROS.live_map_snapshot())
            return
        if u.path == '/api/reports':
            self._json(200, list_reports())
            return
        if u.path == '/api/preview':
            self._json(200, {'cmd': MANAGER.preview(MANAGER.cfg)})
            return
        if u.path == '/api/rl/status':
            self._json(200, RL_TRAIN.status())
            return
        if u.path == '/api/sac/status':
            self._json(200, SAC_TRAIN.status())
            return
        if u.path == '/api/map/occupancy':
            with ROS.lock:
                occ = ROS.occupancy
            if occ is None:
                self._json(200, {'ok': False, 'occupancy': None})
            else:
                self._json(200, {'ok': True, 'occupancy': occ})
            return
        if u.path == '/api/backgrounds':
            self._json(200, {'ok': True, 'items': _list_backgrounds()})
            return
        if u.path.startswith('/user-bg/'):
            name = _safe_bg_name(unquote(u.path[len('/user-bg/'):]))
            if not name:
                self.send_error(404)
                return
            path = (_bg_user_dir() / name).resolve()
            if not str(path).startswith(str(_bg_user_dir().resolve())) or not path.is_file():
                self.send_error(404)
                return
            data = path.read_bytes()
            ctype = {
                '.png': 'image/png',
                '.jpg': 'image/jpeg',
                '.jpeg': 'image/jpeg',
                '.webp': 'image/webp',
                '.gif': 'image/gif',
                '.svg': 'image/svg+xml',
            }.get(path.suffix.lower(), 'application/octet-stream')
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(data)
            return
        if u.path.startswith('/api/'):
            self._json(404, {'error': 'not found'})
            return
        self._serve_static(u.path)

    def do_POST(self) -> None:
        u = urlparse(self.path)
        data = self._read_json()
        if u.path == '/api/start':
            try:
                ROS.reset_live_tracks()
                self._json(200, MANAGER.start(data))
            except Exception as exc:
                self._json(400, {'ok': False, 'error': str(exc)})
            return
        if u.path == '/api/stop':
            out = MANAGER.stop()
            ROS.reset_live_tracks()
            self._json(200, out)
            return
        if u.path == '/api/restart':
            MANAGER.stop()
            ROS.reset_live_tracks()
            time.sleep(1.0)
            self._json(200, MANAGER.start(data or MANAGER.cfg))
            return
        if u.path == '/api/goal':
            x = float(data.get('x', MANAGER.cfg.get('goal_x', 15.0)))
            y = float(data.get('y', MANAGER.cfg.get('goal_y', 0.0)))
            z = float(data.get('z', MANAGER.cfg.get('goal_z', 1.0)))
            yaw = float(data.get('yaw', 0.0))
            ns = str(data.get('namespace', '') or '')
            ok = ROS.publish_goal(x, y, z, yaw, namespace=ns)
            if not ok:
                topic = f'/{ns}/drone/goal' if ns else '/drone/goal'
                try:
                    subprocess.run(
                        ['bash', '-lc',
                         f'source /opt/ros/humble/setup.bash; '
                         f'[ -f "{ws_root()}/install/setup.bash" ] && source "{ws_root()}/install/setup.bash"; '
                         f'ros2 topic pub --once {topic} geometry_msgs/msg/PoseStamped '
                         f'"{{header: {{frame_id: map}}, pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
                         f'orientation: {{w: 1.0}}}}}}"'],
                        check=False, timeout=10,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    ok = True
                except Exception:
                    ok = False
            if not ns:
                MANAGER.cfg.update({'goal_x': x, 'goal_y': y, 'goal_z': z})
            self._json(200, {
                'ok': ok,
                'goal': {'x': x, 'y': y, 'z': z, 'yaw': yaw, 'namespace': ns},
            })
            return
        if u.path == '/api/rl/train':
            self._json(200, RL_TRAIN.start(data))
            return
        if u.path == '/api/rl/stop':
            self._json(200, RL_TRAIN.stop())
            return
        if u.path == '/api/sac/train':
            self._json(200, SAC_TRAIN.start(data))
            return
        if u.path == '/api/sac/stop':
            self._json(200, SAC_TRAIN.stop())
            return
        if u.path == '/api/acceptance/start':
            self._json(200, ACCEPTANCE.start(data))
            return
        if u.path == '/api/acceptance/stop':
            self._json(200, ACCEPTANCE.stop())
            return
        if u.path == '/api/config':
            MANAGER.cfg.update(data)
            # When map changes and goals not explicitly sent, apply recommended pose.
            if 'map' in data and not any(k in data for k in ('goal_x', 'goal_y', 'goal_z')):
                try:
                    mid = normalize_map_id(
                        str(MANAGER.cfg.get('map', 'auto')),
                        planner=str(MANAGER.cfg.get('planner', 'gcopter')))
                    MANAGER.cfg['map'] = mid
                    pose = MAPS[mid]['pose']
                    MANAGER.cfg['goal_x'] = pose['goal_x']
                    MANAGER.cfg['goal_y'] = pose['goal_y']
                    MANAGER.cfg['goal_z'] = pose['goal_z']
                except ValueError as exc:
                    self._json(400, {'ok': False, 'error': str(exc)})
                    return
            self._json(200, {
                'ok': True,
                'config': MANAGER.cfg,
                'cmd': MANAGER.preview(),
            })
            return
        if u.path == '/api/backgrounds':
            item, err = _save_background_upload(
                str(data.get('filename') or ''),
                str(data.get('data') or ''),
            )
            if err or not item:
                self._json(400, {'ok': False, 'error': err or 'upload failed'})
                return
            self._json(200, {'ok': True, 'item': item, 'items': _list_backgrounds()})
            return
        if u.path == '/api/backgrounds/delete':
            ok, msg = _delete_background(str(data.get('id') or ''))
            if not ok:
                self._json(400, {'ok': False, 'error': msg})
                return
            self._json(200, {'ok': True, 'items': _list_backgrounds()})
            return
        self._json(404, {'error': 'not found'})


def create_httpd(host: str = '127.0.0.1', port: int = 8765,
                 workspace: Optional[str] = None) -> ThreadingHTTPServer:
    """Build the dashboard HTTP server (caller owns serve/shutdown lifecycle)."""
    ws = Path(workspace).resolve() if workspace else ws_root()
    _STATE['ws_root'] = ws
    if not STATIC_DIR.is_dir():
        raise FileNotFoundError(f'Missing static dir: {STATIC_DIR}')
    return ThreadingHTTPServer((host, port), Handler)


def serve_httpd(httpd: ThreadingHTTPServer) -> None:
    """Blocking serve loop with ROS/manager cleanup."""
    ROS.start()
    try:
        httpd.serve_forever()
    finally:
        MANAGER.stop()
        ROS.stop()
        httpd.server_close()


def main(argv: Optional[List[str]] = None) -> int:
    import argparse
    parser = argparse.ArgumentParser(description='Drone sim web control panel')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--workspace', default=str(ws_root()))
    args = parser.parse_args(argv)

    try:
        httpd = create_httpd(args.host, args.port, args.workspace)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    except OSError as exc:
        print(
            f'Cannot bind http://{args.host}:{args.port}/ — {exc}\n'
            f'Another dashboard is probably still running. Free it with:\n'
            f'  fuser -k {args.port}/tcp\n'
            f'Or use another port:\n'
            f'  ros2 run drone_bringup dashboard -- --port {args.port + 1}',
            file=sys.stderr, flush=True)
        return 1
    print(f'Drone dashboard: http://{args.host}:{args.port}/', flush=True)
    print('Ctrl+C to quit (also stops active simulation).', flush=True)
    try:
        serve_httpd(httpd)
    except KeyboardInterrupt:
        print('\nShutting down…')
    return 0


if __name__ == '__main__':
    sys.exit(main())

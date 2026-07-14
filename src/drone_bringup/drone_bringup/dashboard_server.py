#!/usr/bin/env python3
"""Local web control panel for Path A/B/C — replaces typing ros2 launch by hand.

Serves a single-page UI and manages one ros2 launch process at a time.
"""

from __future__ import annotations

import json
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import urlparse

# Optional ROS graph helpers (status / goal). Dashboard still works if ROS is only used via CLI.
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    _HAVE_RCLPY = True
except ImportError:  # pragma: no cover
    _HAVE_RCLPY = False


from drone_bringup.maps_catalog import (
    DEFAULT_MAP_BY_PLANNER,
    MAPS,
    map_public_info,
    normalize_map_id,
)

ROOT = Path(__file__).resolve().parent
STATIC_DIR = ROOT / 'dashboard_static'
_STATE = {
    'ws_root': Path(os.environ.get('DRONE_WS', Path.home() / 'drone_ws')).resolve(),
}


def ws_root() -> Path:
    return _STATE['ws_root']

PLANNERS = {
    'homemade': {
        'label': 'Path A — Homemade planner',
        'launch': 'avoidance.launch.py',
        'via': 'planner_sim',
        'desc': 'Self-developed drone_planner + drone_map',
    },
    'ego': {
        'label': 'Path B — Official EGO',
        'launch': 'ego_avoidance.launch.py',
        'via': 'planner_sim',
        'desc': 'ego_planner + map_generator + our plant',
    },
    'gcopter': {
        'label': 'Path C — GCOPTER / MINCO',
        'launch': 'gcopter_avoidance.launch.py',
        'via': 'planner_sim',
        'desc': 'Vendored GCOPTER (yuwei-wu/GCOPTER ros2) + our plant',
    },
}

MULTI_MODES = {
    'ego_swarm': {
        'label_en': 'EGO-Swarm (official)',
        'label_zh': 'EGO-Swarm（官方）',
        'desc_en': 'broadcast_bspline swarm + our plant (2–3 drones)',
        'desc_zh': '官方广播样条集群 + 自研植物（2–3 机）',
        'launch': 'ego_swarm.launch.py',
    },
    'shared_field': {
        'label_en': 'Shared field (homemade)',
        'label_zh': '同场避障（自研）',
        'desc_en': 'Homemade planners + dense map + peer keep-out',
        'desc_zh': '自研规划器 + 密集地图 + 机间 keep-out',
        'launch': 'shared_field.launch.py',
    },
    'formation': {
        'label_en': 'Formation (homemade)',
        'label_zh': '编队（自研）',
        'desc_en': 'Leader + followers with formation_coordinator',
        'desc_zh': '领机 + 从机，formation_coordinator',
        'launch': 'formation.launch.py',
    },
}


class RosStatus:
    """Background rclpy node for live odom + goal publish."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.odom: Optional[Dict[str, float]] = None
        self.swarm_odom: Dict[str, Dict[str, float]] = {}
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
            for i in range(3):
                ns = f'uav{i}'
                self._goal_pubs[ns] = node.create_publisher(
                    PoseStamped, f'/{ns}/drone/goal', 10)
                node.create_subscription(
                    Odometry, f'/{ns}/drone/odom',
                    lambda msg, n=ns: self._on_swarm_odom(n, msg), 20)
            node.create_subscription(Odometry, '/drone/odom', self._on_odom, 20)
            while self._running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.2)
        except Exception:
            pass
        finally:
            try:
                if self._node is not None:
                    self._node.destroy_node()
            except Exception:
                pass

    def _pack_odom(self, msg: Any) -> Dict[str, float]:
        return {
            'x': float(msg.pose.pose.position.x),
            'y': float(msg.pose.pose.position.y),
            'z': float(msg.pose.pose.position.z),
            'vx': float(msg.twist.twist.linear.x),
            'vy': float(msg.twist.twist.linear.y),
            'vz': float(msg.twist.twist.linear.z),
        }

    def _on_odom(self, msg: Any) -> None:
        with self.lock:
            self.odom = self._pack_odom(msg)

    def _on_swarm_odom(self, ns: str, msg: Any) -> None:
        with self.lock:
            self.swarm_odom[ns] = self._pack_odom(msg)

    def publish_goal(
        self, x: float, y: float, z: float, yaw: float = 0.0,
        namespace: str = '',
    ) -> bool:
        import math
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
        c = {**self.cfg, **(cfg or {})}
        mode = str(c.get('mode', 'single')).lower()
        use_rviz = 'true' if c.get('use_rviz', True) else 'false'
        seed = int(c.get('seed', 1))

        if mode == 'multi':
            multi = str(c.get('multi_mode', 'ego_swarm'))
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
                cmd.append(f'num_drones:={int(c.get("num_drones", 2))}')
            elif multi == 'formation':
                form = str(c.get('formation', 'v'))
                cmd.append(f'formation:={form}')
            return cmd

        planner = c.get('planner', 'gcopter')
        if planner not in PLANNERS:
            raise ValueError(f'Unknown planner: {planner}')
        map_id = normalize_map_id(str(c.get('map', 'auto')), planner=planner)
        return [
            'ros2', 'launch', 'drone_bringup', 'planner_sim.launch.py',
            f'planner:={planner}',
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
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        self._kill_sim_nodes()
        self._append_log('[dashboard] stopped')
        return {'ok': True}

    def _kill_sim_nodes(self) -> None:
        patterns = [
            'planner_sim.launch.py', 'ego_avoidance.launch.py',
            'gcopter_avoidance.launch.py', 'avoidance.launch.py',
            'ego_swarm.launch.py', 'shared_field.launch.py', 'formation.launch.py',
            'ego_planner_node', 'global_planning_node', 'random_forest',
            'dynamics_node', 'controller_node', 'planner_node', 'map_node',
            'mockamap_node', 'cloud_bridge', 'formation_coordinator',
            'traj_server', 'ego_cmd_bridge', 'viz_node',
        ]
        for pat in patterns:
            subprocess.run(
                ['pkill', '-f', pat], check=False,
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
            'multi_modes': MULTI_MODES,
            'maps': map_public_info(),
            'map_defaults': DEFAULT_MAP_BY_PLANNER,
            'logs': logs,
            'have_rclpy': _HAVE_RCLPY,
        }


MANAGER = LaunchManager()
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
            ctype = 'image/svg+xml'
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self) -> None:
        u = urlparse(self.path)
        if u.path == '/api/status':
            st = MANAGER.status()
            with ROS.lock:
                st['odom'] = ROS.odom
                st['swarm_odom'] = dict(ROS.swarm_odom)
            self._json(200, st)
            return
        if u.path == '/api/preview':
            self._json(200, {'cmd': MANAGER.preview(MANAGER.cfg)})
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
                self._json(200, MANAGER.start(data))
            except Exception as exc:
                self._json(400, {'ok': False, 'error': str(exc)})
            return
        if u.path == '/api/stop':
            self._json(200, MANAGER.stop())
            return
        if u.path == '/api/restart':
            MANAGER.stop()
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
        self._json(404, {'error': 'not found'})


def main(argv: Optional[List[str]] = None) -> int:
    import argparse
    parser = argparse.ArgumentParser(description='Drone sim web control panel')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--workspace', default=str(ws_root()))
    args = parser.parse_args(argv)

    _STATE['ws_root'] = Path(args.workspace).resolve()
    if not STATIC_DIR.is_dir():
        print(f'Missing static dir: {STATIC_DIR}', file=sys.stderr)
        return 1

    ROS.start()
    try:
        httpd = ThreadingHTTPServer((args.host, args.port), Handler)
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
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\nShutting down…')
    finally:
        MANAGER.stop()
        ROS.stop()
        httpd.server_close()
    return 0


if __name__ == '__main__':
    sys.exit(main())

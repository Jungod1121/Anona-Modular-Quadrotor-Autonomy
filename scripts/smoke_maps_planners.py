#!/usr/bin/env python3
"""Smoke: every planner × every catalog map loads a cloud and stays alive.

Pass criteria (per combo):
  - Point cloud width > 0 on /map/obstacles or /map_generator/global_cloud
    (TRANSIENT_LOCAL QoS — ros2 topic echo --once will miss latch)
  - planner process alive
  - no fatal crash markers in the *prefix* of the launch log (before tear-down)

Usage (workspace root, after source install/setup.bash):
  python3 scripts/smoke_maps_planners.py
  python3 scripts/smoke_maps_planners.py --settle 10 --out report/map_smoke.json
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'drone_bringup'))

from drone_bringup.maps_catalog import MAPS  # noqa: E402

PLANNERS = ('homemade', 'ego', 'gcopter', 'mighty', 'fast_planner')
PLANNER_PROC = {
    'homemade': 'planner_node',
    'ego': 'ego_planner_node',
    'gcopter': 'global_planning_node',
    'mighty': 'mighty_planning_node',
    'fast_planner': 'fast_planning_node',
}
# Keep patterns path-specific so pkill cannot match this script's argv / parent shell.
KILL_PATS = [
    'ros2 launch drone_bringup',
    'planner_sim.launch.py',
    'ego_avoidance.launch.py',
    'gcopter_avoidance.launch.py',
    'lib/drone_planner/planner_node',
    'lib/ego_planner/ego_planner_node',
    'lib/gcopter/global_planning_node',
    'lib/drone_map/map_node',
    'lib/drone_dynamics/dynamics_node',
    'lib/drone_controller/controller_node',
    'lib/drone_visualization/viz_node',
    'lib/drone_bringup/cloud_bridge',
    'lib/drone_bringup/ego_cmd_bridge',
    'lib/drone_bringup/send_goal',
    'lib/mockamap/mockamap_node',
    'lib/map_generator/random_forest',
    'lib/traj_utils/traj_server',
]

WIDTH_RE = re.compile(r'width[=:]\s*(\d+)', re.I)
POINTS_RE = re.compile(r'points[=:]\s*(\d+)', re.I)
CRASH_MARKERS = (
    'runtime_error',
    'process has died',
    'already been added to an executor',
    'Segmentation fault',
    'std::bad_alloc',
)


def kill_all() -> None:
    for pat in KILL_PATS:
        subprocess.run(
            ['pkill', '-9', '-f', pat],
            check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)
    # Second pass for slow-to-die children
    for pat in KILL_PATS:
        subprocess.run(
            ['pkill', '-9', '-f', pat],
            check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.8)


def cloud_width_rclpy(timeout_s: float = 4.0) -> int:
    """Subscribe with TRANSIENT_LOCAL so latched maps are visible."""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2

    rclpy.init(args=None)
    node = rclpy.create_node('map_smoke_probe')
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    got = {'w': 0}

    def _cb(msg: PointCloud2) -> None:
        got['w'] = int(msg.width) if msg.width else int(msg.height) or 0

    for topic in ('/map/obstacles', '/map_generator/global_cloud'):
        node.create_subscription(PointCloud2, topic, _cb, qos)

    end = time.time() + timeout_s
    while time.time() < end and got['w'] <= 0:
        rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return got['w']


def width_from_log(text: str) -> int:
    best = 0
    for m in WIDTH_RE.finditer(text):
        best = max(best, int(m.group(1)))
    for m in POINTS_RE.finditer(text):
        best = max(best, int(m.group(1)))
    return best


def proc_alive(planner: str) -> bool:
    tip = PLANNER_PROC[planner]
    # Match installed binary paths, not bare names (avoids false positives).
    needle = {
        'homemade': 'lib/drone_planner/planner_node',
        'ego': 'lib/ego_planner/ego_planner_node',
        'gcopter': 'lib/gcopter/global_planning_node',
        'mighty': 'lib/drone_mighty/mighty_planning_node',
        'fast_planner': 'lib/drone_fast_planner/fast_planning_node',
    }.get(planner, tip)
    r = subprocess.run(
        ['pgrep', '-f', needle],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return r.returncode == 0


def smoke_one(planner: str, map_id: str, settle: float, log_dir: Path) -> dict:
    kill_all()
    log_path = log_dir / f'{planner}__{map_id}.log'
    if log_path.exists():
        log_path.unlink()
    cmd = [
        'ros2', 'launch', 'drone_bringup', 'planner_sim.launch.py',
        f'planner:={planner}', f'map:={map_id}',
        'use_rviz:=false', 'seed:=1',
    ]
    env = os.environ.copy()
    env['ROS_LOG_DIR'] = str(log_dir / 'ros')
    with open(log_path, 'w') as lf:
        proc = subprocess.Popen(
            cmd, stdout=lf, stderr=subprocess.STDOUT,
            start_new_session=True, env=env, cwd=str(ROOT))
    t0 = time.time()
    time.sleep(settle)

    width = 0
    try:
        width = cloud_width_rclpy(4.0)
    except Exception as exc:
        width = 0
        print(f'(probe err: {exc})', end=' ', flush=True)

    alive = proc_alive(planner)
    # Snapshot log before killing (tear-down may add noise)
    log_text = log_path.read_text(errors='ignore')
    if width <= 0:
        width = width_from_log(log_text)

    crashed = any(m in log_text for m in CRASH_MARKERS)
    ok = width > 0 and alive and not crashed

    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        pass
    time.sleep(0.5)
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass
    kill_all()

    return {
        'planner': planner,
        'map': map_id,
        'ok': ok,
        'width': width,
        'planner_alive': alive,
        'crashed': crashed,
        'elapsed_s': round(time.time() - t0, 1),
        'log': str(log_path),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--settle', type=float, default=10.0)
    ap.add_argument('--out', type=Path,
                    default=ROOT / 'report' / 'map_smoke.json')
    ap.add_argument('--planners', nargs='*', default=list(PLANNERS))
    ap.add_argument('--maps', nargs='*', default=sorted(MAPS.keys()))
    args = ap.parse_args()

    log_dir = ROOT / 'report' / 'map_smoke_logs'
    log_dir.mkdir(parents=True, exist_ok=True)
    (log_dir / 'ros').mkdir(exist_ok=True)

    kill_all()
    results = []
    print(f'Smoke {len(args.planners)} planners × {len(args.maps)} maps '
          f'(settle={args.settle}s)', flush=True)
    for planner in args.planners:
        for map_id in args.maps:
            print(f'  … {planner:10s} × {map_id:20s}', end=' ', flush=True)
            row = smoke_one(planner, map_id, args.settle, log_dir)
            results.append(row)
            flag = 'PASS' if row['ok'] else 'FAIL'
            print(f'{flag}  width={row["width"]} alive={row["planner_alive"]} '
                  f'crash={row["crashed"]}', flush=True)

    failed = [r for r in results if not r['ok']]
    summary = {
        'total': len(results),
        'passed': len(results) - len(failed),
        'failed': len(failed),
        'failures': failed,
        'results': results,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(summary, indent=2))
    print(f'\n{summary["passed"]}/{summary["total"]} passed → {args.out}',
          flush=True)
    if failed:
        print('FAILURES:', flush=True)
        for r in failed:
            print(f'  - {r["planner"]} × {r["map"]}: width={r["width"]} '
                  f'alive={r["planner_alive"]} crashed={r["crashed"]}',
                  flush=True)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())

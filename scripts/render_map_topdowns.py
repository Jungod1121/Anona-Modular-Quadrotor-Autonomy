#!/usr/bin/env python3
"""Render 3D oblique obstacle point-cloud PNGs for catalog maps.

Usage (workspace root, after sourcing ROS + install):
  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 scripts/render_map_topdowns.py
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'drone_bringup'))

from drone_bringup.maps_catalog import MAPS, normalize_map_id, pose_for_map  # noqa: E402

DEFAULT_MAPS = (
    'official_forest',
    'official_perlin',
    'official_maze2d',
    'official_maze3d',
    'dense_field',
    'narrow_corridor',
)


def cloud_xyz(timeout_s: float = 14.0, max_points: int = 40000) -> Tuple[Optional[np.ndarray], int]:
    """Return (xyz Nx3 or empty array, raw_width)."""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2

    rclpy.init(args=None)
    node = rclpy.create_node('map_topdown_probe')
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    got: Dict[str, object] = {'xyz': None, 'width': -1}

    def cb(msg: PointCloud2) -> None:
        if got['xyz'] is not None:
            return
        got['width'] = int(msg.width)
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not pts:
            got['xyz'] = np.zeros((0, 3), dtype=float)
            return
        arr = np.asarray([[p[0], p[1], p[2]] for p in pts], dtype=float)
        if arr.shape[0] > max_points:
            idx = np.linspace(0, arr.shape[0] - 1, max_points, dtype=int)
            arr = arr[idx]
        got['xyz'] = arr

    for topic in ('/map/obstacles', '/map_generator/global_cloud'):
        node.create_subscription(PointCloud2, topic, cb, qos)

    t0 = time.time()
    while got['xyz'] is None and (time.time() - t0) < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    rclpy.shutdown()
    if got['xyz'] is None:
        return None, -1
    return got['xyz'], int(got['width'])  # type: ignore[arg-type]


def _view_limits(xyz: np.ndarray, pose: dict, bounds: dict, margin: float = 1.5):
    xs, ys, zs = [], [], []
    if xyz is not None and xyz.shape[0] > 0:
        xs.append(xyz[:, 0]); ys.append(xyz[:, 1]); zs.append(xyz[:, 2])
    xs.extend([pose['init_x'], pose['goal_x']])
    ys.extend([pose['init_y'], pose['goal_y']])
    zs.extend([pose.get('init_z', 1.0), pose.get('goal_z', 1.0), 0.0])
    if bounds:
        xs.extend([bounds['xmin'], bounds['xmax']])
        ys.extend([bounds['ymin'], bounds['ymax']])
        zs.extend([bounds.get('zmin', 0.0), bounds.get('zmax', 3.0)])
    x = np.concatenate([np.asarray(a, dtype=float).ravel() for a in xs])
    y = np.concatenate([np.asarray(a, dtype=float).ravel() for a in ys])
    z = np.concatenate([np.asarray(a, dtype=float).ravel() for a in zs])
    return (
        float(x.min()) - margin, float(x.max()) + margin,
        float(y.min()) - margin, float(y.max()) + margin,
        float(max(0.0, z.min() - 0.2)), float(z.max()) + margin,
    )


def plot_map_3d(map_id: str, xyz: np.ndarray, pose: dict, out_path: Path) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    meta = MAPS[normalize_map_id(map_id)]
    label = meta.get('label_en') or map_id
    bounds = meta.get('bounds') or {}

    fig = plt.figure(figsize=(4.4, 3.6), dpi=160)
    fig.patch.set_facecolor('white')
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('white')

    if xyz is not None and xyz.shape[0] > 0:
        # Color by height for depth cue.
        c = xyz[:, 2]
        ax.scatter(
            xyz[:, 0], xyz[:, 1], xyz[:, 2],
            c=c, cmap='viridis', s=1.2, alpha=0.35, linewidths=0, rasterized=True)
    else:
        ax.text2D(0.5, 0.5, 'open / few obstacles', transform=ax.transAxes,
                  ha='center', va='center', fontsize=8, color='#9ca3af')

    ax.scatter(
        [pose['init_x']], [pose['init_y']], [pose.get('init_z', 1.0)],
        c='#16a34a', s=48, marker='o', depthshade=False, label='start')
    ax.scatter(
        [pose['goal_x']], [pose['goal_y']], [pose.get('goal_z', 1.0)],
        c='#dc2626', s=64, marker='*', depthshade=False, label='goal')

    xmin, xmax, ymin, ymax, zmin, zmax = _view_limits(xyz, pose, bounds)
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_zlim(zmin, zmax)
    # Keep world proportions so corridors are not squashed.
    try:
        ax.set_box_aspect((xmax - xmin, ymax - ymin, max(zmax - zmin, 1.0)))
    except Exception:
        pass

    ax.view_init(elev=28, azim=-60)
    ax.tick_params(labelsize=6)
    ax.set_xlabel('x [m]', fontsize=7)
    ax.set_ylabel('y [m]', fontsize=7)
    ax.set_zlabel('z [m]', fontsize=7)
    ax.set_title(f'{map_id}\n{label}', fontsize=9, pad=2)
    ax.legend(loc='upper left', fontsize=6, framealpha=0.85)
    fig.tight_layout(pad=0.2)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def stop_proc(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.terminate()
        except Exception:
            pass
    time.sleep(0.8)
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError, OSError):
            try:
                proc.kill()
            except Exception:
                pass


def render_one(map_id: str, out_dir: Path, settle: float, timeout: float) -> Path:
    mid = normalize_map_id(map_id)
    meta = MAPS[mid]
    seed = int(meta.get('seed') or 1)
    pose = pose_for_map(mid)
    out_path = out_dir / f'map_{mid}.png'
    log_path = out_dir / f'_{mid}_launch.log'

    cmd = [
        'ros2', 'launch', 'drone_bringup', 'map_only.launch.py',
        f'map:={mid}', f'seed:={seed}',
    ]
    with log_path.open('w') as logf:
        proc = subprocess.Popen(
            cmd, stdout=logf, stderr=subprocess.STDOUT,
            cwd=str(ROOT), start_new_session=True,
            env={**os.environ, 'PYTHONNOUSERSITE': '1'})
    try:
        time.sleep(settle)
        xyz, width = cloud_xyz(timeout_s=timeout)
        if xyz is None:
            raise RuntimeError(f'no cloud message for map={mid} (see {log_path})')
        plot_map_3d(mid, xyz, pose, out_path)
        print(
            f'[ok] {mid}: width={width} pts={xyz.shape[0]} → {out_path}',
            flush=True)
        return out_path
    finally:
        stop_proc(proc)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--maps', default=','.join(DEFAULT_MAPS))
    parser.add_argument(
        '--out-dir',
        default=str(ROOT / 'report' / 'final_paper' / 'figures' / 'maps'))
    parser.add_argument('--settle', type=float, default=4.5)
    parser.add_argument('--timeout', type=float, default=14.0)
    args = parser.parse_args(argv)

    maps = [m.strip() for m in args.maps.split(',') if m.strip()]
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    failed: List[str] = []
    for mid in maps:
        try:
            render_one(mid, out_dir, settle=args.settle, timeout=args.timeout)
        except Exception as exc:
            failed.append(mid)
            print(f'[FAIL] {mid}: {exc}', flush=True)

    print(f'Done. ok={len(maps) - len(failed)}/{len(maps)} → {out_dir}', flush=True)
    return 1 if failed else 0


if __name__ == '__main__':
    os.environ.setdefault('PYTHONNOUSERSITE', '1')
    sys.exit(main())

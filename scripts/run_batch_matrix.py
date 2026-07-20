#!/usr/bin/env python3
"""Planner × map-tier × seed batch matrix for fair comparison.

Default matrix (small): homemade, vfh, ego × tier_simple_open, tier_medium_corridor × seed 42.
Writes report/batch_matrix/manifest.json. Use --dry-run to emit the manifest without launching ROS.

Usage (workspace root, after source install/setup.bash):
  python3 scripts/run_batch_matrix.py --dry-run
  python3 scripts/run_batch_matrix.py --duration 45
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

ROOT = Path(__file__).resolve().parents[1]
REPORT_DIR = ROOT / 'report' / 'batch_matrix'
sys.path.insert(0, str(ROOT / 'src' / 'drone_bringup'))

from drone_bringup.maps_catalog import pose_for_map  # noqa: E402
from drone_bringup.planner_registry import PLANNERS, RATES, normalize_planner_id  # noqa: E402

# Evaluation tiers → concrete map ids (see MAPS.md).
MAP_TIERS: Dict[str, str] = {
    'tier_simple_open': 'sparse',
    'tier_medium_corridor': 'narrow_corridor',
}

DEFAULT_PLANNERS = ('homemade', 'vfh', 'ego')
DEFAULT_TIERS = ('tier_simple_open', 'tier_medium_corridor')
DEFAULT_SEEDS = (42,)

KILL_PATS = [
    'ros2 launch drone_bringup',
    'planner_sim.launch.py',
    'vfh_planner_node', 'sac_planner_node', 'rl_planner_node', 'safety_supervisor',
    'lib/drone_planner/planner_node',
    'lib/ego_planner/ego_planner_node',
    'lib/drone_map/map_node',
    'lib/drone_dynamics/dynamics_node',
    'lib/drone_controller/controller_node',
    'evaluate_drone',
]


def cleanup() -> None:
    script = ROOT / 'scripts' / 'cleanup_sim.sh'
    if script.is_file():
        subprocess.run(['bash', str(script)], check=False, cwd=str(ROOT))
    for pat in KILL_PATS:
        subprocess.run(
            ['pkill', '-9', '-f', pat],
            check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    time.sleep(1.0)


def parse_summary(path: Path) -> Dict[str, str]:
    out: Dict[str, str] = {}
    if not path.is_file():
        return out
    for line in path.read_text().splitlines():
        if ':' in line:
            k, v = line.split(':', 1)
            out[k.strip()] = v.strip()
    return out


def planner_tags(planner_id: str) -> Dict[str, Any]:
    pid = normalize_planner_id(planner_id)
    meta = PLANNERS.get(pid, {})
    return {
        'planner_id': pid,
        'class': meta.get('class', 'unknown'),
        'pairwise_tag': 'strong' if meta.get('class') == 'strong' else (
            'weak' if meta.get('class') == 'weak' else meta.get('class', 'unknown')
        ),
        'label_en': meta.get('label_en', pid),
    }


def build_matrix(
    planners: List[str],
    tiers: List[str],
    seeds: List[int],
    sac_shield_ablation: bool,
) -> List[Dict[str, Any]]:
    cells = []
    for planner in planners:
        for tier in tiers:
            for seed in seeds:
                map_id = MAP_TIERS[tier]
                pose = pose_for_map(map_id, planner=planner)
                cell = {
                    'planner': normalize_planner_id(planner),
                    'map_tier': tier,
                    'map_id': map_id,
                    'seed': seed,
                    'goal': [pose['goal_x'], pose['goal_y'], pose['goal_z']],
                    **planner_tags(planner),
                    'sac_shield_ablation': False,
                }
                if normalize_planner_id(planner) == 'sac' and sac_shield_ablation:
                    cell['sac_shield_ablation'] = True
                    cell['launch_extra_args'] = ['safety_supervisor:=false']
                cells.append(cell)
    return cells


def wait_for_topic(topic: str, timeout: float = 40.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        proc = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True, text=True, cwd=str(ROOT),
        )
        if topic in proc.stdout:
            return True
        time.sleep(0.5)
    return False


def run_cell(cell: Dict[str, Any], duration: float, eval_delay: float) -> Dict[str, Any]:
    slug = f"{cell['planner']}__{cell['map_tier']}__seed{cell['seed']}"
    out_dir = REPORT_DIR / 'runs' / slug
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / 'launch.log'

    result = dict(cell)
    result['output_dir'] = str(out_dir)
    result['status'] = 'pending'

    cleanup()
    launch_args = [
        'ros2', 'launch', 'drone_bringup', 'planner_sim.launch.py',
        f"planner:={cell['planner']}",
        f"map:={cell['map_id']}",
        f"seed:={cell['seed']}",
        'use_rviz:=false',
    ]
    launch_args.extend(cell.get('launch_extra_args', []))

    env = os.environ.copy()
    with open(log_path, 'w') as launch_log:
        launch_proc = subprocess.Popen(
            launch_args,
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            cwd=str(ROOT),
            preexec_fn=os.setsid,
            env=env,
        )

    try:
        if not wait_for_topic('/drone/odom', timeout=40.0):
            result['status'] = 'fail'
            result['error'] = 'timeout waiting for /drone/odom'
            return result

        time.sleep(eval_delay)
        gx, gy, gz = cell['goal']
        eval_cmd = [
            'ros2', 'run', 'drone_bringup', 'evaluate_drone',
            '--duration', str(duration),
            '--output-dir', str(out_dir),
            '--goal-x', str(gx), '--goal-y', str(gy), '--goal-z', str(gz),
        ]
        eval_proc = subprocess.Popen(eval_cmd, cwd=str(ROOT), env=env)
        deadline = time.time() + duration + 60.0
        summary_path = out_dir / 'summary.txt'
        eval_started = time.time()
        while time.time() < deadline:
            if (summary_path.is_file() and summary_path.stat().st_size > 0 and
                    summary_path.stat().st_mtime >= eval_started - 1.0):
                time.sleep(0.8)
                break
            if eval_proc.poll() is not None:
                break
            time.sleep(0.5)
        if eval_proc.poll() is None:
            eval_proc.send_signal(signal.SIGINT)
            try:
                eval_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                eval_proc.kill()

        result['summary'] = parse_summary(summary_path)
        result['status'] = 'ok' if summary_path.is_file() else 'fail'
        result['evaluate_exit_code'] = eval_proc.returncode
    finally:
        try:
            os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
            launch_proc.wait(timeout=8)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(launch_proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        cleanup()

    return result


def main() -> int:
    ap = argparse.ArgumentParser(description='Run planner × map-tier × seed batch matrix')
    ap.add_argument('--dry-run', action='store_true', help='Write manifest only; do not launch ROS')
    ap.add_argument('--planners', nargs='*', default=list(DEFAULT_PLANNERS))
    ap.add_argument('--tiers', nargs='*', default=list(DEFAULT_TIERS))
    ap.add_argument('--seeds', nargs='*', type=int, default=list(DEFAULT_SEEDS))
    ap.add_argument('--duration', type=float, default=45.0, help='evaluate_drone duration per cell')
    ap.add_argument('--eval-delay', type=float, default=10.0, help='Seconds after launch before evaluate')
    ap.add_argument('--sac-shield-ablation', action='store_true',
                    help='Stub: run SAC cells with safety_supervisor:=false')
    args = ap.parse_args()

    for tier in args.tiers:
        if tier not in MAP_TIERS:
            print(f'Unknown tier {tier!r}; choose from {list(MAP_TIERS)}', file=sys.stderr)
            return 2

    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now(timezone.utc).astimezone().isoformat(timespec='seconds')
    matrix = build_matrix(args.planners, args.tiers, args.seeds, args.sac_shield_ablation)

    manifest: Dict[str, Any] = {
        'timestamp': ts,
        'dry_run': args.dry_run,
        'rates': dict(RATES),
        'map_tiers': MAP_TIERS,
        'matrix_size': len(matrix),
        'duration_s': args.duration,
        'cells': matrix,
        'results': [],
    }

    if args.dry_run:
        manifest_path = REPORT_DIR / 'manifest.json'
        manifest_path.write_text(json.dumps(manifest, indent=2))
        print(f'Dry-run: {len(matrix)} cells → {manifest_path}')
        for c in matrix:
            print(f"  {c['planner']:10s} × {c['map_tier']:22s} seed={c['seed']} "
                  f"({c['pairwise_tag']}) map={c['map_id']}")
        return 0

    results = []
    for i, cell in enumerate(matrix, 1):
        print(f'\n[{i}/{len(matrix)}] {cell["planner"]} × {cell["map_tier"]} seed={cell["seed"]}',
              flush=True)
        row = run_cell(cell, args.duration, args.eval_delay)
        results.append(row)
        print(f"  -> {row.get('status')} final_err={row.get('summary', {}).get('final_pos_err', '-')}",
              flush=True)

    manifest['results'] = results
    manifest_path = REPORT_DIR / 'manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2))
    print(f'\nWrote {manifest_path} ({len(results)} runs)', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())

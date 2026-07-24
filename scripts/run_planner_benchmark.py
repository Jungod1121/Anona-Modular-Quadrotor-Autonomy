#!/usr/bin/env python3
"""Benchmark seven single-drone planners on forest and dense maps.

The runner supports the complete 7×2 matrix and one planner/map cell. Every
trial keeps its raw evaluator artifacts, updates a latest-results manifest, and
regenerates CSV/JSON/Markdown comparison reports plus charts.

Examples (after sourcing the ROS 2 workspace):

  python3 scripts/run_planner_benchmark.py
  python3 scripts/run_planner_benchmark.py --mode single \
      --planner ego --map official_forest
  python3 scripts/run_planner_benchmark.py --report-only
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = ROOT / 'report' / 'planner_benchmark'
sys.path.insert(0, str(ROOT / 'src' / 'drone_bringup'))

from drone_bringup.maps_catalog import MAPS, normalize_map_id, pose_for_map  # noqa: E402
from drone_bringup.planner_registry import PLANNERS, RATES, normalize_planner_id  # noqa: E402


BENCHMARK_PLANNERS: Tuple[str, ...] = (
    'homemade',
    'ego',
    'gcopter',
    'mighty',
    'fast_planner',
    'vfh',
    'sac',
)
BENCHMARK_MAPS: Tuple[str, ...] = ('official_forest', 'dense_field')
DEFAULT_MAP_SEEDS = {'official_forest': 1, 'dense_field': 42}
MAP_LABELS = {
    'official_forest': 'Random Forest',
    'dense_field': 'Dense Obstacle Field',
}


def now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec='seconds')


def run_stamp() -> str:
    return datetime.now().strftime('%Y%m%dT%H%M%S')


def parse_scalar(value: Any, default: float = float('nan')) -> float:
    token = str(value or '').strip().split()[0] if str(value or '').strip() else ''
    try:
        return float(token)
    except (TypeError, ValueError):
        return default


def parse_bool(value: Any) -> bool:
    return str(value or '').strip().lower() in ('true', '1', 'yes')


def parse_summary(path: Path) -> Dict[str, str]:
    summary: Dict[str, str] = {}
    if not path.is_file():
        return summary
    for line in path.read_text(errors='replace').splitlines():
        if ':' in line:
            key, value = line.split(':', 1)
            summary[key.strip()] = value.strip()
    return summary


def read_metrics(path: Path) -> Dict[str, np.ndarray]:
    if not path.is_file():
        return {}
    columns: Dict[str, List[float]] = {}
    with path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            for key, value in row.items():
                if key == 'planner_state':
                    continue
                try:
                    number = float(value)
                except (TypeError, ValueError):
                    number = float('nan')
                columns.setdefault(key, []).append(number)
    return {key: np.asarray(values, dtype=float) for key, values in columns.items()}


def path_length(points: np.ndarray) -> float:
    if points.shape[0] < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def active_jerk(ts: np.ndarray, points: np.ndarray) -> float:
    if points.shape[0] < 5:
        return float('nan')
    dt = np.diff(ts)
    valid = dt > 1e-5
    if not np.all(valid):
        return float('nan')
    velocity = np.diff(points, axis=0) / dt[:, None]
    acceleration = np.diff(velocity, axis=0) / dt[1:, None]
    jerk = np.diff(acceleration, axis=0) / dt[2:, None]
    if jerk.size == 0:
        return float('nan')
    return float(np.mean(np.linalg.norm(jerk, axis=1)))


def first_sustained_hit(errors: np.ndarray, tolerance: float, samples: int = 10) -> Optional[int]:
    if errors.size < samples:
        return None
    inside = np.isfinite(errors) & (errors <= tolerance)
    run = 0
    for index, value in enumerate(inside):
        run = run + 1 if value else 0
        if run >= samples:
            return index - samples + 1
    return None


def derive_metrics(
    metrics_csv: Path,
    summary: Dict[str, str],
    goal: Sequence[float],
    goal_tolerance: float,
    safety_distance: float,
) -> Dict[str, Any]:
    data = read_metrics(metrics_csv)
    required = ('t', 'px', 'py', 'pz', 'pos_err')
    if any(key not in data or data[key].size == 0 for key in required):
        return {
            'success': False,
            'safety_pass': False,
            'error': 'metrics.csv is missing required samples',
        }

    ts = data['t']
    points = np.column_stack((data['px'], data['py'], data['pz']))
    errors = data['pos_err']
    finite_rows = np.isfinite(ts) & np.all(np.isfinite(points), axis=1) & np.isfinite(errors)
    ts, points, errors = ts[finite_rows], points[finite_rows], errors[finite_rows]
    if ts.size < 2:
        return {'success': False, 'safety_pass': False, 'error': 'too few valid samples'}

    displacement = np.linalg.norm(points - points[0], axis=1)
    motion_candidates = np.flatnonzero(displacement >= 0.15)
    motion_index = int(motion_candidates[0]) if motion_candidates.size else 0
    arrival_index = first_sustained_hit(errors, goal_tolerance)
    success = arrival_index is not None
    end_index = int(arrival_index) if success else len(ts) - 1
    end_index = max(end_index, motion_index + 1)
    active_ts = ts[motion_index:end_index + 1]
    active_points = points[motion_index:end_index + 1]

    segment_dt = np.diff(active_ts)
    segment_dist = np.linalg.norm(np.diff(active_points, axis=0), axis=1)
    valid_dt = segment_dt > 1e-5
    speeds = segment_dist[valid_dt] / segment_dt[valid_dt] if np.any(valid_dt) else np.array([])

    start = points[motion_index]
    goal_vec = np.asarray(goal, dtype=float)
    direct_distance = float(np.linalg.norm(goal_vec - start))
    flown = path_length(active_points)
    efficiency = min(1.0, direct_distance / flown) if flown > 1e-6 else 0.0
    travel_time = (
        float(ts[arrival_index] - ts[motion_index])
        if success and arrival_index is not None
        else float('nan')
    )

    obstacle = data.get('min_obstacle_dist', np.array([], dtype=float))
    obstacle = obstacle[np.isfinite(obstacle)]
    min_clearance = float(np.min(obstacle)) if obstacle.size else float('nan')
    safety_pass = math.isfinite(min_clearance) and min_clearance >= safety_distance

    final_error = float(errors[-1])
    mean_speed = float(np.mean(speeds)) if speeds.size else 0.0
    max_speed = float(np.max(speeds)) if speeds.size else 0.0
    jerk = active_jerk(active_ts, active_points)

    rpm_columns = [data.get(f'rpm{i}', np.array([], dtype=float)) for i in range(4)]
    rpm = np.column_stack(rpm_columns) if all(col.size == len(data['t']) for col in rpm_columns) else np.empty((0, 4))
    rpm = rpm[np.all(np.isfinite(rpm), axis=1)] if rpm.size else rpm

    expected_time = direct_distance / 1.0
    arrival_score = 1.0 if success else 0.0
    clearance_score = (
        min(1.0, max(0.0, min_clearance / max(safety_distance * 1.5, 1e-6)))
        if math.isfinite(min_clearance) else 0.0
    )
    accuracy_score = max(0.0, 1.0 - final_error / max(goal_tolerance, 1e-6)) if success else 0.0
    time_score = (
        min(1.0, expected_time / max(travel_time, expected_time))
        if success and math.isfinite(travel_time) else 0.0
    )
    smoothness_score = 1.0 / (1.0 + max(jerk, 0.0) / 10.0) if math.isfinite(jerk) else 0.0
    overall_score = 100.0 * (
        0.30 * arrival_score
        + 0.20 * clearance_score
        + 0.15 * accuracy_score
        + 0.15 * efficiency
        + 0.10 * time_score
        + 0.10 * smoothness_score
    )

    return {
        'success': success,
        'safety_pass': safety_pass,
        'goal_tolerance_m': goal_tolerance,
        'safety_distance_m': safety_distance,
        'time_to_goal_s': travel_time,
        'final_error_m': final_error,
        'min_clearance_m': min_clearance,
        'path_length_m': flown,
        'direct_distance_m': direct_distance,
        'path_efficiency': efficiency,
        'mean_speed_mps': mean_speed,
        'max_speed_mps': max_speed,
        'mean_jerk_mps3': jerk,
        'mean_motor_rpm': float(np.mean(rpm)) if rpm.size else float('nan'),
        'peak_motor_rpm': float(np.max(rpm)) if rpm.size else float('nan'),
        'planner_success_ever': parse_bool(summary.get('planner_success_ever')),
        'fallback_trigger_count': int(parse_scalar(summary.get('fallback_trigger_count'), 0.0)),
        'overall_score': overall_score,
        'sample_count': int(ts.size),
    }


def cleanup() -> None:
    script = ROOT / 'scripts' / 'cleanup_sim.sh'
    if script.is_file():
        subprocess.run(
            ['bash', str(script)],
            cwd=str(ROOT),
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    time.sleep(0.8)


def wait_for_topic(topic: str, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        proc = subprocess.run(
            ['ros2', 'topic', 'list'],
            cwd=str(ROOT),
            capture_output=True,
            text=True,
            check=False,
        )
        if topic in proc.stdout.splitlines():
            return True
        time.sleep(0.5)
    return False


def stop_process_group(proc: Optional[subprocess.Popen], timeout: float = 8.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=timeout)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def case_key(planner: str, map_id: str) -> str:
    return f'{planner}__{map_id}'


def build_cases(
    mode: str,
    planner: Optional[str],
    map_id: Optional[str],
    seed_override: Optional[int],
) -> List[Dict[str, Any]]:
    if mode == 'single':
        if not planner or not map_id:
            raise ValueError('--mode single requires both --planner and --map')
        planners = [normalize_planner_id(planner)]
        maps = [normalize_map_id(map_id, planner=planners[0])]
    else:
        planners = list(BENCHMARK_PLANNERS)
        maps = list(BENCHMARK_MAPS)

    unknown = [pid for pid in planners if pid not in BENCHMARK_PLANNERS]
    if unknown:
        raise ValueError(
            f'Unsupported benchmark planner(s): {unknown}; choose from {list(BENCHMARK_PLANNERS)}')
    invalid_maps = [mid for mid in maps if mid not in BENCHMARK_MAPS]
    if invalid_maps:
        raise ValueError(
            f'Unsupported benchmark map(s): {invalid_maps}; choose from {list(BENCHMARK_MAPS)}')

    cases: List[Dict[str, Any]] = []
    for pid in planners:
        for mid in maps:
            pose = pose_for_map(mid, planner=pid)
            seed = seed_override if seed_override is not None else DEFAULT_MAP_SEEDS[mid]
            cases.append({
                'key': case_key(pid, mid),
                'planner': pid,
                'planner_label': PLANNERS[pid]['label_en'],
                'planner_class': PLANNERS[pid]['class'],
                'map_id': mid,
                'map_label': MAP_LABELS[mid],
                'seed': int(seed),
                'start': [pose['init_x'], pose['init_y'], pose['init_z']],
                'goal': [pose['goal_x'], pose['goal_y'], pose['goal_z']],
                'safety_distance_m': float(MAPS[mid].get('safety_radius', 0.35)),
            })
    return cases


def run_case(
    case: Dict[str, Any],
    output_dir: Path,
    duration: float,
    eval_delay: float,
    topic_timeout: float,
    goal_tolerance: float,
) -> Dict[str, Any]:
    run_id = run_stamp()
    run_dir = output_dir / 'runs' / case['key'] / run_id
    suffix = 1
    while run_dir.exists():
        run_dir = output_dir / 'runs' / case['key'] / f'{run_id}_{suffix}'
        suffix += 1
    run_dir.mkdir(parents=True, exist_ok=False)
    launch_log_path = run_dir / 'launch.log'
    eval_log_path = run_dir / 'evaluate.log'
    metadata_path = run_dir / 'case.json'

    result = {
        **case,
        'run_id': run_dir.name,
        'started_at': now_iso(),
        'status': 'running',
        'output_dir': str(run_dir),
        'duration_s': duration,
    }
    metadata_path.write_text(json.dumps(result, indent=2))

    cleanup()
    env = os.environ.copy()
    launch_cmd = [
        'ros2', 'launch', 'drone_bringup', 'planner_sim.launch.py',
        f"planner:={case['planner']}",
        f"map:={case['map_id']}",
        f"seed:={case['seed']}",
        'use_rviz:=false',
    ]
    launch_proc: Optional[subprocess.Popen] = None
    eval_proc: Optional[subprocess.Popen] = None
    launch_log = launch_log_path.open('w')
    eval_log = eval_log_path.open('w')
    try:
        launch_proc = subprocess.Popen(
            launch_cmd,
            cwd=str(ROOT),
            env=env,
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        if not wait_for_topic('/drone/odom', topic_timeout):
            result.update(status='fail', error='timeout waiting for /drone/odom')
            return result

        time.sleep(max(0.0, eval_delay))
        gx, gy, gz = case['goal']
        eval_cmd = [
            'ros2', 'run', 'drone_bringup', 'evaluate_drone',
            '--duration', str(duration),
            '--output-dir', str(run_dir),
            '--goal-x', str(gx),
            '--goal-y', str(gy),
            '--goal-z', str(gz),
            '--safety-distance', str(case['safety_distance_m']),
        ]
        eval_started = time.time()
        eval_proc = subprocess.Popen(
            eval_cmd,
            cwd=str(ROOT),
            env=env,
            stdout=eval_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        summary_path = run_dir / 'summary.txt'
        deadline = time.time() + duration + 75.0
        while time.time() < deadline:
            if (
                summary_path.is_file()
                and summary_path.stat().st_size > 0
                and summary_path.stat().st_mtime >= eval_started - 1.0
            ):
                time.sleep(0.5)
                break
            if eval_proc.poll() is not None:
                break
            time.sleep(0.5)

        if eval_proc.poll() is None:
            stop_process_group(eval_proc, timeout=5.0)
        summary = parse_summary(summary_path)
        result['raw_summary'] = summary
        result.update(derive_metrics(
            run_dir / 'metrics.csv',
            summary,
            case['goal'],
            goal_tolerance,
            case['safety_distance_m'],
        ))
        result['evaluate_exit_code'] = eval_proc.returncode
        result['status'] = 'ok' if (run_dir / 'metrics.csv').is_file() else 'fail'
        if result['status'] == 'fail' and 'error' not in result:
            result['error'] = 'evaluate_drone did not produce metrics.csv'
    except Exception as exc:
        result.update(status='fail', success=False, safety_pass=False, error=str(exc))
    finally:
        stop_process_group(eval_proc, timeout=3.0)
        stop_process_group(launch_proc)
        launch_log.close()
        eval_log.close()
        cleanup()
        result['completed_at'] = now_iso()
        metadata_path.write_text(json.dumps(result, indent=2, allow_nan=True))
    return result


def load_latest(output_dir: Path) -> Dict[str, Dict[str, Any]]:
    path = output_dir / 'latest_results.json'
    if not path.is_file():
        return {}
    try:
        payload = json.loads(path.read_text())
    except (json.JSONDecodeError, OSError):
        return {}
    rows = payload.get('results', payload if isinstance(payload, list) else [])
    return {row['key']: row for row in rows if isinstance(row, dict) and row.get('key')}


def finite(value: Any) -> Optional[float]:
    number = parse_scalar(value)
    return number if math.isfinite(number) else None


def result_rows(latest: Dict[str, Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows = []
    for planner in BENCHMARK_PLANNERS:
        for map_id in BENCHMARK_MAPS:
            row = latest.get(case_key(planner, map_id))
            if row:
                rows.append(row)
    return rows


def save_latest(output_dir: Path, latest: Dict[str, Dict[str, Any]]) -> None:
    rows = result_rows(latest)
    payload = {
        'updated_at': now_iso(),
        'matrix_size': len(BENCHMARK_PLANNERS) * len(BENCHMARK_MAPS),
        'completed_cases': len(rows),
        'planners': list(BENCHMARK_PLANNERS),
        'maps': list(BENCHMARK_MAPS),
        'rates': dict(RATES),
        'results': rows,
    }
    (output_dir / 'latest_results.json').write_text(
        json.dumps(payload, indent=2, allow_nan=True))

    fields = [
        'planner', 'planner_label', 'map_id', 'map_label', 'seed', 'status',
        'success', 'safety_pass', 'overall_score', 'time_to_goal_s',
        'final_error_m', 'min_clearance_m', 'path_length_m', 'path_efficiency',
        'mean_speed_mps', 'max_speed_mps', 'mean_jerk_mps3',
        'fallback_trigger_count', 'output_dir', 'completed_at',
    ]
    with (output_dir / 'comparison_results.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def append_history(output_dir: Path, result: Dict[str, Any]) -> None:
    with (output_dir / 'history.jsonl').open('a') as stream:
        stream.write(json.dumps(result, allow_nan=True) + '\n')


def aggregate_by_planner(rows: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    aggregates = []
    for planner in BENCHMARK_PLANNERS:
        subset = [row for row in rows if row.get('planner') == planner]
        if not subset:
            continue

        def mean_of(key: str) -> float:
            values = [value for row in subset if (value := finite(row.get(key))) is not None]
            return float(np.mean(values)) if values else float('nan')

        aggregates.append({
            'planner': planner,
            'planner_label': PLANNERS[planner]['label_en'],
            'cases': len(subset),
            'success_rate': sum(bool(row.get('success')) for row in subset) / len(subset),
            'safety_rate': sum(bool(row.get('safety_pass')) for row in subset) / len(subset),
            'overall_score': mean_of('overall_score'),
            'time_to_goal_s': mean_of('time_to_goal_s'),
            'path_efficiency': mean_of('path_efficiency'),
            'min_clearance_m': mean_of('min_clearance_m'),
            'mean_jerk_mps3': mean_of('mean_jerk_mps3'),
        })
    return sorted(
        aggregates,
        key=lambda row: finite(row.get('overall_score')) or -1.0,
        reverse=True,
    )


def generate_charts(output_dir: Path, rows: Sequence[Dict[str, Any]]) -> List[Path]:
    if not rows:
        return []
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return []

    chart_dir = output_dir / 'charts'
    chart_dir.mkdir(parents=True, exist_ok=True)
    labels = [PLANNERS[planner]['label_en'].split('—', 1)[0].strip() for planner in BENCHMARK_PLANNERS]
    x = np.arange(len(BENCHMARK_PLANNERS))
    colors = {'official_forest': '#4da3ff', 'dense_field': '#ffb454'}

    def values_for(map_id: str, key: str) -> np.ndarray:
        by_planner = {
            row['planner']: finite(row.get(key))
            for row in rows if row.get('map_id') == map_id
        }
        return np.asarray([
            by_planner.get(planner) if by_planner.get(planner) is not None else np.nan
            for planner in BENCHMARK_PLANNERS
        ], dtype=float)

    overview = chart_dir / 'overview.png'
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    width = 0.36
    for offset, map_id in zip((-width / 2, width / 2), BENCHMARK_MAPS):
        axes[0, 0].bar(
            x + offset, values_for(map_id, 'overall_score'), width,
            label=MAP_LABELS[map_id], color=colors[map_id])
        axes[0, 1].bar(
            x + offset, values_for(map_id, 'time_to_goal_s'), width,
            label=MAP_LABELS[map_id], color=colors[map_id])
        axes[1, 0].bar(
            x + offset, values_for(map_id, 'path_efficiency'), width,
            label=MAP_LABELS[map_id], color=colors[map_id])
        clearance_ratio = values_for(map_id, 'min_clearance_m') / float(
            MAPS[map_id].get('safety_radius', 0.35))
        axes[1, 1].bar(
            x + offset, clearance_ratio, width,
            label=MAP_LABELS[map_id], color=colors[map_id])

    titles = (
        ('Overall score', 'score [0–100]'),
        ('Time to goal', 'seconds'),
        ('Path efficiency', 'direct / flown'),
        ('Normalized minimum clearance', 'clearance / safety limit'),
    )
    for axis, (title, ylabel) in zip(axes.flat, titles):
        axis.set_title(title, loc='left', fontweight='bold')
        axis.set_ylabel(ylabel)
        axis.set_xticks(x, labels, rotation=20, ha='right')
        axis.grid(axis='y', alpha=0.25)
    axes[1, 1].axhline(1.0, color='#e35d6a', linestyle='--', linewidth=1, label='safety limit')
    axes[0, 0].legend(fontsize=8)
    axes[1, 1].legend(fontsize=8)
    fig.suptitle('Seven-planner benchmark — Random Forest vs Dense Obstacle Field', fontsize=15)
    fig.savefig(overview, dpi=180)
    plt.close(fig)

    heatmap = chart_dir / 'success_safety_heatmap.png'
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), constrained_layout=True)
    for axis, key, title in (
        (axes[0], 'success', 'Goal reached'),
        (axes[1], 'safety_pass', 'Safety clearance passed'),
    ):
        matrix = np.full((len(BENCHMARK_PLANNERS), len(BENCHMARK_MAPS)), np.nan)
        for row in rows:
            if row.get('planner') in BENCHMARK_PLANNERS and row.get('map_id') in BENCHMARK_MAPS:
                i = BENCHMARK_PLANNERS.index(row['planner'])
                j = BENCHMARK_MAPS.index(row['map_id'])
                matrix[i, j] = 1.0 if row.get(key) else 0.0
        axis.imshow(matrix, vmin=0, vmax=1, cmap='RdYlGn', aspect='auto')
        axis.set_xticks(range(len(BENCHMARK_MAPS)), [MAP_LABELS[mid] for mid in BENCHMARK_MAPS])
        axis.set_yticks(range(len(BENCHMARK_PLANNERS)), labels)
        axis.set_title(title, loc='left', fontweight='bold')
        for i in range(matrix.shape[0]):
            for j in range(matrix.shape[1]):
                text = '—' if np.isnan(matrix[i, j]) else ('PASS' if matrix[i, j] else 'FAIL')
                axis.text(j, i, text, ha='center', va='center', fontsize=8)
    fig.savefig(heatmap, dpi=180)
    plt.close(fig)

    generated = [overview, heatmap]
    for map_id in BENCHMARK_MAPS:
        path = chart_dir / f'trajectories_{map_id}.png'
        fig, axis = plt.subplots(figsize=(9, 6), constrained_layout=True)
        plotted = False
        for planner in BENCHMARK_PLANNERS:
            row = next(
                (candidate for candidate in rows
                 if candidate.get('planner') == planner and candidate.get('map_id') == map_id),
                None,
            )
            if not row:
                continue
            data = read_metrics(Path(row['output_dir']) / 'metrics.csv')
            if 'px' not in data or 'py' not in data:
                continue
            axis.plot(data['px'], data['py'], linewidth=1.2, label=planner)
            plotted = True
        if plotted:
            pose = pose_for_map(map_id)
            axis.scatter([pose['init_x']], [pose['init_y']], marker='o', c='#2ca02c', s=55, label='start')
            axis.scatter([pose['goal_x']], [pose['goal_y']], marker='*', c='#d62728', s=90, label='goal')
            axis.set_title(f'Trajectory comparison — {MAP_LABELS[map_id]}', loc='left', fontweight='bold')
            axis.set_xlabel('x [m]')
            axis.set_ylabel('y [m]')
            axis.set_aspect('equal', adjustable='datalim')
            axis.grid(alpha=0.25)
            axis.legend(ncol=3, fontsize=8)
            fig.savefig(path, dpi=180)
            generated.append(path)
        plt.close(fig)
    return generated


def fmt_number(value: Any, digits: int = 2, suffix: str = '') -> str:
    number = finite(value)
    return f'{number:.{digits}f}{suffix}' if number is not None else '—'


def generate_report(output_dir: Path, latest: Dict[str, Dict[str, Any]]) -> Path:
    rows = result_rows(latest)
    save_latest(output_dir, latest)
    charts = generate_charts(output_dir, rows)
    aggregates = aggregate_by_planner(rows)
    report_path = output_dir / 'comparison_report.md'

    lines = [
        '# Seven-planner comparative benchmark',
        '',
        f'- Updated: `{now_iso()}`',
        f'- Matrix: `{len(BENCHMARK_PLANNERS)} planners × {len(BENCHMARK_MAPS)} maps = 14 cases`',
        f'- Completed latest cases: `{len(rows)}/14`',
        f'- Maps: `{MAP_LABELS[BENCHMARK_MAPS[0]]}`, `{MAP_LABELS[BENCHMARK_MAPS[1]]}`',
        '',
        '## Metric definitions',
        '',
        '- **Success**: position error remains within the goal tolerance for 10 evaluator samples.',
        '- **Time to goal**: elapsed time from first 0.15 m displacement to sustained arrival; launch delay is excluded.',
        '- **Path efficiency**: straight-line distance divided by flown distance (higher is better).',
        '- **Safety**: minimum point-cloud clearance is at least the map safety radius.',
        '- **Smoothness**: mean finite-difference jerk during active travel (lower is better).',
        '- **Overall score**: arrival 30%, clearance 20%, final accuracy 15%, path efficiency 15%, time 10%, smoothness 10%.',
        '',
    ]
    if charts:
        lines.extend([
            '## Comparison charts',
            '',
            '![overview](charts/overview.png)',
            '',
            '![success and safety](charts/success_safety_heatmap.png)',
            '',
        ])
        for map_id in BENCHMARK_MAPS:
            chart = output_dir / 'charts' / f'trajectories_{map_id}.png'
            if chart in charts:
                lines.extend([
                    f'![{MAP_LABELS[map_id]} trajectories](charts/{chart.name})',
                    '',
                ])

    lines.extend([
        '## Overall ranking',
        '',
        '| Rank | Planner | Cases | Success | Safety | Score | Avg time [s] | Efficiency | Avg clearance [m] |',
        '|---:|---|---:|---:|---:|---:|---:|---:|---:|',
    ])
    for rank, row in enumerate(aggregates, 1):
        lines.append(
            f"| {rank} | {row['planner_label']} | {row['cases']}/2 | "
            f"{row['success_rate'] * 100:.0f}% | {row['safety_rate'] * 100:.0f}% | "
            f"{fmt_number(row['overall_score'], 1)} | {fmt_number(row['time_to_goal_s'])} | "
            f"{fmt_number(row['path_efficiency'], 3)} | {fmt_number(row['min_clearance_m'], 3)} |"
        )

    lines.extend([
        '',
        '## Per-case results',
        '',
        '| Planner | Map | Status | Success | Safety | Score | Time [s] | Final error [m] | Clearance [m] | Path [m] | Efficiency | Mean speed [m/s] | Jerk [m/s³] |',
        '|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|',
    ])
    for row in rows:
        lines.append(
            f"| {row.get('planner_label', row['planner'])} | {row.get('map_label', row['map_id'])} | "
            f"{row.get('status', '—')} | {'PASS' if row.get('success') else 'FAIL'} | "
            f"{'PASS' if row.get('safety_pass') else 'FAIL'} | {fmt_number(row.get('overall_score'), 1)} | "
            f"{fmt_number(row.get('time_to_goal_s'))} | {fmt_number(row.get('final_error_m'), 3)} | "
            f"{fmt_number(row.get('min_clearance_m'), 3)} | {fmt_number(row.get('path_length_m'))} | "
            f"{fmt_number(row.get('path_efficiency'), 3)} | {fmt_number(row.get('mean_speed_mps'))} | "
            f"{fmt_number(row.get('mean_jerk_mps3'))} |"
        )

    missing = [
        case_key(planner, map_id)
        for planner in BENCHMARK_PLANNERS
        for map_id in BENCHMARK_MAPS
        if case_key(planner, map_id) not in latest
    ]
    if missing:
        lines.extend(['', '## Missing cases', '', *[f'- `{key}`' for key in missing]])

    lines.extend([
        '',
        '## Artifacts',
        '',
        '- `latest_results.json`: latest result for every planner/map cell',
        '- `comparison_results.csv`: flat comparison table',
        '- `history.jsonl`: append-only history of every trial',
        '- `runs/<planner>__<map>/<timestamp>/`: raw CSV, logs, summaries and per-run plots',
        '',
    ])
    report_path.write_text('\n'.join(lines))
    return report_path


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description='Benchmark seven planners on Random Forest and Dense Obstacle Field')
    parser.add_argument('--mode', choices=('all', 'single'), default='all')
    parser.add_argument('--planner', help=f"Single mode planner: {', '.join(BENCHMARK_PLANNERS)}")
    parser.add_argument('--map', dest='map_id',
                        help=f"Single mode map: {', '.join(BENCHMARK_MAPS)}")
    parser.add_argument('--seed', type=int, default=None,
                        help='Override the map default seed')
    parser.add_argument('--duration', type=float, default=90.0,
                        help='Evaluator duration for each trial')
    parser.add_argument('--eval-delay', type=float, default=1.0,
                        help='Delay after odometry appears before starting evaluator')
    parser.add_argument('--topic-timeout', type=float, default=45.0)
    parser.add_argument('--goal-tolerance', type=float, default=0.5)
    parser.add_argument('--output-dir', type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument('--dry-run', action='store_true',
                        help='Print the selected matrix without launching ROS')
    parser.add_argument('--report-only', action='store_true',
                        help='Regenerate reports/charts from latest saved results')
    args = parser.parse_args(argv)

    if (args.planner or args.map_id) and args.mode == 'all':
        args.mode = 'single'

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    if args.report_only:
        report = generate_report(output_dir, load_latest(output_dir))
        print(f'BENCHMARK_REPORT {report}', flush=True)
        return 0

    try:
        cases = build_cases(args.mode, args.planner, args.map_id, args.seed)
    except ValueError as exc:
        parser.error(str(exc))

    if args.dry_run:
        print(f'Benchmark matrix: {len(cases)} case(s)')
        for index, case in enumerate(cases, 1):
            print(
                f"[{index}/{len(cases)}] {case['planner']} × {case['map_id']} "
                f"seed={case['seed']} goal={case['goal']}")
        return 0

    latest = load_latest(output_dir)
    completed = 0
    try:
        for index, case in enumerate(cases, 1):
            print(
                f"BENCHMARK_PROGRESS {index} {len(cases)} {case['planner']} {case['map_id']}",
                flush=True,
            )
            result = run_case(
                case,
                output_dir,
                duration=max(1.0, args.duration),
                eval_delay=max(0.0, args.eval_delay),
                topic_timeout=max(1.0, args.topic_timeout),
                goal_tolerance=max(0.05, args.goal_tolerance),
            )
            latest[result['key']] = result
            append_history(output_dir, result)
            save_latest(output_dir, latest)
            completed += 1
            print(
                f"BENCHMARK_RESULT {result['key']} status={result.get('status')} "
                f"success={result.get('success', False)} "
                f"score={fmt_number(result.get('overall_score'), 1)}",
                flush=True,
            )
    except KeyboardInterrupt:
        print('BENCHMARK_CANCELLED', flush=True)
    finally:
        cleanup()
        report = generate_report(output_dir, latest)
        print(f'BENCHMARK_REPORT {report}', flush=True)
        print(f'BENCHMARK_DONE {completed} {len(cases)}', flush=True)
    return 0 if completed == len(cases) else 130


if __name__ == '__main__':
    sys.exit(main())

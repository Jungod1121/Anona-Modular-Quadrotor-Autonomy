#!/usr/bin/env python3
"""Benchmark seven single-drone planners on forest and dense maps.

Default mission is a closed square with map-specific corners (not the catalog
straight-line start→goal). The runner supports the complete 7×2 matrix and one
planner/map cell. Every trial keeps raw evaluator artifacts, updates a
latest-results manifest, and regenerates CSV/JSON/Markdown reports plus charts.

Examples (after sourcing the ROS 2 workspace):

  python3 scripts/run_planner_benchmark.py
  python3 scripts/run_planner_benchmark.py --mode single \
      --planner ego --map official_forest
  python3 scripts/run_planner_benchmark.py --mission catalog   # legacy A→B
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

from drone_bringup.maps_catalog import (  # noqa: E402
    MAPS,
    benchmark_square_corners,
    benchmark_square_waypoints,
    normalize_map_id,
    pose_for_benchmark_square,
    pose_for_map,
)
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
# Square perimeter is longer than a straight A→B hop; give trials more time.
DEFAULT_DURATION_S = 120.0
# Match square_mission arrival_tol (~1.5–1.8): 1.2 m was rejecting near-miss
# corner passes (e.g. GCOPTER dense WP2 at 1.30 m → scored 3/4).
DEFAULT_WAYPOINT_TOL_M = 1.5
# Stage gate at each corner: must remain inside the tolerance ball for dwell_s
# (not a one-sample skim). Mean-speed cap is a soft "near-stop" check — set
# high enough for tracking jitter under the square speed caps (~0.65–1.1 m/s).
DEFAULT_WAYPOINT_DWELL_S = 1.0
DEFAULT_WAYPOINT_DWELL_SPEED_MPS = 1.20
# After the last corner dwell, stay near home — no post-lap flyaway.
DEFAULT_POST_MISSION_RADIUS_M = 3.0
# Catalog map safety_radius is 0.35–0.40 m (acceptance-style). On dense
# multi-waypoint squares that bar fails almost everyone (1/14 at catalog).
# Benchmark floor 0.08 m: fails hard cloud collisions / body-scale contacts
# but accepts grazing clearances (successful square flights often land at
# ~0.08–0.14 m min clearance under voxelized obstacle clouds).
DEFAULT_SAFETY_DISTANCE_M = 0.08
BENCHMARK_SAFETY_BY_MAP = {
    'official_forest': 0.08,
    'dense_field': 0.08,
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


def _window_mean_speed(
    points: np.ndarray,
    ts: np.ndarray,
    start: int,
    end: int,
) -> float:
    """Mean finite-difference speed over samples [start, end] inclusive."""
    if end <= start:
        return 0.0
    seg_p = points[start:end + 1]
    seg_t = ts[start:end + 1]
    dt = np.diff(seg_t)
    dist = np.linalg.norm(np.diff(seg_p, axis=0), axis=1)
    valid = dt > 1e-5
    if not np.any(valid):
        return 0.0
    return float(np.mean(dist[valid] / dt[valid]))


def first_waypoint_dwell(
    points: np.ndarray,
    ts: np.ndarray,
    waypoint: Sequence[float],
    tolerance: float,
    search_from: int,
    dwell_s: float,
    dwell_speed_mps: float,
) -> Optional[Tuple[int, int, float]]:
    """First sustained near-stop at waypoint, searching only from search_from.

    Returns (enter_index, exit_index, mean_speed) when a contiguous window of
    duration ≥ dwell_s stays within tolerance and mean speed ≤ dwell_speed_mps.
    """
    if search_from >= len(ts):
        return None
    goal = np.asarray(waypoint, dtype=float)
    distances = np.linalg.norm(points - goal, axis=1)
    inside = np.isfinite(distances) & (distances <= tolerance)

    i = int(search_from)
    n = len(ts)
    while i < n:
        if not inside[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and inside[j + 1]:
            j += 1
        # Slide start within this inside-run so "arrive then stop" still counts.
        for start in range(i, j + 1):
            end = start
            while end <= j and (float(ts[end]) - float(ts[start])) < dwell_s:
                end += 1
            if end > j or (float(ts[end]) - float(ts[start])) < dwell_s:
                break
            speed = _window_mean_speed(points, ts, start, end)
            if speed <= dwell_speed_mps:
                return start, end, speed
        i = j + 1
    return None


def waypoint_visit_stats(
    points: np.ndarray,
    ts: np.ndarray,
    waypoints: Sequence[Sequence[float]],
    tolerance: float,
    dwell_s: float = DEFAULT_WAYPOINT_DWELL_S,
    dwell_speed_mps: float = DEFAULT_WAYPOINT_DWELL_SPEED_MPS,
    home: Optional[Sequence[float]] = None,
    post_mission_radius_m: float = DEFAULT_POST_MISSION_RADIUS_M,
) -> Dict[str, Any]:
    """Sequential stage gates: dwell at WP1, then WP2, … then stay near home.

    Spawn coinciding with the return corner must not count until prior stages
    complete. Brief fly-throughs without a near-stop dwell do not count.
    After the final dwell, any excursion beyond post_mission_radius from home
    (or a final pose outside tolerance) fails the mission.
    """
    details: List[Dict[str, Any]] = []
    visited = 0
    search_from = 0
    last_exit_index: Optional[int] = None
    fail_reason = ''

    for index, waypoint in enumerate(waypoints):
        dwell = first_waypoint_dwell(
            points, ts, waypoint, tolerance, search_from,
            dwell_s, dwell_speed_mps,
        )
        if dwell is None:
            details.append({
                'index': index,
                'waypoint': [float(v) for v in waypoint],
                'visited': False,
                'first_hit_t': float('nan'),
                'dwell_end_t': float('nan'),
                'dwell_speed_mps': float('nan'),
            })
            fail_reason = fail_reason or (
                f'stage {index + 1}/{len(waypoints)}: no {dwell_s:.1f}s '
                f'near-stop within {tolerance:.2f} m'
            )
            # Do not search later stages — order is mandatory.
            for later in range(index + 1, len(waypoints)):
                details.append({
                    'index': later,
                    'waypoint': [float(v) for v in waypoints[later]],
                    'visited': False,
                    'first_hit_t': float('nan'),
                    'dwell_end_t': float('nan'),
                    'dwell_speed_mps': float('nan'),
                })
            break

        enter_i, exit_i, speed = dwell
        details.append({
            'index': index,
            'waypoint': [float(v) for v in waypoint],
            'visited': True,
            'first_hit_t': float(ts[enter_i]),
            'dwell_end_t': float(ts[exit_i]),
            'dwell_speed_mps': float(speed),
        })
        visited += 1
        last_exit_index = exit_i
        # Next stage only after this dwell completes (blocks spawn=return cheat).
        search_from = exit_i + 1

    stages_pass = visited >= len(waypoints) and len(waypoints) > 0
    home_vec = np.asarray(
        home if home is not None else (waypoints[-1] if waypoints else points[0]),
        dtype=float,
    )
    post_ok = False
    final_home_err = float('nan')
    max_post_home_err = float('nan')
    if stages_pass and last_exit_index is not None:
        post = points[last_exit_index:]
        home_dist = np.linalg.norm(post - home_vec, axis=1)
        final_home_err = float(home_dist[-1])
        max_post_home_err = float(np.max(home_dist))
        radius = max(float(post_mission_radius_m), float(tolerance))
        if final_home_err > tolerance:
            fail_reason = (
                f'post-mission: final pose {final_home_err:.2f} m from home '
                f'(tol {tolerance:.2f} m)'
            )
        elif max_post_home_err > radius:
            fail_reason = (
                f'post-mission flyaway: max {max_post_home_err:.2f} m from home '
                f'(limit {radius:.2f} m)'
            )
        else:
            post_ok = True
    elif stages_pass:
        fail_reason = fail_reason or 'post-mission: missing dwell exit index'

    return {
        'visited': visited,
        'total': len(waypoints),
        'pass': bool(stages_pass and post_ok),
        'stages_pass': bool(stages_pass),
        'post_mission_pass': bool(post_ok),
        'last_hit_index': last_exit_index,
        'dwell_s': float(dwell_s),
        'dwell_speed_mps': float(dwell_speed_mps),
        'post_mission_radius_m': float(post_mission_radius_m),
        'final_home_error_m': final_home_err,
        'max_post_home_error_m': max_post_home_err,
        'fail_reason': fail_reason,
        'details': details,
    }


def derive_metrics(
    metrics_csv: Path,
    summary: Dict[str, str],
    goal: Sequence[float],
    goal_tolerance: float,
    safety_distance: float,
    waypoints: Optional[Sequence[Sequence[float]]] = None,
    ideal_path_m: Optional[float] = None,
    dwell_s: float = DEFAULT_WAYPOINT_DWELL_S,
    dwell_speed_mps: float = DEFAULT_WAYPOINT_DWELL_SPEED_MPS,
    post_mission_radius_m: float = DEFAULT_POST_MISSION_RADIUS_M,
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

    wp_stats = None
    if waypoints:
        home = points[0]
        # Prefer explicit mission home (return corner / catalog goal) when given.
        goal_vec = np.asarray(goal, dtype=float)
        if goal_vec.size >= 3 and np.all(np.isfinite(goal_vec)):
            home = goal_vec
        wp_stats = waypoint_visit_stats(
            points, ts, waypoints, goal_tolerance,
            dwell_s=dwell_s,
            dwell_speed_mps=dwell_speed_mps,
            home=home,
            post_mission_radius_m=post_mission_radius_m,
        )
        success = bool(wp_stats['pass'])
        arrival_index = wp_stats['last_hit_index']
    else:
        arrival_index = first_sustained_hit(errors, goal_tolerance)
        success = arrival_index is not None

    end_index = int(arrival_index) if success and arrival_index is not None else len(ts) - 1
    end_index = max(end_index, motion_index + 1)
    active_ts = ts[motion_index:end_index + 1]
    active_points = points[motion_index:end_index + 1]

    segment_dt = np.diff(active_ts)
    segment_dist = np.linalg.norm(np.diff(active_points, axis=0), axis=1)
    valid_dt = segment_dt > 1e-5
    speeds = segment_dist[valid_dt] / segment_dt[valid_dt] if np.any(valid_dt) else np.array([])

    start = points[motion_index]
    goal_vec = np.asarray(goal, dtype=float)
    if ideal_path_m is not None and math.isfinite(ideal_path_m) and ideal_path_m > 1e-6:
        direct_distance = float(ideal_path_m)
    else:
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

    out = {
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
    if wp_stats is not None:
        out['waypoints_visited'] = wp_stats['visited']
        out['waypoints_total'] = wp_stats['total']
        out['waypoint_visits'] = wp_stats
        if wp_stats.get('fail_reason'):
            out['success_fail_reason'] = wp_stats['fail_reason']
    return out


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
    mission: str = 'square',
    safety_distance: Optional[float] = None,
    planners_filter: Optional[Sequence[str]] = None,
) -> List[Dict[str, Any]]:
    if mode == 'single':
        if not planner or not map_id:
            raise ValueError('--mode single requires both --planner and --map')
        planners = [normalize_planner_id(planner)]
        maps = [normalize_map_id(map_id, planner=planners[0])]
    else:
        planners = list(BENCHMARK_PLANNERS)
        maps = list(BENCHMARK_MAPS)
        if planners_filter:
            wanted = [normalize_planner_id(p) for p in planners_filter]
            unknown = [p for p in wanted if p not in BENCHMARK_PLANNERS]
            if unknown:
                raise ValueError(
                    f'Unsupported planner filter(s): {unknown}; '
                    f'choose from {list(BENCHMARK_PLANNERS)}')
            planners = [p for p in planners if p in wanted]

    unknown = [pid for pid in planners if pid not in BENCHMARK_PLANNERS]
    if unknown:
        raise ValueError(
            f'Unsupported benchmark planner(s): {unknown}; choose from {list(BENCHMARK_PLANNERS)}')
    invalid_maps = [mid for mid in maps if mid not in BENCHMARK_MAPS]
    if invalid_maps:
        raise ValueError(
            f'Unsupported benchmark map(s): {invalid_maps}; choose from {list(BENCHMARK_MAPS)}')

    mission = str(mission or 'square').strip().lower()
    if mission not in ('square', 'catalog'):
        raise ValueError("mission must be 'square' or 'catalog'")

    cases: List[Dict[str, Any]] = []
    for pid in planners:
        for mid in maps:
            seed = seed_override if seed_override is not None else DEFAULT_MAP_SEEDS[mid]
            if mission == 'square':
                pose = pose_for_benchmark_square(mid)
                corners = [list(pt) for pt in benchmark_square_corners(mid)]
                waypoints = [list(pt) for pt in benchmark_square_waypoints(mid)]
                ideal = path_length(np.asarray(corners + [corners[0]], dtype=float))
            else:
                pose = pose_for_map(mid, planner=pid)
                corners = []
                waypoints = []
                ideal = float(np.linalg.norm([
                    pose['goal_x'] - pose['init_x'],
                    pose['goal_y'] - pose['init_y'],
                    pose['goal_z'] - pose['init_z'],
                ]))
            if safety_distance is not None:
                safety_m = float(safety_distance)
            else:
                safety_m = float(BENCHMARK_SAFETY_BY_MAP.get(
                    mid, DEFAULT_SAFETY_DISTANCE_M))
            cases.append({
                'key': case_key(pid, mid),
                'planner': pid,
                'planner_label': PLANNERS[pid]['label_en'],
                'planner_class': PLANNERS[pid]['class'],
                'map_id': mid,
                'map_label': MAP_LABELS[mid],
                'seed': int(seed),
                'mission': mission,
                'start': [pose['init_x'], pose['init_y'], pose['init_z']],
                'goal': [pose['goal_x'], pose['goal_y'], pose['goal_z']],
                'corners': corners,
                'waypoints': waypoints,
                'ideal_path_m': float(ideal),
                'safety_distance_m': safety_m,
            })
    return cases


def rescore_row(
    row: Dict[str, Any],
    safety_distance: float,
    goal_tolerance: float,
    dwell_s: float = DEFAULT_WAYPOINT_DWELL_S,
    dwell_speed_mps: float = DEFAULT_WAYPOINT_DWELL_SPEED_MPS,
    post_mission_radius_m: float = DEFAULT_POST_MISSION_RADIUS_M,
) -> Dict[str, Any]:
    """Recompute success/safety/score from saved metrics.csv with new thresholds."""
    run_dir = Path(row.get('output_dir') or '')
    metrics_csv = run_dir / 'metrics.csv'
    summary = parse_summary(run_dir / 'summary.txt')
    goal = row.get('goal') or [0.0, 0.0, 1.0]
    waypoints = row.get('waypoints') or None
    ideal = row.get('ideal_path_m')
    derived = derive_metrics(
        metrics_csv,
        summary,
        goal,
        goal_tolerance,
        safety_distance,
        waypoints=waypoints,
        ideal_path_m=ideal,
        dwell_s=dwell_s,
        dwell_speed_mps=dwell_speed_mps,
        post_mission_radius_m=post_mission_radius_m,
    )
    updated = dict(row)
    updated.update(derived)
    updated['safety_distance_m'] = float(safety_distance)
    updated['rescored_at'] = now_iso()
    return updated


def run_case(
    case: Dict[str, Any],
    output_dir: Path,
    duration: float,
    eval_delay: float,
    topic_timeout: float,
    goal_tolerance: float,
    dwell_s: float = DEFAULT_WAYPOINT_DWELL_S,
    dwell_speed_mps: float = DEFAULT_WAYPOINT_DWELL_SPEED_MPS,
    post_mission_radius_m: float = DEFAULT_POST_MISSION_RADIUS_M,
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
        f"mission:={case.get('mission', 'square')}",
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
            waypoints=case.get('waypoints') or None,
            ideal_path_m=case.get('ideal_path_m'),
            dwell_s=dwell_s,
            dwell_speed_mps=dwell_speed_mps,
            post_mission_radius_m=post_mission_radius_m,
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


def read_obstacles_xy(path: Path, max_points: int = 12000) -> Optional[np.ndarray]:
    if not path.is_file():
        return None
    try:
        with path.open(newline='') as stream:
            reader = csv.DictReader(stream)
            xs, ys = [], []
            for row in reader:
                try:
                    xs.append(float(row['x']))
                    ys.append(float(row['y']))
                except (KeyError, TypeError, ValueError):
                    continue
    except OSError:
        return None
    if not xs:
        return None
    points = np.column_stack((np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)))
    if points.shape[0] > max_points:
        idx = np.linspace(0, points.shape[0] - 1, max_points, dtype=int)
        points = points[idx]
    return points


def plot_trial_trajectory(
    row: Dict[str, Any],
    output_path: Path,
    plt,
) -> bool:
    run_dir = Path(row.get('output_dir') or '')
    metrics_path = run_dir / 'metrics.csv'
    data = read_metrics(metrics_path)
    if 'px' not in data or 'py' not in data or data['px'].size == 0:
        return False

    map_id = str(row.get('map_id') or '')
    planner = str(row.get('planner') or '')
    fig, axis = plt.subplots(figsize=(8.5, 6.5), constrained_layout=True)

    obstacles = read_obstacles_xy(run_dir / 'obstacles.csv')
    if obstacles is not None:
        axis.scatter(
            obstacles[:, 0], obstacles[:, 1],
            s=2.5, c='#9aa0a6', alpha=0.35, linewidths=0, label='obstacles', zorder=1)
    else:
        axis.text(
            0.02, 0.98, 'no obstacles.csv (re-run trial to capture cloud)',
            transform=axis.transAxes, va='top', ha='left', fontsize=8, color='#666666')

    axis.plot(data['px'], data['py'], color='#1f77b4', linewidth=1.6, label='flown', zorder=3)

    corners = row.get('corners') or []
    if not corners:
        try:
            corners = [list(pt) for pt in benchmark_square_corners(map_id)]
        except KeyError:
            corners = []
    if corners:
        xs = [c[0] for c in corners] + [corners[0][0]]
        ys = [c[1] for c in corners] + [corners[0][1]]
        axis.plot(xs, ys, '--', color='#444444', linewidth=1.0, label='square', zorder=2)
        axis.scatter([corners[0][0]], [corners[0][1]], marker='o', c='#2ca02c', s=55, label='start', zorder=4)
        axis.scatter(
            [c[0] for c in corners[1:]], [c[1] for c in corners[1:]],
            marker='x', c='#d62728', s=45, label='corners', zorder=4)
    else:
        start = row.get('start') or []
        goal = row.get('goal') or []
        if len(start) >= 2:
            axis.scatter([start[0]], [start[1]], marker='o', c='#2ca02c', s=55, label='start', zorder=4)
        if len(goal) >= 2:
            axis.scatter([goal[0]], [goal[1]], marker='*', c='#d62728', s=90, label='goal', zorder=4)

    title = (
        f"{row.get('planner_label', planner)} × {row.get('map_label', map_id)}"
    )
    subtitle = (
        f"success={'PASS' if row.get('success') else 'FAIL'}  "
        f"safety={'PASS' if row.get('safety_pass') else 'FAIL'}  "
        f"score={fmt_number(row.get('overall_score'), 1)}"
    )
    axis.set_title(f'{title}\n{subtitle}', loc='left', fontweight='bold', fontsize=11)
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
    axis.set_aspect('equal', adjustable='datalim')
    axis.grid(alpha=0.25)
    axis.legend(loc='best', fontsize=8)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return True


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
    trials_dir = chart_dir / 'trials'
    chart_dir.mkdir(parents=True, exist_ok=True)
    trials_dir.mkdir(parents=True, exist_ok=True)
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
            BENCHMARK_SAFETY_BY_MAP.get(map_id, DEFAULT_SAFETY_DISTANCE_M))
        axes[1, 1].bar(
            x + offset, clearance_ratio, width,
            label=MAP_LABELS[map_id], color=colors[map_id])

    titles = (
        ('Overall score', 'score [0–100]'),
        ('Time to goal', 'seconds'),
        ('Path efficiency', 'ideal / flown'),
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
    fig.suptitle('Seven-planner benchmark — aggregate comparison', fontsize=15)
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
    # Drop legacy combined-map trajectory overlays if present.
    for legacy in (
        chart_dir / 'trajectories_official_forest.png',
        chart_dir / 'trajectories_dense_field.png',
    ):
        if legacy.is_file():
            try:
                legacy.unlink()
            except OSError:
                pass

    for row in rows:
        key = str(row.get('key') or case_key(row.get('planner', ''), row.get('map_id', '')))
        trial_chart = trials_dir / f'{key}.png'
        run_dir = Path(row.get('output_dir') or '')
        run_chart = run_dir / 'trajectory.png' if run_dir else None
        if plot_trial_trajectory(row, trial_chart, plt):
            generated.append(trial_chart)
            row['chart_path'] = str(trial_chart.relative_to(output_dir))
            if run_chart is not None:
                try:
                    plot_trial_trajectory(row, run_chart, plt)
                    row['run_chart_path'] = str(run_chart)
                except Exception:
                    pass
    return generated


def fmt_number(value: Any, digits: int = 2, suffix: str = '') -> str:
    number = finite(value)
    return f'{number:.{digits}f}{suffix}' if number is not None else '—'


def generate_report(output_dir: Path, latest: Dict[str, Dict[str, Any]]) -> Path:
    rows = result_rows(latest)
    charts = generate_charts(output_dir, rows)
    # Persist chart paths that generate_charts stamped onto row dicts.
    save_latest(output_dir, latest)
    aggregates = aggregate_by_planner(rows)
    report_path = output_dir / 'comparison_report.md'

    lines = [
        '# Seven-planner comparative benchmark',
        '',
        f'- Updated: `{now_iso()}`',
        f'- Matrix: `{len(BENCHMARK_PLANNERS)} planners × {len(BENCHMARK_MAPS)} maps = 14 cases`',
        f'- Completed latest cases: `{len(rows)}/14`',
        f'- Maps: `{MAP_LABELS[BENCHMARK_MAPS[0]]}`, `{MAP_LABELS[BENCHMARK_MAPS[1]]}`',
        '- **Single independent runs merge into this report** by planner×map key '
        '(they replace only that cell; other cells stay).',
        '',
        '## Metric definitions',
        '',
        '- **Mission**: map-specific closed square (Random Forest ≈16×12 m; Dense Field ≈23×10 m).',
        '- **Max speed (square mission)**: controller/planner capped at **0.65 m/s** '
        '(acc 1.1 m/s²) to improve tracking and clearance.',
        '- **Success (staged square)**: visit corners **in order**; at each corner '
        f'dwell ≥ **{DEFAULT_WAYPOINT_DWELL_S:.1f} s** inside the waypoint tolerance '
        f'with mean speed ≤ **{DEFAULT_WAYPOINT_DWELL_SPEED_MPS:.2f} m/s** during that '
        'window (stage gate — brief skims do not count). The return corner (spawn) '
        'only counts after prior stages. '
        'After the last dwell, remain within '
        f'**{DEFAULT_POST_MISSION_RADIUS_M:.1f} m** of home and finish inside the '
        'waypoint tolerance — post-lap flyaways fail.',
        '- **Time to goal**: elapsed time from first 0.15 m displacement to completion '
        'of the final corner dwell; launch delay is excluded.',
        '- **Path efficiency**: ideal square perimeter divided by flown distance (higher is better).',
        '- **Safety**: minimum obstacle-cloud clearance ≥ **0.08 m** '
        '(benchmark floor; catalog/acceptance radii 0.35–0.40 m are stricter and '
        'fail almost all dense square flights).',
        '- **Smoothness**: mean finite-difference jerk during active travel (lower is better).',
        '- **Overall score**: arrival 30%, clearance 20%, final accuracy 15%, path efficiency 15%, time 10%, smoothness 10%.',
        '',
    ]
    if charts:
        lines.extend([
            '## Aggregate comparison charts',
            '',
            '![overview](charts/overview.png)',
            '',
            '![success and safety](charts/success_safety_heatmap.png)',
            '',
            '## Per-trial trajectory charts',
            '',
            'One figure per planner×map trial (obstacles overlaid when `obstacles.csv` is present).',
            '',
        ])
        for row in rows:
            rel = row.get('chart_path') or f"charts/trials/{row['key']}.png"
            chart = output_dir / rel
            if chart.is_file():
                label = f"{row.get('planner_label', row['planner'])} × {row.get('map_label', row['map_id'])}"
                lines.extend([f'![{label}]({rel})', ''])

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
        '- `latest_results.json`: latest result for every planner/map cell '
        '(single-cell runs upsert one key and keep the rest)',
        '- `comparison_results.csv`: flat comparison table',
        '- `history.jsonl`: append-only history of every trial',
        '- `charts/overview.png` / `charts/success_safety_heatmap.png`: aggregate summary only',
        '- `charts/trials/<planner>__<map>.png`: one trajectory figure per trial',
        '- `runs/<planner>__<map>/<timestamp>/`: raw CSV, logs, summaries, `trajectory.png`, `obstacles.csv`',
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
    parser.add_argument('--duration', type=float, default=DEFAULT_DURATION_S,
                        help='Evaluator duration for each trial')
    parser.add_argument('--eval-delay', type=float, default=1.0,
                        help='Delay after odometry appears before starting evaluator')
    parser.add_argument('--topic-timeout', type=float, default=45.0)
    parser.add_argument('--goal-tolerance', type=float, default=DEFAULT_WAYPOINT_TOL_M,
                        help='Waypoint visit / final-goal tolerance [m]')
    parser.add_argument('--waypoint-dwell', type=float, default=DEFAULT_WAYPOINT_DWELL_S,
                        help='Required near-stop dwell at each corner [s]')
    parser.add_argument(
        '--waypoint-dwell-speed', type=float, default=DEFAULT_WAYPOINT_DWELL_SPEED_MPS,
        help='Max mean speed during a corner dwell to count as stopped [m/s]')
    parser.add_argument(
        '--post-mission-radius', type=float, default=DEFAULT_POST_MISSION_RADIUS_M,
        help='Max distance from home after last dwell (flyaway fail) [m]')
    parser.add_argument(
        '--safety-distance', type=float, default=DEFAULT_SAFETY_DISTANCE_M,
        help='Min obstacle clearance for safety PASS [m] '
             f'(benchmark default {DEFAULT_SAFETY_DISTANCE_M}; catalog maps use 0.35–0.40)')
    parser.add_argument(
        '--planners', type=str, default='',
        help='Comma-separated planner subset for --mode all (e.g. gcopter,fast_planner)')
    parser.add_argument(
        '--mission', choices=('square', 'catalog'), default='square',
        help='square = map-specific closed square; catalog = single catalog A→B goal')
    parser.add_argument('--output-dir', type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument('--dry-run', action='store_true',
                        help='Print the selected matrix without launching ROS')
    parser.add_argument('--report-only', action='store_true',
                        help='Regenerate reports/charts from latest saved results')
    parser.add_argument(
        '--rescore-only', action='store_true',
        help='Recompute safety/success/score from existing metrics.csv with current thresholds')
    args = parser.parse_args(argv)

    if (args.planner or args.map_id) and args.mode == 'all' and not args.planners:
        args.mode = 'single'

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    safety_m = max(0.05, float(args.safety_distance))
    goal_tol = max(0.05, float(args.goal_tolerance))
    dwell_s = max(0.2, float(args.waypoint_dwell))
    dwell_speed = max(0.05, float(args.waypoint_dwell_speed))
    post_radius = max(goal_tol, float(args.post_mission_radius))

    if args.report_only and not args.rescore_only:
        report = generate_report(output_dir, load_latest(output_dir))
        print(f'BENCHMARK_REPORT {report}', flush=True)
        return 0

    if args.rescore_only:
        latest = load_latest(output_dir)
        rescored = {}
        for key, row in latest.items():
            thr = safety_m
            rescored[key] = rescore_row(
                row, thr, goal_tol,
                dwell_s=dwell_s,
                dwell_speed_mps=dwell_speed,
                post_mission_radius_m=post_radius,
            )
            reason = rescored[key].get('success_fail_reason') or ''
            print(
                f"RESCORE {key} safety={rescored[key].get('safety_pass')} "
                f"success={rescored[key].get('success')} "
                f"wp={rescored[key].get('waypoints_visited')}/"
                f"{rescored[key].get('waypoints_total')} "
                f"score={fmt_number(rescored[key].get('overall_score'), 1)} "
                f"clr={fmt_number(rescored[key].get('min_clearance_m'), 3)}"
                + (f" reason={reason}" if reason and not rescored[key].get('success') else ''),
                flush=True,
            )
        report = generate_report(output_dir, rescored)
        print(f'BENCHMARK_REPORT {report}', flush=True)
        print(f'BENCHMARK_RESCORE {len(rescored)}', flush=True)
        return 0

    planners_filter = None
    if args.planners.strip():
        planners_filter = [p.strip() for p in args.planners.split(',') if p.strip()]

    try:
        cases = build_cases(
            args.mode, args.planner, args.map_id, args.seed,
            mission=args.mission,
            safety_distance=safety_m,
            planners_filter=planners_filter,
        )
    except ValueError as exc:
        parser.error(str(exc))

    if args.dry_run:
        print(
            f'Benchmark matrix: {len(cases)} case(s) mission={args.mission} '
            f'safety>={safety_m:.2f}m')
        for index, case in enumerate(cases, 1):
            print(
                f"[{index}/{len(cases)}] {case['planner']} × {case['map_id']} "
                f"seed={case['seed']} start={case['start']} "
                f"safety={case['safety_distance_m']} "
                f"waypoints={case.get('waypoints') or case['goal']}")
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
                goal_tolerance=goal_tol,
                dwell_s=dwell_s,
                dwell_speed_mps=dwell_speed,
                post_mission_radius_m=post_radius,
            )
            latest[result['key']] = result
            append_history(output_dir, result)
            save_latest(output_dir, latest)
            completed += 1
            print(
                f"BENCHMARK_RESULT {result['key']} status={result.get('status')} "
                f"success={result.get('success', False)} "
                f"safety={result.get('safety_pass', False)} "
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

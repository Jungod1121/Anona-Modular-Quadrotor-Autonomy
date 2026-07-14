#!/usr/bin/env python3
"""
Automated PLAN.md acceptance runner for all six scenarios.

Launches each scenario headless (no RViz), runs evaluate_drone, parses metrics,
and writes report/acceptance_report.md + report/acceptance_results.json.
"""

from __future__ import annotations

import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Tuple

WS = Path(__file__).resolve().parents[1]
REPORT_DIR = WS / 'report'
OUTPUT_ROOT = REPORT_DIR / 'acceptance_runs'


@dataclass
class ScenarioSpec:
    id: int
    name: str
    launch: str
    launch_args: List[str]
    eval_delay: float
    eval_duration: float
    goal: Tuple[float, float, float]
    criteria: Dict[str, str]
    notes: str = ''
    extra_processes: List[List[str]] = field(default_factory=list)


def parse_summary(path: Path) -> Dict[str, str]:
    out: Dict[str, str] = {}
    if not path.is_file():
        return out
    for line in path.read_text().splitlines():
        if ':' in line:
            k, v = line.split(':', 1)
            out[k.strip()] = v.strip()
    return out


def metric_value(summary: Dict[str, str], key: str) -> float:
    raw = summary.get(key, '')
    token = raw.split()[0] if raw else ''
    return parse_float(token)


def parse_float(val: str, default: float = float('nan')) -> float:
    if val is None:
        return default
    # summary.txt values look like "0.1597 m" or "True"
    token = str(val).strip().split()[0] if str(val).strip() else ''
    try:
        return float(token)
    except (TypeError, ValueError):
        return default


def parse_bool(val: str) -> bool:
    return val.strip().lower() in ('true', '1', 'yes')


def waypoint_visits(csv_path: Path, waypoints: List[Tuple[float, float, float]], tol: float = 0.5) -> Dict:
    if not csv_path.is_file():
        return {'visited': 0, 'total': len(waypoints), 'pass': False}
    import csv

    pts = []
    with csv_path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append((float(row['px']), float(row['py']), float(row['pz'])))

    visited = 0
    details = []
    for i, wp in enumerate(waypoints):
        hit = any(math.dist((p[0], p[1]), (wp[0], wp[1])) <= tol for p in pts)
        if hit:
            visited += 1
        details.append({'index': i, 'waypoint': wp, 'visited': hit})
    return {
        'visited': visited,
        'total': len(waypoints),
        'pass': visited >= len(waypoints),
        'details': details,
        'tolerance_m': tol,
    }


def square_waypoints(side: float, z: float) -> List[Tuple[float, float, float]]:
    h = side * 0.5
    return [(h, h, z), (-h, h, z), (-h, -h, z), (h, -h, z), (h, h, z)]


def cleanup_sim() -> None:
    script = WS / 'scripts' / 'cleanup_sim.sh'
    if script.is_file():
        subprocess.run(['bash', str(script)], check=False, cwd=str(WS))
    # Extra sweep for stray ROS2 nodes that can leak goals / maps across scenarios.
    patterns = [
        'dynamics_node', 'controller_node', 'map_node', 'planner_node', 'viz_node',
        'send_goal', 'evaluate_drone', 'waypoint_publisher', 'rviz2',
        'ros2 launch drone_bringup',
    ]
    for pat in patterns:
        subprocess.run(['pkill', '-9', '-f', pat], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)


def wait_for_topic(topic: str, timeout: float = 30.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        proc = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            cwd=str(WS),
        )
        if topic in proc.stdout:
            return True
        time.sleep(0.5)
    return False


def run_scenario(spec: ScenarioSpec, dry_run: bool = False) -> Dict:
    out_dir = OUTPUT_ROOT / f'scenario_{spec.id:02d}_{spec.launch.replace(".launch.py", "")}'
    if out_dir.exists():
        for old in out_dir.glob('*'):
            try:
                old.unlink()
            except IsADirectoryError:
                pass
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / 'launch.log'
    summary_path = out_dir / 'summary.txt'

    result: Dict = {
        'id': spec.id,
        'name': spec.name,
        'launch': spec.launch,
        'output_dir': str(out_dir),
        'pass': False,
        'checks': {},
        'summary': {},
        'notes': spec.notes,
    }

    if dry_run:
        result['status'] = 'dry_run'
        return result

    cleanup_sim()
    time.sleep(1.0)

    launch_cmd = [
        'ros2', 'launch', 'drone_bringup', spec.launch,
        'use_rviz:=false',
        *spec.launch_args,
    ]

    env = os.environ.copy()
    launch_log = open(log_path, 'w')
    launch_proc = subprocess.Popen(
        launch_cmd,
        stdout=launch_log,
        stderr=subprocess.STDOUT,
        cwd=str(WS),
        preexec_fn=os.setsid,
        env=env,
    )

    extra_procs = []
    try:
        if not wait_for_topic('/drone/odom', timeout=40.0):
            result['status'] = 'fail'
            result['error'] = 'timeout waiting for /drone/odom'
            return result

        for cmd in spec.extra_processes:
            extra_procs.append(subprocess.Popen(cmd, cwd=str(WS), env=env))

        time.sleep(spec.eval_delay)

        eval_cmd = [
            'ros2', 'run', 'drone_bringup', 'evaluate_drone',
            '--duration', str(spec.eval_duration),
            '--output-dir', str(out_dir),
            '--goal-x', str(spec.goal[0]),
            '--goal-y', str(spec.goal[1]),
            '--goal-z', str(spec.goal[2]),
        ]
        eval_started = time.time()
        eval_proc = subprocess.Popen(eval_cmd, cwd=str(WS), env=env)
        deadline = time.time() + spec.eval_duration + 60.0
        while time.time() < deadline:
            if (summary_path.is_file() and summary_path.stat().st_size > 0 and
                    summary_path.stat().st_mtime >= eval_started - 1.0):
                # Give export a moment to finish writing.
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
        result['evaluate_exit_code'] = eval_proc.returncode

        summary = parse_summary(out_dir / 'summary.txt')
        result['summary'] = summary

        checks: Dict[str, bool] = {}
        for key, expr in spec.criteria.items():
            if expr.startswith('summary:'):
                field_name = expr.split(':', 1)[1]
                if '_lte_' in field_name:
                    name, limit_s = field_name.rsplit('_lte_', 1)
                    checks[key] = parse_float(summary.get(name, 'inf')) <= float(limit_s)
                elif '_gt_' in field_name:
                    name, limit_s = field_name.rsplit('_gt_', 1)
                    checks[key] = parse_float(summary.get(name, '0')) > float(limit_s)
                elif field_name.endswith('_true'):
                    checks[key] = parse_bool(summary.get(field_name[:-5], 'false'))
                else:
                    checks[key] = parse_bool(summary.get(field_name, 'false'))
            elif expr.startswith('waypoints:'):
                side = float(expr.split(':')[1])
                wp = square_waypoints(side, spec.goal[2])
                wp_result = waypoint_visits(out_dir / 'metrics.csv', wp[:-1], tol=0.6)
                result['waypoint_visits'] = wp_result
                checks[key] = wp_result['pass']
            elif expr.startswith('log_not:'):
                needle = expr.split(':', 1)[1]
                log_text = log_path.read_text(errors='replace') if log_path.is_file() else ''
                checks[key] = needle not in log_text
            elif expr.startswith('log:'):
                pattern = expr.split(':', 1)[1]
                log_text = log_path.read_text(errors='replace') if log_path.is_file() else ''
                checks[key] = bool(re.search(pattern, log_text))

        result['checks'] = checks
        result['pass'] = all(checks.values()) if checks else False
        result['status'] = 'pass' if result['pass'] else 'fail'

        launch_log.flush()
        launch_log.close()
        launch_log = None
        result['launch_log_tail'] = '\n'.join(log_path.read_text(errors='replace').splitlines()[-15:])

    finally:
        if launch_log is not None:
            launch_log.close()
        for p in extra_procs:
            p.send_signal(signal.SIGINT)
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
        try:
            os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
            launch_proc.wait(timeout=8)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(launch_proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        cleanup_sim()
        time.sleep(1.0)

    return result


SCENARIOS = [
    ScenarioSpec(
        id=1,
        name='悬停',
        launch='hover.launch.py',
        launch_args=[],
        eval_delay=8.0,
        eval_duration=45.0,
        goal=(0.0, 0.0, 1.5),
        criteria={
            '位置误差≤0.3m(末段均值)': 'summary:hover_pass_0.3m_true',
            '最终位置误差≤0.3m': 'summary:goal_pass_0.3m_true',
        },
        notes='目标 (0,0,1.5)，3s 后自动发 goal',
    ),
    ScenarioSpec(
        id=2,
        name='单目标点',
        launch='single_goal.launch.py',
        launch_args=[],
        eval_delay=8.0,
        eval_duration=50.0,
        goal=(2.0, 1.0, 1.5),
        criteria={
            '到达目标误差≤0.3m': 'summary:goal_pass_0.3m_true',
            '最大误差≤3.0m(起飞过程)': 'summary:max_pos_err_lte_3.0',
        },
        notes='目标 (2,1,1.5)',
    ),
    ScenarioSpec(
        id=3,
        name='多目标点(正方形4点)',
        launch='multi_goal.launch.py',
        launch_args=['pattern:=square'],
        eval_delay=10.0,
        eval_duration=110.0,
        goal=(1.0, 1.0, 1.5),
        criteria={
            '4个角点均访问': 'waypoints:2.0',
            '最终回到起点附近': 'summary:goal_pass_0.3m_true',
        },
        notes='square side=2m, hold=8s/点, 5s 后开始发航点',
    ),
    ScenarioSpec(
        id=4,
        name='静态避障',
        launch='avoidance.launch.py',
        launch_args=['seed:=42'],
        eval_delay=12.0,
        eval_duration=150.0,
        goal=(17.0, 5.0, 1.5),
        criteria={
            '到达目标误差≤0.5m': 'summary:final_pos_err_lte_0.5',
            '最小障碍距离>0.35m': 'summary:avoidance_pass_0.35m_true',
            '规划器曾报告success': 'summary:planner_success_ever_true',
        },
        notes='dense_field 80圆柱+围墙, seed=42',
    ),
    ScenarioSpec(
        id=5,
        name='狭窄通道绕行',
        launch='narrow_passage.launch.py',
        launch_args=[],
        eval_delay=12.0,
        eval_duration=150.0,
        goal=(17.0, 5.0, 1.5),
        criteria={
            '到达目标误差≤0.5m': 'summary:final_pos_err_lte_0.5',
            '最小障碍距离>0.35m': 'summary:avoidance_pass_0.35m_true',
            '无A*失败日志': 'log_not:A* failed',
        },
        notes='narrow_corridor 80圆柱+窄门+围墙',
    ),
    ScenarioSpec(
        id=6,
        name='稳定性展示',
        launch='stability_demo.launch.py',
        launch_args=['run_eval:=false'],
        eval_delay=8.0,
        eval_duration=90.0,
        goal=(0.0, 0.0, 1.5),
        criteria={
            '风扰下悬停误差≤0.3m': 'summary:hover_pass_0.3m_true',
        },
        notes='wind_enable+imu_noise_enable, 手动跑 evaluate',
    ),
]


def write_report(results: List[Dict], merge_existing: bool = False) -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    if merge_existing:
        prev_path = REPORT_DIR / 'acceptance_results.json'
        if prev_path.is_file():
            try:
                prev = json.loads(prev_path.read_text())
                by_id = {r['id']: r for r in prev.get('results', [])}
                for r in results:
                    by_id[r['id']] = r
                results = [by_id[i] for i in sorted(by_id)]
            except (json.JSONDecodeError, KeyError):
                pass

    ts = datetime.now(timezone.utc).astimezone().strftime('%Y-%m-%d %H:%M:%S %Z')
    passed = sum(1 for r in results if r.get('pass'))
    total = len(results)

    lines = [
        '# PLAN.md 验收对照表',
        '',
        f'> 自动生成时间：{ts}',
        f'> 通过：**{passed}/{total}** 场景',
        '',
        '## 总览',
        '',
        '| # | 场景 | launch | 结果 | 关键指标 |',
        '|---|------|--------|------|----------|',
    ]

    for r in results:
        sid = r['id']
        name = r['name']
        launch = r['launch']
        status = '✅ PASS' if r.get('pass') else ('❌ FAIL' if r.get('status') != 'dry_run' else '⏭ skip')
        summary = r.get('summary', {})
        metrics = []
        if 'mean_pos_err' in summary:
            metrics.append(f"mean_err={summary['mean_pos_err']}")
        if 'final_pos_err' in summary:
            metrics.append(f"final={summary['final_pos_err']}")
        if 'min_obstacle_distance' in summary:
            metrics.append(f"min_obs={summary['min_obstacle_distance']}")
        if 'planner_success_ever' in summary:
            metrics.append(f"planner_ok={summary['planner_success_ever']}")
        wp = r.get('waypoint_visits')
        if wp:
            metrics.append(f"wp={wp['visited']}/{wp['total']}")
        lines.append(f'| {sid} | {name} | `{launch}` | {status} | {"; ".join(metrics) or "-"} |')

    lines.extend([
        '',
        '## 分项检查',
        '',
    ])

    for r in results:
        lines.append(f'### 场景 {r["id"]}：{r["name"]}')
        lines.append('')
        if r.get('notes'):
            lines.append(f'- 说明：{r["notes"]}')
        checks = r.get('checks', {})
        if checks:
            lines.append('- 检查项：')
            for k, v in checks.items():
                mark = '✅' if v else '❌'
                lines.append(f'  - {mark} {k}')
        summary = r.get('summary', {})
        if summary:
            lines.append('- 原始指标：')
            for k, v in summary.items():
                lines.append(f'  - `{k}`: {v}')
        if r.get('launch_log_tail'):
            lines.append('- launch 日志末尾：')
            lines.append('```')
            lines.append(r['launch_log_tail'])
            lines.append('```')
        lines.append('')

    lines.extend([
        '## PLAN.md 硬指标对照',
        '',
        '| 硬指标 | PLAN 要求 | 验收方式 | 当前状态 |',
        '|--------|-----------|----------|----------|',
    ])

    hover = next((r for r in results if r['id'] == 1), {})
    avoid = next((r for r in results if r['id'] == 4), {})
    narrow = next((r for r in results if r['id'] == 5), {})
    stab = next((r for r in results if r['id'] == 6), {})

    def row(name, req, method, res):
        mark = '✅' if res else '❌'
        return f'| {name} | {req} | {method} | {mark} |'

    lines.append(row(
        '悬停误差',
        '≤ 0.3 m',
        'scenario 1 evaluate.py',
        hover.get('pass', False),
    ))
    lines.append(row(
        '避障最小距离',
        '> 安全距离 0.35 m',
        'scenario 4 min_obstacle_distance',
        avoid.get('checks', {}).get('最小障碍距离>0.35m', False),
    ))
    lines.append(row(
        '狭窄通道',
        '规划路径+实际轨迹可展示',
        'scenario 5 到达+无A*失败',
        narrow.get('pass', False),
    ))
    lines.append(row(
        '稳定性',
        '误差/RPM曲线',
        'scenario 6 CSV+PNG',
        stab.get('pass', False) and (OUTPUT_ROOT / 'scenario_06_stability_demo' / 'evaluation.png').is_file(),
    ))

    lines.extend([
        '',
        '## 产物路径',
        '',
        f'- JSON：`report/acceptance_results.json`',
        f'- 各场景原始数据：`report/acceptance_runs/scenario_XX_*/metrics.csv`',
        '',
        '## 复现命令',
        '',
        '```bash',
        'source /opt/ros/humble/setup.bash',
        'cd ~/drone_ws && source install/setup.bash',
        'python3 scripts/run_acceptance.py',
        '```',
        '',
    ])

    (REPORT_DIR / 'acceptance_report.md').write_text('\n'.join(lines) + '\n')
    (REPORT_DIR / 'acceptance_results.json').write_text(
        json.dumps({'timestamp': ts, 'passed': passed, 'total': total, 'results': results}, indent=2)
    )


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description='Run PLAN.md six-scenario acceptance suite')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--only', type=str, default='', help='Comma-separated scenario ids, e.g. 1,4')
    args = parser.parse_args()

    only = {int(x.strip()) for x in args.only.split(',') if x.strip()} if args.only else None
    specs = [s for s in SCENARIOS if only is None or s.id in only]

    # Scenario 6: check evaluation.png exists after run
    results = []
    for spec in specs:
        print(f'\n=== Scenario {spec.id}: {spec.name} ===', flush=True)
        r = run_scenario(spec, dry_run=args.dry_run)
        if spec.id == 6 and not args.dry_run:
            png = OUTPUT_ROOT / 'scenario_06_stability_demo' / 'evaluation.png'
            r['checks']['评测图已生成'] = png.is_file()
            r['pass'] = all(r.get('checks', {}).values()) if r.get('checks') else False
            r['status'] = 'pass' if r['pass'] else 'fail'
        results.append(r)
        print(f"  -> {r.get('status', 'unknown')} pass={r.get('pass')}", flush=True)

    if not args.dry_run:
        write_report(results, merge_existing=bool(only))
        print(f'\nReport written to {REPORT_DIR / "acceptance_report.md"}', flush=True)

    return 0 if all(r.get('pass') for r in results) else 1


if __name__ == '__main__':
    sys.exit(main())

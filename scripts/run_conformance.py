#!/usr/bin/env python3
"""One-planner contract-conformance run: launch stack headless, check, clean up.

    python3 scripts/run_conformance.py --planner ego [--map official_forest]

Exit code 0 = the backend honours its declared command channels.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

WS = Path(__file__).resolve().parent.parent


def wait_for_topic(topic: str, timeout: float = 40.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = subprocess.run(
            ['timeout', '3', 'ros2', 'topic', 'echo', topic, '--once',
             '--field', 'header'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if r.returncode == 0:
            return True
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--planner', default='ego')
    parser.add_argument('--map', default='auto')
    parser.add_argument('--window', type=float, default=60.0)
    parser.add_argument('--goal-x', type=float, default=None)
    parser.add_argument('--goal-y', type=float, default=None)
    args = parser.parse_args()

    env = os.environ.copy()
    launch_cmd = [
        'ros2', 'launch', 'drone_bringup', 'planner_sim.launch.py',
        f'planner:={args.planner}',
        *( [f'map:={args.map}'] if args.map != 'auto' else ['map:=auto'] ),
        'use_rviz:=false',
        *(['mission:=catalog'] if args.planner not in ('fuel_explore',) else []),
    ]
    print(f'[conformance] launching: {" ".join(launch_cmd)}')
    launch_proc = subprocess.Popen(
        launch_cmd, cwd=str(WS), env=env,
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    try:
        if not wait_for_topic('/drone/odom'):
            print('[conformance] FAIL — /drone/odom never appeared')
            return 1
        # Give the planner node a moment to finish its own init/subscriptions.
        time.sleep(5.0)

        checker_args = ['--planner', args.planner, '--window', str(args.window)]
        if args.goal_x is not None:
            checker_args += ['--goal-x', str(args.goal_x)]
        if args.goal_y is not None:
            checker_args += ['--goal-y', str(args.goal_y)]
        rc = subprocess.call(
            ['ros2', 'run', 'drone_bringup', 'planner_conformance'] + checker_args,
            cwd=str(WS), env=env)
        return rc
    finally:
        try:
            os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
            launch_proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(launch_proc.pid), signal.SIGKILL)
            except Exception:
                pass


if __name__ == '__main__':
    sys.exit(main())

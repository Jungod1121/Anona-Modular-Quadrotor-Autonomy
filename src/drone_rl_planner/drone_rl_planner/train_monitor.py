"""Compatibility entrypoint — redirects to ``train_sac_monitor``.

Older scripts/docs may still call::

    python3 -m drone_rl_planner.train_monitor

That now opens the same unified Path H monitor (curriculum-aware curves +
rollout map).
"""

from drone_rl_planner.train_sac_monitor import main

if __name__ == '__main__':
    main()

# Path H curriculum results (snapshot)

**Date:** 2026-07-22  
**Algo:** Polar DrQ-SAC (`shared_drq_v2`), n-step=3, DrQ K=2  
**Weights:** local only (`*.pt` gitignored) under `checkpoints/`

## Stage ladder

| Stage | CLI | Domain | Best success | Checkpoint (local) | Notes |
|-------|-----|--------|--------------|--------------------|-------|
| fast | `--fast` | easy / sparse | **~92.5%** | `sac_polar_fast_best.pt` | High success quickly |
| 2 | `--stage2` | medium | **~96%** | `sac_polar_mid_best.pt` | Strong bridge prior |
| 3 / 3b | `--stage3` / `--stage3b` | full `dense_heavy` | mixed; peak ~75% then regressions | `sac_polar_dense_best.pt` | Collision-buffer continue hurt Live |
| 4 | `--stage4` | mix **15%** dense | **85%** (early-stop) | `sac_polar_mixb_best.pt` | Eval on same mix |
| 5 | `--stage5` | mix **30%** dense | **81.7%** (early-stop @160k) | `sac_polar_mixc_best.pt` | Auto-advanced to stage6 |
| 6 | `--stage6` | mix **50%** dense | **65%** @400k (target 75% missed) | `sac_polar_mixd_best.pt` | Long plateau ~60–65% |

Failed / forensic: `sac_polar_mix_best.pt` (40% dense + dense-only eval, Best stuck ~10%);
`sac_polar_dense_best_collapsed18.pt` (clobber snapshot).

## Takeaways

1. Easy / medium are **solved** for this stack (~90%+).
2. Raising dense fraction lowers the achievable Best; **50% mix ≈ 65% ceiling** in a 400k budget.
3. Full dense needs **longer** training (order of **millions** of env steps), plus careful buffer resets — not blind continue on a collision-filled replay.
4. Open-source code reference: [denisyarats/drq](https://github.com/denisyarats/drq), [facebookresearch/drqv2](https://github.com/facebookresearch/drqv2). SACPlanner is paper-only.

## Reproduce

```bash
export PYTHONPATH=src/drone_rl_planner:$PYTHONPATH
python3 -m drone_rl_planner.train_sac_polar --stage4 --device cuda
# after mixb_best exists:
python3 -m drone_rl_planner.train_sac_polar --stage5 --device cuda
bash src/drone_rl_planner/checkpoints/sac_mix_ramp_supervisor.sh
```

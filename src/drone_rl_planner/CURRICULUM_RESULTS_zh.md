# 路径 H 课程训练结果（快照）

**日期：** 2026-07-22  
**算法：** 极坐标 DrQ-SAC（`shared_drq_v2`），n-step=3，DrQ K=2  
**权重：** 仅本机（`*.pt` 已 gitignore），目录 `checkpoints/`

## 阶段阶梯

| 阶段 | 命令 | 域 | Best 成功率 | 本机权重 | 备注 |
|------|------|----|-------------|----------|------|
| fast | `--fast` | easy / 稀疏 | **~92.5%** | `sac_polar_fast_best.pt` | 很快拉高 |
| 2 | `--stage2` | medium | **~96%** | `sac_polar_mid_best.pt` | 强先验 |
| 3 / 3b | `--stage3` / `--stage3b` | 满 `dense_heavy` | 曾峰值 ~75%，后有回退 | `sac_polar_dense_best.pt` | 撞墙 buffer 续训伤 Live |
| 4 | `--stage4` | 混合 **15%** dense | **85%**（早停） | `sac_polar_mixb_best.pt` | 同分布评测 |
| 5 | `--stage5` | 混合 **30%** dense | **81.7%**（160k 早停） | `sac_polar_mixc_best.pt` | 自动晋级 stage6 |
| 6 | `--stage6` | 混合 **50%** dense | **65%** @400k（未达 75%） | `sac_polar_mixd_best.pt` | 长期平台 ~60–65% |

失败/取证：`sac_polar_mix_best.pt`（40% dense + 纯 dense 评测，Best≈10%）；
`sac_polar_dense_best_collapsed18.pt`（误覆盖快照）。

## 结论

1. easy / medium 对本栈已基本「搞定」（90%+）。
2. dense 比例升高会拉低 Best 上限；**50% 混合在 40 万步预算下约 65% 封顶**。
3. 满 dense 需要更长训练（量级 **百万环境步**），并注意清空 / 更换 buffer，不要在撞墙数据上盲目续训。
4. 开源代码参考 DrQ / DrQ-v2；SACPlanner 仅有论文。

## 复现

```bash
export PYTHONPATH=src/drone_rl_planner:$PYTHONPATH
python3 -m drone_rl_planner.train_sac_polar --stage4 --device cuda
python3 -m drone_rl_planner.train_sac_polar --stage5 --device cuda
bash src/drone_rl_planner/checkpoints/sac_mix_ramp_supervisor.sh
```

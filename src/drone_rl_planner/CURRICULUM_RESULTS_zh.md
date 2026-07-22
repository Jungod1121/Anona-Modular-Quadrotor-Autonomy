# 路径 H 课程训练结果（快照）

**日期：** 2026-07-22  
**算法：** 极坐标 DrQ-SAC（`shared_drq_v2`），n-step=3，DrQ K=2  
**权重：** 仅保存在本机（`*.pt` 已被 gitignore），目录为 `checkpoints/`

## 阶段阶梯

| 阶段 | 命令行 | 域 | Best 成功率 | 检查点（本机） | 备注 |
|------|--------|----|-------------|--------------|------|
| fast | `--fast` | easy / 稀疏 | **~92.5%** | `sac_polar_fast_best.pt` | 很快达到高成功率 |
| 2 | `--stage2` | medium | **~96%** | `sac_polar_mid_best.pt` | 强桥接先验 |
| 3 / 3b | `--stage3` / `--stage3b` | 满 `dense_heavy` | 波动；峰值约 75% 后回退 | `sac_polar_dense_best.pt` | 撞墙 buffer 续训伤害 Live |
| 4 | `--stage4` | 混合 **15%** dense | **85%**（早停） | `sac_polar_mixb_best.pt` | 在同分布上评测 |
| 5 | `--stage5` | 混合 **30%** dense | **81.7%**（160k 早停） | `sac_polar_mixc_best.pt` | 自动晋级到 stage6 |
| 6 | `--stage6` | 混合 **50%** dense | **65%** @400k（未达目标 75%） | `sac_polar_mixd_best.pt` | 长期平台约 60–65% |

失败 / 取证：`sac_polar_mix_best.pt`（40% dense + 仅 dense 评测，Best 卡在约 10%）；
`sac_polar_dense_best_collapsed18.pt`（误覆盖快照）。

## 结论

1. 对本栈而言，easy / medium 已基本**解决**（约 90%+）。
2. 提高 dense 比例会降低可达 Best；在 40 万步预算下，**50% 混合约等于 65% 上限**。
3. 满 dense 需要**更长**训练（量级为**数百万**环境步），并仔细重置 buffer — 不要在充满碰撞的 replay 上盲目续训。
4. 开源代码参考：[denisyarats/drq](https://github.com/denisyarats/drq)、[facebookresearch/drqv2](https://github.com/facebookresearch/drqv2)。SACPlanner 仅有论文。

## 复现

```bash
export PYTHONPATH=src/drone_rl_planner:$PYTHONPATH
python3 -m drone_rl_planner.train_sac_polar --stage4 --device cuda
# 在 mixb_best 存在之后：
python3 -m drone_rl_planner.train_sac_polar --stage5 --device cuda
bash src/drone_rl_planner/checkpoints/sac_mix_ramp_supervisor.sh
```

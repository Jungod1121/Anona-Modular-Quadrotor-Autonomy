# Contributing to Anona

## 分支与 PR 流程（Branch & PR workflow）

CI 在每次 push 到 `main` 以及每个 PR 上运行
（构建 31 个包 + 功能测试 + hover 冒烟，见 `.github/workflows/ci.yml`）。
**让 CI 先行验证，再合并** —— 这是标准流程：

```bash
# 1. 从最新 main 开出工作分支
git checkout main && git pull
git checkout -b fix/<topic>        # 或 feat/、vendor/

# 2. 开发 + 本地门禁（与 CI 完全一致的入口）
bash scripts/test.sh               # 构建 + 功能测试（linter 已豁免）
colcon test-result --all           # 必须 0 failures

# 3. 提交并推送分支
git push -u origin fix/<topic>

# 4. 开 PR（gh 或网页）
gh pr create --fill
# CI 绿了之后 squash-merge：
gh pr merge --squash --delete-branch
```

规则速查：

| 事项 | 约定 |
|---|---|
| 直接 push 到 `main` | 仅限一行级热修；其余一律走 PR |
| commit 信息 | 祈使句、首行 ≤72 字符、说明"为什么" |
| 验收相关改动 | 除 `scripts/test.sh` 外必须附 `python3 scripts/run_acceptance.py`
  结果（场景 4 允许 1 次重试，见 `attempts` 字段） |
| vendor 目录 | 修改需同步更新对应 `VENDOR_NOTES.md` 的溯源/偏差记录 |

## 本地测试

```bash
source /opt/ros/humble/setup.bash
bash scripts/test.sh                       # 全量构建+功能测试（~10 min）
python3 scripts/run_acceptance.py          # 六场景验收（~25 min）
python3 scripts/run_acceptance.py --only 1,5   # 单场景冒烟
```

## vendor 包修改须知

`src/*_vendor` 下除上游算法核心外均为维护中的移植。改动前先读对应
`VENDOR_NOTES.md`：凡偏离上游的行为（如 EGO 的虚拟地板、body-frame twist
旋转）都已登记；新偏差必须同样登记，注明动机。

# AI usage records — Cursor Plan mode archive

This folder collects **Cursor planning-mode plans** written for the `drone_ws` project,
together with **dialogue excerpts** from the main agent transcript around each plan.

Use this as material for `ai_usage.md` / course AI-use documentation.

## 中文说明

这里归档了本仓库在 Cursor **Plan 模式**下写过的方案（至少 8 次正式 CreatePlan），以及每次建计划前后的对话摘录，方便写作业要求的 AI 使用记录。

每个主题一个子目录：`plan.md`（定稿方案）+ `dialogue.md`（对话）+ `meta.json`（在总 transcript 里的锚点）。草稿版在 `99_other_or_draft_plans/`，Cursor 原始文件全量在 `_raw_cursor_plans/`。

- Archived: 2026-07-21T21:03:26
- Raw Cursor plan files also copied to `_raw_cursor_plans/` (15 files)
- Primary transcript: `/home/jungod/.cursor/projects/home-jungod-drone-ws/agent-transcripts/4388ed19-9096-4f14-8c7c-34ff76388ddd/4388ed19-9096-4f14-8c7c-34ff76388ddd.jsonl`

## How each entry is organized

```
NN_topic/
  plan.md           # canonical plan (usually the completed revision)
  dialogue.md       # user/assistant turns around SwitchMode→plan / CreatePlan
  meta.json         # anchors into the transcript
  *.plan.md         # original Cursor filename copy
```

## Index (canonical plans)

| # | Folder | Title | Canonical file | Dialogue anchors |
|---|--------|-------|----------------|------------------|
| 01 | `01_dual_planner_ego_map/` | Dual planner EGO map | `dual_planner_ego_map_668444cd.plan.md` | 3 |
| 02 | `02_multi_planner_switch/` | Multi planner switch | `multi_planner_switch_9abd0d10.plan.md` | 2 |
| 03 | `03_maps_goals_dashboard/` | Maps Goals Dashboard | `maps_goals_dashboard_b9cb5625.plan.md` | 2 |
| 04 | `04_fuel_style_exploration/` | FUEL-style exploration | `fuel-style_exploration_b7847763.plan.md` | 2 |
| 05 | `05_three_planner_integration/` | Three Planner Integration | `three_planner_integration_35dc6f85.plan.md` | 1 |
| 06 | `06_review_led_architecture_upgrade/` | review-led architecture upgrade | `review-led_architecture_upgrade_eafb51b1.plan.md` | 3 |
| 07 | `07_glass_revert_xy_track/` | Glass revert XY track | `glass_revert_xy_track_e08cf224.plan.md` | 1 |
| 08 | `08_map_heading_layout_fix/` | Map heading layout fix | `map_heading_layout_fix_bd27cdb1.plan.md` | 1 |

## Also included

- `00_workspace_PLAN_md/` — repo-root [`PLAN.md`](../PLAN.md) (course/Claude Code execution plan)
- `99_other_or_draft_plans/` — earlier draft `.plan.md` variants (todos still pending)
- `_raw_cursor_plans/` — full dump of `~/.cursor/plans/*.plan.md`

## Notes

- Dialogue extracts are **trimmed** (long tool payloads truncated) so they stay readable.
- Some plan sessions were revised twice (draft → completed); the **completed** file is `plan.md`.
- Path H / SAC curriculum work after mid-July may appear in the same transcript but often
  without a formal CreatePlan artifact — see the live chat + `PLAN.md` for those threads.

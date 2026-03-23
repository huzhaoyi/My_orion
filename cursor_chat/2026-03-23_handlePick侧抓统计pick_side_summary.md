# 2026-03-23 handlePick 侧抓统计 pick_side_summary

## 变更

文件：`src/orion_mtc/src/orchestration/task_manager.cpp`

每次 PICK 在缆绳侧抓候选循环上增加计数器，并在结束时输出一行可 grep 的汇总：

- **成功**：`INFO` — `pick_side_summary candidates_total=... grasp_ik_fail=... pregrasp_ik_fail=... out_of_bounds=... collision=... init_failed=... plan_failed=... exec_failed=... accepted=1 candidate_index=...`
- **全部失败**：`ERROR` — 原 `CABLE_SIDE_GRASP` 文案后追加相同字段，`accepted=0`

含义：`grasp_ik_fail` / `pregrasp_ik_fail` 分别统计 grasp / pregrasp 的 `setFromIK` 失败次数；`collision` 为预抓姿态场景碰撞（原 `PREGRASP_IN_COLLISION`）。

## 联调用法

```bash
grep pick_side_summary ~/.ros/log/latest/*/stdout*
```

根据 grasp vs pregrasp 占比调整 `pick_params.yaml` 中对应候选组。

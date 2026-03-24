# 双模式抓取：deterministic / robust

## 配置（`orion_mtc_params.yaml`）

- `pick_strategy.mode`: `deterministic` | `robust`（默认 deterministic）
- `pick_strategy.pregrasp_planner`: `ptp` | `ompl`（仅 robust 生效；deterministic 强制 PTP）
- `pick_strategy.allow_candidate_fallback`: 是否失败换下一候选（robust；deterministic 始终单候选、失败即停）
- `pick_strategy.freeze_perception_at_job_start`: 任务内不再二次读取 `object_axis`（默认 true）
- `pick_strategy.enable_pick_diagnostics`: 每次尝试输出 `PickAttemptReport`（默认 true）
- `pick_strategy.deterministic_pregrasp_offset_m` / `deterministic_grasp_depth_m`: deterministic 单组几何

## 实现要点

- **`decision/candidate_selector.cpp`**：deterministic 用窄化 `CableGraspConfig`（单 offset/depth、绕轴 0°、无轴向偏移）生成通常唯一候选；robust 用全量 `generateCableSideGrasps`。
- **`PickTaskBuilder`**：`PickBuildOptions` 控制 pregrasp 用 Pilz PTP 或 OMPL；deterministic 跳过「自碰 ACM」阶段；`cablePickStageDisplayNames` 与轨迹段数一致。
- **`TaskManager::handlePick`**：按策略组 `build_opts`；`try_next_on_failure = robust && allow_candidate_fallback`；执行阶段名用动态 `pick_stage_names`。
- **`pick_diagnostics`**：`logPickAttemptReport` 汇总模式、候选下标、pregrasp 规划器、预检/规划/执行结果。

## 行为变更说明

- 默认 **deterministic + 无 fallback**：不再自动遍历全部候选；需「旧版多候选回退」时设 `mode: robust` 且 `allow_candidate_fallback: true`。

# 侧抓预检 IK 超时与 feasibility 对齐

## 当日进展

推进 **orion_mtc** 侧抓预检与 **feasibility** 参数一致性：将 `cable_side_pick_precheck` 中硬编码的 `0.15s` 改为使用与 `FeasibilityChecker` 相同的 `feasibility.ik_timeout`（经 `MTCConfig::ik_timeout_sec` 加载并传入 `precheckCableSideGraspCandidate`）；`orion_mtc_params.yaml` 中 `ik_timeout` 默认调至 `0.4` 并加注释说明仿真可调。

## 完成情况

任务已完成；`colcon build --packages-select orion_mtc` 通过。

## 问题/需求

若仍出现 **NO_IK**，需区分：超时不足（继续增大 `feasibility.ik_timeout`）与目标位姿在关节/姿态上不可解（需 `pick_strategy` 多候选、`approach_normal_sign` 或几何检查）。无额外资源需求。

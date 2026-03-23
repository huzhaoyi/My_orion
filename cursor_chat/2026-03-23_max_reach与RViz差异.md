# 2026-03-23 max_reach_hard 与 RViz 可达性差异说明

## 用户问题
终端报：`缆绳中心距 base_link 原点 1.491 m > feasibility.max_reach_hard=1.200 m`，但认为按运动学 RViz 可以到达该点。

## 结论摘要
1. **预检不是 URDF 运动学包络**：`r_cable` 与 `max_reach_hard` 的比较来自 `task_manager.cpp` 与 `pick_params.yaml` 的配置阈值，**不是**从模型自动计算的最大臂展。
2. **比较对象与 RViz 不一致**：预检是 **缆绳中心（object_pose 位置）** 到 **base_link 原点** 的欧氏距离；RViz 交互规划通常针对 **gripper_tcp / 末端**，且侧向抓取时 TCP 相对缆绳中心有固定偏移。
3. **1.20 m 偏保守**：相对 `orion.urdf` 各关节 translation 粗加，到 `gripper_tcp` 的链长量级明显高于 1.2 m（实际最大可达距离还受关节限位与姿态约束，但 1.2 m 为工程上的硬门限而非精确运动学）。
4. **可调手段**：在确认仿真/实机安全前提下，按需增大 `feasibility.max_reach_hard` / `max_reach_soft`；或后续改为基于候选 grasp 的 TCP 距离预检。

## 相关文件
- `src/orion_mtc/src/orchestration/task_manager.cpp`（`r_cable` 与预检）
- `src/orion_mtc/config/pick_params.yaml`（`feasibility.max_reach_hard`）

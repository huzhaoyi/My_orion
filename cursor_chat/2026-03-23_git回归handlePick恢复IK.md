# 主题：按 git 对比恢复 handlePick 侧抓 IK（对齐 5d52666）

## 结论

- 与「夹取功能全正常」提交 `5d52666` 对比，当前 `handlePick` 中侧抓预检 IK 曾改为：多种子（优先 `joint_states`）+ 显式 `model_frame` 变换 + `KinematicsQueryOptions`。在 HoloOcean 联调中仍出现大量 `grasp IK fail`。
- **Git 侧可见差异**：`4f7be1a` 同时改了 URDF 关节限位、腕部 `continuous`、夹爪 mimic，以及 SRDF 将 `arm` 链 tip 从 `Link6` 改为 `gripper_tcp`；这些会影响可达工作空间，但 **IK 预检代码路径** 与当时能跑通抓取的版本应保持一致以便对照。
- **本次修改**：将预检 IK **恢复为与 `5d52666` 相同**——`PlanningScene` 若可用则用 `getCurrentState()`，否则 `setToDefaultValues()`；对 `cand.grasp_pose` / `cand.pregrasp_pose`（`cable_side_grasp.hpp` 约定为 **base_link** 下）直接调用 `setFromIK(..., hand_frame=gripper_tcp, PICK_IK_TIMEOUT_SEC=0.15)`；pregrasp 前再次从 scene 或默认重置状态（与旧版一致）。**保留** `pick_side_summary` 统计与失败汇总日志。
- **移除**：`setGetLatestJointStateCallback`、`orion_mtc_node` 对 `joint_states` 的重复订阅（可行性模块等已有 joint_state 用法时可避免双路种子不一致）。

## 后续若仍全失败

优先核对：URDF 收紧后的关节限位是否使大量候选姿态在 **joint limit** 内无解；或 `feasibility.max_reach_*` 与感知标定是否一致（与本次代码改动独立）。

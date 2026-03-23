# handlePick 侧抓 IK：与 FeasibilityChecker / MoveIt 约定对齐

## 问题

侧抓预检长期全 `grasp IK fail`，而 RViz 能规划。根因不是「候选不够」，而是 **IK 种子与位姿坐标系** 与审批/运动学约定不一致。

## MoveIt 约定

`RobotState::setFromIK` 的位姿在 **kinematic model 参考系**（即 `RobotModel` 的 model frame，本机 URDF 根为 `world`），见 `moveit/robot_state/robot_state.h` 注释。

缆绳候选里 `cand.grasp_pose` 为 **base_link** 下 `gripper_tcp` 位姿，应使用  
`T_world_tcp = T_world_base * T_base_tcp`；在已更新 `RobotState` 上取 `getGlobalLinkTransform("base_link")` 再右乘候选。

## 与 FeasibilityChecker 一致

`feasibility_checker.cpp` 对 IK 使用：**先**用 `joint_states` 按关节名 `setVariablePosition`，再 `state.update()`，**不用**「仅 PlanningScene 或全零默认」作唯一种子。`handlePick` 原先主要依赖 GetPlanningScene / 默认，与仿真真实关节不一致，KDL 易无解。

## 代码改动

- `pick_seed_robot_state_for_ik`：`joint_states`（非空）→ `PlanningScene` → `setToDefaultValues()`，每次均 `update()`。
- `tcp_pose_in_model_frame(state, pose_base_tcp)`：`getGlobalLinkTransform("base_link") * pose_base_tcp`。
- `orion_mtc_node` 再次订阅 `joint_states` 并 `setGetLatestJointStateCallback`。
- 进入候选循环前打一条 INFO，说明种子顺序与模型系变换。

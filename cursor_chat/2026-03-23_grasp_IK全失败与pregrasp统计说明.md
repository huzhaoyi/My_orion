# grasp IK 全失败与 pregrasp_ik_fail=0 说明

## 结论

1. **`pregrasp_ik_fail=0` 不是统计写错**  
   `handlePick` 里对每个候选**先**对 `grasp_pose` 做 `setFromIK(gripper_tcp)`，只有 `grasp_ik_ok` 为真才会对 `pregrasp_pose` 再解 IK。若**从未**有一次 grasp 成功，则**不会**进入 pregrasp IK 分支，因此 `pregrasp_ik_fail` 保持为 0。

2. **`grasp_ik_fail=1512` 表示 1512 个候选的 grasp 位姿在 KDL 下均无解**（在给定超时与种子状态下），属于**运动学/工作空间/姿态**问题，而不是「只坏了 grasp 统计」这类分支错误。

3. **可达性预检与逐候选失败可同时成立**  
   预检用的是候选集合上 `min( max(‖grasp‖, ‖pregrasp‖) )` 与 `max_reach_hard` 比较：只要**存在**一组候选使「grasp 与 pregrasp 中较远者」足够近，预检就通过。但循环里会对**每一个**候选单独解 grasp IK；大量候选的 grasp TCP 可能仍较远或姿态不可解，从而全部 `grasp IK fail`。

## 建议排查

- 在 RViz / MoveIt 中对日志里某一帧的 `grasp` 位姿（`base_link` + `gripper_tcp`）手动试 IK。  
- 核对 `object_pose`、标定与单位（米）。  
- 若距离在边界附近：考虑 Trac-IK、放宽姿态枚举、或调整 `feasibility` 与目标生成策略。

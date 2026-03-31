# object_pose_fused 数值与「不对」的澄清

## 结论（代数核对）

当前静态 TF：`base_link→camera` 单位、`camera→sensor_link`（0.5 四元数）、`sensor_link→left_arm_base` 平移 (1.55, 0.5653, -0.283628)。若 **`base_link` 与 `camera` 重合**，则某点在 `left_arm_base` 下约为 (1.022, -0.566, 0.265) 时，变到 `base_link` 约为 **(-0.019, 2.572, -0.0005)**；与用户看到的 **(-0.257, 2.571, ~0)** 在 **y/z** 一致，**x** 差约 0.24 来自**中心线弧长点**与**某一 kp 索引**不是同一点（反推 (-0.257,2.571,0) 对应 left 系约 (1.021,-0.565,0.027)，z 与 kp 行不同是正常的）。

因此：**很大概率 TF 与 `tryPublishFusedGrasp` 已在工作**；若仍与终端 `kp[*]|left_arm_base` 逐行对比不一致，主因是 **frame_id 不同 + 抓取点不是单个 keypoint**。

## 姿态 qz=qw=0.707

为 **`left_arm_base` 系侧抓四元数经 TF 转到 `base_link`** 后的结果，与未变换前的 (0.5,-0.5,-0.5,0.5) 之类不一致是预期现象。

## 前端

`PerceptionCard` 原先写死「base_link」易误导；已改为显示 **`PoseStamped.header.frame_id`**（`stateStore` 存 `fusedObjectPoseFrame`）。

## 若物理仍不对

再查 **`base_link→camera` 是否应为非单位**（与真机外参），或 HoloOcean/URDF 中臂根与标定是否一致。

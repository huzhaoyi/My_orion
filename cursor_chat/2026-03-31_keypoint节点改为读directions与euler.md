# keypoint_to_arm_tf：去掉中心线自算，改读 directions / euler_angles

## 变更

- 删除 PCA、去重、中值滤波、弧长抓取点、`localPcaTangent`、`centerline_grasp_test` 及相关 ROS 参数。
- **姿态**：从 Keypoints **`directions`**（声呐/视觉系向量，与点同源 `frame_id`）经 TF 旋到 `left_arm_frame` 后，按 **`target_sensor_to_object_pose` 同款侧向叉乘** 得四元数；**`fused_orientation_source`**=`euler_angles` 时用 **`euler_angles`**（度，内旋 ZYX R-P-Y）得源系四元数再左乘 **`T_left←source` 的旋转**。
- **`auto`**：`||directions||≥1e-6` 用 directions，否则 euler。
- **位置**：**`fused_grasp_position_mode`**=`mean`（默认）或 **`keypoint_index`**。
- **`fused_apply_tcp_frame_correction`**：默认 **false**；**true** 时恢复旧 **R_corr** 左乘（与桥接不一致，仅兼容历史 TCP 调试）。
- mock：新增 **`mock_direction_x/y/z`**。

## 假设

- `euler_angles` 与感知文档一致（roll/pitch/yaw 度、ZYX）；若现场定义不同需改 `vectorEulerZyxDegToQuat` 顺序或换 Source。

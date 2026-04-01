# keypoint_to_arm_tf：移除全关键点 y/z 强制覆盖

## 变更

- 删除参数 **`override_all_keypoints_y`**、**`override_keypoint_y_value`**、**`override_all_keypoints_z`**、**`override_keypoint_z_value`** 及 `processKeypoints` 内对应改写逻辑。
- 订阅到的 **`Keypoints.keypoints`** 始终按消息原始坐标参与 TF / 融合；调试仍可用 **`use_mock_keypoints`** 与 **`mock_kp_*`** / **`mock_preset`**。

## 构建

`colcon build --packages-select orion_mtc` 已通过。

**说明**：若外部 launch 仍传入上述已删除参数名，ROS2 可能产生「未用参数」告警，从上层配置中去掉即可。

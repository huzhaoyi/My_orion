# keypoint_arm_tf 独立启动 fused TF 断树

## 现象

`ros2 launch orion_mtc keypoint_arm_tf.launch.py`（默认 `tf_under_manipulator=false`）时日志：

`fused grasp publish: TF left_arm_base->base_link failed: ... not part of the same tree`

## 原因

默认小 TF 树仅为 `camera → sensor_link → left_arm_base / right_arm_base`，不存在 `base_link`。节点参数 `output_grasp_frame` 默认 `base_link`，`tryPublishFusedGrasp` 需从 `left_arm_frame` 变换到 `output_grasp_frame`，故失败。

launch 头部说明已写明与 URDF `base_link` 树不连通；与 `pick_holoocean` 联调应使用 `tf_under_manipulator:=true`。

## 修改

在本地调试分支 `keypoint_params` 中设置 `output_grasp_frame: left_arm_base`，与中心线拟合坐标系一致，同帧变换可解。

`tf_under_manipulator=true` / `use_platform_tf=true` 分支未改，仍按需使用默认或既有 `base_link` 链。

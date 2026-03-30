# keypoint_arm_tf 对接 sealien_ctrlpilot_location

## 当日进展

推进 **sealien_ctrlpilot_payload_orion / orion_mtc** 的 Keypoints TF 调试 launch，完成与 **sealien_ctrlpilot_location**（`rov.urdf_simulate.xml` + `robot_state_publisher`）的帧名与启动行为对齐。

## 完成情况

- **`keypoint_arm_tf.launch.py`**：新增 **`use_platform_tf`**（默认 `false`）。为 `true` 时仅启动 `keypoint_to_arm_tf_node`，参数为 `sensor_camera1`、`sensor_left_roboticarm`、`sensor_right_roboticarm`，不启动三条 `static_transform_publisher`，避免与机体 URDF 重复静态 TF。
- **README**：补充 `use_platform_tf:=true` 的启动示例与参数说明。

## 问题/需求

- 实车需已运行 location 侧 URDF/融合，保证 **Keypoints 源帧** 与 **`sensor_camera1`** 一致（或通过 `force_source_frame` 覆盖）；MoveIt **`base_link`** 与机体 **`base_link`** 语义需现场再核对。

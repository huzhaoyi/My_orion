# keypoint TF 转机械臂坐标节点

## 当日进展

- 在 `orion_mtc_msgs` 增加 `Keypoints.msg`（与 sealien 侧字段对齐）。
- 在 `orion_mtc` 新增可执行文件 `keypoint_to_arm_tf_node`：订阅 `/keypoints`，将每个 keypoint 作为 `PointStamped`（`header.frame_id` 空则用 `sensor_link`），经 TF 变换到 `left_arm_base` / `right_arm_base`，并用 `RCLCPP_INFO` 打印源系与两臂系坐标。
- 新增独立 launch `launch/keypoint_arm_tf.launch.py`：两个 `static_transform_publisher`（sensor_link→左右臂，平移 1.55,±0.5653,-0.283628）+ 上述节点，与现有 pick/TF launch 分离。

## 使用

```bash
source install/setup.bash
ros2 launch orion_mtc keypoint_arm_tf.launch.py
# 另终端发布测试（示例）
ros2 topic pub /keypoints orion_mtc_msgs/msg/Keypoints "{header: {frame_id: sensor_link}, keypoints: [{x: 2.0, y: 0.0, z: 0.0}]}"
```

## 参数

`input_topic`、`source_frame_override`、`left_arm_frame`、`right_arm_frame`、`tf_timeout_sec`。

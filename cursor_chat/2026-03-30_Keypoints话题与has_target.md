# Keypoints 话题与语义对齐

## 当日进展

- 对照 `sealien_ctrlpilot_msgmanagement/msg/Keypoints.msg`（声纳/感知侧最新定义，含 `has_target`、`task_type`、`directions`、欧拉角等）。
- `keypoint_to_arm_tf_node`：默认 `input_topic` 改为 `/perception/sonar/keypoints`；**不因 `has_target` 跳过**，只要 `keypoints` 非空即做 TF；mock 注入使用 `has_target=true`、`is_available=true` 便于与典型真值一致（对转换逻辑无强制要求）。
- `keypoint_arm_tf.launch.py`：两处 `input_topic` 与说明文字与上述话题一致。
- `orion_mtc_msgs/msg/Keypoints.msg` 与 msgmanagement 版本字段对齐（本地文档/其它引用与统一类型一致）。

## 完成情况

- 已完成：`colcon build --packages-select orion_mtc_msgs orion_mtc` 通过。

## 问题/需求

- 无。若发布端 QoS 与订阅不一致，仍可通过 `qos_best_effort` 等参数调整。

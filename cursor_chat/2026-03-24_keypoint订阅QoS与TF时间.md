# keypoint_to_arm_tf 问题修复

- **QoS**：订阅由默认 Reliable 改为可配置，默认 `qos_best_effort=true`、`qos_depth=10`，与常见 `/keypoints` 发布端（Best Effort）一致，避免完全收不到消息。
- **TF 时间戳**：默认 `tf_use_latest_timestamp=true`，`PointStamped` 用 `rclcpp::Time(0)` 查最新静态 TF，避免 `header.stamp` 与仿真/系统时钟不一致导致 `lookupTransform` 失败。
- 首次收到 Keypoints 时打一条 `first /Keypoints received` 日志，便于确认订阅生效。

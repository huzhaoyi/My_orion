# （已调整）Keypoints 订阅 QoS

曾将默认改为 BEST_EFFORT；现场 `topic info -v` 显示发布为 **RELIABLE** 时，应与发布端一致改回 **RELIABLE 订阅**（`qos_best_effort` 默认 **false**）。

详见：`2026-04-01_Keypoints_QoS与topic_info对齐.md`。

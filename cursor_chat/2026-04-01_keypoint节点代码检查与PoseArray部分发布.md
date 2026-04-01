# keypoint_to_arm_tf 代码检查摘要

## 审查结论（逻辑 / 鲁棒性）

1. **订阅 QoS**：默认 **RELIABLE + depth**，若发布端为 **BEST_EFFORT** 可能无匹配，表现为「echo 有、节点无」。启动现增加 **INFO** 打印当前 QoS，便于对照 `ros2 topic info -v`。
2. **`use_mock_keypoints:=true`**：不创建订阅，仅定时器数据——易与「真话题在打印」混淆。
3. **`tryPublishKeypointsPoseArray`（已修）**：原先任一点 TF 失败即 **整帧不发布**，Web 侧像「收不到」。现为 **跳过失败点**，只要 `poses` 非空即发布；失败统计 **WARN_THROTTLE**。批量变换时用 **`tryTransformPoint(..., log_warnings=false)`** 避免逐点刷屏。
4. **`computeGraspPositionInArmFrame`**：任一点 TF 失败仍会导致 **整帧不融合**（设计如此：均值/弧长需完整集合）；与 UI 用 PoseArray 的「部分点」策略不同。
5. **空 `keypoints`**：有 **WARN_THROTTLE**，属于「收到消息但无点」。

## 构建

`colcon build --packages-select orion_mtc` 已通过。

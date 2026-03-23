# 网页急停 / 回 ready 无反应：rosbridge 先 advertise

## 原因

ROS2 下 `rosbridge_library` 在 **`publish` 且图中尚不能可靠推断话题类型** 时，`MultiPublisher` 会 `TopicNotEstablishedException`，急停与回 ready 的首次发布可能无效。

## 修改

`web/js/wsClient.js`：在 WebSocket `onopen` 里于 `subscribeTopics()` 之后调用 **`advertiseEmptyPublishTopics()`**，对

- `{prefix}/emergency_stop`
- `{prefix}/go_to_ready`

发送 `op: advertise`，`type: std_msgs/msg/Empty`。

之后再点按钮执行原有 `publish` 即可建立发布者与类型。

## 说明

若仍无反应：确认浏览器连的是与 `orion_mtc` 同一 rosbridge（默认 `ws://主机:9090`）；回 ready 在 **PICKING / worker 执行 job** 时后端会拒绝（设计如此），急停应始终有效。

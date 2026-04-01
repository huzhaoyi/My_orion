# Web：Keypoints 轨迹不显示 — 前端修复

## 原因归纳

- rosbridge **PoseArray** JSON：`poses` 偶发非数组对象；**position** 数值可能为 **string**。
- 话题名必须与后端 **`keypoints_posearray_topic`** 一致；`?ns=` 变了但节点话题仍是默认绝对路径时，订阅可能对不上。

## 改动

- **wsClient**：`getKeypointsSubscribeTopic()`（默认 `{prefix}/keypoints_base_link`），**`?keypoints_topic=` / `?keypoints=`** 可写全名覆盖；连接后 **系统日志** 打印实际订阅名；`inferType` / `handleMessage` 用 **`subscribedKeypointsTopic` + `/keypoints_base_link` 后缀** 匹配。
- **stateStore.setKeypointsTrace**：`poses` 支持 `Object.values`；坐标 **Number** 解析；解析失败 **10s 节流 WARN** 写入底部日志。
- **README / main.js**：文档化查询参数。

## 排查

1. 看底部日志是否出现 **「订阅 Keypoints 轨迹: …」** 与后端 `ros2 topic list | grep keypoints` 一致。
2. 若仍无点列，看是否出现 **keypoints_base_link 解析** 类 WARN（rosbridge 类型须为 **geometry_msgs/msg/PoseArray**）。

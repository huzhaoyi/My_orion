# rosbridge：刷新后仍 WARN WebSocketClosedError

## 原因

新客户端已连接并订阅（日志可见），但 **刷新** 时旧 Tab 的 WebSocket 已关，ROS 侧 **~1Hz** 话题仍可能短暂对已注销客户端排队发送 → **对已关闭套接字 write**。

## 改动

1. **`web/js/data/wsClient.js`**：`pagehide`/`beforeunload` 时 **清重连定时器、close(1000)、置 pageUnloading**，避免卸载后仍触发 `scheduleReconnect`；`onclose` 在 `pageUnloading` 时不打「已断开」+ 不重连。
2. **`orion_mtc/launch/rosbridge_websocket_keepalive.launch.py`**：`websocket_ping_interval=25`、`websocket_ping_timeout=120`，启用 Tornado 层 ping。
3. **`pick_holoocean.launch.py`**：用上述 launch 替换原 `rosbridge_server` XML。
4. **`package.xml`**：`exec_depend` **rosbridge_server**、**rosapi**。**README** 说明。

## 假设

- 若 WARN 仍偶发，多为 rosbridge 内部竞态；可 `sudo apt upgrade ros-humble-rosbridge-*` 或无视（不影响新连接）。

# rosbridge `websocket_ping_interval` 类型错误与修复确认

## 现象

`InvalidParameterTypeException`: 将 `websocket_ping_interval` 设为 `25.0`（DOUBLE）时，节点此前因默认值声明为整型而期望 INTEGER。

## 原因

1. Launch 通过 `parameters` 字典传入浮点字面量，与节点首次声明的类型不一致会触发异常。
2. 仅改源码、未 `colcon build` 时，`ros2 launch` 仍从 `install/share/orion_mtc/launch/` 读取旧 launch，会继续报错。

## 处理

- Launch 中通过 **`arguments=['--websocket_ping_interval', str(...), ...]`** 走 argparse，使参数以 DOUBLE 声明。
- 修改后执行 **`colcon build --packages-select orion_mtc`**，再启动。

## 验证

在 `sealien_ws` 下 rebuild 后，`ros2 launch orion_mtc rosbridge_websocket_keepalive.launch.py` 可正常启动，`Rosbridge WebSocket server started on port 9090`，无类型异常。

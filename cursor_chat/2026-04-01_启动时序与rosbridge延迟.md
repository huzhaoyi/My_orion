# 2026-04-01 启动时序与 rosbridge 延迟

## 现象

`pick_holoocean.launch.py` 曾用 `actions.insert(1, rosbridge)` 把 rosbridge 插在列表前部，Web 很快连上并轮询 `get_queue_state`，而 `mtc_node` 服务注册偏晚，rosbridge 报 `InvalidServiceException: ... get_queue_state does not exist`。

## 改动

1. **`pick_holoocean.launch.py`**：`OpaqueFunction` + `TimerAction`，在 demo/桥接/MTC/keypoint **之后**再启动 rosbridge；新增参数 **`rosbridge_startup_delay_sec`**（默认 `5.0`，`0` 表示无 Timer、仅顺序置后）。
2. **`web/js/data/wsClient.js`**：`callService` 支持 **`service_missing_retries`**；`getQueueState` 默认 **10** 次退避重试（解析回包 JSON 中是否存在类 “does not exist / InvalidServiceException” 文案）。
3. **`README.md`**：补充上述前置与参数说明。

## 话题侧

感知/Keypoints 仍依赖发布端与订阅端 QoS、全名一致；本次未改话题名，仅减轻 **服务** 竞态。

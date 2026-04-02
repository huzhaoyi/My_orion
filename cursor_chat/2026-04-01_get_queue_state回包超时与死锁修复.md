# get_queue_state 回包超时（RMW failed to send response）

## 含义

`failed to send response to /manipulator/get_queue_state (timeout): client will not receive response`：服务端已处理完回调，但向客户端写出响应时 RMW 认为会话已过期（常见原因：处理过晚，或阻塞过久）。

## 根因（本仓库）

1. **`handleGetQueueState` 锁顺序错误**：先 `queue_->size()/peekFront`（队列锁），再 `getCurrentJobId()` 等（`worker_mutex_`）；而 `submitJob` 先持 `worker_mutex_` 再 `queue_->push`。两路径交错时可 **死锁**，`get_queue_state` 长期不返回 → 回包超时。
2. **单 MutuallyExclusive 回调组**：轻量查询与 `check_pick` / `go_to_ready` 等长任务串行排队，高负载时拉长响应时间。
3. **`orion_mtc_node` 未加入 executor**：`FeasibilityChecker` 的 `joint_states` 订阅不 spin（顺带修复）。

## 改动摘要

- `TaskManager::fillGetQueueStateResponse`：在持 `worker_mutex_` 期间访问队列，与 `submitJob` 一致。
- `get_robot_state` / `get_queue_state` / `get_recent_jobs`：`Reentrant` 回调组。
- `main`：`MultiThreadedExecutor` 同时 `add_node` action_client 与 planning 节点。

# handlePick 成功分支 state_mutex_ 自死锁修复

## 现象

`executePickSolution` 已打印 `all segments finished, return true`，但 worker / 前端仍像卡在任务末尾；根因是 **`handlePick()` 成功分支在已持有 `state_mutex_` 时调用 `setState()` 与 `getHeldObject()`**，二者内部再次对同一非递归 `mutex` 加锁，同一线程死锁。

## 根因（代码事实）

- `TaskManager::setState()`：`std::lock_guard` 锁 `state_mutex_`。
- `TaskManager::getHeldObject()`：同样锁 `state_mutex_`。
- 原成功分支：`{ lock; held_object_=...; setState(...); held_object_state_fn_(getHeldObject()); }` → 第二行即死锁。

## 修改

文件：`src/orion_mtc/src/orchestration/task_manager.cpp`

- 在短暂临界区内只做 `held_object_ = new_held`，并拷贝 `held_copy = held_object_`。
- 释放锁后依次：`setState(HOLDING_TRACKED)`、`held_object_state_fn_(held_copy)`。

与 `handlePlaceSingle` 成功路径（先清 `held_object_.valid` 再锁外 `setState` + 回调）一致。

## 验证

- `colcon build --packages-select orion_mtc` 通过。

## 未改项

- `removeWorldObject cable_seg_*` 的 “scene 中不存在” 警告仍为独立问题，与本次死锁无关。

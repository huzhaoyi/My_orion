# 服务提前注册与 FeasibilityChecker 延后

## 现象

`pick_holoocean` 在 rosbridge（约 5s 延迟）已连上后仍长时间报 `get_queue_state does not exist`。原因是 **`OrionMTCNode` 构造在 `initInterfaces` 之前执行 `FeasibilityChecker`（RobotModelLoader）**，耗时可达数十秒，**所有 `/manipulator` 服务尚未 `create_service`**，DDS 上看不到服务。

## 改动

1. **`initModules`**：不再创建 FeasibilityChecker。
2. **`initInterfaces`**：`ManipulatorInterfaceContext` 中 `feasibility_checker` 先为 `nullptr`。
3. **`initFeasibility`**（新）：在 **`setupPlanningScene()` 开头**调用，与 executor **spin 并行**加载重型模型；完成后 `task_manager->setFeasibilityChecker` + `manipulator_iface->setFeasibilityChecker`。
4. **`ManipulatorRosInterface`**：`setFeasibilityChecker` + `handleCheckPick` 对 `ctx_.feasibility_checker` 加互斥，避免与注入竞态。

未就绪时 `check_pick` 仍为「审批模块未就绪」；`handlePick` 内本即可在 `feasibility_checker_ == nullptr` 时跳过工作空间硬限。

## README

补充说明 mtc 尽早 advertise、`FeasibilityChecker` 延后加载。

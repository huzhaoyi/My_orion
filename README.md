# My_orion

ROS 2 工作空间，用于 Orion 机械臂与 **HoloOcean** 仿真联调：关节状态来自 HoloOcean ArmSensor，MoveIt 规划、MTC 抓取，轨迹经桥接发给 HoloOcean 执行。

## 包结构

| 包名 | 说明 |
|------|------|
| **orion_description** | 机器人 URDF、网格与描述资源 |
| **orion_moveit_config** | MoveIt 配置（SRDF、关节限位、OMPL、Pilz PTP/LIN、运动学、控制器等） |
| **orion_mtc_msgs** | 抓取相关接口：Pick Action；GetRobotState、GetQueueState、SubmitJob、CancelJob、GetRecentJobs、ResetHeldObject、SyncHeldObject、CheckPick 等服务；**TargetSet**（多目标候选）等消息 |
| **orion_mtc** | 基于 MoveIt Task Constructor 的**抓取**节点（仅 pick / 夹爪 / 持物同步，无放置任务）。内部按 **app / interface / perception / decision / planning / execution / scene / orchestration** 分层：`ManipulatorRosInterface` 负责 ROS 订阅与服务；`TaskManager` 编排；`PickTaskBuilder` + MTC 只做运动序列；`SolutionExecutor`/`TrajectoryExecutor` 执行；`PlanningSceneManager` 管理 scene；`FeasibilityChecker` 与 `cable_side_pick_precheck` 做审批与缆绳侧抓预检；**`target_set` + `TargetSelector`** 与 **`object_pose`** 二选一供抓取目标。规划末端参考系为 URDF **`gripper_tcp`**。任务队列 + Worker，支持优先级与去重；夹取失败时自动回到 ready 并设 IDLE；另提供 **打开/闭合夹爪** 服务（仅动夹爪，臂关节保持当前 joint_states）；Action 即时执行，话题/SubmitJob 异步入队 |
| **orion_holoocean_bridge** | HoloOcean 桥接：ArmSensor → `joint_states`；FollowJointTrajectory 经 **trajectory_to_agent_bridge** 转为 AgentCommand 发往 HoloOcean 执行；**target_sensor_to_object_pose**：TargetSensor（positions + directions）+ ROV 里程计 → base_link 下 `/object_pose`；**cable_sensor_to_object_pose**：CableSensor（单缆绳）→ world 下变换到 base_link 后发布 `/object_pose` |

## 依赖

- ROS 2（建议 Humble 或更高）
- MoveIt 2
- MoveIt Task Constructor（core + msgs）
- Pilz Industrial Motion Planner（PTP/LIN）
- holoocean-ros（含 `holoocean_interfaces`），通过环境变量 `HOLOOCEAN_ROS_INSTALL` 指定其 install 目录，或先 `source` 该工作区

## 构建

```bash
cd /path/to/My_orion
colcon build --symlink-install
source install/setup.bash
```

## orion_mtc 接口说明

任务可经 **Action（即时执行）** 或 **话题/SubmitJob（异步入队）** 提交；队列按优先级调度，支持去重与取消未执行 job。

- **抓取**：使用 `/manipulator/object_pose`（PoseStamped，可为任意 frame，执行前变换到 `base_link`）。缆绳侧向抓取需 **`/manipulator/object_axis`**（Vector3Stamped，缆绳轴向）。可选 **`/manipulator/target_set`**（`orion_mtc_msgs/TargetSet`）：若 `targets` 非空，`pick_trigger` 优先取**第一个**目标，否则回退到 `object_pose` 的 latest。话题触发的 PICK 在执行时会取**最新**位姿（先等待一次更新再取 latest）。夹不到时判失败并自动回到 ready（臂+手张开）再设 IDLE。抓稳后保存持物上下文，状态变为 HOLDING_TRACKED/HOLDING_UNTRACKED。
- **重置持物 / 同步持物**：通过对应服务或 SubmitJob 入队，用于场景与持物状态一致。
- **打开/闭合夹爪**：仅对手 group 做 MoveTo open/close，**臂关节保持当前 joint_states**（来自 joystick/编码器等），通过服务触发并异步入队执行。

| 类型 | 名称 | 说明 |
|------|------|------|
| Action | `/manipulator/pick` | 即时抓取：Goal 为 `object_pose`（base_link）、可选 `object_id` |
| 服务 | `/manipulator/get_robot_state` | 当前 `mode`、`task_id`、`held_object_id`、`has_held_object`、`last_error` |
| 服务 | `/manipulator/get_queue_state` | 队列与 Worker：`queue_size`、`current_job_id/type`、`next_job_id/type`、`worker_status`、`worker_running`、`queue_empty`、`last_error` |
| 服务 | `/manipulator/submit_job` | 异步入队：提交 PICK、RESET_HELD_OBJECT、SYNC_HELD_OBJECT、OPEN_GRIPPER、CLOSE_GRIPPER（`job_type` 0～4），返回 `job_id` 或拒绝原因 |
| 服务 | `/manipulator/cancel_job` | 取消队列中未执行的 job（`job_id`）；正在执行的返回 cannot cancel running job |
| 服务 | `/manipulator/get_recent_jobs` | 最近 N 条执行记录（含 result_code、created_at/started_at/finished_at），便于长跑排查 |
| 服务 | `/manipulator/reset_held_object` | 清空持物状态并清理 scene 中的 attach |
| 服务 | `/manipulator/sync_held_object` | 同步持物状态（tracked/untracked）到 scene |
| 服务 | `/manipulator/open_gripper` | 打开夹爪（std_srvs/srv/Trigger）：异步入队，仅手 group 到 open，臂关节保持当前 joint_states |
| 服务 | `/manipulator/close_gripper` | 闭合夹爪（std_srvs/srv/Trigger）：异步入队，仅手 group 到 close，臂关节保持当前 joint_states |
| 服务 | `/manipulator/check_pick` | 抓取可行性审批（几何/IK/碰撞诊断） |
| 话题 | `/manipulator/object_pose` | 物体位姿（PoseStamped）：位置=目标中心；**侧向抓取**时姿态为抓取系（由 direction/圆柱轴构造：y=闭合方向、z=接近方向）。桥接由 TargetSensor 或 CableSensor 发布；执行抓取时取最新一帧 |
| 话题 | `/manipulator/object_axis` | 缆绳轴向（Vector3Stamped），侧向抓取必填 |
| 话题 | `/manipulator/target_set` | 多目标候选（`orion_mtc_msgs/TargetSet`）；`pick_trigger` 优先用 `targets[0]`，否则用 `object_pose` |
| 话题 | `/manipulator/pick_trigger` | 空消息：异步入队**抓取**（需有 `object_pose` 或 `target_set`，或等待 3s） |
| 话题 | `/manipulator/emergency_stop` | `std_msgs/msg/Empty`：急停——取消当前 FollowJointTrajectory、清空待执行队列、中止后续 solution 段 |
| 话题 | `/manipulator/go_to_ready` | `std_msgs/msg/Empty`：回到 SRDF ready 并张开手（**仅在非抓取且 worker 未执行 job 时**接受；否则拒绝并打日志） |
| 话题 | `/manipulator/left_arm_gripped` | `std_msgs/Float32`：夹爪传感器反馈（如 0=张开、1=夹紧）；**持物态是否与物理一致以此为准**——当反馈为「张开」且当前为持物态（非 PICKING）时，节点会清除内部 `held_object`、置 `task_mode` 为 IDLE、清理 planning scene 中 attach，并发布 `held_object_state`（与 `reset_held_object` 的 scene 清理一致） |
| 话题 | `/manipulator/tf`、`/manipulator/tf_static` | 机械臂 TF（HoloOcean 联调 launch 下由 robot_state_publisher / move_group 发布并订阅，与全局 `/tf` 隔离） |

业务规则：已持物时禁止再次 pick；**夹爪 locked（有物，来自 /left_arm_gripped）时禁止 pick**，避免 object 在夹爪上导致规划失败，需先 reset_held_object 或 open_gripper。队列默认按优先级调度（RESET/SYNC 高于 PICK 等），同类型同目标短时间窗口内去重；配置见 `orion_mtc/config/orion_mtc_params.yaml`（抓取几何 `feasibility` / `cable_side_grasp` + `runtime_policy`）。

## 运行（HoloOcean 联调）

关节状态来自 HoloOcean 的 ArmSensor，规划在 MoveIt 中完成，轨迹通过桥接节点发给 HoloOcean 执行。

**启动：**

```bash
# 确保已设置 HOLOOCEAN_ROS_INSTALL 或已 source holoocean-ros 的 install
ros2 launch orion_mtc pick_holoocean.launch.py
```

桥接提供两种目标源（launch 中均会启动）：**target_sensor_to_object_pose** 从 TargetSensor（positions + directions）+ ROV 里程计变换到 base_link 并发布 `/manipulator/object_pose`；**cable_sensor_to_object_pose** 从 CableSensor（单缆绳）变换到 base_link 并发布 `/manipulator/object_pose`。位姿为**侧向抓取系**（由 direction/圆柱轴构造：y=闭合方向、z=接近方向）。目标会随传感器实时更新；话题触发的抓取在执行前会取最新位姿。所有接口均在 `/manipulator` 命名空间下。**TF**：本 launch 下机械臂相关 TF 发布/订阅在 `/manipulator/tf`、`/manipulator/tf_static`（由 `tf_under_manipulator:=true` 启用），与仿真侧或其他 TF 源隔离。

**前置条件（抓取）**：需有 `/manipulator/object_pose` 或 `/manipulator/target_set` 中有效目标（或等待 3s），否则 pick_trigger 不入队。若 HoloOcean 未跑或暂无目标，可：① 先启动 HoloOcean 并确保有目标；或 ② 手动发布 object_pose 再 pick_trigger；或 ③ 用 Action `/manipulator/pick` 并在 Goal 里带 `object_pose`。

**HoloOcean 重启**：仿真时间回退时，若 `map→rov0` TF 仍用 PoseSensor 头时间戳，tf2 会报 `TF_OLD_DATA`。桥接默认 `use_pose_sensor_stamp_for_rov_tf:=false`，对 ROV 相关 TF/感知消息使用 ROS 节点时钟，与 `joint_states` 桥接一致，无需重连 ROS 节点即可恢复；若全栈 `use_sim_time` 且 `/clock` 随仿真重置，可改回 `true`。

**触发方式：**

- 仅抓取（需已存在 `/manipulator/object_pose`，例如由桥接 target_sensor_to_object_pose 发布）：
  ```bash
  ros2 topic pub --once /manipulator/pick_trigger std_msgs/msg/Empty "{}"
  ```
- 打开/闭合夹爪（臂关节保持当前 joint_states，仅手爪动）：
  ```bash
  ros2 service call /manipulator/open_gripper std_srvs/srv/Trigger "{}"
  ros2 service call /manipulator/close_gripper std_srvs/srv/Trigger "{}"
  ```
- 无 HoloOcean/目标时，先手动发物体位姿再抓取：
  ```bash
  ros2 topic pub --once /manipulator/object_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.35, y: -0.15, z: -0.4}, orientation: {w: 1.0}}}"
    
  ros2 topic pub --once /manipulator/pick_trigger std_msgs/msg/Empty "{}"
  ```
- Action 仅抓取：
  ```bash
  ros2 action send_goal /manipulator/pick orion_mtc_msgs/action/Pick \
    "{object_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.35, y: -0.15, z: 0.4}, orientation: {w: 1.0}}}, object_id: 'cube_1'}"
  ```

**配置：**

- 桥接与目标索引：`orion_holoocean_bridge/config/holoocean_bridge_params.yaml`（如 `target_index`、`trajectory_to_agent_bridge`、`cable_sensor_to_object_pose` 的 `cable_sensor_topic` 等）
- `orion_mtc` 抓取与运行策略（单文件）：`orion_mtc/config/orion_mtc_params.yaml`（`cable_side_grasp`、`feasibility`、`runtime_policy` 等）
- **MoveIt**（规划器/限位/控制器等）仍使用 `orion_moveit_config/config/` 下多文件，与 `orion_mtc` 应用参数分离，符合 MoveIt 惯例
- **工作空间限值**（水平/垂直 min～max，防止抓取目标配置超出）：见 [docs/workspace_limits.md](docs/workspace_limits.md)；**TF 与坐标系**（world/ROV/base_link）：见 [docs/tf_conversion.md](docs/tf_conversion.md)

**仅查看机器人模型：**

```bash
ros2 launch orion_description display.launch.py
```

**仅 MoveIt + RViz（无 MTC，关节由 GUI 或外部发布）：**

```bash
ros2 launch orion_moveit_config demo.launch.py
```

## Web 上位机

仓库内提供基于浏览器的上位机界面（`web/`），通过 **rosbridge** 与 orion_mtc 通信，与上文「orion_mtc 接口说明」中的话题/服务一一对应。

**机器人模型同步**：修改 `orion_description` 或 `rov_urdf` 的 URDF/STL 后，运行 `web/sync_robot_model.sh` 将 `src/orion_description` 与可选 `rov_urdf/meshes/stl` 同步到 `web/robot/`（URDF + meshes/stl），供 3D 视图加载。

**前置**：先启动 rosbridge（如 `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`）和 orion_mtc（如 `pick_holoocean.launch.py`）。

**打开**：用浏览器打开 `web/index.html`（或由任意 HTTP 服务器托管 `web/`）。可选 URL 参数：
- `?ws=ws://host:port` — WebSocket 地址，默认 `ws://localhost:9090`
- `?ns=/manipulator` 或 `?topic_prefix=/manipulator` — 话题/服务命名空间，默认 `/manipulator`

**功能对应**：
- **订阅话题**：`runtime_status`、`job_event`、`task_stage`、`held_object_state`、`recovery_event`、`object_pose`、`joint_states`
- **发布话题（急停/回 ready）**：通过 rosbridge `op: publish` 向 `emergency_stop`、`go_to_ready` 发 `std_msgs/Empty`（{}）；顶部栏提供「急停」「回 ready」按钮
- **调用服务**：`get_robot_state`（连接时同步）、`get_queue_state`、`get_recent_jobs`（底部「最近执行」Tab）、`submit_job`、`cancel_job`、`reset_held_object`、`sync_held_object`、`open_gripper`、`close_gripper`、`check_pick`
- **界面**：左侧当前执行/队列/持物/感知/最近错误，中间 3D 视图与视角/显示层，右侧任务操作（抓取、夹爪、审批抓取）与调试工具，底部事件流/最近执行/系统日志

## 如何测试

1. **编译与接口检查**

   ```bash
   cd /path/to/My_orion
   colcon build --symlink-install
   source install/setup.bash
   ros2 interface show orion_mtc_msgs/action/Pick
   ros2 interface show orion_mtc_msgs/msg/TargetSet
   ros2 interface show orion_mtc_msgs/msg/PerceptionState
   ros2 interface show orion_mtc_msgs/srv/GetRobotState
   ros2 interface show orion_mtc_msgs/srv/GetQueueState
   ros2 interface show orion_mtc_msgs/srv/SubmitJob
   ros2 interface show orion_mtc_msgs/srv/CancelJob
   ros2 interface show orion_mtc_msgs/srv/GetRecentJobs
   ros2 interface show std_srvs/srv/Trigger
   ```

2. **启动 HoloOcean 场景与 launch**

   终端 1：`ros2 launch orion_mtc pick_holoocean.launch.py`

3. **终端 2：触发与状态查询**

   - **抓取**（需先有 object_pose 或 target_set）：`ros2 topic pub --once /manipulator/pick_trigger std_msgs/msg/Empty "{}"`；若 3s 内无有效目标则不入队。
   - **打开/闭合夹爪**：`ros2 service call /manipulator/open_gripper std_srvs/srv/Trigger "{}"`、`ros2 service call /manipulator/close_gripper std_srvs/srv/Trigger "{}"`（臂关节保持当前，仅手爪动）。
   - **队列与 Worker**：`ros2 service call /manipulator/get_queue_state orion_mtc_msgs/srv/GetQueueState "{}"`（含 `next_job_id`、`next_job_priority`）。
   - **机器人状态**：`ros2 service call /manipulator/get_robot_state orion_mtc_msgs/srv/GetRobotState "{}"`
   - **最近执行记录**：`ros2 service call /manipulator/get_recent_jobs orion_mtc_msgs/srv/GetRecentJobs "{max_count: 20}"`
   - **取消排队任务**：`ros2 service call /manipulator/cancel_job orion_mtc_msgs/srv/CancelJob "{job_id: 'job_xxx'}"`
   - **即时执行**：Action `/manipulator/pick`

4. **业务规则**：已持物时 pick 被拒绝；**夹爪 locked（有物）时 pick 被拒绝**，需先 reset_held_object 或 open_gripper。队列满或策略 `reject_new_jobs_while_busy` 时 submit_job 返回拒绝原因。抓取使用 object_pose（侧向抓取姿态由 direction 推导，话题触发时执行前取最新位姿）。夹取失败时自动回到 ready 再设 IDLE。

## 许可证

Apache-2.0

# 依赖说明

本文档列出 **Sealien_CtrlPilot_Payload_Orion** 仓库运行与编译所需的系统库、ROS 2 包，以及**不在 Ubuntu apt 源中**、需单独准备的工作空间依赖。

**推荐环境**：Ubuntu 22.04 + ROS 2 **Humble**。其它 ROS 2 发行版（如 Jazzy）需自行替换 apt 包名前缀 `ros-humble-*`，并确认 MoveIt 2 / MTC / Pilz 在该发行版可用。

---

## 一键安装（推荐）

前提：已按官方文档安装对应 ROS 2 发行版，且已配置 `packages.ros.org` apt 源。

```bash
cd /path/to/sealien_ctrlpilot_payload_orion
chmod +x scripts/install_ros_dependencies.sh   # 仅首次
./scripts/install_ros_dependencies.sh
```

- 默认使用 **`rosdep install`**，根据各包的 `package.xml` 解析并安装系统依赖。
- 会**跳过**下列键（需你本地另行提供，见下文）：`holoocean_interfaces`、`sealien_ctrlpilot_msgmanagement`。
- 若 `rosdep` 因网络或键缺失失败，可改用固定列表：

```bash
./scripts/install_ros_dependencies.sh --apt-explicit
```

其它发行版示例：

```bash
ROS_DISTRO=jazzy ./scripts/install_ros_dependencies.sh
```

---

## 按仓库包归纳的 ROS 依赖

下列与 `src/*/package.xml` 一致，便于对照；具体 apt 包名由 `rosdep` 映射（Humble 下多为 `ros-humble-<(kebab-case)>`）。

| 包名 | 用途 | 主要依赖（ROS / 工具） |
|------|------|------------------------|
| **orion_description** | URDF / 网格 | `ament_cmake`、`xacro`、`robot_state_publisher`、`joint_state_publisher_gui` |
| **orion_moveit_config** | MoveIt 配置 | `moveit_ros_move_group`、`moveit_planners_ompl`、`pilz_industrial_motion_planner`、`moveit_ros_visualization`、`rviz2`、`moveit_simple_controller_manager` 等 |
| **orion_mtc_msgs** | 接口消息 / Action | `rosidl_default_generators`、`action_msgs`、`geometry_msgs`、`std_msgs` 等 |
| **orion_mtc** | MTC 抓取节点 | **MoveIt Task Constructor**：`moveit_task_constructor_core`、`moveit_task_constructor_msgs`；MoveIt：`moveit_ros_planning_interface`、`moveit_core`、`moveit_ros_planning`；常用消息与 TF：`control_msgs`、`trajectory_msgs`、`tf2*`、`Eigen3`；运行时：`rosbridge_server`、`rosapi`（Web 联调） |
| **orion_holoocean_bridge** | HoloOcean 桥 | `rclpy`、`holoocean_interfaces`（**外部**）、`control_msgs`、`tf2_ros`、`orion_mtc_msgs` 等 |
| **orion_joy_arm_bridge** | 手柄桥 | `rclpy`、`sensor_msgs`、`control_msgs`、`trajectory_msgs` 等 |

---

## 必须通过其它工作空间提供的包

以下项**不在**本仓库 `rosdep` 默认解析链中（已在安装脚本中 `--skip-keys`），需你方单独编译并 `source` 其 `install/setup.bash`，或与当前仓库放在**同一 colcon 工作空间**一并构建。

| 依赖 | 用途 |
|------|------|
| **holoocean_interfaces** | `orion_holoocean_bridge`：ArmSensor / AgentCommand / CableSensor 等消息。来自 **holoocean-ros** 工作区；可通过环境变量 **`HOLOOCEAN_ROS_INSTALL`** 指向其 install，或先 source 该 overlay。 |
| **sealien_ctrlpilot_msgmanagement** | `orion_mtc` 中 **`keypoint_to_arm_tf_node`**：`/keypoints` 类型与电缆检测等发布端一致。编译本仓库前需已可被 CMake/ament 找到（同一 `colcon` 工作空间或预先 overlay）。 |

---

## 系统级 / 工具（脚本会尝试安装）

- **编译工具**：`build-essential`、`cmake`、`libeigen3-dev`
- **构建工具**：`python3-colcon-common-extensions`
- **依赖解析**：`python3-rosdep`（`rosdep init` / `update` 需网络）

---

## 上位机 Web（`web/`）

静态页面 + Python 标准库 HTTP 服务，无 npm 依赖。需要系统自带 **Python 3**。

联调 ROS 时需本机或局域网内可访问 **`rosbridge_server`**（已在 `orion_mtc` 的 `exec_depend` 中列出，可由安装脚本或 `rosdep` 安装）。默认 WebSocket 端口为 **9091**（`rosbridge_websocket_keepalive` / `pick_holoocean` 参数 `rosbridge_port`）；页面从本机打开时 Web 默认 `ws://127.0.0.1:9091`，从局域网 IP 打开时默认与页面同主机 `ws://<该IP>:9091`。**rosbridge 在其它机器**或端口不一致时用 `?ws=`；与同机 9090 并存时可统一改 `rosbridge_port`。

---

## 安装后自检建议

```bash
source /opt/ros/humble/setup.bash   # 或你的发行版
# 若已编译本仓库：
# source /path/to/colcon_ws/install/setup.bash

ros2 pkg list | grep -E 'moveit_task_constructor_core|pilz_industrial_motion_planner'
```

若缺少 MTC 核心包，CMake 会在配置 `orion_mtc` 时报告找不到 `moveit_task_constructor_core` 的 `*Config.cmake`。

---

## 与 README 的关系

仓库根目录 [README.md](../README.md) 中「依赖」小节为简要列表；**完整列表与安装步骤以本文档与 `scripts/install_ros_dependencies.sh` 为准**。

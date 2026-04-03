# arm_bag_tools：臂控制链录制与分析

本目录用于在 HoloOcean + MTC 联调时，**录制**关节相关 ROS2 话题，并**一键生成**对比曲线与 CSV，便于排查指令/反馈是否饱和、抖动或与限位不一致。

**说明**：仓库内 `tools/` 下除本目录外，另有根级脚本如 `compute_max_reach_from_urdf.py`（由 URDF 推算臂展供 feasibility 配置）；**带子目录的工具仅 `arm_bag_tools/`**，其子文件夹含义见下表。

## 目录说明

| 路径 | 用途 |
|------|------|
| **`bags/`** | **默认 rosbag 输出目录**。`record_arm_chain.sh` 无参数时，在此下创建 `sim_arm_debug_时间戳/` 等子目录；每个子目录内为 `ros2 bag record` 的标准结构（如 `*.db3`、`metadata.yaml`）。仅保留空目录占位文件 `.gitkeep`；实际 bag **默认不被 git 跟踪**（见仓库根目录 `.gitignore`）。 |
| **`analysis/`** | **分析结果输出目录**。`analyze_arm_bag.sh` 在选中某个 bag 后，在其下创建与 bag **同名**的子目录，内含 `arm_comparison.png`、`summary_stats.csv`、`aligned_series.csv`。同样用 `.gitkeep` 占位，生成内容默认被 `.gitignore`。 |

其余文件均在本目录**根下**（不再放入子文件夹）：

| 文件 | 用途 |
|------|------|
| `record_arm_chain.sh` | 启动录制：订阅 `joint_states`、`ArmSensor`、`AgentCommand` 及两臂 `FollowJointTrajectory` 的 `_action` 话题（含 hidden）。输出见 `bags/`。 |
| `analyze_arm_bag.sh` | 一键调用 `plot_arm_bag.py`：默认选 `bags/` 下**最近修改**的 bag，结果写入 `analysis/`。需已 `source` ROS2 与工作空间。 |
| `plot_arm_bag.py` | 读 bag 目录，画 cmd / sensor / joint_states 对比图；导出 `summary_stats.csv`（原始 cmd−sensor）、**`summary_stats_wrapped.csv`**（J1–J6 最短弧折叠误差）、**`cmd_sensor_error.csv`**、**`anomaly_report.txt`**（各轴首次超阈值时刻，含 J4 摘要）、`aligned_series.csv`；参数 **`--spike-deg`** 控制异常阈值。 |
| `clean_arm_bags.sh` | 清空 **`bags/*`** 与 **`analysis/*`**，保留两目录下的 `.gitkeep`。 |
| `requirements.txt` | 分析脚本依赖：`numpy`、`matplotlib`（与 ROS 环境分离安装时可用 `pip install -r requirements.txt`）。 |

## 典型流程

1. 终端内：`source /opt/ros/$ROS_DISTRO/setup.bash` 与 `install/setup.bash`（需含 `holoocean_interfaces`）。
2. `cd` 到本目录，执行 `./record_arm_chain.sh`，结束后在 `bags/` 下得到新子目录。
3. 执行 `./analyze_arm_bag.sh`（或 `./analyze_arm_bag.sh <bag 子目录名>`），在 `analysis/<同名>/` 查看 PNG 与 CSV。
4. 不需要旧数据时：`./clean_arm_bags.sh` 或 `./clean_arm_bags.sh -y`。
5. 调整「首次异常」阈值：`ARM_BAG_SPIKE_DEG=45 ./analyze_arm_bag.sh`（默认 60°）。

## 与环境变量的关系

- **`ARM_BAG_OPEN=1`**：与 `./analyze_arm_bag.sh` 同用时，若存在图形界面，会尝试用系统默认程序打开生成的 PNG。

## 与 git

- `bags/`、`analysis/` 内**除 `.gitkeep 外**的内容默认忽略，避免大文件入库。
- 若需提交某次 bag 作样例，须自行改 `.gitignore` 或使用仓库外的备份路径录制。

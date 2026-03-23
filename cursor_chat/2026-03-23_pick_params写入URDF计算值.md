# 2026-03-23 pick_params 写入 URDF 计算后的 max_reach

## 用户意图
不要仅靠运行时 `0` 自动算，而是根据 URDF/模型把**具体米数**写进配置。

## 做法
- 新增 `tools/compute_max_reach_from_urdf.py`：从 URDF 沿 `gripper_tcp`→`base_link` 父链累加各关节 `origin xyz` 的范数（与 `reach_kinematics.cpp` 思路一致），再乘 `margin`、乘 `0.92` 得 soft。
- `pick_params.yaml` 中 `feasibility.max_reach_hard` / `max_reach_soft` 已写为 **2.039** / **1.876**（当前 `orion.urdf`），并注明 URDF 变更后重跑脚本。

## 重算命令
`python3 tools/compute_max_reach_from_urdf.py --urdf src/orion_description/urdf/orion.urdf`

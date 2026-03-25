# 2026-03-25 左手柄 TGL/SW 键位表对齐 YAML

## 硬件表（用户）

- TGL1..4：11/12、13/14、15/16、17/18 → 关节 1..4  
- SW1-2：5/6 → 关节 5；SW3-4：7/8 → 关节 6  
- SW5-6：9/10 → 手动夹爪开/合  

## 约定

每对「先 minus / 后 plus」：`arm_button_minus_indices`、`arm_button_plus_indices`。  
夹爪：`gripper_open=9`，`gripper_close=10`。  

## 修改

- `joy_manipulator.yaml`：臂 minus 由 `[13,15,17,19,7,9]` 改为 `[11,13,15,17,5,7]`；夹爪 10/11 → 9/10。  
- `joy_manipulator_node.py`：`declare_parameter` 默认值与 YAML 一致。

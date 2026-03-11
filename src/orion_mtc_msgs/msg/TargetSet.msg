# 多目标集合：同一帧下多个目标的位置与轴向（如 base_link）
# 用于“目标集合选择 + 单目标抓取”流程
std_msgs/Header header
int32 num_targets
float64[] positions   # 长度 3*num_targets：依次 [x0,y0,z0, x1,y1,z1, ...]，单位 m
float64[] directions  # 长度 3*num_targets：圆柱轴方向 [dx0,dy0,dz0, ...]，建议源端已归一化

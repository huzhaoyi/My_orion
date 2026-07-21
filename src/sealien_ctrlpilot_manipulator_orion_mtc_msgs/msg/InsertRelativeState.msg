# 插孔段插头相对目标孔位姿（孔系下 T_hole_peg 及几何误差）
# 用于 world model: s_t 来自本消息；s_{t+1} 为下一帧；a_t 对齐 AgentCommand
std_msgs/Header header

bool active
string task_stage

# 几何误差（与 insert_task_builder 中 e_lat / 轴向 / 杆轴对齐语义一致）
float64 lateral_error_m
float64 axial_error_m
float64 angle_error_rad

# peg 在孔系下的位姿（relative_pose.header.frame_id = hole_frame_id）
geometry_msgs/Pose relative_pose
string hole_frame_id

int32 hole_index
float32 latch

std_msgs/Header header

bool has_target  # 是否检测到目标,如果为false,则后面的数据都不可用

geometry_msgs/Point[2] corner_points # 目标检测框左上角和右下角点

geometry_msgs/Point[] keypoints #数组类型，存放检测到的关键点信息

float32 yaw_degree  # 平面内朝向（度），与 Orion MTC Keypoints 对齐；未使用时可为 0

float32 score  # 目标框置信度

float32 score_threshold # 当前的目标框置信度阈值
bool use_score_threshold  # 是否启用阈值过滤
bool is_available  # 是否可用
string task_type  # 任务类型，keypoint_detect： 关键点检测 object_detect: 目标检测

# 以下数据输出电缆在声纳坐标系下的姿态，用于机械臂抓取电缆
float32[3] directions  #电缆方向向量，用于机械臂抓取, 顺序：x,y,z
float32[3] euler_angles_with_coordinate #电缆和三个坐标轴的夹角，顺序：x,y,z
float32[3] euler_angles #电缆自身的旋转角roll(默认为0)，电缆和xoy平面夹角pitch， 电缆xoy平面内投影和y轴正方向夹角yaw


# 感知状态：缆绳与 TargetSensor 分栏、ROV 位姿，供网页/客户端单话题订阅
# object_pose：与 /object_pose 抓取话题一致（当前选中源）；cable/target_sensor 分源展示
std_msgs/Header header
geometry_msgs/PoseStamped object_pose
geometry_msgs/PoseStamped cable_object_pose
geometry_msgs/PoseStamped target_sensor_object_pose
geometry_msgs/Vector3 object_axis_direction
float32 object_confidence
geometry_msgs/PoseStamped rov_pose_in_arm_base_link
geometry_msgs/PoseStamped rov_pose_in_world
# TargetSensor 桥当前用于 /object_pose 的候选下标（0 起）；缆绳等非 Target 场景填 -1
int32 target_sensor_selected_index

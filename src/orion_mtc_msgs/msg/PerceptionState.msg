# 感知状态：单缆绳位姿、ROV 位姿，供网页/客户端单话题订阅（无多目标）
std_msgs/Header header
geometry_msgs/PoseStamped object_pose
geometry_msgs/Vector3 object_axis_direction
float32 object_confidence
geometry_msgs/PoseStamped rov_pose_in_base_link
geometry_msgs/PoseStamped rov_pose_in_world

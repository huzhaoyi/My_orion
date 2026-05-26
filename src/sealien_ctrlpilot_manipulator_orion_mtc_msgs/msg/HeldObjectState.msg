# 手上拿着什么：持物状态单独话题
std_msgs/Header header

bool valid
bool tracked

string object_id
string scene_attach_id
string attach_link

geometry_msgs/Pose object_pose_at_grasp
geometry_msgs/Pose tcp_pose_at_grasp

float32 weight

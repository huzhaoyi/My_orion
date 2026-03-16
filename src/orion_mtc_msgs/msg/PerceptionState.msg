# 感知状态：物体位姿、ROV 世界系/基座系位姿、多目标集合（世界系+机械臂系），供网页/客户端单话题订阅
std_msgs/Header header
geometry_msgs/PoseStamped object_pose
geometry_msgs/PoseStamped rov_pose_in_base_link
geometry_msgs/PoseStamped rov_pose_in_world
orion_mtc_msgs/TargetSet target_set
orion_mtc_msgs/TargetSet target_set_world

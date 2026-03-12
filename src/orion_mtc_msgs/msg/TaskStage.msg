# 执行阶段进度，偏调试：当前跑到哪一步
# MTC 阶段：move_to_ready, open_hand, move_to_pregrasp, approach_object, close_hand, attach_object, lift_object, move_to_preplace, lower_to_place, detach_object, retreat, return_home 等
std_msgs/Header header

string job_id
string task_type      # PICK / PLACE / PLACE_RELEASE / OPEN_GRIPPER / CLOSE_GRIPPER
string stage_name
string stage_state   # ENTER / RUNNING / DONE / FAILED / SKIPPED

string detail

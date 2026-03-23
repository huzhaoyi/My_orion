# 执行阶段进度，偏调试：当前跑到哪一步
# MTC 阶段：move_to_ready, open_hand, move_to_pregrasp, approach_object, close_hand, attach_object, retreat 等（随任务构图变化）
std_msgs/Header header

string job_id
string task_type      # PICK / OPEN_GRIPPER / CLOSE_GRIPPER / …
string stage_name
string stage_state   # ENTER / RUNNING / DONE / FAILED / SKIPPED

string detail

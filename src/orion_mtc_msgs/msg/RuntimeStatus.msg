# 机器人运行状态总看板：给上位机 / rviz / 日志看“现在整体在干嘛”
# 变化频率不高，最核心
std_msgs/Header header

string worker_status   # STOPPED / IDLE / RUNNING_JOB / RECOVERING / ERROR
string task_mode       # IDLE / PICKING / HOLDING_TRACKED / HOLDING_UNTRACKED / ERROR

string current_job_id
string current_job_type
string next_job_type

bool worker_running
bool queue_empty
uint32 queue_size

bool has_held_object
string held_object_id
string held_scene_attach_id

string last_error

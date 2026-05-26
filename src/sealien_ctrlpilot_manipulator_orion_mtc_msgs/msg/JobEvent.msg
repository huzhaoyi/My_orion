# 任务级状态：某 job 生命周期事件，供调度 / 任务监控
# 一个 job 从提交到结束：SUBMITTED / STARTED / SUCCEEDED / FAILED / REJECTED / CANCELLED
std_msgs/Header header

string job_id
string job_type
string source

uint32 priority

string event_type    # SUBMITTED / STARTED / SUCCEEDED / FAILED / REJECTED / CANCELLED
bool success
string reason

builtin_interfaces/Time created_at
builtin_interfaces/Time started_at
builtin_interfaces/Time finished_at

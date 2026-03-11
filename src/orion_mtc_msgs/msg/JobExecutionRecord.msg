# 单条 job 执行记录，用于 GetRecentJobs 可回看
string job_id
string job_type
string source
uint8 result_code
string message
int64 created_at_ns
int64 started_at_ns
int64 finished_at_ns

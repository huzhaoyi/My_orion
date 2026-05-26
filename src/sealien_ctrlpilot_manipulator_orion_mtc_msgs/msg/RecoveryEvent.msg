# 恢复/异常事件：恢复逻辑单独报，比塞进 last_error 更清楚
std_msgs/Header header

string recovery_type   # CLEAR_SCENE / RESET_HELD / GO_HOME / AUTO_RECOVERY
string trigger_reason
bool success
string detail

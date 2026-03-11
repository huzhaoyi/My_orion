/* 执行器/Worker 运行态，与业务态 RobotTaskMode 分离，便于长期运行可观测 */

#ifndef ORION_MTC_CORE_RUNTIME_STATUS_HPP
#define ORION_MTC_CORE_RUNTIME_STATUS_HPP

namespace orion_mtc
{

/** 后台 worker 状态：业务态是 IDLE 时 worker 可能未启动或正在恢复 */
enum class WorkerStatus
{
  STOPPED,
  IDLE,
  RUNNING_JOB,
  RECOVERING,
  ERROR
};

inline const char* toCString(WorkerStatus s)
{
  switch (s)
  {
    case WorkerStatus::STOPPED:
      return "STOPPED";
    case WorkerStatus::IDLE:
      return "IDLE";
    case WorkerStatus::RUNNING_JOB:
      return "RUNNING_JOB";
    case WorkerStatus::RECOVERING:
      return "RECOVERING";
    case WorkerStatus::ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

}  // namespace orion_mtc

#endif  // ORION_MTC_CORE_RUNTIME_STATUS_HPP

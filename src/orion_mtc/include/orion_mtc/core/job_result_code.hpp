/* Job 执行结果码：供内部日志与执行记录使用 */

#ifndef ORION_MTC_CORE_JOB_RESULT_CODE_HPP
#define ORION_MTC_CORE_JOB_RESULT_CODE_HPP

namespace orion_mtc
{

enum class JobResultCode : uint8_t
{
  SUCCESS = 0,
  REJECTED = 1,
  PLAN_FAILED = 2,
  EXEC_FAILED = 3,
  RECOVERY_FAILED = 4,
  NO_POSE = 5,
  POLICY_REJECTED = 6,
  CANCELLED = 7,
  UNKNOWN = 255
};

inline const char* jobResultCodeToCString(JobResultCode c)
{
  switch (c)
  {
    case JobResultCode::SUCCESS:
      return "SUCCESS";
    case JobResultCode::REJECTED:
      return "REJECTED";
    case JobResultCode::PLAN_FAILED:
      return "PLAN_FAILED";
    case JobResultCode::EXEC_FAILED:
      return "EXEC_FAILED";
    case JobResultCode::RECOVERY_FAILED:
      return "RECOVERY_FAILED";
    case JobResultCode::NO_POSE:
      return "NO_POSE";
    case JobResultCode::POLICY_REJECTED:
      return "POLICY_REJECTED";
    case JobResultCode::CANCELLED:
      return "CANCELLED";
    default:
      return "UNKNOWN";
  }
}

}  // namespace orion_mtc

#endif  // ORION_MTC_CORE_JOB_RESULT_CODE_HPP

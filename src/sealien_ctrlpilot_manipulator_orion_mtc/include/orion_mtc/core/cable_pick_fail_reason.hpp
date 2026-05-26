/* 缆绳侧抓候选失败原因：决策/预检与编排层共用，便于统计与诊断 */

#ifndef ORION_MTC_CORE_CABLE_PICK_FAIL_REASON_HPP
#define ORION_MTC_CORE_CABLE_PICK_FAIL_REASON_HPP

namespace orion_mtc
{

enum class CablePickFailReason
{
    NO_IK,
    OUT_OF_BOUNDS,
    PREGRASP_IN_COLLISION,
    INIT_FAILED,
    PLAN_FAILED,
    EXECUTION_FAILED
};

inline const char* cablePickFailReasonTag(CablePickFailReason r)
{
    switch (r)
    {
    case CablePickFailReason::NO_IK:
        return "NO_IK";
    case CablePickFailReason::OUT_OF_BOUNDS:
        return "OUT_OF_BOUNDS";
    case CablePickFailReason::PREGRASP_IN_COLLISION:
        return "PREGRASP_IN_COLLISION";
    case CablePickFailReason::INIT_FAILED:
        return "INIT_FAILED";
    case CablePickFailReason::PLAN_FAILED:
        return "PLAN_FAILED";
    case CablePickFailReason::EXECUTION_FAILED:
        return "EXECUTION_FAILED";
    default:
        return "UNKNOWN";
    }
}

}  // namespace orion_mtc

#endif  // ORION_MTC_CORE_CABLE_PICK_FAIL_REASON_HPP

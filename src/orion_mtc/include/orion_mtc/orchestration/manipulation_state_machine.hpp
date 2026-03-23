/* 操纵业务态转移规则：编排层统一查询，避免散落 if（仅抓取与持物相关） */

#ifndef ORION_MTC_ORCHESTRATION_MANIPULATION_STATE_MACHINE_HPP
#define ORION_MTC_ORCHESTRATION_MANIPULATION_STATE_MACHINE_HPP

#include "orion_mtc/core/task_state.hpp"

namespace orion_mtc
{

class ManipulationStateMachine
{
public:
    bool canAcceptPick(RobotTaskMode mode) const
    {
        return orion_mtc::canAcceptPick(mode);
    }

    RobotTaskMode onPickStarted(RobotTaskMode current) const
    {
        if (orion_mtc::canAcceptPick(current))
        {
            return RobotTaskMode::PICKING;
        }
        return current;
    }

    RobotTaskMode onPickSucceeded(bool tracked) const
    {
        return tracked ? RobotTaskMode::HOLDING_TRACKED : RobotTaskMode::HOLDING_UNTRACKED;
    }

    RobotTaskMode onPickFailed(RobotTaskMode previous) const
    {
        (void)previous;
        return RobotTaskMode::ERROR;
    }
};

}  // namespace orion_mtc

#endif  // ORION_MTC_ORCHESTRATION_MANIPULATION_STATE_MACHINE_HPP

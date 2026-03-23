#include "orion_mtc/decision/cable_side_pick_precheck.hpp"
#include "orion_mtc/core/constants.hpp"
#include "orion_mtc/planning/collision_object_utils.hpp"
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_state/robot_state.h>

namespace orion_mtc
{

bool precheckCableSideGraspCandidate(
    const rclcpp::Logger& logger,
    std::size_t candidate_index,
    const CableGraspCandidate& cand,
    const moveit::core::RobotModelConstPtr& robot_model,
    planning_scene::PlanningScenePtr scene_for_ik_seed,
    bool has_scene_base_msg,
    const moveit_msgs::msg::PlanningScene& scene_base_msg,
    const std::vector<CableSegment>& segments,
    const std::string& plan_frame,
    const std::string& arm_group_name,
    const std::string& hand_frame,
    CablePickFailReason* out_reason)
{
    if (out_reason != nullptr)
    {
        *out_reason = CablePickFailReason::NO_IK;
    }
    if (!robot_model)
    {
        return true;
    }
    const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(arm_group_name);
    moveit::core::RobotState state(robot_model);
    if (scene_for_ik_seed)
    {
        state = scene_for_ik_seed->getCurrentState();
    }
    else
    {
        state.setToDefaultValues();
    }

    bool grasp_ik_ok = false;
    if (jmg != nullptr)
    {
        grasp_ik_ok = state.setFromIK(jmg, cand.grasp_pose, hand_frame, 0.15);
    }
    RCLCPP_INFO(logger, "handlePick: candidate %zu grasp IK %s", candidate_index,
                grasp_ik_ok ? "success" : "fail");
    if (!grasp_ik_ok)
    {
        if (out_reason != nullptr)
        {
            *out_reason = CablePickFailReason::NO_IK;
        }
        return false;
    }

    bool pregrasp_ik_ok = false;
    if (scene_for_ik_seed)
    {
        state = scene_for_ik_seed->getCurrentState();
    }
    else
    {
        state.setToDefaultValues();
    }
    if (jmg != nullptr)
    {
        pregrasp_ik_ok = state.setFromIK(jmg, cand.pregrasp_pose, hand_frame, 0.15);
    }
    RCLCPP_INFO(logger, "handlePick: candidate %zu pregrasp IK %s", candidate_index,
                pregrasp_ik_ok ? "success" : "fail");
    if (!pregrasp_ik_ok)
    {
        if (out_reason != nullptr)
        {
            *out_reason = CablePickFailReason::NO_IK;
        }
        return false;
    }

    state.update();
    if (jmg != nullptr && !state.satisfiesBounds(jmg))
    {
        RCLCPP_WARN(logger, "handlePick: candidate %zu OUT_OF_BOUNDS", candidate_index);
        if (out_reason != nullptr)
        {
            *out_reason = CablePickFailReason::OUT_OF_BOUNDS;
        }
        return false;
    }
    if (has_scene_base_msg)
    {
        planning_scene::PlanningScenePtr scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
        scene->setPlanningSceneMsg(scene_base_msg);
        for (const auto& seg : segments)
        {
            moveit_msgs::msg::CollisionObject obj =
                makeSegmentCollisionObject(seg, plan_frame, moveit_msgs::msg::CollisionObject::ADD);
            scene->processCollisionObjectMsg(obj);
        }
        collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();
        for (int idx : cand.local_segment_indices)
        {
            if (idx >= 0 && idx < static_cast<int>(segments.size()))
            {
                for (const auto& link : CABLE_LOCAL_PREGRASP_ALLOWED_LINKS)
                {
                    acm.setEntry(segments[static_cast<std::size_t>(idx)].id, link, true);
                }
            }
        }
        collision_detection::CollisionRequest req;
        req.contacts = true;
        req.max_contacts = 1;
        collision_detection::CollisionResult res;
        scene->checkCollision(req, res, state, acm);
        RCLCPP_INFO(logger, "handlePick: candidate %zu collision %s", candidate_index,
                    res.collision ? "yes" : "no");
        if (res.collision)
        {
            if (out_reason != nullptr)
            {
                *out_reason = CablePickFailReason::PREGRASP_IN_COLLISION;
            }
            return false;
        }
    }
    return true;
}

}  // namespace orion_mtc

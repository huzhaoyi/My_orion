/* 缆绳侧抓：MTC 规划前的轻量预检（IK / 关节界 / 带缆段场景的 pregrasp 碰撞） */

#ifndef ORION_MTC_DECISION_CABLE_SIDE_PICK_PRECHECK_HPP
#define ORION_MTC_DECISION_CABLE_SIDE_PICK_PRECHECK_HPP

#include "orion_mtc/core/cable_pick_fail_reason.hpp"
#include "orion_mtc/planning/cable_segments.hpp"
#include "orion_mtc/planning/cable_side_grasp.hpp"
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/logger.hpp>
#include <cstddef>
#include <vector>

namespace orion_mtc
{

/**
 * 对单个 CableGraspCandidate 做 pregrasp IK、关节界、（若 has_scene）带缆段局部 ACM 的碰撞预检。
 * @return true 表示可进入 MTC plan；false 时 out_reason 有效。
 */
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
    CablePickFailReason* out_reason);

}  // namespace orion_mtc

#endif  // ORION_MTC_DECISION_CABLE_SIDE_PICK_PRECHECK_HPP

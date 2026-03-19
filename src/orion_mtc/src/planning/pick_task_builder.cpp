#include "orion_mtc/planning/pick_task_builder.hpp"
#include "orion_mtc/planning/collision_object_utils.hpp"
#include "orion_mtc/planning/cable_side_grasp.hpp"
#include "orion_mtc/core/constants.hpp"
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

namespace mtc = moveit::task_constructor;

namespace orion_mtc
{

PickTaskBuilder::PickTaskBuilder(const rclcpp::Node::SharedPtr& node, const MTCConfig& config)
  : node_(node), config_(config)
{
}

mtc::Task PickTaskBuilder::buildFromCableCandidate(
    const std::vector<CableSegment>& segments,
    const CableGraspCandidate& candidate,
    const std::string& plan_frame)
{
  mtc::Task task;
  task.stages()->setName("orion pick (cable side, segmented)");
  task.loadRobotModel(node_);
  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "gripper_tcp";
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.stages()->properties().set("group", arm_group_name);
  task.stages()->properties().set("eef", hand_group_name);
  task.stages()->properties().set("ik_frame", hand_frame);

  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  auto ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  ptp_planner->setPlannerId("PTP");
  auto lin_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  lin_planner->setPlannerId("LIN");
  auto ompl_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "move_group");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  auto stage_ready = std::make_unique<mtc::stages::MoveTo>("move to ready", ptp_planner);
  stage_ready->setGroup(arm_group_name);
  stage_ready->setGoal("ready");
  task.add(std::move(stage_ready));

  {
    auto stage_add = std::make_unique<mtc::stages::ModifyPlanningScene>("add_cable_segments");
    for (const auto& seg : segments)
    {
      moveit_msgs::msg::CollisionObject obj =
          makeSegmentCollisionObject(seg, plan_frame, moveit_msgs::msg::CollisionObject::ADD);
      obj.header.stamp = node_->now();
      stage_add->addObject(obj);
    }
    task.add(std::move(stage_add));
  }

  auto stage_open = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open->setGroup(hand_group_name);
  stage_open->setGoal("open");
  task.add(std::move(stage_open));

  {
    auto stage_acm = std::make_unique<mtc::stages::ModifyPlanningScene>("allow self-collision (pregrasp)");
    stage_acm->allowCollisions("Link1", std::vector<std::string>{ "Link6", "Link7", "Link8" }, true);
    stage_acm->allowCollisions("Link2", std::vector<std::string>{ "Link8" }, true);
    stage_acm->allowCollisions("Link7", std::vector<std::string>{ "base_link" }, true);
    stage_acm->allowCollisions("Link7", std::vector<std::string>{ "Link2" }, true);
    stage_acm->allowCollisions("Link8", std::vector<std::string>{ "base_link" }, true);
    task.add(std::move(stage_acm));
  }

  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

  /* 仅对抓取局部段放宽末端碰撞（Link1/Link2 不放开） */
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (cable local) for pregrasp");
    for (int idx : candidate.local_segment_indices)
    {
      if (idx >= 0 && idx < static_cast<int>(segments.size()))
      {
        stage->allowCollisions(segments[idx].id, CABLE_LOCAL_PREGRASP_ALLOWED_LINKS, true);
      }
    }
    grasp->insert(std::move(stage));
  }

  const rclcpp::Time now = node_->now();
  geometry_msgs::msg::PoseStamped pregrasp_ps = toPoseStamped(candidate.pregrasp_pose, plan_frame, now);
  auto stage_pregrasp = std::make_unique<mtc::stages::MoveTo>("move to pregrasp", ompl_planner);
  stage_pregrasp->setGroup(arm_group_name);
  stage_pregrasp->setGoal(pregrasp_ps);
  stage_pregrasp->setIKFrame(hand_frame);
  grasp->insert(std::move(stage_pregrasp));

  /* approach 阶段继续使用局部段 ACM（同上） */
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (cable local) for approach");
    for (int idx : candidate.local_segment_indices)
    {
      if (idx >= 0 && idx < static_cast<int>(segments.size()))
      {
        stage->allowCollisions(segments[idx].id, CABLE_LOCAL_APPROACH_ALLOWED_LINKS, true);
      }
    }
    grasp->insert(std::move(stage));
  }

  geometry_msgs::msg::Vector3Stamped approach_v = toVector3Stamped(candidate.approach_dir, plan_frame, now);
  auto stage_approach = std::make_unique<mtc::stages::MoveRelative>("approach to grasp (LIN)", lin_planner);
  stage_approach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_approach->setMinMaxDistance(static_cast<float>(candidate.approach_dist),
                                    static_cast<float>(candidate.approach_dist));
  stage_approach->setIKFrame(hand_frame);
  stage_approach->setDirection(approach_v);
  grasp->insert(std::move(stage_approach));

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("close");
    grasp->insert(std::move(stage));
  }

  /* 简化版：删除全部缆绳段并标记抓取，不 attach 几何体；executor 根据本阶段设置 held 状态 */
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("remove_cable_segments");
    for (const auto& seg : segments)
    {
      stage->removeObject(seg.id);
    }
    grasp->insert(std::move(stage));
  }

  geometry_msgs::msg::Vector3Stamped retreat_v = toVector3Stamped(candidate.retreat_dir, plan_frame, now);
  auto stage_retreat = std::make_unique<mtc::stages::MoveRelative>("retreat short", cartesian_planner);
  stage_retreat->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_retreat->setMinMaxDistance(static_cast<float>(candidate.retreat_dist),
                                  static_cast<float>(candidate.retreat_dist));
  stage_retreat->setIKFrame(hand_frame);
  stage_retreat->setDirection(retreat_v);
  grasp->insert(std::move(stage_retreat));

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(static_cast<float>(config_.lift_object_min_dist),
                             static_cast<float>(config_.lift_object_max_dist));
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "lift_object");
    geometry_msgs::msg::Vector3Stamped v;
    v.header.frame_id = plan_frame;
    v.header.stamp = now;
    v.vector.x = 0.0;
    v.vector.y = 0.0;
    v.vector.z = 1.0;
    stage->setDirection(v);
    grasp->insert(std::move(stage));
  }

  task.add(std::move(grasp));
  return task;
}

}  // namespace orion_mtc

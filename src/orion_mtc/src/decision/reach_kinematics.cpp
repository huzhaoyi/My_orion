#include "orion_mtc/decision/reach_kinematics.hpp"
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/parameter.hpp>
#include <algorithm>
#include <cmath>

namespace orion_mtc
{

namespace
{
constexpr double k_param_eps = 1e-9;
constexpr double k_fallback_hard_m = 2.0;
constexpr double k_default_kinematic_margin = 1.08;
}  // namespace

double computeKinematicReachUpperBoundM(const moveit::core::RobotModel& model,
                                        const std::string& base_link,
                                        const std::string& tip_link)
{
  const moveit::core::LinkModel* tip = model.getLinkModel(tip_link);
  if (!tip)
  {
    return -1.0;
  }
  double sum = 0.0;
  const moveit::core::LinkModel* cur = tip;
  while (cur && cur->getName() != base_link)
  {
    sum += cur->getJointOriginTransform().translation().norm();
    cur = cur->getParentLinkModel();
  }
  if (!cur || cur->getName() != base_link)
  {
    return -1.0;
  }
  return sum;
}

ResolvedReachLimits resolveFeasibilityReachLimits(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<const moveit::core::RobotModel>& robot_model,
    const std::string& base_link,
    const std::string& tip_link)
{
  ResolvedReachLimits out;
  double hard_param = 0.0;
  double soft_param = 0.0;
  double margin = k_default_kinematic_margin;
  rclcpp::Parameter p;
  if (node)
  {
    if (node->get_parameter("feasibility.max_reach_hard", p))
    {
      hard_param = p.as_double();
    }
    if (node->get_parameter("feasibility.max_reach_soft", p))
    {
      soft_param = p.as_double();
    }
    if (node->get_parameter("feasibility.max_reach_hard_kinematic_margin", p))
    {
      margin = p.as_double();
    }
  }
  margin = std::max(margin, 1.0);

  if (hard_param > k_param_eps)
  {
    out.hard_m = hard_param;
    out.manual_hard = true;
  }
  else
  {
    if (robot_model)
    {
      out.kinematic_ub_m = computeKinematicReachUpperBoundM(*robot_model, base_link, tip_link);
      if (out.kinematic_ub_m >= 0.0)
      {
        out.hard_m = out.kinematic_ub_m * margin;
        out.used_kinematic = true;
      }
      else
      {
        out.hard_m = k_fallback_hard_m;
      }
    }
    else
    {
      out.hard_m = k_fallback_hard_m;
    }
  }

  if (soft_param > k_param_eps)
  {
    out.soft_m = soft_param;
  }
  else
  {
    out.soft_m = out.hard_m * KINEMATIC_SOFT_REACH_RATIO;
  }
  return out;
}

}  // namespace orion_mtc

/* 持物上下文：抓取成功后保存，place 时用 tcp_to_object 反推末端目标 */

#ifndef ORION_MTC_CORE_HELD_OBJECT_HPP
#define ORION_MTC_CORE_HELD_OBJECT_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include <string>

namespace orion_mtc
{

/* 几何由 planning 层 makeTargetCollisionObject 统一描述 */
struct HeldObjectContext
{
  bool valid = false;
  std::string object_id;
  std::string attach_link;
  /* "object" 来自 pick，"held_tracked" 来自 sync(tracked)，place/release 用此 id 做 allow/detach */
  std::string scene_attach_id;
  geometry_msgs::msg::Pose object_pose_at_grasp;
  geometry_msgs::msg::Pose tcp_pose_at_grasp;
  Eigen::Isometry3d tcp_to_object = Eigen::Isometry3d::Identity();
  double weight = 0.0;
};

inline void clearHeldObject(HeldObjectContext& ctx)
{
  ctx.valid = false;
  ctx.object_id.clear();
  ctx.attach_link.clear();
  ctx.scene_attach_id.clear();
}

inline bool isTracked(const HeldObjectContext& ctx)
{
  return ctx.valid && !ctx.scene_attach_id.empty() &&
         (ctx.scene_attach_id == "object" || ctx.scene_attach_id == "held_tracked");
}

inline bool isUntracked(const HeldObjectContext& ctx)
{
  return ctx.valid && ctx.scene_attach_id == "held_unknown";
}

}  // namespace orion_mtc

#endif  // ORION_MTC_CORE_HELD_OBJECT_HPP

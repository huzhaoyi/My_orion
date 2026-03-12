/* Planning Scene 管理层：world/attached 增删改、场景清理，不持有业务状态 */

#ifndef ORION_MTC_SCENE_PLANNING_SCENE_MANAGER_HPP
#define ORION_MTC_SCENE_PLANNING_SCENE_MANAGER_HPP

#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <rclcpp/client.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace rclcpp
{
class Node;
}

namespace orion_mtc
{

class PlanningSceneManager
{
public:
  explicit PlanningSceneManager(rclcpp::Node* node);
  ~PlanningSceneManager() = default;

  /* 放置成功后把 planning scene 中物体更新到目标位姿，供 move to ready 及后续规划使用 */
  bool applyObjectPoseToPlanningScene(double px, double py, double pz,
                                      double qx, double qy, double qz, double qw);

  /* HOLDING_UNTRACKED 时往 scene 里 attach 保守包络体，使规划知道手上占空间 */
  bool applyAttachedHeldUnknownToScene();

  /* 从 planning scene 移除已 attach 的物体（held_unknown / object），尽力清理 */
  bool clearAttachedObjectFromPlanningScene(const std::string& object_id);

  /* 从 world 中移除指定 id 的碰撞体（抓取 attach 后兜底清理，避免 scene 双份物体） */
  bool removeWorldObject(const std::string& object_id);

  /* sync(tracked=true) 时把 target 几何 attach 到 Link6，id=held_tracked */
  bool applyAttachedTrackedObjectToScene(const Eigen::Isometry3d& tcp_to_object);

  /* 应用 scene diff（用于执行 sub_trajectory 时应用 attach/detach 等） */
  bool applySceneDiff(const moveit_msgs::msg::PlanningScene& scene_diff);

private:
  rclcpp::Node* node_;
  std::shared_ptr<rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>> apply_planning_scene_client_;

  bool ensureClient();
};

}  // namespace orion_mtc

#endif  // ORION_MTC_SCENE_PLANNING_SCENE_MANAGER_HPP

/* 审批模式：抓取可达性与风险诊断，独立于执行接口 */

#ifndef ORION_MTC_DECISION_FEASIBILITY_CHECKER_HPP
#define ORION_MTC_DECISION_FEASIBILITY_CHECKER_HPP

#include <orion_mtc_msgs/msg/diagnostic_item.hpp>
#include <orion_mtc_msgs/srv/check_pick.hpp>
#include <rclcpp/node.hpp>
#include <memory>
#include <string>
#include <vector>

namespace orion_mtc
{

struct MTCConfig;

class FeasibilityChecker
{
public:
  /** node 需能提供 robot_description 参数（与 move_group 同源） */
  explicit FeasibilityChecker(rclcpp::Node::SharedPtr node);
  ~FeasibilityChecker();

  /** 抓取审批：几何范围 + IK + 关节余量，不执行规划/执行 */
  void checkPick(const orion_mtc_msgs::srv::CheckPick::Request::SharedPtr req,
                 orion_mtc_msgs::srv::CheckPick::Response::SharedPtr res);

  /** 注入 MTC 配置（gripper 偏移等），可选；未设置则用节点参数 */
  void setMTCConfig(const MTCConfig* config);

private:
  struct FeasibilityParams
  {
    double max_reach_hard = 1.20;
    double max_reach_soft = 1.10;
    double min_reach_safe = 0.12;
    double z_min = -0.50;
    double z_max = 0.80;
    double joint_margin_warning_rad = 0.10;
    double ik_timeout = 0.2;
    double approach_angle_max_deg = 55.0;   /* 接近方向与竖直向下允许最大夹角 */
    double suggestion_perturb_xy = 0.02;     /* 建议修正扰动 xy [m] */
    double suggestion_perturb_z = 0.03;
    double suggestion_perturb_yaw_rad = 0.15;
  };

  void loadParams();
  void onJointState(const void* msg);
  void addItem(std::vector<orion_mtc_msgs::msg::DiagnosticItem>& items,
               const std::string& code, int32_t level, const std::string& message,
               const std::string& field, double value, double threshold,
               const std::string& suggestion);
  /* 内部：对给定末端参考 link（默认 gripper_tcp）位姿做 IK，返回是否可解及关节余量诊断 */
  bool runIkAndJointMargin(const std::string& group_name, const std::string& link_name,
                          double px, double py, double pz,
                          double qx, double qy, double qz, double qw,
                          std::vector<orion_mtc_msgs::msg::DiagnosticItem>& items);
  /* 仅做 IK 可解性检查，不写 items（用于建议修正扰动搜索） */
  bool runIkOnly(double px, double py, double pz, double qx, double qy, double qz, double qw);
  /* 抓取失败时尝试扰动搜索，若找到可行位姿则写入 res 的 best_candidate_pose 并添加建议项 */
  void trySuggestCorrectionPick(const orion_mtc_msgs::srv::CheckPick::Request::SharedPtr req,
                                orion_mtc_msgs::srv::CheckPick::Response::SharedPtr res,
                                double obj_z, double grasp_z);
  /* 检查目标位姿是否与规划场景碰撞（可选，依赖 GetPlanningScene） */
  bool checkTargetCollision(double px, double py, double pz,
                           double qx, double qy, double qz, double qw,
                           std::vector<orion_mtc_msgs::msg::DiagnosticItem>& items);

  rclcpp::Node::SharedPtr node_;
  const MTCConfig* mtc_config_ = nullptr;
  FeasibilityParams params_;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_DECISION_FEASIBILITY_CHECKER_HPP

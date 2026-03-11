/* MTC pick-and-place node for Orion robot.
 * 抓取与放置拆成两个独立业务接口，底层共享同一套规划与执行；
 * 状态机：IDLE -> PICKING -> HOLDING -> PLACING -> IDLE
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit_task_constructor_msgs/msg/sub_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <orion_mtc_msgs/action/pick.hpp>
#include <orion_mtc_msgs/action/place.hpp>
#include <orion_mtc_msgs/srv/get_robot_state.hpp>
#include <moveit/robot_state/robot_state.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <atomic>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <ctime>
#include <future>
#include <iomanip>
#include <mutex>
#include <unordered_map>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc");
namespace mtc = moveit::task_constructor;

/* 控制器名与关节列表（与 orion_moveit_config moveit_controllers.yaml 一致） */
static const std::vector<std::string> ARM_JOINTS = {
    "joint_base_link_Link1", "joint_Link1_Link2", "joint_Link2_Link3",
    "joint_LinkVirtual_Link4", "joint_Link4_Link5", "joint_Link5_Link6"
};
static const std::vector<std::string> HAND_JOINTS = { "joint_Link6_Link7", "joint_Link6_Link8" };

/* 机器人任务状态：拆分 pick/place 后必须用 HOLDING 连接 */
enum class RobotTaskMode
{
  IDLE,
  PICKING,
  HOLDING,
  PLACING,
  ERROR
};

/* 抓取成功后保存的持物上下文，place 时用 tcp_to_object 反推末端目标 */
struct HeldObjectContext
{
  bool valid = false;
  std::string object_id;
  std::string attach_link;
  geometry_msgs::msg::Pose object_pose_at_grasp;
  geometry_msgs::msg::Pose tcp_pose_at_grasp;
  Eigen::Isometry3d tcp_to_object = Eigen::Isometry3d::Identity();
  shape_msgs::msg::SolidPrimitive shape;
  double weight = 0.0;
};

using PickAction = orion_mtc_msgs::action::Pick;
using PlaceAction = orion_mtc_msgs::action::Place;
using GetRobotStateSrv = orion_mtc_msgs::srv::GetRobotState;

class OrionMTCTaskNode
{
public:
  OrionMTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();
  mtc::Task buildPickTask(double obj_x, double obj_y, double obj_z,
                          const geometry_msgs::msg::Quaternion& object_orientation,
                          const std::string& object_id);
  mtc::Task buildPlaceTask(double place_x, double place_y, double place_z,
                           double place_qx, double place_qy, double place_qz, double place_qw);
  bool executeSolutionLocally(const mtc::SolutionBase& solution);
  bool executePickSolutionLocally(const mtc::SolutionBase& solution,
                                 const geometry_msgs::msg::Pose& object_pose_at_grasp,
                                 const std::string& object_id);
  bool executeSubTrajectory(const moveit_task_constructor_msgs::msg::SubTrajectory& sub);
  bool sendJointTrajectory(const std::string& controller_name,
                           const trajectory_msgs::msg::JointTrajectory& jt);

  bool computeTcpPoseFromTrajectoryEnd(const moveit::core::RobotModelConstPtr& robot_model,
                                      const trajectory_msgs::msg::JointTrajectory& traj,
                                      const std::string& hand_frame,
                                      geometry_msgs::msg::Pose& tcp_pose_out);
  static bool sceneDiffHasAttach(const moveit_task_constructor_msgs::msg::SubTrajectory& sub);
  void setState(RobotTaskMode mode);
  void setStateError(const std::string& err);
  std::string genTaskId(const char* prefix);

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr action_client_node_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_client_;
  std::unordered_map<std::string,
                     rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr>
      follow_jt_clients_;
  std::mutex follow_jt_mutex_;

  RobotTaskMode task_mode_ = RobotTaskMode::IDLE;
  HeldObjectContext held_object_;
  std::string current_task_id_;
  std::string last_error_;
  std::mutex state_mutex_;

  geometry_msgs::msg::PoseStamped object_pose_from_topic_;
  bool has_object_pose_from_topic_ = false;
  std::mutex object_pose_mutex_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_object_pose_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_pick_place_trigger_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_pick_trigger_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_place_trigger_;
  std::atomic<bool> do_task_running_{ false };

  geometry_msgs::msg::PoseStamped place_pose_from_topic_;
  bool has_place_pose_from_topic_ = false;
  std::mutex place_pose_mutex_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_place_pose_;

  std::atomic<double> left_arm_gripped_{ 0.0 };
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_left_arm_gripped_;

  rclcpp_action::Server<PickAction>::SharedPtr pick_action_server_;
  rclcpp_action::Server<PlaceAction>::SharedPtr place_action_server_;
  rclcpp::Service<GetRobotStateSrv>::SharedPtr get_robot_state_srv_;

  void onObjectPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onPickPlaceTriggerReceived(const std_msgs::msg::Empty::SharedPtr msg);
  void onPickTriggerReceived(const std_msgs::msg::Empty::SharedPtr msg);
  void onPlaceTriggerReceived(const std_msgs::msg::Empty::SharedPtr msg);
  void onPlacePoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onLeftArmGrippedReceived(const std_msgs::msg::Float32::SharedPtr msg);
  rclcpp_action::GoalResponse handlePickGoalRequest(const rclcpp_action::GoalUUID& uuid,
                                                    std::shared_ptr<const PickAction::Goal> goal);
  rclcpp_action::CancelResponse handlePickGoalCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickAction>>& goal_handle);
  void handlePickGoalAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickAction>>& goal_handle);
  rclcpp_action::GoalResponse handlePlaceGoalRequest(const rclcpp_action::GoalUUID& uuid,
                                                     std::shared_ptr<const PlaceAction::Goal> goal);
  rclcpp_action::CancelResponse handlePlaceGoalCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PlaceAction>>& goal_handle);
  void handlePlaceGoalAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PlaceAction>>& goal_handle);
  void handleGetRobotState(const std::shared_ptr<GetRobotStateSrv::Request> req,
                           std::shared_ptr<GetRobotStateSrv::Response> res);

  /* 业务编排：仅抓取 / 仅放置（内部用 object_pose 或 place_pose 来自话题或 action goal） */
  void doPick();
  void doPlace();
  bool doPickFromGoal(const geometry_msgs::msg::PoseStamped& object_pose, const std::string& object_id);
  bool doPlaceFromGoal(const geometry_msgs::msg::PoseStamped& target_pose);

  bool waitForGripped(bool expect_gripped, double timeout_sec = 5.0);
  bool isHandOnlySegment(const moveit_task_constructor_msgs::msg::SubTrajectory& sub) const;
  bool isGripperClosedInSegment(const moveit_task_constructor_msgs::msg::SubTrajectory& sub) const;
};

OrionMTCTaskNode::OrionMTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("orion_mtc_node", options) }
  , action_client_node_{ std::make_shared<rclcpp::Node>("orion_mtc_action_client", options) }
{
  /* 仅当 launch 未传入时声明 MTC pick-place 参数，避免与 params-file 重复声明导致 ParameterAlreadyDeclaredException */
  auto declare_if_not_set = [this](const char* name, double val) {
    try
    {
      node_->declare_parameter<double>(name, val);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
    {
      /* 已由 launch 的 params 提供，跳过 */
    }
  };
  auto declare_str_if_not_set = [this](const char* name, const std::string& val) {
    try
    {
      node_->declare_parameter<std::string>(name, val);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
    {
    }
  };
  /* HoloOcean + 当前几何下，approach 常会差几毫米导致整个 pick 失败；默认放宽到 0.08m */
  declare_if_not_set("approach_object_min_dist", 0.08);
  declare_if_not_set("approach_object_max_dist", 0.15);
  declare_if_not_set("lift_object_min_dist", 0.05);
  declare_if_not_set("lift_object_max_dist", 0.25);
  declare_str_if_not_set("support_surface_link", "");
  declare_if_not_set("place_pose_x", 0.35);
  declare_if_not_set("place_pose_y", 0.15);
  declare_if_not_set("place_pose_z", 0.4);
  declare_if_not_set("place_pose_qx", 0.0);
  declare_if_not_set("place_pose_qy", 0.0);
  declare_if_not_set("place_pose_qz", 0.0);
  declare_if_not_set("place_pose_qw", 1.0);
  declare_if_not_set("retreat_min_dist", 0.12);
  declare_if_not_set("retreat_max_dist", 0.25);
  declare_if_not_set("lower_to_place_min_dist", 0.05);
  declare_if_not_set("lower_to_place_max_dist", 0.12);
  /* 物体位姿由 /object_pose 话题提供；pregrasp 由物体位置计算 */

  /* 物体位姿话题：base_link 下 PoseStamped；抓取仍由内部计算（订阅放在 action_client_node_ 上以便 executor 能收到回调） */
  sub_object_pose_ = action_client_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "object_pose", 10, std::bind(&OrionMTCTaskNode::onObjectPoseReceived, this, std::placeholders::_1));
  sub_pick_place_trigger_ = action_client_node_->create_subscription<std_msgs::msg::Empty>(
      "pick_place_trigger", 10, std::bind(&OrionMTCTaskNode::onPickPlaceTriggerReceived, this, std::placeholders::_1));
  sub_pick_trigger_ = action_client_node_->create_subscription<std_msgs::msg::Empty>(
      "pick_trigger", 10, std::bind(&OrionMTCTaskNode::onPickTriggerReceived, this, std::placeholders::_1));
  sub_place_trigger_ = action_client_node_->create_subscription<std_msgs::msg::Empty>(
      "place_trigger", 10, std::bind(&OrionMTCTaskNode::onPlaceTriggerReceived, this, std::placeholders::_1));
  sub_place_pose_ = action_client_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "place_pose", 10, std::bind(&OrionMTCTaskNode::onPlacePoseReceived, this, std::placeholders::_1));
  sub_left_arm_gripped_ = action_client_node_->create_subscription<std_msgs::msg::Float32>(
      "left_arm_gripped", 10, std::bind(&OrionMTCTaskNode::onLeftArmGrippedReceived, this, std::placeholders::_1));

  pick_action_server_ = rclcpp_action::create_server<PickAction>(
      action_client_node_, "pick",
      std::bind(&OrionMTCTaskNode::handlePickGoalRequest, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&OrionMTCTaskNode::handlePickGoalCancel, this, std::placeholders::_1),
      std::bind(&OrionMTCTaskNode::handlePickGoalAccepted, this, std::placeholders::_1));
  place_action_server_ = rclcpp_action::create_server<PlaceAction>(
      action_client_node_, "place",
      std::bind(&OrionMTCTaskNode::handlePlaceGoalRequest, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&OrionMTCTaskNode::handlePlaceGoalCancel, this, std::placeholders::_1),
      std::bind(&OrionMTCTaskNode::handlePlaceGoalAccepted, this, std::placeholders::_1));
  get_robot_state_srv_ = action_client_node_->create_service<GetRobotStateSrv>(
      "get_robot_state", std::bind(&OrionMTCTaskNode::handleGetRobotState, this,
                                   std::placeholders::_1, std::placeholders::_2));
}

void OrionMTCTaskNode::onObjectPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (msg->header.frame_id != "base_link")
  {
    RCLCPP_WARN(LOGGER, "object_pose frame_id is '%s', expected base_link; ignore (or add tf transform)",
                msg->header.frame_id.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(object_pose_mutex_);
  object_pose_from_topic_ = *msg;
  has_object_pose_from_topic_ = true;
  RCLCPP_DEBUG(LOGGER, "object_pose received: (%.3f, %.3f, %.3f)", msg->pose.position.x, msg->pose.position.y,
               msg->pose.position.z);
}

void OrionMTCTaskNode::onPickPlaceTriggerReceived(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  std::thread(&OrionMTCTaskNode::doTask, this).detach();
}

void OrionMTCTaskNode::onPickTriggerReceived(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  std::thread(&OrionMTCTaskNode::doPick, this).detach();
}

void OrionMTCTaskNode::onPlaceTriggerReceived(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  std::thread(&OrionMTCTaskNode::doPlace, this).detach();
}

void OrionMTCTaskNode::onPlacePoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (msg->header.frame_id != "base_link")
  {
    RCLCPP_WARN(LOGGER, "place_pose frame_id is '%s', expected base_link; ignore",
                msg->header.frame_id.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(place_pose_mutex_);
  place_pose_from_topic_ = *msg;
  has_place_pose_from_topic_ = true;
  RCLCPP_DEBUG(LOGGER, "place_pose received: (%.3f, %.3f, %.3f)", msg->pose.position.x, msg->pose.position.y,
               msg->pose.position.z);
}

void OrionMTCTaskNode::onLeftArmGrippedReceived(const std_msgs::msg::Float32::SharedPtr msg)
{
  double v = static_cast<double>(msg->data);
  left_arm_gripped_.store(v);
  /* HOLDING 仅由 pick 成功设置（有 held_object_ 才能放置）；话题 lock 仅用于夹爪松开时清空持物，不根据 locked 设为 HOLDING */
  const double grip_threshold = 0.5;
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (task_mode_ == RobotTaskMode::HOLDING && v < grip_threshold)
  {
    held_object_.valid = false;
    task_mode_ = RobotTaskMode::IDLE;
    RCLCPP_INFO(LOGGER, "left_arm_gripped=%.3f < %.3f: topic unlocked, clear holding -> IDLE", v, grip_threshold);
  }
}

bool OrionMTCTaskNode::waitForGripped(bool expect_gripped, double timeout_sec)
{
  const double threshold = 0.5;
  const int total_ticks = static_cast<int>(timeout_sec * 20.0);  /* 50 ms per tick */
  for (int i = 0; i < total_ticks; ++i)
  {
    double v = left_arm_gripped_.load();
    if (expect_gripped && v >= threshold)
    {
      RCLCPP_INFO(LOGGER, "waitForGripped: gripped (%.3f >= %.3f)", v, threshold);
      return true;
    }
    if (!expect_gripped && v < threshold)
    {
      RCLCPP_INFO(LOGGER, "waitForGripped: unlocked (%.3f < %.3f)", v, threshold);
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  RCLCPP_WARN(LOGGER, "waitForGripped: timeout (expect_gripped=%d, last=%.3f)", expect_gripped,
              left_arm_gripped_.load());
  return false;
}

bool OrionMTCTaskNode::isHandOnlySegment(const moveit_task_constructor_msgs::msg::SubTrajectory& sub) const
{
  const auto& names = sub.trajectory.joint_trajectory.joint_names;
  for (const auto& n : names)
  {
    if (std::find(ARM_JOINTS.begin(), ARM_JOINTS.end(), n) != ARM_JOINTS.end())
      return false;
  }
  return std::find_first_of(names.begin(), names.end(), HAND_JOINTS.begin(), HAND_JOINTS.end()) != names.end();
}

bool OrionMTCTaskNode::isGripperClosedInSegment(const moveit_task_constructor_msgs::msg::SubTrajectory& sub) const
{
  const auto& traj = sub.trajectory.joint_trajectory;
  if (traj.points.empty() || traj.joint_names.size() != traj.points.back().positions.size())
    return false;
  /* Orion 夹爪闭合 (0, 0) rad，张开 (0.4, -0.4) rad */
  std::vector<size_t> hand_idx;
  for (const auto& hn : HAND_JOINTS)
  {
    for (size_t i = 0; i < traj.joint_names.size(); ++i)
    {
      if (traj.joint_names[i] == hn)
      {
        hand_idx.push_back(i);
        break;
      }
    }
  }
  if (hand_idx.size() != 2u)
    return false;
  const auto& p = traj.points.back().positions;
  double j0 = p[hand_idx[0]];
  double j1 = p[hand_idx[1]];
  return std::abs(j0) < 0.15 && std::abs(j1) < 0.15;
}

void OrionMTCTaskNode::setState(RobotTaskMode mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  task_mode_ = mode;
  if (mode == RobotTaskMode::ERROR)
  {
    last_error_.clear();
  }
}

void OrionMTCTaskNode::setStateError(const std::string& err)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  task_mode_ = RobotTaskMode::ERROR;
  last_error_ = err;
  RCLCPP_ERROR(LOGGER, "state ERROR: %s", err.c_str());
}

std::string OrionMTCTaskNode::genTaskId(const char* prefix)
{
  auto now = std::chrono::system_clock::now();
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  std::ostringstream oss;
  oss << prefix << "_" << ns;
  return oss.str();
}

bool OrionMTCTaskNode::sceneDiffHasAttach(const moveit_task_constructor_msgs::msg::SubTrajectory& sub)
{
  return sub.scene_diff.is_diff &&
         !sub.scene_diff.robot_state.attached_collision_objects.empty();
}

bool OrionMTCTaskNode::computeTcpPoseFromTrajectoryEnd(
    const moveit::core::RobotModelConstPtr& robot_model,
    const trajectory_msgs::msg::JointTrajectory& traj,
    const std::string& hand_frame,
    geometry_msgs::msg::Pose& tcp_pose_out)
{
  if (traj.points.empty() || traj.joint_names.empty())
  {
    return false;
  }
  const auto& last_pt = traj.points.back();
  if (last_pt.positions.size() != traj.joint_names.size())
  {
    return false;
  }
  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();
  for (size_t i = 0; i < traj.joint_names.size(); ++i)
  {
    const std::string& name = traj.joint_names[i];
    if (state.getRobotModel()->hasJointModel(name))
    {
      state.setVariablePosition(name, last_pt.positions[i]);
    }
  }
  state.update();
  if (!state.knowsFrameTransform(hand_frame))
  {
    return false;
  }
  const Eigen::Isometry3d& T = state.getGlobalLinkTransform(hand_frame);
  tcp_pose_out.position.x = T.translation().x();
  tcp_pose_out.position.y = T.translation().y();
  tcp_pose_out.position.z = T.translation().z();
  Eigen::Quaterniond q(T.rotation());
  tcp_pose_out.orientation.x = q.x();
  tcp_pose_out.orientation.y = q.y();
  tcp_pose_out.orientation.z = q.z();
  tcp_pose_out.orientation.w = q.w();
  return true;
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr OrionMTCTaskNode::getNodeBaseInterface()
{
  /* executor 只 spin action_client_node_，避免与 Task 冲突；Task 使用 node_ */
  return action_client_node_->get_node_base_interface();
}

void OrionMTCTaskNode::setupPlanningScene()
{
  /* 物体由任务内 ModifyPlanningScene("add object") 加入，不依赖 /collision_object 与 move_group 同步，逻辑更确定 */
  RCLCPP_INFO(LOGGER, "setupPlanningScene: object added in-task (add object stage), no /collision_object publish");
}

bool OrionMTCTaskNode::sendJointTrajectory(const std::string& controller_name,
                                           const trajectory_msgs::msg::JointTrajectory& jt)
{
  if (jt.points.empty())
  {
    RCLCPP_DEBUG(LOGGER, "sendJointTrajectory: empty trajectory for %s", controller_name.c_str());
    return true;
  }
  RCLCPP_DEBUG(LOGGER, "sendJointTrajectory: %s joints=%zu points=%zu",
               controller_name.c_str(), jt.joint_names.size(), jt.points.size());

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr client;
  bool need_wait_server = false;
  {
    std::lock_guard<std::mutex> lk(follow_jt_mutex_);
    auto it = follow_jt_clients_.find(controller_name);
    if (it == follow_jt_clients_.end())
    {
      std::string action_name = "/" + controller_name + "/follow_joint_trajectory";
      auto c = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
          action_client_node_, action_name);
      follow_jt_clients_.emplace(controller_name, c);
      client = c;
      need_wait_server = true;
    }
    else
    {
      client = it->second;
    }
  }
  if (need_wait_server && !client->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(LOGGER, "sendJointTrajectory: action /%s/follow_joint_trajectory not available",
                 controller_name.c_str());
    return false;
  }

  auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
  goal_msg.trajectory = jt;
  goal_msg.trajectory.header.stamp.sec = 0;
  goal_msg.trajectory.header.stamp.nanosec = 0;  /* 置零最兼容，避免 VM/仿真下“已过期”误判 */
  auto goal_handle_future = client->async_send_goal(goal_msg);
  /* 等待 server 接受 goal；长轨迹或仿真延迟时需更长时间，取 15s */
  if (goal_handle_future.wait_for(std::chrono::seconds(15)) != std::future_status::ready)
  {
    RCLCPP_ERROR(LOGGER, "sendJointTrajectory: send_goal timeout for %s", controller_name.c_str());
    return false;
  }
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(LOGGER, "sendJointTrajectory: goal rejected for %s", controller_name.c_str());
    return false;
  }
  auto result_future = client->async_get_result(goal_handle);
  if (result_future.wait_for(std::chrono::seconds(60)) != std::future_status::ready)
  {
    RCLCPP_ERROR(LOGGER, "sendJointTrajectory: get_result timeout for %s", controller_name.c_str());
    client->async_cancel_goal(goal_handle);
    return false;
  }
  auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_ERROR(LOGGER, "sendJointTrajectory: %s failed result code %d",
                 controller_name.c_str(), static_cast<int>(result.code));
    return false;
  }
  return true;
}

bool OrionMTCTaskNode::executeSubTrajectory(
    const moveit_task_constructor_msgs::msg::SubTrajectory& sub)
{
  /* 先应用 scene_diff（attach/detach/ACM 等）；只要 is_diff 就 apply，避免漏掉只改 name/transforms 等字段的 diff */
  if (sub.scene_diff.is_diff)
  {
    if (!apply_planning_scene_client_)
    {
      apply_planning_scene_client_ =
          action_client_node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    }
    if (apply_planning_scene_client_)
    {
      if (!apply_planning_scene_client_->service_is_ready())
        apply_planning_scene_client_->wait_for_service(std::chrono::seconds(2));
    }
    if (apply_planning_scene_client_ && apply_planning_scene_client_->service_is_ready())
    {
      auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
      req->scene = sub.scene_diff;
      auto fut = apply_planning_scene_client_->async_send_request(req);
      if (fut.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
      {
        auto res = fut.get();
        if (!res || !res->success)
          RCLCPP_WARN(LOGGER, "executeSubTrajectory: apply_planning_scene returned false");
        else
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      else
      {
        RCLCPP_WARN(LOGGER, "executeSubTrajectory: apply_planning_scene timed out");
      }
    }
  }

  const auto& traj = sub.trajectory.joint_trajectory;
  if (traj.points.empty())
  {
    return true;
  }

  /* controller_names 为空时按 joint_names 推断控制器，避免漏执行 */
  std::vector<std::string> ctrl_names(sub.execution_info.controller_names.begin(),
                                      sub.execution_info.controller_names.end());
  if (ctrl_names.empty())
  {
    auto is_in = [](const std::string& j, const std::vector<std::string>& set) {
      return std::find(set.begin(), set.end(), j) != set.end();
    };
    bool any_arm = false;
    bool any_hand = false;
    bool any_other = false;
    for (const auto& j : traj.joint_names)
    {
      if (is_in(j, ARM_JOINTS))
        any_arm = true;
      else if (is_in(j, HAND_JOINTS))
        any_hand = true;
      else
        any_other = true;
    }
    if (traj.joint_names.empty())
    {
      RCLCPP_WARN(LOGGER, "executeSubTrajectory: trajectory has points but no joint_names");
      return true;
    }
    if (any_other)
    {
      ctrl_names = { "arm_controller" };
    }
    else if (any_arm && any_hand)
    {
      ctrl_names = { "arm_controller", "hand_controller" };
    }
    else if (any_arm)
    {
      ctrl_names = { "arm_controller" };
    }
    else if (any_hand)
    {
      ctrl_names = { "hand_controller" };
    }
    else
    {
      ctrl_names = { "arm_controller" };
    }
    RCLCPP_DEBUG(LOGGER, "executeSubTrajectory: controller_names empty, inferred %zu controller(s)",
                 ctrl_names.size());
  }

  /* 按控制器拆分：用索引抽取子轨迹，以最大索引越界为安全判断（非 indices.size()） */
  std::vector<std::string> arm_joints(ARM_JOINTS.begin(), ARM_JOINTS.end());
  std::vector<std::string> hand_joints(HAND_JOINTS.begin(), HAND_JOINTS.end());

  auto jointIndex = [&traj](const std::vector<std::string>& names) {
    std::vector<size_t> idx;
    for (const auto& n : names)
    {
      for (size_t i = 0; i < traj.joint_names.size(); ++i)
      {
        if (traj.joint_names[i] == n)
        {
          idx.push_back(i);
          break;
        }
      }
    }
    return idx;
  };

  /* 按索引抽取，以 max(index) < src.size() 为越界判断，避免 indices.size()<=src.size() 误判 */
  auto safe_extract = [](std::vector<double>& dst, const std::vector<double>& src,
                         const std::vector<size_t>& idx) -> bool {
    if (src.empty() || idx.empty())
      return false;
    size_t max_i = *std::max_element(idx.begin(), idx.end());
    if (max_i >= src.size())
      return false;
    for (size_t i : idx)
      dst.push_back(src[i]);
    return true;
  };

  auto extractSubTrajectory = [&traj, &safe_extract](const std::vector<size_t>& indices)
      -> trajectory_msgs::msg::JointTrajectory {
    trajectory_msgs::msg::JointTrajectory out;
    out.header = traj.header;
    if (indices.empty())
      return out;
    for (size_t i : indices)
    {
      if (i < traj.joint_names.size())
        out.joint_names.push_back(traj.joint_names[i]);
    }
    if (out.joint_names.size() != indices.size())
      return out;
    bool warned_vel = false;
    bool warned_acc = false;
    for (const auto& pt : traj.points)
    {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      if (!safe_extract(p.positions, pt.positions, indices))
      {
        RCLCPP_ERROR(LOGGER, "executeSubTrajectory: point positions index out of range");
        return trajectory_msgs::msg::JointTrajectory();
      }
      if (!pt.velocities.empty() && !safe_extract(p.velocities, pt.velocities, indices) && !warned_vel)
      {
        RCLCPP_WARN(LOGGER, "executeSubTrajectory: velocities dimension mismatch, leaving empty");
        warned_vel = true;
      }
      if (!pt.accelerations.empty() && !safe_extract(p.accelerations, pt.accelerations, indices) && !warned_acc)
      {
        RCLCPP_WARN(LOGGER, "executeSubTrajectory: accelerations dimension mismatch, leaving empty");
        warned_acc = true;
      }
      p.time_from_start = pt.time_from_start;
      out.points.push_back(p);
    }
    return out;
  };

  std::vector<std::pair<std::string, trajectory_msgs::msg::JointTrajectory>> to_send;
  for (const auto& ctrl : ctrl_names)
  {
    trajectory_msgs::msg::JointTrajectory jt_to_send;
    if (ctrl == "arm_controller")
    {
      auto idx = jointIndex(arm_joints);
      if (idx.empty())
        continue;
      jt_to_send = extractSubTrajectory(idx);
      if (jt_to_send.points.empty() && !idx.empty())
      {
        RCLCPP_ERROR(LOGGER, "executeSubTrajectory: arm segment extraction failed (index out of range)");
        return false;
      }
    }
    else if (ctrl == "hand_controller")
    {
      auto idx = jointIndex(hand_joints);
      if (idx.empty())
        continue;
      jt_to_send = extractSubTrajectory(idx);
      if (jt_to_send.points.empty() && !idx.empty())
      {
        RCLCPP_ERROR(LOGGER, "executeSubTrajectory: hand segment extraction failed (index out of range)");
        return false;
      }
    }
    else
    {
      jt_to_send = traj;
    }
    if (!jt_to_send.points.empty())
      to_send.emplace_back(ctrl, jt_to_send);
  }

  if (!traj.points.empty() && to_send.empty())
  {
    RCLCPP_WARN(LOGGER,
                "executeSubTrajectory: trajectory non-empty but no controller selected; joint_names=%zu",
                traj.joint_names.size());
  }

  /* 单控制器时同步发送；多控制器时先并发发送再统一等待（arm+hand 同步执行） */
  if (to_send.size() == 1u)
  {
    if (!sendJointTrajectory(to_send[0].first, to_send[0].second))
      return false;
  }
  else if (to_send.size() > 1u)
  {
    std::vector<std::shared_future<bool>> results;
    for (const auto& p : to_send)
    {
      /* 按值捕获 p，避免并发时引用到循环变量导致两线程发同一 controller */
      results.push_back(
          std::async(std::launch::async, [this, p]() {
            return sendJointTrajectory(p.first, p.second);
          })
              .share());
    }
    for (auto& r : results)
    {
      if (!r.get())
        return false;
    }
  }
  return true;
}

bool OrionMTCTaskNode::executeSolutionLocally(const mtc::SolutionBase& solution)
{
  /* 同一进程内直接用 C++ SolutionBase::toMsg 得到 Solution 消息，不依赖 get_solution 服务或 solution 话题 */
  moveit_task_constructor_msgs::msg::Solution solution_msg;
  solution.toMsg(solution_msg, &task_.introspection());

  if (solution_msg.sub_trajectory.empty())
  {
    RCLCPP_WARN(LOGGER, "executeSolutionLocally: no sub_trajectory in solution");
    return false;
  }

  task_.introspection().publishSolution(solution);  /* 供 RViz 显示 */

  RCLCPP_INFO(LOGGER, "executeSolutionLocally: executing %zu trajectory segments",
              solution_msg.sub_trajectory.size());

  bool have_waited_gripped = false;  /* 已等待过“抓稳”，后续手部张开段则等待“松开” */
  for (size_t i = 0; i < solution_msg.sub_trajectory.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "Executing segment %zu / %zu", i + 1, solution_msg.sub_trajectory.size());
    if (!executeSubTrajectory(solution_msg.sub_trajectory[i]))
    {
      RCLCPP_ERROR(LOGGER, "executeSolutionLocally: segment %zu failed", i);
      return false;
    }
    /* 动态抓取：仅手部 segment 后根据闭合/张开等待 gripped 或 unlock */
    const auto& sub = solution_msg.sub_trajectory[i];
    if (isHandOnlySegment(sub))
    {
      /* 仅对“抓取时闭合”等待 gripped；回位时的 close hand (return home) 不再等待（手中无物） */
      if (isGripperClosedInSegment(sub) && !have_waited_gripped)
      {
        if (!waitForGripped(true))
          RCLCPP_WARN(LOGGER, "executeSolutionLocally: wait gripped timeout, continue anyway");
        have_waited_gripped = true;
      }
      else if (have_waited_gripped)
      {
        if (!waitForGripped(false))
          RCLCPP_WARN(LOGGER, "executeSolutionLocally: wait unlock timeout, continue anyway");
      }
    }
  }
  return true;
}

bool OrionMTCTaskNode::executePickSolutionLocally(const mtc::SolutionBase& solution,
                                                 const geometry_msgs::msg::Pose& object_pose_at_grasp,
                                                 const std::string& object_id)
{
  moveit_task_constructor_msgs::msg::Solution solution_msg;
  solution.toMsg(solution_msg, &task_.introspection());
  if (solution_msg.sub_trajectory.empty())
  {
    RCLCPP_WARN(LOGGER, "executePickSolutionLocally: no sub_trajectory");
    return false;
  }
  task_.introspection().publishSolution(solution);

  const std::string hand_frame = "Link6";
  trajectory_msgs::msg::JointTrajectory last_trajectory;  // 用于 attach 段后 FK 计算 TCP

  bool have_waited_gripped = false;
  for (size_t i = 0; i < solution_msg.sub_trajectory.size(); ++i)
  {
    const auto& sub = solution_msg.sub_trajectory[i];
    RCLCPP_INFO(LOGGER, "Executing pick segment %zu / %zu", i + 1, solution_msg.sub_trajectory.size());
    if (!executeSubTrajectory(sub))
    {
      RCLCPP_ERROR(LOGGER, "executePickSolutionLocally: segment %zu failed", i);
      return false;
    }
    if (!sub.trajectory.joint_trajectory.points.empty())
    {
      last_trajectory = sub.trajectory.joint_trajectory;
    }
    if (sceneDiffHasAttach(sub) && !last_trajectory.points.empty())
    {
      geometry_msgs::msg::Pose tcp_pose;
      if (computeTcpPoseFromTrajectoryEnd(task_.getRobotModel(), last_trajectory, hand_frame, tcp_pose))
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        held_object_.valid = true;
        held_object_.object_id = object_id.empty() ? "object" : object_id;
        held_object_.attach_link = hand_frame;
        held_object_.object_pose_at_grasp = object_pose_at_grasp;
        held_object_.tcp_pose_at_grasp = tcp_pose;
        Eigen::Isometry3d T_base_tcp = Eigen::Isometry3d::Identity();
        T_base_tcp.translate(Eigen::Vector3d(tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z));
        T_base_tcp.rotate(Eigen::Quaterniond(tcp_pose.orientation.w, tcp_pose.orientation.x,
                                            tcp_pose.orientation.y, tcp_pose.orientation.z));
        Eigen::Isometry3d T_base_obj = Eigen::Isometry3d::Identity();
        T_base_obj.translate(Eigen::Vector3d(object_pose_at_grasp.position.x, object_pose_at_grasp.position.y,
                                              object_pose_at_grasp.position.z));
        T_base_obj.rotate(Eigen::Quaterniond(object_pose_at_grasp.orientation.w,
                                              object_pose_at_grasp.orientation.x,
                                              object_pose_at_grasp.orientation.y,
                                              object_pose_at_grasp.orientation.z));
        held_object_.tcp_to_object = T_base_tcp.inverse() * T_base_obj;
        held_object_.shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        held_object_.shape.dimensions = { 0.1, 0.02 };
        RCLCPP_INFO(LOGGER, "executePickSolutionLocally: saved held context object_id=%s",
                    held_object_.object_id.c_str());
      }
      else
      {
        RCLCPP_WARN(LOGGER, "executePickSolutionLocally: FK for TCP at grasp failed, held context incomplete");
      }
    }
    if (isHandOnlySegment(sub))
    {
      if (isGripperClosedInSegment(sub) && !have_waited_gripped)
      {
        if (!waitForGripped(true))
          RCLCPP_WARN(LOGGER, "executePickSolutionLocally: wait gripped timeout");
        have_waited_gripped = true;
      }
      else if (have_waited_gripped)
      {
        if (!waitForGripped(false))
          RCLCPP_WARN(LOGGER, "executePickSolutionLocally: wait unlock timeout");
      }
    }
  }
  return true;
}

void OrionMTCTaskNode::doTask()
{
  if (do_task_running_.exchange(true))
  {
    RCLCPP_WARN(LOGGER, "doTask: already running, skip (wait for current task to finish)");
    return;
  }
  /* 完全由话题驱动：无 /object_pose 不执行；若尚未收到则短暂等待 */
  {
    const int wait_ticks = 60;  /* 约 3 s，50 ms/次 */
    for (int i = 0; i < wait_ticks; ++i)
    {
      {
        std::lock_guard<std::mutex> lock(object_pose_mutex_);
        if (has_object_pose_from_topic_)
          break;
      }
      if (i == 0)
        RCLCPP_INFO(LOGGER, "doTask: waiting for /object_pose (e.g. from target_sensor_to_object_pose)...");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::lock_guard<std::mutex> lock(object_pose_mutex_);
    if (!has_object_pose_from_topic_)
    {
      RCLCPP_WARN(LOGGER, "doTask: no object pose from topic after wait, skip (publish /object_pose then /pick_place_trigger)");
      do_task_running_ = false;
      return;
    }
  }
  task_ = createTask();

  try
  {
    task_.init();
    task_.enableIntrospection(true);
    task_.introspection().publishTaskDescription();  /* 触发 introspection 创建 publisher/service */
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    do_task_running_ = false;
    return;
  }

  moveit::core::MoveItErrorCode plan_result = task_.plan(5);
  if (!plan_result)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed (code " << plan_result.val << ")");
    task_.explainFailure(std::cout);
    std::cout.flush();
    do_task_running_ = false;
    return;
  }
  if (task_.solutions().empty())
  {
    RCLCPP_ERROR(LOGGER, "Task planning returned success but no solutions");
    do_task_running_ = false;
    return;
  }

  /* 本地执行：取 solution 消息并执行；内部会 publish 供 RViz 显示 */
  if (!executeSolutionLocally(*task_.solutions().front()))
  {
    RCLCPP_ERROR(LOGGER, "Local task execution failed");
    do_task_running_ = false;
    return;
  }
  RCLCPP_INFO(LOGGER, "Task execution finished successfully");
  do_task_running_ = false;
}

void OrionMTCTaskNode::doPick()
{
  if (do_task_running_.exchange(true))
  {
    RCLCPP_WARN(LOGGER, "doPick: already running, skip");
    return;
  }
  const int wait_ticks = 60;
  for (int i = 0; i < wait_ticks; ++i)
  {
    {
      std::lock_guard<std::mutex> lock(object_pose_mutex_);
      if (has_object_pose_from_topic_)
        break;
    }
    if (i == 0)
      RCLCPP_INFO(LOGGER, "doPick: waiting for /object_pose");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(object_pose_mutex_);
    if (!has_object_pose_from_topic_)
    {
      RCLCPP_WARN(LOGGER, "doPick: no object pose after wait, abort");
      do_task_running_ = false;
      return;
    }
    pose = object_pose_from_topic_;
  }
  bool ok = doPickFromGoal(pose, "");
  do_task_running_ = false;
  (void)ok;
}

void OrionMTCTaskNode::doPlace()
{
  if (do_task_running_.exchange(true))
  {
    RCLCPP_WARN(LOGGER, "doPlace: already running, skip");
    return;
  }
  const int wait_ticks = 60;
  for (int i = 0; i < wait_ticks; ++i)
  {
    {
      std::lock_guard<std::mutex> lock(place_pose_mutex_);
      if (has_place_pose_from_topic_)
        break;
    }
    if (i == 0)
      RCLCPP_INFO(LOGGER, "doPlace: waiting for /place_pose");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(place_pose_mutex_);
    if (!has_place_pose_from_topic_)
    {
      RCLCPP_WARN(LOGGER, "doPlace: no place pose after wait, use params");
      place_pose_from_topic_.header.frame_id = "base_link";
      node_->get_parameter("place_pose_x", place_pose_from_topic_.pose.position.x);
      node_->get_parameter("place_pose_y", place_pose_from_topic_.pose.position.y);
      node_->get_parameter("place_pose_z", place_pose_from_topic_.pose.position.z);
      node_->get_parameter("place_pose_qx", place_pose_from_topic_.pose.orientation.x);
      node_->get_parameter("place_pose_qy", place_pose_from_topic_.pose.orientation.y);
      node_->get_parameter("place_pose_qz", place_pose_from_topic_.pose.orientation.z);
      node_->get_parameter("place_pose_qw", place_pose_from_topic_.pose.orientation.w);
    }
    pose = place_pose_from_topic_;
  }
  bool ok = doPlaceFromGoal(pose);
  do_task_running_ = false;
  (void)ok;
}

bool OrionMTCTaskNode::doPickFromGoal(const geometry_msgs::msg::PoseStamped& object_pose,
                                      const std::string& object_id)
{
  RobotTaskMode mode;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    mode = task_mode_;
  }
  if (mode == RobotTaskMode::HOLDING)
  {
    RCLCPP_ERROR(LOGGER, "doPickFromGoal: already holding, reject (clear or place first)");
    return false;
  }
  if (mode != RobotTaskMode::IDLE && mode != RobotTaskMode::ERROR)
  {
    RCLCPP_ERROR(LOGGER, "doPickFromGoal: busy (mode not IDLE/ERROR), reject");
    return false;
  }

  setState(RobotTaskMode::PICKING);
  current_task_id_ = genTaskId("pick");

  double obj_x = object_pose.pose.position.x;
  double obj_y = object_pose.pose.position.y;
  double obj_z = object_pose.pose.position.z;
  geometry_msgs::msg::Quaternion object_orientation = object_pose.pose.orientation;
  if (object_pose.header.frame_id != "base_link")
  {
    RCLCPP_WARN(LOGGER, "doPickFromGoal: frame_id '%s', expected base_link", object_pose.header.frame_id.c_str());
  }

  task_ = buildPickTask(obj_x, obj_y, obj_z, object_orientation, object_id);
  try
  {
    task_.init();
    task_.enableIntrospection(true);
    task_.introspection().publishTaskDescription();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    setStateError(std::string("pick init: ") + e.what());
    return false;
  }

  moveit::core::MoveItErrorCode plan_result = task_.plan(5);
  if (!plan_result)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Pick planning failed (code " << plan_result.val << ")");
    task_.explainFailure(std::cout);
    setStateError("pick plan failed");
    return false;
  }
  if (task_.solutions().empty())
  {
    RCLCPP_ERROR(LOGGER, "Pick plan returned no solutions");
    setStateError("pick no solutions");
    return false;
  }

  geometry_msgs::msg::Pose object_pose_at_grasp;
  object_pose_at_grasp.position.x = obj_x;
  object_pose_at_grasp.position.y = obj_y;
  object_pose_at_grasp.position.z = obj_z;
  object_pose_at_grasp.orientation = object_orientation;

  if (!executePickSolutionLocally(*task_.solutions().front(), object_pose_at_grasp, object_id))
  {
    RCLCPP_ERROR(LOGGER, "Pick execution failed");
    setStateError("pick execution failed");
    return false;
  }

  setState(RobotTaskMode::HOLDING);
  RCLCPP_INFO(LOGGER, "Pick finished successfully, state=HOLDING");
  return true;
}

bool OrionMTCTaskNode::doPlaceFromGoal(const geometry_msgs::msg::PoseStamped& target_pose)
{
  bool has_held = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    has_held = held_object_.valid;
    if (task_mode_ != RobotTaskMode::HOLDING || !has_held)
    {
      RCLCPP_ERROR(LOGGER, "doPlaceFromGoal: not holding (mode=%d, held_valid=%d), reject",
                   static_cast<int>(task_mode_), held_object_.valid ? 1 : 0);
      return false;
    }
  }

  setState(RobotTaskMode::PLACING);
  current_task_id_ = genTaskId("place");

  double px = target_pose.pose.position.x;
  double py = target_pose.pose.position.y;
  double pz = target_pose.pose.position.z;
  double qx = target_pose.pose.orientation.x;
  double qy = target_pose.pose.orientation.y;
  double qz = target_pose.pose.orientation.z;
  double qw = target_pose.pose.orientation.w;
  if (target_pose.header.frame_id != "base_link")
  {
    RCLCPP_WARN(LOGGER, "doPlaceFromGoal: frame_id '%s', expected base_link", target_pose.header.frame_id.c_str());
  }

  task_ = buildPlaceTask(px, py, pz, qx, qy, qz, qw);
  if (task_.stages()->numChildren() == 0)
  {
    setStateError("buildPlaceTask: no held context");
    setState(RobotTaskMode::HOLDING);
    return false;
  }

  try
  {
    task_.init();
    task_.enableIntrospection(true);
    task_.introspection().publishTaskDescription();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    setStateError(std::string("place init: ") + e.what());
    setState(RobotTaskMode::HOLDING);
    return false;
  }

  moveit::core::MoveItErrorCode plan_result = task_.plan(5);
  if (!plan_result)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Place planning failed (code " << plan_result.val << ")");
    task_.explainFailure(std::cout);
    setStateError("place plan failed");
    setState(RobotTaskMode::HOLDING);
    return false;
  }
  if (task_.solutions().empty())
  {
    RCLCPP_ERROR(LOGGER, "Place plan returned no solutions");
    setStateError("place no solutions");
    setState(RobotTaskMode::HOLDING);
    return false;
  }

  if (!executeSolutionLocally(*task_.solutions().front()))
  {
    RCLCPP_ERROR(LOGGER, "Place execution failed");
    setStateError("place execution failed");
    setState(RobotTaskMode::HOLDING);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    held_object_.valid = false;
  }
  setState(RobotTaskMode::IDLE);
  RCLCPP_INFO(LOGGER, "Place finished successfully, state=IDLE");
  return true;
}

mtc::Task OrionMTCTaskNode::buildPickTask(double obj_x, double obj_y, double obj_z,
                                          const geometry_msgs::msg::Quaternion& object_orientation,
                                          const std::string& object_id)
{
  (void)object_id;  /* 场景中物体 id 固定为 "object"，object_id 仅用于 held_object_ 上报 */
  double approach_min = 0.10, approach_max = 0.15;
  double lift_min = 0.05, lift_max = 0.25;
  std::string support_surface_link;
  node_->get_parameter("approach_object_min_dist", approach_min);
  node_->get_parameter("approach_object_max_dist", approach_max);
  node_->get_parameter("lift_object_min_dist", lift_min);
  node_->get_parameter("lift_object_max_dist", lift_max);
  node_->get_parameter("support_surface_link", support_surface_link);

  mtc::Task task;
  task.stages()->setName("orion pick");
  task.loadRobotModel(node_);
  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "Link6";
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.stages()->properties().set("group", arm_group_name);
  task.stages()->properties().set("eef", hand_group_name);
  task.stages()->properties().set("ik_frame", hand_frame);

  task.add(std::make_unique<mtc::stages::CurrentState>("current"));
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "base_link";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { 0.1f, 0.02f };
    geometry_msgs::msg::Pose pose;
    pose.position.x = static_cast<float>(obj_x);
    pose.position.y = static_cast<float>(obj_y);
    pose.position.z = static_cast<float>(obj_z);
    pose.orientation = object_orientation;
    object.pose = pose;
    auto stage_add = std::make_unique<mtc::stages::ModifyPlanningScene>("add object");
    stage_add->addObject(object);
    task.add(std::move(stage_add));
  }

  auto ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  ptp_planner->setPlannerId("PTP");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  auto stage_ready = std::make_unique<mtc::stages::MoveTo>("move to ready", ptp_planner);
  stage_ready->setGroup(arm_group_name);
  stage_ready->setGoal("ready");
  task.add(std::move(stage_ready));

  auto stage_open = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open->setGroup(hand_group_name);
  stage_open->setGoal("open");
  task.add(std::move(stage_open));

  {
    std::vector<std::string> object_allowed_links =
        task.getRobotModel()->getJointModelGroup(arm_group_name)->getLinkModelNamesWithCollisionGeometry();
    std::vector<std::string> hand_links =
        task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry();
    object_allowed_links.insert(object_allowed_links.end(), hand_links.begin(), hand_links.end());
    if (std::find(object_allowed_links.begin(), object_allowed_links.end(), hand_frame) == object_allowed_links.end())
      object_allowed_links.push_back(hand_frame);
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (arm+hand,object)");
    stage->allowCollisions("object", object_allowed_links, true);
    task.add(std::move(stage));
  }

  /* PTP 到 pregrasp 时轨迹常触发 Link1/Link2-Link8、Link7/Link8-base_link 自碰，此处放宽 ACM 使规划通过 */
  {
    auto stage_acm = std::make_unique<mtc::stages::ModifyPlanningScene>("allow self-collision (pregrasp)");
    stage_acm->allowCollisions("Link1", std::vector<std::string>{ "Link8" }, true);
    stage_acm->allowCollisions("Link2", std::vector<std::string>{ "Link8" }, true);
    stage_acm->allowCollisions("Link7", std::vector<std::string>{ "base_link" }, true);
    stage_acm->allowCollisions("Link8", std::vector<std::string>{ "base_link" }, true);
    task.add(std::move(stage_acm));
  }

  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

  geometry_msgs::msg::PoseStamped pregrasp;
  pregrasp.header.frame_id = "base_link";
  pregrasp.pose.position.x = obj_x;
  pregrasp.pose.position.y = obj_y;
  pregrasp.pose.position.z = obj_z + approach_max;
  pregrasp.pose.orientation.x = 1.0;
  pregrasp.pose.orientation.y = 0.0;
  pregrasp.pose.orientation.z = 0.0;
  pregrasp.pose.orientation.w = 0.0;
  auto stage_pregrasp = std::make_unique<mtc::stages::MoveTo>("move to pregrasp", ptp_planner);
  stage_pregrasp->setGroup(arm_group_name);
  stage_pregrasp->setGoal(pregrasp);
  stage_pregrasp->setIKFrame(hand_frame);
  grasp->insert(std::move(stage_pregrasp));

  auto stage_approach = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
  stage_approach->properties().set("marker_ns", "approach_object");
  stage_approach->properties().set("link", hand_frame);
  stage_approach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  /* 允许笛卡尔路径达成度不足时仍接受（避免 min_fraction 卡死整个 pick） */
  stage_approach->properties().set("min_fraction", 0.55);
  stage_approach->setMinMaxDistance(static_cast<float>(approach_min), static_cast<float>(approach_max));
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = hand_frame;
  vec.vector.x = 0.0;
  vec.vector.y = 0.0;
  vec.vector.z = 1.0;
  stage_approach->setDirection(vec);
  grasp->insert(std::move(stage_approach));

  {
    std::vector<std::string> hand_and_wrist =
        task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry();
    hand_and_wrist.push_back(hand_frame);
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision before close");
    stage->allowCollisions("object", hand_and_wrist, true);
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("close");
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject("object", hand_frame);
    grasp->insert(std::move(stage));
  }

  if (!support_surface_link.empty())
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,support)");
    stage->allowCollisions("object", std::vector<std::string>{ support_surface_link }, true);
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(static_cast<float>(lift_min), static_cast<float>(lift_max));
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "lift_object");
    geometry_msgs::msg::Vector3Stamped v;
    v.header.frame_id = "base_link";
    v.vector.x = 0.0;
    v.vector.y = 0.0;
    v.vector.z = 1.0;
    stage->setDirection(v);
    grasp->insert(std::move(stage));
  }

  if (!support_surface_link.empty())
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,surface)");
    stage->allowCollisions("object", std::vector<std::string>{ support_surface_link }, false);
    grasp->insert(std::move(stage));
  }

  task.add(std::move(grasp));
  return task;
}

mtc::Task OrionMTCTaskNode::buildPlaceTask(double place_x, double place_y, double place_z,
                                           double place_qx, double place_qy, double place_qz, double place_qw)
{
  double retreat_min = 0.12, retreat_max = 0.25;
  double lower_min = 0.05, lower_max = 0.12;
  node_->get_parameter("retreat_min_dist", retreat_min);
  node_->get_parameter("retreat_max_dist", retreat_max);
  node_->get_parameter("lower_to_place_min_dist", lower_min);
  node_->get_parameter("lower_to_place_max_dist", lower_max);

  HeldObjectContext held;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!held_object_.valid)
    {
      RCLCPP_ERROR(LOGGER, "buildPlaceTask: no held object context");
      mtc::Task empty;
      empty.loadRobotModel(node_);
      return empty;
    }
    held = held_object_;
  }

  /* 放置目标为物体位姿；TCP 目标 = T_base_object_target * inv(T_tcp_object) */
  Eigen::Isometry3d T_base_obj_target = Eigen::Isometry3d::Identity();
  T_base_obj_target.translate(Eigen::Vector3d(place_x, place_y, place_z));
  T_base_obj_target.rotate(Eigen::Quaterniond(place_qw, place_qx, place_qy, place_qz));
  Eigen::Isometry3d T_base_tcp_target = T_base_obj_target * held.tcp_to_object.inverse();

  /* pre-place 高度：目标 z + lower_max，再 LIN 下降，避免直接怼到桌面 */
  Eigen::Vector3d pre_place_trans = T_base_tcp_target.translation();
  pre_place_trans.z() += lower_max;

  geometry_msgs::msg::Pose pre_place_pose;
  pre_place_pose.position.x = pre_place_trans.x();
  pre_place_pose.position.y = pre_place_trans.y();
  pre_place_pose.position.z = pre_place_trans.z();
  Eigen::Quaterniond q(T_base_tcp_target.rotation());
  pre_place_pose.orientation.x = q.x();
  pre_place_pose.orientation.y = q.y();
  pre_place_pose.orientation.z = q.z();
  pre_place_pose.orientation.w = q.w();

  mtc::Task task;
  task.stages()->setName("orion place");
  task.loadRobotModel(node_);
  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "Link6";
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  auto ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  ptp_planner->setPlannerId("PTP");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  geometry_msgs::msg::PoseStamped pre_place_stamped;
  pre_place_stamped.header.frame_id = "base_link";
  pre_place_stamped.pose = pre_place_pose;
  auto stage_pre = std::make_unique<mtc::stages::MoveTo>("move to pre-place", ptp_planner);
  stage_pre->setGroup(arm_group_name);
  stage_pre->setGoal(pre_place_stamped);
  stage_pre->setIKFrame(hand_frame);
  task.add(std::move(stage_pre));

  auto place = std::make_unique<mtc::SerialContainer>("place object");
  /* 根容器 property 继承不可靠，直接在 place 上设置供子阶段 PARENT 使用 */
  place->properties().set("eef", hand_group_name);
  place->properties().set("group", arm_group_name);
  place->properties().set("ik_frame", hand_frame);

  auto stage_lower = std::make_unique<mtc::stages::MoveRelative>("lower to place", cartesian_planner);
  stage_lower->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_lower->setMinMaxDistance(static_cast<float>(lower_min), static_cast<float>(lower_max));
  stage_lower->setIKFrame(hand_frame);
  stage_lower->properties().set("marker_ns", "lower_place");
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = hand_frame;
  vec.vector.x = 0.0;
  vec.vector.y = 0.0;
  vec.vector.z = -1.0;
  stage_lower->setDirection(vec);
  place->insert(std::move(stage_lower));

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("open");
    place->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage->detachObject("object", hand_frame);
    place->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(static_cast<float>(retreat_min), static_cast<float>(retreat_max));
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "retreat");
    geometry_msgs::msg::Vector3Stamped v;
    v.header.frame_id = hand_frame;
    v.vector.x = 0.0;
    v.vector.y = 0.0;
    v.vector.z = -1.0;
    stage->setDirection(v);
    place->insert(std::move(stage));
  }

  {
    std::vector<std::string> hand_and_wrist =
        task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry();
    hand_and_wrist.push_back(hand_frame);
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
    stage->allowCollisions("object", hand_and_wrist, false);
    place->insert(std::move(stage));
  }

  task.add(std::move(place));
  return task;
}

mtc::Task OrionMTCTaskNode::createTask()
{
  /* 从参数服务器读取 pick-place 参数（默认见 config/pick_place_params.yaml） */
  double approach_min = 0.10, approach_max = 0.15;
  double lift_min = 0.05, lift_max = 0.25;
  std::string support_surface_link;
  double place_x = 0.35, place_y = 0.15, place_z = 0.4;
  double place_qx = 0.0, place_qy = 0.0, place_qz = 0.0, place_qw = 1.0;
  double retreat_min = 0.12, retreat_max = 0.25;
  double lower_min = 0.05, lower_max = 0.12;
  double obj_x, obj_y, obj_z;

  node_->get_parameter("approach_object_min_dist", approach_min);
  node_->get_parameter("approach_object_max_dist", approach_max);
  node_->get_parameter("lift_object_min_dist", lift_min);
  node_->get_parameter("lift_object_max_dist", lift_max);
  node_->get_parameter("support_surface_link", support_surface_link);
  node_->get_parameter("place_pose_x", place_x);
  node_->get_parameter("place_pose_y", place_y);
  node_->get_parameter("place_pose_z", place_z);
  node_->get_parameter("place_pose_qx", place_qx);
  node_->get_parameter("place_pose_qy", place_qy);
  node_->get_parameter("place_pose_qz", place_qz);
  node_->get_parameter("place_pose_qw", place_qw);
  node_->get_parameter("retreat_min_dist", retreat_min);
  node_->get_parameter("retreat_max_dist", retreat_max);
  node_->get_parameter("lower_to_place_min_dist", lower_min);
  node_->get_parameter("lower_to_place_max_dist", lower_max);

  /* 放置位姿：优先来自 /place_pose 话题（动态放置），否则用参数 */
  {
    std::lock_guard<std::mutex> lock(place_pose_mutex_);
    if (has_place_pose_from_topic_)
    {
      place_x = place_pose_from_topic_.pose.position.x;
      place_y = place_pose_from_topic_.pose.position.y;
      place_z = place_pose_from_topic_.pose.position.z;
      place_qx = place_pose_from_topic_.pose.orientation.x;
      place_qy = place_pose_from_topic_.pose.orientation.y;
      place_qz = place_pose_from_topic_.pose.orientation.z;
      place_qw = place_pose_from_topic_.pose.orientation.w;
      RCLCPP_INFO(LOGGER, "createTask: place pose from topic (%.3f, %.3f, %.3f)", place_x, place_y, place_z);
    }
  }

  /* 物体位姿仅来自 /object_pose（doTask 已保证收到后才调用 createTask） */
  geometry_msgs::msg::Quaternion object_orientation;
  {
    std::lock_guard<std::mutex> lock(object_pose_mutex_);
    obj_x = object_pose_from_topic_.pose.position.x;
    obj_y = object_pose_from_topic_.pose.position.y;
    obj_z = object_pose_from_topic_.pose.position.z;
    object_orientation = object_pose_from_topic_.pose.orientation;
  }
  RCLCPP_INFO(LOGGER, "createTask: object pose from topic (%.3f, %.3f, %.3f)", obj_x, obj_y, obj_z);

  mtc::Task task;
  task.stages()->setName("orion pick place");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "Link6";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.stages()->properties().set("group", arm_group_name);
  task.stages()->properties().set("eef", hand_group_name);
  task.stages()->properties().set("ik_frame", hand_frame);

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));

  /* 在任务内将物体加入规划场景（位姿：来自 /object_pose 或参数 object_position_*） */
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "base_link";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { 0.1f, 0.02f };
    geometry_msgs::msg::Pose pose;
    pose.position.x = static_cast<float>(obj_x);
    pose.position.y = static_cast<float>(obj_y);
    pose.position.z = static_cast<float>(obj_z);
    pose.orientation = object_orientation;
    object.pose = pose;

    auto stage_add_object = std::make_unique<mtc::stages::ModifyPlanningScene>("add object");
    stage_add_object->addObject(object);
    task.add(std::move(stage_add_object));
  }

  /* 工业风格：PTP 到 pregrasp（最小关节运动、轨迹规矩）；Connect/return 也用 PTP */
  auto ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  ptp_planner->setPlannerId("PTP");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  /* 先从当前状态移到 ready，避免从全零/奇异位形直接 PTP 到 pregrasp 导致自碰（Link1-Link7 等） */
  auto stage_move_to_ready = std::make_unique<mtc::stages::MoveTo>("move to ready", ptp_planner);
  stage_move_to_ready->setGroup(arm_group_name);
  stage_move_to_ready->setGoal("ready");
  task.add(std::move(stage_move_to_ready));

  /* 官方骨架：… → OpenHand → … → Pick → Connect(preplace) → Place → ReturnHome */
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  /* 抓取阶段允许 object 与 arm+hand 碰撞，否则 MoveTo pregrasp/approach 路径易报 object-Link5 等碰撞导致规划失败 */
  {
    std::vector<std::string> object_allowed_links =
        task.getRobotModel()
            ->getJointModelGroup(arm_group_name)
            ->getLinkModelNamesWithCollisionGeometry();
    std::vector<std::string> hand_links =
        task.getRobotModel()
            ->getJointModelGroup(hand_group_name)
            ->getLinkModelNamesWithCollisionGeometry();
    object_allowed_links.insert(object_allowed_links.end(), hand_links.begin(), hand_links.end());
    if (std::find(object_allowed_links.begin(), object_allowed_links.end(), hand_frame) == object_allowed_links.end())
    {
      object_allowed_links.push_back(hand_frame);
    }
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (arm+hand,object)");
    stage->allowCollisions("object", object_allowed_links, true);
    task.add(std::move(stage));
  }

  {
    auto stage_acm =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow self-collision (pregrasp)");
    stage_acm->allowCollisions("Link1", std::vector<std::string>{ "Link8" }, true);
    stage_acm->allowCollisions("Link2", std::vector<std::string>{ "Link8" }, true);
    stage_acm->allowCollisions("Link7", std::vector<std::string>{ "base_link" }, true);
    stage_acm->allowCollisions("Link8", std::vector<std::string>{ "base_link" }, true);
    task.add(std::move(stage_acm));
  }

  mtc::Stage* attach_object_stage = nullptr;

  /* Pick 容器：PTP pregrasp → LIN approach → close → attach → lift */
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    /* 1) PTP 到 pregrasp（物体上方 approach_max），夹爪朝下；再 LIN approach 直线下压 → 工业风格 上方→下压 抓取 */
    {
      geometry_msgs::msg::PoseStamped pregrasp;
      pregrasp.header.frame_id = "base_link";
      pregrasp.pose.position.x = obj_x;
      pregrasp.pose.position.y = obj_y;
      pregrasp.pose.position.z = obj_z + approach_max;
      pregrasp.pose.orientation.x = 1.0;
      pregrasp.pose.orientation.y = 0.0;
      pregrasp.pose.orientation.z = 0.0;
      pregrasp.pose.orientation.w = 0.0;

      auto stage = std::make_unique<mtc::stages::MoveTo>("move to pregrasp", ptp_planner);
      stage->setGroup(arm_group_name);
      stage->setGoal(pregrasp);
      stage->setIKFrame(hand_frame);
      grasp->insert(std::move(stage));
    }

    /* 2) 笛卡尔 approach：沿 hand_frame(Link6) 的 z 接近物体 */
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->properties().set("min_fraction", 0.55);
      stage->setMinMaxDistance(static_cast<float>(approach_min), static_cast<float>(approach_max));
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.x = 0.0;
      vec.vector.y = 0.0;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    {
      std::vector<std::string> hand_and_wrist_links =
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry();
      hand_and_wrist_links.push_back(hand_frame);
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision before close");
      stage->allowCollisions("object", hand_and_wrist_links, true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    /* 有支撑面时：Lift 前允许 object 与支撑面碰撞（与 MTC demo 一致） */
    if (!support_surface_link.empty())
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,support)");
      stage->allowCollisions("object", std::vector<std::string>{ support_surface_link }, true);
      grasp->insert(std::move(stage));
    }

    /* 抬升：沿 base_link +Z，距离由参数 lift_object_* 指定 */
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(static_cast<float>(lift_min), static_cast<float>(lift_max));
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base_link";
      vec.vector.x = 0.0;
      vec.vector.y = 0.0;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /* 有支撑面时：Lift 后禁止 object 与支撑面碰撞 */
    if (!support_surface_link.empty())
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,surface)");
      stage->allowCollisions("object", std::vector<std::string>{ support_surface_link }, false);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  /* Connect 到预放置：PTP */
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "connect preplace",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, ptp_planner },
                                                  { hand_group_name, interpolation_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  /* Place 容器：PlaceIK → Lower(笛卡尔) → Open → Detach → Retreat(笛卡尔) */
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    place->properties().set("eef", hand_group_name);
    place->properties().set("group", arm_group_name);
    place->properties().set("ik_frame", hand_frame);

    /* 工业流程：preplace 位姿 = 放置点上方（place_z + lower_max），再 LIN 沿手爪 -Z 下降至放置高度，避免压进桌面 */
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "base_link";
      target_pose_msg.pose.position.x = place_x;
      target_pose_msg.pose.position.y = place_y;
      target_pose_msg.pose.position.z = place_z + lower_max;  /* preplace 高度 = 最终放置高度 + 下降段，避免 IK 直接解到桌面再 lower 压进去 */
      target_pose_msg.pose.orientation.x = 1.0;
      target_pose_msg.pose.orientation.y = 0.0;
      target_pose_msg.pose.orientation.z = 0.0;
      target_pose_msg.pose.orientation.w = 0.0;  /* 末端朝下，与抓取一致，避免 IK 腕部翻转/肘朝下 */

      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(1);   /* 只取 1 个解，与 pick 一致减少奇怪姿态 */
      wrapper->setMinSolutionDistance(1.0f);
      wrapper->setIgnoreCollisions(true);
      wrapper->setIKFrame(hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    /* 沿手爪 Z 轴下降（hand_frame -Z），工业流程 LIN lower；沿 TCP 下降避免斜着压 */
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lower to place", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(static_cast<float>(lower_min), static_cast<float>(lower_max));
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lower_place");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.x = 0.0;
      vec.vector.y = 0.0;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    /* 沿手爪坐标系 -z 退离，距离由参数 retreat_* 指定 */
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(static_cast<float>(retreat_min), static_cast<float>(retreat_max));
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.x = 0.0;
      vec.vector.y = 0.0;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    {
      std::vector<std::string> hand_and_wrist_links =
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry();
      hand_and_wrist_links.push_back(hand_frame);
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object", hand_and_wrist_links, false);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  /* 回 home：臂 PTP 到 ready，再夹爪闭合（Place 后夹爪为 open，回位后闭合） */
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", ptp_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand (return home)", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("close");
    task.add(std::move(stage));
  }

  return task;
}

rclcpp_action::GoalResponse OrionMTCTaskNode::handlePickGoalRequest(
    const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const PickAction::Goal> /*goal*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (task_mode_ == RobotTaskMode::HOLDING)
  {
    RCLCPP_INFO(LOGGER, "Pick goal rejected: already holding");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (task_mode_ != RobotTaskMode::IDLE && task_mode_ != RobotTaskMode::ERROR)
  {
    RCLCPP_INFO(LOGGER, "Pick goal rejected: busy");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OrionMTCTaskNode::handlePickGoalCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickAction>>& /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void OrionMTCTaskNode::handlePickGoalAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickAction>>& goal_handle)
{
  std::thread([this, goal_handle]() {
    const auto goal = goal_handle->get_goal();
    bool ok = doPickFromGoal(goal->object_pose, goal->object_id.empty() ? "object" : goal->object_id);
    auto result = std::make_shared<PickAction::Result>();
    result->success = ok;
    result->task_id = current_task_id_;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      result->held_object_id = ok ? held_object_.object_id : "";
      result->message = ok ? "pick success" : last_error_;
    }
    goal_handle->succeed(result);
  }).detach();
}

rclcpp_action::GoalResponse OrionMTCTaskNode::handlePlaceGoalRequest(
    const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const PlaceAction::Goal> /*goal*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (task_mode_ != RobotTaskMode::HOLDING || !held_object_.valid)
  {
    RCLCPP_INFO(LOGGER, "Place goal rejected: not holding");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OrionMTCTaskNode::handlePlaceGoalCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PlaceAction>>& /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void OrionMTCTaskNode::handlePlaceGoalAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PlaceAction>>& goal_handle)
{
  std::thread([this, goal_handle]() {
    const auto goal = goal_handle->get_goal();
    bool ok = doPlaceFromGoal(goal->target_pose);
    auto result = std::make_shared<PlaceAction::Result>();
    result->success = ok;
    result->task_id = current_task_id_;
    result->message = ok ? "place success" : last_error_;
    goal_handle->succeed(result);
  }).detach();
}

void OrionMTCTaskNode::handleGetRobotState(const std::shared_ptr<GetRobotStateSrv::Request> /*req*/,
                                           std::shared_ptr<GetRobotStateSrv::Response> res)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  switch (task_mode_)
  {
    case RobotTaskMode::IDLE:
      res->mode = "IDLE";
      break;
    case RobotTaskMode::PICKING:
      res->mode = "PICKING";
      break;
    case RobotTaskMode::HOLDING:
      res->mode = "HOLDING";
      break;
    case RobotTaskMode::PLACING:
      res->mode = "PLACING";
      break;
    case RobotTaskMode::ERROR:
      res->mode = "ERROR";
      break;
    default:
      res->mode = "UNKNOWN";
  }
  res->task_id = current_task_id_;
  res->held_object_id = held_object_.valid ? held_object_.object_id : "";
  res->has_held_object = held_object_.valid;
  res->last_error = last_error_;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<OrionMTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  /* 完全由话题驱动：不主动执行；先发 /object_pose，再发 /pick_place_trigger 触发一次 pick-place */
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}

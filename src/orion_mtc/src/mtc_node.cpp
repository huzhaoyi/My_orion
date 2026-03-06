/* MTC pick-and-place node for Orion robot */

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
#include <future>
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

class OrionMTCTaskNode
{
public:
  OrionMTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();
  /* 将 solution 转为 msg 后按 sub_trajectory 顺序本地执行，绕过 move_group 轨迹验证 */
  bool executeSolutionLocally(const mtc::SolutionBase& solution);
  /* 对 segment 应用 scene_diff（attach/detach 等），再向 execution_info 中的控制器发送轨迹 */
  bool executeSubTrajectory(const moveit_task_constructor_msgs::msg::SubTrajectory& sub);
  /* 向指定控制器发送 joint_trajectory，等待完成 */
  bool sendJointTrajectory(const std::string& controller_name,
                           const trajectory_msgs::msg::JointTrajectory& jt);

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  /* 仅将 action_client_node_ 加入 executor，避免与 Task 内部节点冲突导致 "already been added" */
  rclcpp::Node::SharedPtr action_client_node_;
  /* ApplyPlanningScene 客户端复用，避免每段重复创建 */
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_client_;
  /* FollowJointTrajectory action 客户端按 controller 名缓存，避免每次 send 都 create_client */
  std::unordered_map<std::string,
                     rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr>
      follow_jt_clients_;
  std::mutex follow_jt_mutex_;

  /* 物体位姿话题输入：有则 createTask 中优先使用，抓取仍由内部计算 */
  geometry_msgs::msg::PoseStamped object_pose_from_topic_;
  bool has_object_pose_from_topic_ = false;
  std::mutex object_pose_mutex_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_object_pose_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_pick_place_trigger_;
  std::atomic<bool> do_task_running_{ false };  /* 防止连续触发导致重入竞态 */

  void onObjectPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onPickPlaceTriggerReceived(const std_msgs::msg::Empty::SharedPtr msg);
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
  declare_if_not_set("approach_object_min_dist", 0.10);
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
  /* 在独立线程中执行，避免阻塞 executor */
  std::thread(&OrionMTCTaskNode::doTask, this).detach();
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

  for (size_t i = 0; i < solution_msg.sub_trajectory.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "Executing segment %zu / %zu", i + 1, solution_msg.sub_trajectory.size());
    if (!executeSubTrajectory(solution_msg.sub_trajectory[i]))
    {
      RCLCPP_ERROR(LOGGER, "executeSolutionLocally: segment %zu failed", i);
      return false;
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
  /* 完全由话题驱动：无 /object_pose 不执行 */
  {
    std::lock_guard<std::mutex> lock(object_pose_mutex_);
    if (!has_object_pose_from_topic_)
    {
      RCLCPP_WARN(LOGGER, "doTask: no object pose from topic yet, skip (publish /object_pose then /pick_place_trigger)");
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

  /* 工业风格：PTP 到 pregrasp（最小关节运动、轨迹规矩，避免 OMPL 关节空间绕圈导致掏档）；Connect/return 也用 PTP */
  auto ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  ptp_planner->setPlannerId("PTP");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  /* 官方骨架：CurrentState → OpenHand → Connect(pregrasp) → Pick → Connect(preplace) → Place → ReturnHome */
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
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

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

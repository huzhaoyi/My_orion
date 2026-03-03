/* MTC pick-and-place node for Orion robot */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
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

#include <algorithm>
#include <chrono>
#include <future>
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
};

OrionMTCTaskNode::OrionMTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("orion_mtc_node", options) }
  , action_client_node_{ std::make_shared<rclcpp::Node>("orion_mtc_action_client", options) }
{
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

  auto it = follow_jt_clients_.find(controller_name);
  if (it == follow_jt_clients_.end())
  {
    std::string action_name = "/" + controller_name + "/follow_joint_trajectory";
    auto client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        action_client_node_, action_name);
    if (!client->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(LOGGER, "sendJointTrajectory: action %s not available", action_name.c_str());
      return false;
    }
    it = follow_jt_clients_.emplace(controller_name, std::move(client)).first;
  }
  auto& client = it->second;
  auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
  goal_msg.trajectory = jt;
  goal_msg.trajectory.header.stamp = action_client_node_->now();
  auto goal_handle_future = client->async_send_goal(goal_msg);
  /* 节点已在其他线程的 executor 中 spin，仅等待 future 完成 */
  if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
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
      if (fut.wait_for(std::chrono::seconds(5)) == std::future_status::ready && fut.get()->success)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
    return;
  }

  moveit::core::MoveItErrorCode plan_result = task_.plan(5);
  if (!plan_result)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed (code " << plan_result.val << ")");
    /* 直接输出到 cout 并 flush，否则 MTC 的失败说明会因缓冲只在进程退出时打印 */
    task_.explainFailure(std::cout);
    std::cout.flush();
    return;
  }
  if (task_.solutions().empty())
  {
    RCLCPP_ERROR(LOGGER, "Task planning returned success but no solutions");
    return;
  }

  /* 本地执行：取 solution 消息并执行；内部会 publish 供 RViz 显示 */
  if (!executeSolutionLocally(*task_.solutions().front()))
  {
    RCLCPP_ERROR(LOGGER, "Local task execution failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Task execution finished successfully");
}

mtc::Task OrionMTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("orion pick place");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "Link6";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  /* 指向“物体已加入场景”之后的阶段，供 GenerateGraspPose 的 setMonitoredStage 使用，否则会从 current 取场景导致 object 不在场景 */
  mtc::Stage* stage_with_object_ptr = nullptr;

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));

  /* 在任务内将物体加入规划场景，确保 generate grasp pose 时 object 已在场景中（不依赖 /collision_object 同步） */
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "base_link";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { 0.1f, 0.02f };
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.35f;
    pose.position.y = -0.15f;
    pose.position.z = 0.4f;
    pose.orientation.w = 1.0f;
    object.pose = pose;

    auto stage_add_object = std::make_unique<mtc::stages::ModifyPlanningScene>("add object");
    stage_add_object->addObject(object);
    stage_with_object_ptr = stage_add_object.get();
    task.add(std::move(stage_add_object));
  }

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  {
    std::vector<std::string> hand_and_wrist_links =
        task.getRobotModel()
            ->getJointModelGroup(hand_group_name)
            ->getLinkModelNamesWithCollisionGeometry();
    hand_and_wrist_links.push_back(hand_frame);  // Link6 手腕，抓取时也可能接触物体
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    stage->allowCollisions("object", hand_and_wrist_links, true);
    task.add(std::move(stage));
  }

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage = nullptr;

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.0, 0.15);  // 最小 0 以通过；Achieved 0 时多为笛卡尔路径未生成

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 8.0);
      stage->setMonitoredStage(stage_with_object_ptr);

      // 抓取框：手系下 z 向上偏移 0.1，便于从上往下抓；单位旋转减少不可达姿态
      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      grasp_frame_transform.translation().z() = 0.1;

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(16);
      wrapper->setMinSolutionDistance(0.5);
      wrapper->setIgnoreCollisions(true);  // 抓取时手与物体允许接触，避免 Link7-object 碰撞导致无解
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
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

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base_link";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, interpolation_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "object";
      target_pose_msg.pose.position.y = 0.5;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIgnoreCollisions(true);  // 放置时夹持物体与放置面可能接触
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
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

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base_link";
      vec.vector.x = -0.5;
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

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
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
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}

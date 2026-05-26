/* app 层：装配 perception / decision / execution / scene / orchestration，接口委托 ManipulatorRosInterface */

#include "orion_mtc/app/orion_mtc_node.hpp"
#include "orion_mtc/config/mtc_config.hpp"
#include "orion_mtc/config/runtime_policy_loader.hpp"
#include "orion_mtc/decision/feasibility_checker.hpp"
#include "orion_mtc/decision/target_selector.hpp"
#include "orion_mtc/interface/manipulator_ros_interface.hpp"
#include "orion_mtc/perception/perception_snapshot_provider.hpp"
#include "orion_mtc/perception/pose_cache.hpp"
#include "orion_mtc/perception/target_cache.hpp"
#include "orion_mtc/perception/vector3_cache.hpp"
#include "orion_mtc/planning/collision_object_utils.hpp"
#include "orion_mtc/scene/planning_scene_manager.hpp"
#include "orion_mtc/execution/trajectory_executor.hpp"
#include "orion_mtc/execution/solution_executor.hpp"
#include "orion_mtc/orchestration/task_manager.hpp"
#include "orion_mtc/core/constants.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <chrono>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc");

namespace orion_mtc
{

namespace
{

visualization_msgs::msg::MarkerArray buildPanelObstacleMarkers(
    const rclcpp::Time& stamp,
    const std::vector<moveit_msgs::msg::CollisionObject>& collision_objects)
{
  visualization_msgs::msg::MarkerArray out;
  if (collision_objects.empty())
  {
    return out;
  }
  const std::string frame_id = collision_objects.front().header.frame_id.empty()
                                   ? "arm_base_link"
                                   : collision_objects.front().header.frame_id;
  visualization_msgs::msg::Marker delete_all;
  delete_all.header.frame_id = frame_id;
  delete_all.header.stamp = stamp;
  delete_all.ns = "panel_obstacles";
  delete_all.id = 0;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  out.markers.push_back(delete_all);

  int32_t marker_id = 1;
  for (const auto& co : collision_objects)
  {
    if (co.primitives.empty() || co.primitive_poses.empty())
    {
      continue;
    }
    const shape_msgs::msg::SolidPrimitive& prim = co.primitives.front();
    if (prim.type != shape_msgs::msg::SolidPrimitive::BOX || prim.dimensions.size() < 3u)
    {
      continue;
    }
    visualization_msgs::msg::Marker m;
    m.header.frame_id = co.header.frame_id.empty() ? frame_id : co.header.frame_id;
    m.header.stamp = stamp;
    m.ns = "panel_obstacles";
    m.id = marker_id++;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = co.primitive_poses.front();
    m.scale.x = prim.dimensions[0];
    m.scale.y = prim.dimensions[1];
    m.scale.z = prim.dimensions[2];
    m.color.r = 0.85f;
    m.color.g = 0.45f;
    m.color.b = 0.12f;
    m.color.a = 0.42f;
    out.markers.push_back(m);
  }
  return out;
}

/*
 * 将面板四角点从 src_frame 变换到 dst_frame；corners_xyz 为 x,y,z 交替共 12 个数。
 */
bool transformPanelCorners(const tf2_ros::Buffer& tf_buffer,
                           const std::string& src_frame,
                           const std::string& dst_frame,
                           const std::vector<double>& corners_xyz,
                           std::vector<double>& corners_out)
{
  if (corners_xyz.size() < 12u || src_frame.empty() || dst_frame.empty())
  {
    return false;
  }
  if (src_frame == dst_frame)
  {
    corners_out = corners_xyz;
    return true;
  }
  corners_out.clear();
  corners_out.reserve(12u);
  const tf2::Duration timeout = tf2::durationFromSec(2.0);
  for (size_t i = 0; i < 4u; ++i)
  {
    geometry_msgs::msg::PointStamped pin;
    pin.header.frame_id = src_frame;
    pin.header.stamp.sec = 0;
    pin.header.stamp.nanosec = 0;
    pin.point.x = corners_xyz[i * 3u + 0u];
    pin.point.y = corners_xyz[i * 3u + 1u];
    pin.point.z = corners_xyz[i * 3u + 2u];
    try
    {
      const geometry_msgs::msg::PointStamped pout = tf_buffer.transform(pin, dst_frame, timeout);
      corners_out.push_back(pout.point.x);
      corners_out.push_back(pout.point.y);
      corners_out.push_back(pout.point.z);
    }
    catch (const std::exception&)
    {
      return false;
    }
  }
  return corners_out.size() >= 12u;
}

}  // namespace

/*
 * 双节点构造：orion_mtc_node 承载参数与 FeasibilityChecker；action_client 承载订阅/服务/TF。
 * 顺序：declare/load 参数 → initModules（缓存、scene、executor、TaskManager、TF 回调）→ initInterfaces（ROS 注册）。
 */
OrionMTCNode::OrionMTCNode(const rclcpp::NodeOptions& options)
  : node_(std::make_shared<rclcpp::Node>("orion_mtc_node", options))
  , action_client_node_(std::make_shared<rclcpp::Node>("orion_mtc_action_client", options))
{
    declareParameters(node_.get());
    declareRuntimePolicyParameters(node_);
    loadFromNode(node_.get(), config_);
    loadRuntimePolicyFromNode(node_, runtime_policy_);
    initModules();
    initInterfaces();
}

/* 默认析构；模块均为 shared_ptr，不按逆序手动释放。 */
OrionMTCNode::~OrionMTCNode() = default;

/*
 * 创建感知缓存、PlanningSceneManager、Trajectory/SolutionExecutor、TaskManager；
 * 注入 wait_for_gripped（轮询 left_arm_gripped）、gripper_locked、latest pose/axis、外系 TF 变换与 FeasibilityChecker。
 */
void OrionMTCNode::initModules()
{
    object_pose_cache_ = std::make_shared<PoseCache>("");
    object_pose_fused_cache_ = std::make_shared<PoseCache>("");
    target_cache_ = std::make_shared<TargetCache>();
    object_axis_cache_ = std::make_shared<Vector3Cache>("");
    object_axis_fused_cache_ = std::make_shared<Vector3Cache>("");
    perception_snapshot_ = std::make_shared<PerceptionSnapshotProvider>(
        object_pose_cache_, target_cache_, action_client_node_->get_clock());
    target_selector_ = std::make_shared<TargetSelector>();
    action_client_reentrant_cb_group_ =
        action_client_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    scene_manager_ = std::make_shared<PlanningSceneManager>(action_client_node_,
                                                            action_client_reentrant_cb_group_);
    trajectory_executor_ =
        std::make_shared<TrajectoryExecutor>(action_client_node_, action_client_reentrant_cb_group_);
    solution_executor_ =
        std::make_shared<SolutionExecutor>(scene_manager_.get(), trajectory_executor_.get());

    double gripper_lock_threshold = 0.5;
    (void)node_->get_parameter("gripper_feedback.lock_threshold", gripper_lock_threshold);
    double gripper_unlock_threshold = 0.45;
    (void)node_->get_parameter("gripper_feedback.unlock_threshold", gripper_unlock_threshold);
    double gripper_wait_timeout_sec = 8.0;
    (void)node_->get_parameter("gripper_feedback.wait_timeout_sec", gripper_wait_timeout_sec);
    double gripper_settle_delay_sec = 1.0;
    (void)node_->get_parameter("gripper_feedback.settle_delay_sec", gripper_settle_delay_sec);
    double gripper_poll_hz = 20.0;
    (void)node_->get_parameter("gripper_feedback.poll_hz", gripper_poll_hz);
    int gripper_consecutive_samples = 3;
    (void)node_->get_parameter("gripper_feedback.consecutive_samples", gripper_consecutive_samples);
    const int effective_gripper_consecutive_samples = std::max(1, gripper_consecutive_samples);
    const double effective_gripper_poll_hz = (gripper_poll_hz > 1.0e-6) ? gripper_poll_hz : 20.0;
    const int sleep_ms = static_cast<int>(1000.0 / effective_gripper_poll_hz);

    RCLCPP_INFO(LOGGER,
                "gripper feedback params: lock=%.3f unlock=%.3f timeout=%.2fs settle=%.2fs poll=%.1fHz consecutive=%d",
                gripper_lock_threshold,
                gripper_unlock_threshold,
                gripper_wait_timeout_sec,
                gripper_settle_delay_sec,
                effective_gripper_poll_hz,
                effective_gripper_consecutive_samples);

    WaitForGrippedFn wait_fn = [this,
                                gripper_lock_threshold,
                                gripper_unlock_threshold,
                                gripper_wait_timeout_sec,
                                gripper_settle_delay_sec,
                                effective_gripper_poll_hz,
                                effective_gripper_consecutive_samples,
                                sleep_ms](bool expect_gripped, double timeout_sec) {
        const double threshold = expect_gripped ? gripper_lock_threshold : gripper_unlock_threshold;
        const double effective_timeout_sec = std::max(timeout_sec, gripper_wait_timeout_sec);
        const int total_ticks = std::max(1, static_cast<int>(effective_timeout_sec * effective_gripper_poll_hz));
        int satisfied_consecutive = 0;

        if (expect_gripped && gripper_settle_delay_sec > 1.0e-6)
        {
            RCLCPP_INFO(LOGGER, "waitForGripped: settle for %.2fs before lock check", gripper_settle_delay_sec);
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(gripper_settle_delay_sec * 1000.0)));
        }

        for (int i = 0; i < total_ticks; ++i)
        {
            double v = left_arm_gripped_.load();
            const bool sample_ok = expect_gripped ? (v >= threshold) : (v < threshold);
            if (sample_ok)
            {
                ++satisfied_consecutive;
            }
            else
            {
                satisfied_consecutive = 0;
            }

            if (satisfied_consecutive >= effective_gripper_consecutive_samples)
            {
                if (expect_gripped)
                {
                    RCLCPP_INFO(LOGGER,
                                "waitForGripped: gripped (%.3f >= %.3f), consecutive=%d",
                                v,
                                threshold,
                                effective_gripper_consecutive_samples);
                }
                else
                {
                    RCLCPP_INFO(LOGGER,
                                "waitForGripped: unlocked (%.3f < %.3f), consecutive=%d",
                                v,
                                threshold,
                                effective_gripper_consecutive_samples);
                }
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        RCLCPP_WARN(LOGGER,
                    "waitForGripped: timeout (expect_gripped=%d, threshold=%.3f, consecutive_need=%d, last=%.3f)",
                    expect_gripped,
                    threshold,
                    effective_gripper_consecutive_samples,
                    left_arm_gripped_.load());
        return false;
    };

    task_manager_ = std::make_shared<TaskManager>(
        node_, config_, scene_manager_.get(), trajectory_executor_.get(),
        solution_executor_.get(), std::move(wait_fn));
    task_manager_->setPolicy(runtime_policy_);
    task_manager_->setGripperLockedCallback([this, gripper_lock_threshold]() {
        return left_arm_gripped_.load() >= gripper_lock_threshold;
    });
    task_manager_->setGraspChainPerceptionCallbacks(
        [this]() { return object_pose_cache_->latest(); },
        [this]() { return object_axis_cache_->latest(); },
        [this]() { return object_pose_fused_cache_->latest(); },
        [this]() { return object_axis_fused_cache_->latest(); });
    task_manager_->setGetLatestTargetSetCallback(
        [this]() { return target_cache_ ? target_cache_->latest() : std::optional<orion_mtc_msgs::msg::TargetSet>(); });
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(action_client_node_->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, action_client_node_, true);

    rclcpp::NodeOptions manip_tf_opts;
    manip_tf_opts.arguments(
        { "--ros-args", "-r", "tf:=/manipulator/tf", "-r", "tf_static:=/manipulator/tf_static" });
    manipulator_tf_node_ = std::make_shared<rclcpp::Node>("orion_mtc_manip_tf", manip_tf_opts);
    manipulator_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(manipulator_tf_node_->get_clock());
    manipulator_tf_buffer_->setUsingDedicatedThread(true);
    manipulator_tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*manipulator_tf_buffer_, manipulator_tf_node_, true);

    task_manager_->setTransformToArmBaseLinkCallback(
        [this](geometry_msgs::msg::PoseStamped& pose, geometry_msgs::msg::Vector3Stamped* axis) {
            const std::string& tf_target = config_.manipulator_frame.tf_target_frame;
            const std::string& plan_frame = config_.manipulator_frame.planning_frame_id;
            const std::string src_frame = pose.header.frame_id;

            if (src_frame == plan_frame)
            {
                return true;
            }
            /* 旧接口/Web 曾用 base_link 表示臂根；location 的 /tf 上 base_link 为 ROV，禁止误变换。 */
            if (src_frame == "base_link")
            {
                RCLCPP_WARN_THROTTLE(
                    LOGGER,
                    *action_client_node_->get_clock(),
                    30000,
                    "Manipulator pose frame_id=base_link is legacy (planning frame is %s; colleague ROV uses "
                    "base_link on /tf). Remapping to %s without TF — use arm_base_link or %s explicitly.",
                    plan_frame.c_str(),
                    plan_frame.c_str(),
                    tf_target.c_str());
                pose.header.frame_id = plan_frame;
                if (axis != nullptr)
                {
                    axis->header.frame_id = plan_frame;
                }
                return true;
            }
            if (src_frame == tf_target)
            {
                pose.header.frame_id = plan_frame;
                if (axis != nullptr)
                {
                    axis->header.frame_id = plan_frame;
                }
                RCLCPP_INFO(LOGGER,
                            "transform: %s 与 %s 同安装点，仅改 frame_id -> %s pos=(%.4f, %.4f, %.4f)",
                            tf_target.c_str(),
                            plan_frame.c_str(),
                            plan_frame.c_str(),
                            pose.pose.position.x,
                            pose.pose.position.y,
                            pose.pose.position.z);
                return true;
            }

            const std::string lookup_target =
                tf_target.empty() ? plan_frame : tf_target;

            auto apply_transform = [&](const geometry_msgs::msg::TransformStamped& T, bool used_latest) {
                geometry_msgs::msg::PoseStamped out;
                tf2::doTransform(pose, out, T);
                pose = out;
                pose.header.frame_id = plan_frame;
                if (axis != nullptr)
                {
                    geometry_msgs::msg::Vector3Stamped axis_out;
                    tf2::doTransform(*axis, axis_out, T);
                    axis->vector = axis_out.vector;
                    axis->header.frame_id = plan_frame;
                }
                RCLCPP_INFO(LOGGER,
                            "transform: %s -> %s (TF 目标=%s%s) pos=(%.4f, %.4f, %.4f)",
                            src_frame.c_str(),
                            plan_frame.c_str(),
                            lookup_target.c_str(),
                            used_latest ? ", latest" : "",
                            pose.pose.position.x,
                            pose.pose.position.y,
                            pose.pose.position.z);
            };

            try
            {
                rclcpp::Time t(pose.header.stamp.sec, pose.header.stamp.nanosec);
                geometry_msgs::msg::TransformStamped T =
                    tf_buffer_->lookupTransform(lookup_target, src_frame, t);
                apply_transform(T, false);
                return true;
            }
            catch (const std::exception& e)
            {
                try
                {
                    geometry_msgs::msg::TransformStamped T_latest =
                        tf_buffer_->lookupTransform(lookup_target, src_frame, tf2::TimePointZero);
                    apply_transform(T_latest, true);
                    RCLCPP_WARN(LOGGER,
                                "transform %s -> %s fallback to latest TF: %s",
                                src_frame.c_str(),
                                plan_frame.c_str(),
                                e.what());
                    return true;
                }
                catch (const std::exception& e2)
                {
                    RCLCPP_WARN(LOGGER,
                                "transform %s -> %s (via %s) failed: %s; fallback failed: %s",
                                src_frame.c_str(),
                                plan_frame.c_str(),
                                lookup_target.c_str(),
                                e.what(),
                                e2.what());
                    return false;
                }
            }
        });
    feasibility_checker_ = std::make_shared<FeasibilityChecker>(node_);
    feasibility_checker_->setMTCConfig(&config_);
    task_manager_->setFeasibilityChecker(feasibility_checker_.get());
    feasibility_checker_->setEmergencyStopPredicate(
        [this]() { return task_manager_->isEmergencyStopRequested(); });
}

/* 装配 ManipulatorInterfaceContext 并注册订阅/服务与状态发布。 */
void OrionMTCNode::initInterfaces()
{
    ManipulatorInterfaceContext ctx{ LOGGER,
                                     action_client_node_,
                                     task_manager_,
                                     feasibility_checker_,
                                     object_pose_cache_,
                                     target_cache_,
                                     object_axis_cache_,
                                     object_pose_fused_cache_,
                                     object_axis_fused_cache_,
                                     perception_snapshot_,
                                     target_selector_,
                                     &left_arm_gripped_,
                                     tf_buffer_,
                                     manipulator_tf_buffer_ };

    manipulator_iface_ = std::make_unique<ManipulatorRosInterface>(ctx);
    manipulator_iface_->registerSubscriptionsAndServices();
    manipulator_iface_->registerStatusPublishersAndCallbacks();
}

/* MultiThreadedExecutor：action_client 承载订阅/服务/TF；orion_mtc_node 承载 FeasibilityChecker 的 joint_states 等。 */
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr OrionMTCNode::getNodeBaseInterface()
{
    return action_client_node_->get_node_base_interface();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr OrionMTCNode::getPlanningNodeBaseInterface()
{
    return node_->get_node_base_interface();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr OrionMTCNode::getManipulatorTfNodeBaseInterface()
{
    return manipulator_tf_node_ ? manipulator_tf_node_->get_node_base_interface() : nullptr;
}

/*
 * 面板四角从 poc.frame_id（map/odom）变换到 planning_frame（arm_base_link）后写入 MoveIt。
 * 优先 /manipulator/tf（map→world→arm_base_link）；odom 时回退 /tf→sensor_left 再按同安装点写入臂系。
 */
bool OrionMTCNode::tryApplyPanelObstacles()
{
    const PanelObstaclesConfig& poc = config_.panel_obstacles;
    if (!poc.enable || poc.panels.empty() || !scene_manager_)
    {
        return false;
    }

    const std::string& plan_frame = config_.manipulator_frame.planning_frame_id;
    const std::string& tf_target = config_.manipulator_frame.tf_target_frame;

    if (manipulator_tf_buffer_ && (poc.frame_id == "map" || poc.frame_id == "world"))
    {
        std::string tf_err;
        const bool have_tf = manipulator_tf_buffer_->canTransform(plan_frame,
                                                                  poc.frame_id,
                                                                  tf2::TimePointZero,
                                                                  tf2::durationFromSec(8.0),
                                                                  &tf_err);
        if (!have_tf)
        {
            RCLCPP_WARN(LOGGER,
                        "panel_obstacles: manipulator TF %s<-%s not ready within 8s: %s "
                        "(pick_holoocean 需 static map→world 与 world→arm_base_link)",
                        plan_frame.c_str(),
                        poc.frame_id.c_str(),
                        tf_err.c_str());
        }
    }
    else if (tf_buffer_ && poc.frame_id == "odom" && !tf_target.empty())
    {
        std::string tf_err;
        const bool have_tf = tf_buffer_->canTransform(tf_target,
                                                    poc.frame_id,
                                                    tf2::TimePointZero,
                                                    tf2::durationFromSec(8.0),
                                                    &tf_err);
        if (!have_tf)
        {
            RCLCPP_WARN(LOGGER,
                        "panel_obstacles: global TF %s<-%s not ready within 8s: %s "
                        "(需 sealien_ctrlpilot_location)",
                        tf_target.c_str(),
                        poc.frame_id.c_str(),
                        tf_err.c_str());
        }
    }

    std::vector<moveit_msgs::msg::CollisionObject> objs;
    objs.reserve(poc.panels.size());
    for (const auto& panel : poc.panels)
    {
        if (panel.id.empty() || panel.corners_xyz.size() < 12u)
        {
            RCLCPP_WARN(LOGGER,
                        "panel_obstacles: skip panel id='%s' corners size=%zu (need 12 floats)",
                        panel.id.c_str(),
                        panel.corners_xyz.size());
            continue;
        }

        std::vector<double> corners_plan;
        bool transformed = false;
        if (manipulator_tf_buffer_ &&
            (poc.frame_id == "map" || poc.frame_id == "world" || poc.frame_id == plan_frame))
        {
            transformed = transformPanelCorners(*manipulator_tf_buffer_,
                                                poc.frame_id,
                                                plan_frame,
                                                panel.corners_xyz,
                                                corners_plan);
        }
        if (!transformed && tf_buffer_ && poc.frame_id == "odom" && !tf_target.empty())
        {
            std::vector<double> corners_sensor;
            if (transformPanelCorners(*tf_buffer_, poc.frame_id, tf_target, panel.corners_xyz, corners_sensor))
            {
                corners_plan = std::move(corners_sensor);
                transformed = true;
            }
        }

        const std::string& co_frame = transformed ? plan_frame : poc.frame_id;
        const std::vector<double>& co_corners = transformed ? corners_plan : panel.corners_xyz;
        if (!transformed)
        {
            RCLCPP_WARN(LOGGER,
                        "panel_obstacles: panel '%s' corners not transformed (%s -> %s), "
                        "MoveIt may reject frame",
                        panel.id.c_str(),
                        poc.frame_id.c_str(),
                        plan_frame.c_str());
        }

        moveit_msgs::msg::CollisionObject co = makePanelBoxCollisionObject(panel.id,
                                                                           co_frame,
                                                                           co_corners,
                                                                           poc.unit_scale,
                                                                           poc.wall_thickness_m,
                                                                           poc.aabb_margin_m,
                                                                           moveit_msgs::msg::CollisionObject::ADD);
        co.header.stamp = action_client_node_->now();
        objs.push_back(std::move(co));
    }

    if (objs.empty())
    {
        return false;
    }

    const bool sync_ok = scene_manager_->syncWorldCollisionObjects(objs, false);
    if (!sync_ok)
    {
        RCLCPP_WARN(LOGGER,
                    "panel_obstacles: syncWorldCollisionObjects failed; planning scene unchanged");
        return false;
    }

    if (poc.publish_markers)
    {
        if (!panel_obstacles_markers_pub_)
        {
            auto qos = rclcpp::QoS(1).reliable().transient_local();
            panel_obstacles_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
                PANEL_OBSTACLES_MARKERS_TOPIC, qos);
        }
        const visualization_msgs::msg::MarkerArray markers = buildPanelObstacleMarkers(node_->now(), objs);
        panel_obstacles_markers_pub_->publish(markers);
    }

    RCLCPP_INFO(LOGGER,
                "panel_obstacles: applied %zu box(es) in frame=%s (src=%s) publish_markers=%s",
                objs.size(),
                plan_frame.c_str(),
                poc.frame_id.c_str(),
                poc.publish_markers ? "true" : "false");
    return true;
}

/*
 * 启动后调用：可选将 panel_obstacles 写入 PlanningScene 并发布 MarkerArray；若策略 auto_start_worker 为真则起 Worker。
 */
void OrionMTCNode::setupPlanningScene()
{
    const PanelObstaclesConfig& poc = config_.panel_obstacles;
    if (poc.enable && !poc.panels.empty())
    {
        if (!tryApplyPanelObstacles())
        {
            panel_obstacles_retry_timer_ = node_->create_wall_timer(
                std::chrono::seconds(5),
                [this]() {
                    if (tryApplyPanelObstacles())
                    {
                        panel_obstacles_retry_timer_->cancel();
                        panel_obstacles_retry_timer_.reset();
                    }
                });
            RCLCPP_INFO(LOGGER,
                        "panel_obstacles: initial sync failed or TF not ready; will retry every 5s");
        }
    }
    else
    {
        RCLCPP_INFO(LOGGER,
                    "setupPlanningScene: panel_obstacles disabled or empty (object still added in-task)");
    }
    if (task_manager_->getPolicy().auto_start_worker)
    {
        task_manager_->startWorker();
    }
}

}  // namespace orion_mtc

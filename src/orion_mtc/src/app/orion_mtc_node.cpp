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
#include "orion_mtc/scene/planning_scene_manager.hpp"
#include "orion_mtc/execution/trajectory_executor.hpp"
#include "orion_mtc/execution/solution_executor.hpp"
#include "orion_mtc/orchestration/task_manager.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc");

namespace orion_mtc
{

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

OrionMTCNode::~OrionMTCNode() = default;

void OrionMTCNode::initModules()
{
    object_pose_cache_ = std::make_shared<PoseCache>("");
    target_cache_ = std::make_shared<TargetCache>();
    object_axis_cache_ = std::make_shared<Vector3Cache>("");
    perception_snapshot_ = std::make_shared<PerceptionSnapshotProvider>(
        object_pose_cache_, target_cache_, action_client_node_->get_clock());
    target_selector_ = std::make_shared<TargetSelector>();
    scene_manager_ = std::make_shared<PlanningSceneManager>(action_client_node_.get());
    trajectory_executor_ = std::make_shared<TrajectoryExecutor>(action_client_node_.get());
    solution_executor_ =
        std::make_shared<SolutionExecutor>(scene_manager_.get(), trajectory_executor_.get());

    WaitForGrippedFn wait_fn = [this](bool expect_gripped, double timeout_sec) {
        const double threshold = 0.5;
        const int total_ticks = static_cast<int>(timeout_sec * 20.0);
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
    };

    task_manager_ = std::make_shared<TaskManager>(
        node_, config_, scene_manager_.get(), trajectory_executor_.get(),
        solution_executor_.get(), std::move(wait_fn));
    task_manager_->setPolicy(runtime_policy_);
    task_manager_->setGripperLockedCallback([this]() {
        const double threshold = 0.5;
        return left_arm_gripped_.load() >= threshold;
    });
    task_manager_->setGetLatestObjectPoseCallback([this]() { return object_pose_cache_->latest(); });
    task_manager_->setGetLatestObjectAxisCallback([this]() { return object_axis_cache_->latest(); });
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(action_client_node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, action_client_node_, false);
    task_manager_->setTransformToBaseLinkCallback(
        [this](geometry_msgs::msg::PoseStamped& pose, geometry_msgs::msg::Vector3Stamped* axis) {
            try
            {
                rclcpp::Time t(pose.header.stamp.sec, pose.header.stamp.nanosec);
                geometry_msgs::msg::TransformStamped T =
                    tf_buffer_->lookupTransform("base_link", pose.header.frame_id, t);
                geometry_msgs::msg::PoseStamped out;
                tf2::doTransform(pose, out, T);
                pose = out;
                pose.header.frame_id = "base_link";
                if (axis != nullptr)
                {
                    geometry_msgs::msg::Vector3Stamped axis_out;
                    tf2::doTransform(*axis, axis_out, T);
                    axis->vector = axis_out.vector;
                    axis->header.frame_id = "base_link";
                }
                return true;
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(LOGGER, "transform to base_link failed: %s", e.what());
                return false;
            }
        });
    feasibility_checker_ = std::make_shared<FeasibilityChecker>(node_);
    feasibility_checker_->setMTCConfig(&config_);
}

void OrionMTCNode::initInterfaces()
{
    ManipulatorInterfaceContext ctx{ LOGGER,
                                     action_client_node_,
                                     task_manager_,
                                     feasibility_checker_,
                                     object_pose_cache_,
                                     target_cache_,
                                     object_axis_cache_,
                                     perception_snapshot_,
                                     target_selector_,
                                     &left_arm_gripped_ };

    manipulator_iface_ = std::make_unique<ManipulatorRosInterface>(ctx);
    manipulator_iface_->registerSubscriptionsAndServices();
    manipulator_iface_->registerStatusPublishersAndCallbacks();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr OrionMTCNode::getNodeBaseInterface()
{
    return action_client_node_->get_node_base_interface();
}

void OrionMTCNode::setupPlanningScene()
{
    RCLCPP_INFO(LOGGER, "setupPlanningScene: object added in-task (add object stage), no /collision_object publish");
    if (task_manager_->getPolicy().auto_start_worker)
    {
        task_manager_->startWorker();
    }
}

}  // namespace orion_mtc

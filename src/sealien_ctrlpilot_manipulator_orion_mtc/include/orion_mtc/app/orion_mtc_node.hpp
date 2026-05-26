/* app 层：参数声明、模块装配、生命周期；ROS 话题/服务注册在 interface/ManipulatorRosInterface */

#ifndef ORION_MTC_APP_ORION_MTC_NODE_HPP
#define ORION_MTC_APP_ORION_MTC_NODE_HPP

#include "orion_mtc/config/mtc_config.hpp"
#include "orion_mtc/config/runtime_policy.hpp"
#include "orion_mtc/core/constants.hpp"
#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <atomic>
#include <memory>
#include <string>

namespace orion_mtc
{
class PoseCache;
class TargetCache;
class Vector3Cache;
class PerceptionSnapshotProvider;
class TargetSelector;
class PlanningSceneManager;
class TrajectoryExecutor;
class SolutionExecutor;
class TaskManager;
class FeasibilityChecker;
class ManipulatorRosInterface;
}

namespace orion_mtc
{

class OrionMTCNode
{
public:
    explicit OrionMTCNode(const rclcpp::NodeOptions& options);
    ~OrionMTCNode();
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getPlanningNodeBaseInterface();
    /** 订阅 /manipulator/tf，供面板与 TCP 查询 arm_base_link */
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getManipulatorTfNodeBaseInterface();
    void setupPlanningScene();

private:
    void initModules();
    void initInterfaces();
    bool tryApplyPanelObstacles();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Node::SharedPtr action_client_node_;
    rclcpp::CallbackGroup::SharedPtr action_client_reentrant_cb_group_;
    MTCConfig config_;
    RuntimePolicy runtime_policy_;
    std::shared_ptr<PoseCache> object_pose_cache_;
    std::shared_ptr<PoseCache> object_pose_fused_cache_;
    std::shared_ptr<TargetCache> target_cache_;
    std::shared_ptr<Vector3Cache> object_axis_cache_;
    std::shared_ptr<Vector3Cache> object_axis_fused_cache_;
    std::shared_ptr<PerceptionSnapshotProvider> perception_snapshot_;
    std::shared_ptr<TargetSelector> target_selector_;
    std::shared_ptr<PlanningSceneManager> scene_manager_;
    std::shared_ptr<TrajectoryExecutor> trajectory_executor_;
    std::shared_ptr<SolutionExecutor> solution_executor_;
    std::shared_ptr<TaskManager> task_manager_;
    std::shared_ptr<FeasibilityChecker> feasibility_checker_;
    /** /tf：同事 odom→base_link(ROV)→sensor_left，Action 外参 */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Node::SharedPtr manipulator_tf_node_;
    std::shared_ptr<tf2_ros::Buffer> manipulator_tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> manipulator_tf_listener_;
    rclcpp::TimerBase::SharedPtr panel_obstacles_retry_timer_;

    std::atomic<double> left_arm_gripped_{ 0.0 };

    std::unique_ptr<ManipulatorRosInterface> manipulator_iface_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr panel_obstacles_markers_pub_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_APP_ORION_MTC_NODE_HPP

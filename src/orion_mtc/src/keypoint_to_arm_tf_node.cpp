/* 订阅 Keypoints（与 cable_detect 等发布端一致：sealien_ctrlpilot_msgmanagement/msg/Keypoints），
 * 或将 use_mock_keypoints 打开，用定时器注入假数据做离线 TF 测试。 */

#include <geometry_msgs/msg/point_stamped.hpp>
#include <sealien_ctrlpilot_msgmanagement/msg/keypoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <chrono>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cstddef>
#include <memory>
#include <string>

namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("keypoint_to_arm_tf");
}

class KeypointToArmTfNode final : public rclcpp::Node
{
public:
    static std::shared_ptr<KeypointToArmTfNode> create()
    {
        auto node = std::shared_ptr<KeypointToArmTfNode>(new KeypointToArmTfNode());
        node->tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(node->tf_buffer_, node, false);
        return node;
    }

private:
    KeypointToArmTfNode()
      : rclcpp::Node("keypoint_to_arm_tf")
      , tf_buffer_(get_clock())
    {
        input_topic_ = declare_parameter<std::string>("input_topic", "/keypoints");
        /* header.frame_id 为空时使用 */
        source_frame_override_ = declare_parameter<std::string>("source_frame_override", "sensor_link");
        left_arm_frame_ = declare_parameter<std::string>("left_arm_frame", "left_arm_base");
        right_arm_frame_ = declare_parameter<std::string>("right_arm_frame", "right_arm_base");
        tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.5);
        /* 非空则始终用此帧，忽略 header.frame_id（用于强制 sensor_link 等调试） */
        force_source_frame_ = declare_parameter<std::string>("force_source_frame", "");
        /* 静态 TF / 与仿真时钟不一致时，应用消息时间戳易导致 TF 查询失败；true 时用最新可用变换 */
        tf_use_latest_timestamp_ = declare_parameter<bool>("tf_use_latest_timestamp", true);
        /* cable_detect 等为 Reliable；与发布端一致时设 false */
        qos_best_effort_ = declare_parameter<bool>("qos_best_effort", false);
        qos_depth_ = declare_parameter<int>("qos_depth", 10);
        if (qos_depth_ < 1)
        {
            qos_depth_ = 1;
        }

        rclcpp::QoS qos(static_cast<size_t>(qos_depth_));
        if (qos_best_effort_)
        {
            qos.best_effort();
        }
        else
        {
            qos.reliable();
        }
        qos.durability_volatile();

        use_mock_keypoints_ = declare_parameter<bool>("use_mock_keypoints", false);
        mock_frame_id_ = declare_parameter<std::string>("mock_frame_id", "camera");
        mock_kp_x_ = declare_parameter<double>("mock_kp_x", 0.0);
        mock_kp_y_ = declare_parameter<double>("mock_kp_y", 2.9054482685810803);
        mock_kp_z_ = declare_parameter<double>("mock_kp_z", -9.536743164059724e-05);
        mock_period_sec_ = declare_parameter<double>("mock_period_sec", 1.0);
        if (mock_period_sec_ < 0.05)
        {
            mock_period_sec_ = 0.05;
        }

        if (use_mock_keypoints_)
        {
            mock_timer_ = create_wall_timer(
                std::chrono::duration<double>(mock_period_sec_),
                std::bind(&KeypointToArmTfNode::onMockTimer, this));
            RCLCPP_WARN(LOGGER,
                        "use_mock_keypoints=true: NOT subscribing; injecting mock Keypoints every %.3fs "
                        "(frame=%s pt=(%.6f,%.6f,%.6f))",
                        mock_period_sec_,
                        mock_frame_id_.c_str(),
                        mock_kp_x_,
                        mock_kp_y_,
                        mock_kp_z_);
        }
        else
        {
            sub_ = create_subscription<sealien_ctrlpilot_msgmanagement::msg::Keypoints>(
                input_topic_,
                qos,
                std::bind(&KeypointToArmTfNode::onKeypoints, this, std::placeholders::_1));
        }

        RCLCPP_INFO(LOGGER,
                    "keypoint_to_arm_tf: mock=%s sub=%s qos=%s depth=%d tf_latest_stamp=%s empty_frame_fallback=%s "
                    "force_frame=%s -> [%s | %s]",
                    use_mock_keypoints_ ? "on" : "off",
                    input_topic_.c_str(),
                    qos_best_effort_ ? "best_effort" : "reliable",
                    qos_depth_,
                    tf_use_latest_timestamp_ ? "true" : "false",
                    source_frame_override_.c_str(),
                    force_source_frame_.empty() ? "(use msg header.frame_id)" : force_source_frame_.c_str(),
                    left_arm_frame_.c_str(),
                    right_arm_frame_.c_str());
    }

    void onMockTimer()
    {
        auto msg = std::make_shared<sealien_ctrlpilot_msgmanagement::msg::Keypoints>();
        msg->header.stamp = now();
        msg->header.frame_id = mock_frame_id_;
        msg->has_target = false;
        msg->corner_points[0].x = 0.0f;
        msg->corner_points[0].y = 0.0f;
        msg->corner_points[0].z = 0.0f;
        msg->corner_points[1].x = 0.0f;
        msg->corner_points[1].y = 0.0f;
        msg->corner_points[1].z = 0.0f;
        msg->keypoints.resize(1);
        msg->keypoints[0].x = static_cast<float>(mock_kp_x_);
        msg->keypoints[0].y = static_cast<float>(mock_kp_y_);
        msg->keypoints[0].z = static_cast<float>(mock_kp_z_);
        msg->yaw_degree = 0.0f;
        msg->score = 0.0f;
        msg->score_threshold = 0.0f;
        msg->use_score_threshold = false;
        msg->is_available = false;
        processKeypoints(msg);
    }

    void onKeypoints(const sealien_ctrlpilot_msgmanagement::msg::Keypoints::SharedPtr msg)
    {
        processKeypoints(msg);
    }

    void processKeypoints(const sealien_ctrlpilot_msgmanagement::msg::Keypoints::SharedPtr msg)
    {
        if (!received_keypoints_once_)
        {
            received_keypoints_once_ = true;
            RCLCPP_INFO(LOGGER, "first Keypoints processed (keypoints.size=%zu)", msg->keypoints.size());
        }

        if (msg->keypoints.empty())
        {
            RCLCPP_WARN_THROTTLE(LOGGER, *get_clock(), 5000, "keypoints array empty, skip");
            return;
        }

        std::string frame_id;
        if (!force_source_frame_.empty())
        {
            frame_id = force_source_frame_;
        }
        else
        {
            frame_id = msg->header.frame_id;
            if (frame_id.empty())
            {
                frame_id = source_frame_override_;
            }
        }

        const tf2::Duration timeout = tf2::durationFromSec(tf_timeout_sec_);

        for (size_t i = 0; i < msg->keypoints.size(); ++i)
        {
            geometry_msgs::msg::PointStamped pt_in;
            if (tf_use_latest_timestamp_)
            {
                const rclcpp::Time zero(0, 0, get_clock()->get_clock_type());
                pt_in.header.stamp = zero;
            }
            else
            {
                pt_in.header.stamp = msg->header.stamp;
            }
            pt_in.header.frame_id = frame_id;
            pt_in.point = msg->keypoints[i];

            try
            {
                geometry_msgs::msg::PointStamped left_out =
                    tf_buffer_.transform(pt_in, left_arm_frame_, timeout);
                geometry_msgs::msg::PointStamped right_out =
                    tf_buffer_.transform(pt_in, right_arm_frame_, timeout);

                RCLCPP_INFO(LOGGER,
                            "keypoint[%zu] in '%s': (%.6f, %.6f, %.6f)",
                            i,
                            frame_id.c_str(),
                            pt_in.point.x,
                            pt_in.point.y,
                            pt_in.point.z);
                RCLCPP_INFO(LOGGER,
                            "  -> '%s': (%.6f, %.6f, %.6f)",
                            left_arm_frame_.c_str(),
                            left_out.point.x,
                            left_out.point.y,
                            left_out.point.z);
                RCLCPP_INFO(LOGGER,
                            "  -> '%s': (%.6f, %.6f, %.6f)",
                            right_arm_frame_.c_str(),
                            right_out.point.x,
                            right_out.point.y,
                            right_out.point.z);
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(LOGGER,
                            "TF failed for keypoint[%zu]: %s",
                            i,
                            ex.what());
            }
        }
    }

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sealien_ctrlpilot_msgmanagement::msg::Keypoints>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr mock_timer_;

    std::string input_topic_;
    std::string source_frame_override_;
    std::string force_source_frame_;
    std::string left_arm_frame_;
    std::string right_arm_frame_;
    double tf_timeout_sec_;
    bool tf_use_latest_timestamp_{true};
    bool qos_best_effort_{false};
    int qos_depth_{10};
    bool received_keypoints_once_{false};
    bool use_mock_keypoints_{false};
    std::string mock_frame_id_;
    double mock_kp_x_{0.0};
    double mock_kp_y_{0.0};
    double mock_kp_z_{0.0};
    double mock_period_sec_{1.0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(KeypointToArmTfNode::create());
    rclcpp::shutdown();
    return 0;
}

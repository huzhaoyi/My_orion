/* 订阅 sealien_ctrlpilot_msgmanagement/msg/Keypoints，将关键点与消息内电缆姿态（directions / euler_angles）
 * 经 TF 变到 left_arm_frame ，发布 /manipulator/object_pose_fused 与 object_axis_fused 供 MTC/UI；
 * 另可选发布 keypoints_posearray_topic（geometry_msgs/PoseArray）：各关键点已变换到 output_grasp_frame，供 Web 3D 与感知卡片。
 * 位置：mean | keypoint_index | centerline_arclength（全局 PCA 排序→可选去重→可选沿折线窗口 3D 均值→弧长插值；无分量中值、无局部 PCA 切向）；
 * 姿态：读 directions 侧向叉乘系，或 euler_angles（度 ZYX）再乘源系→left_arm；auto 时 directions 优先。
 * fused_grasp_orientation_correction_*、fused_grasp_position_z_offset_m 仍为可选标定。 */

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sealien_ctrlpilot_msgmanagement/msg/keypoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <cctype>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("keypoint_to_arm_tf");
static constexpr double kDegToRad = 0.017453292519943295;
static constexpr double kDirectionNormMin = 1e-6;

/* mock_preset:=sonar_cable_9 时注入的 9 点 */
static const std::array<std::array<float, 3>, 9> MOCK_KP_SONAR_CABLE_9 = {{
    {{0.871f, 2.9f, 0.0f}},
    {{0.704f, 2.9f, 0.0f}},
    {{0.221f, 2.9f, 0.0f}},
    {{0.061f, 2.9f, 0.0f}},
    {{-0.099f, 2.9f, 0.0f}},
    {{0.542f, 2.9f, 0.0f}},
    {{-0.098f, 2.9f, 0.0f}},
    {{0.220f, 2.9f, 0.0f}},
    {{-0.411f, 2.9f, 0.0f}},
}};

Eigen::Vector3d pointToEigen(const geometry_msgs::msg::Point& p)
{
    return Eigen::Vector3d(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
}

geometry_msgs::msg::Quaternion eigenQuatToMsg(const Eigen::Quaterniond& q)
{
    geometry_msgs::msg::Quaternion out;
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    out.w = q.w();
    return out;
}

Eigen::Quaterniond vectorEulerZyxDegToQuat(double roll_deg, double pitch_deg, double yaw_deg)
{
    const double roll = roll_deg * kDegToRad;
    const double pitch = pitch_deg * kDegToRad;
    const double yaw = yaw_deg * kDegToRad;
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    q.normalize();
    if (q.w() < 0.0)
    {
        q.coeffs() *= -1.0;
    }
    return q;
}

/*
 * 与 target_sensor_to_object_pose_node._rotation_matrix_side_grasp_from_direction 一致：列 x,y,z；
 * 可选再左乘 R_corr（旧 gripper_tcp 调试）；默认与桥接一致不乘。
 */
geometry_msgs::msg::Quaternion sideGraspFromDirectionInArmFrame(Eigen::Vector3d a, bool apply_tcp_correction)
{
    double n = a.norm();
    if (n < 1e-9)
    {
        a = Eigen::Vector3d(0.0, 0.0, 1.0);
    }
    else
    {
        a /= n;
    }
    if (a.z() < 0.0)
    {
        a = -a;
    }
    Eigen::Vector3d ref(0.0, 0.0, 1.0);
    if (std::fabs(a.dot(ref)) > 0.95)
    {
        ref = Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    Eigen::Vector3d y = a.cross(ref);
    double ny = y.norm();
    if (ny < 1e-9)
    {
        y = (std::fabs(a.x()) < 0.9) ? Eigen::Vector3d(1.0, 0.0, 0.0) : Eigen::Vector3d(0.0, 1.0, 0.0);
        y = y - y.dot(a) * a;
        y.normalize();
    }
    else
    {
        y /= ny;
    }
    Eigen::Vector3d z = y.cross(a);
    z.normalize();
    Eigen::Vector3d x = y.cross(z);
    x.normalize();
    Eigen::Matrix3d R_legacy;
    R_legacy.col(0) = x;
    R_legacy.col(1) = y;
    R_legacy.col(2) = z;
    Eigen::Matrix3d R = R_legacy;
    if (apply_tcp_correction)
    {
        Eigen::Matrix3d R_corr;
        R_corr << 0.0, 0.0, -1.0,
            1.0, 0.0, 0.0,
            0.0, -1.0, 0.0;
        R = R_corr * R_legacy;
    }
    Eigen::Quaterniond q(R);
    q.normalize();
    if (q.w() < 0.0)
    {
        q.coeffs() *= -1.0;
    }
    return eigenQuatToMsg(q);
}

geometry_msgs::msg::Quaternion applyFusedOrientationCorrection(const geometry_msgs::msg::Quaternion& q_side,
                                                               const Eigen::Quaterniond& q_corr)
{
    Eigen::Quaterniond q_in(q_side.w, q_side.x, q_side.y, q_side.z);
    Eigen::Quaterniond q_out = q_corr * q_in;
    q_out.normalize();
    if (q_out.w() < 0.0)
    {
        q_out.coeffs() *= -1.0;
    }
    geometry_msgs::msg::Quaternion out;
    out.x = q_out.x();
    out.y = q_out.y();
    out.z = q_out.z();
    out.w = q_out.w();
    return out;
}

/* 沿已排序折线在窗口 [i_begin, i_end] 上对真实顶点做 3D 算术均值（凸组合，避免分量中值拼出的离杆点）。 */
Eigen::Vector3d polyline_window_mean(const std::vector<Eigen::Vector3d>& pts, size_t i_begin, size_t i_end_inclusive)
{
    Eigen::Vector3d s = Eigen::Vector3d::Zero();
    const size_t n = i_end_inclusive - i_begin + 1U;
    for (size_t k = 0U; k < n; ++k)
    {
        s += pts[i_begin + k];
    }
    return s / static_cast<double>(n);
}

/* 无序关键点已在 left_arm_frame；PCA 主轴投影排序→去重→沿折线窗口均值→按弧长比例取 3D 点。 */
bool centerline_arclength_grasp_position(const std::vector<Eigen::Vector3d>& pts_arm, double dedupe_radius_m,
    int smooth_window_in, double arclength_fraction, Eigen::Vector3d* out_p)
{
    if (out_p == nullptr || pts_arm.empty())
    {
        return false;
    }
    if (pts_arm.size() == 1U)
    {
        *out_p = pts_arm[0];
        return true;
    }

    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& p : pts_arm)
    {
        mean += p;
    }
    mean /= static_cast<double>(pts_arm.size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : pts_arm)
    {
        const Eigen::Vector3d d = p - mean;
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(pts_arm.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    if (es.info() != Eigen::Success)
    {
        RCLCPP_WARN(LOGGER, "centerline: eigen decomposition failed, use centroid");
        *out_p = mean;
        return true;
    }
    Eigen::Vector3d axis = es.eigenvectors().col(2);
    const double an = axis.norm();
    if (an < 1e-12)
    {
        *out_p = mean;
        return true;
    }
    axis /= an;

    std::vector<std::pair<double, Eigen::Vector3d>> scored;
    scored.reserve(pts_arm.size());
    for (const auto& p : pts_arm)
    {
        const double t = (p - mean).dot(axis);
        scored.emplace_back(t, p);
    }
    std::sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

    std::vector<Eigen::Vector3d> chain;
    chain.reserve(scored.size());
    for (const auto& sp : scored)
    {
        if (chain.empty() || (sp.second - chain.back()).norm() >= dedupe_radius_m)
        {
            chain.push_back(sp.second);
        }
    }

    if (chain.empty())
    {
        return false;
    }
    if (chain.size() == 1U)
    {
        *out_p = chain[0];
        return true;
    }

    int mw = smooth_window_in;
    if (mw < 1)
    {
        mw = 1;
    }
    if ((mw % 2) == 0)
    {
        ++mw;
    }
    const int half = mw / 2;
    const size_t nc = chain.size();
    std::vector<Eigen::Vector3d> smoothed(nc);
    for (size_t i = 0; i < nc; ++i)
    {
        const int bi = static_cast<int>(i) - half;
        const int ei = static_cast<int>(i) + half;
        const int i0 = std::max(0, bi);
        const int i1 = std::min(static_cast<int>(nc) - 1, ei);
        smoothed[i] = polyline_window_mean(chain, static_cast<size_t>(i0), static_cast<size_t>(i1));
    }

    double total_len = 0.0;
    for (size_t i = 0; i + 1U < smoothed.size(); ++i)
    {
        total_len += (smoothed[i + 1U] - smoothed[i]).norm();
    }
    double f = arclength_fraction;
    if (f < 0.0)
    {
        f = 0.0;
    }
    else if (f > 1.0)
    {
        f = 1.0;
    }
    if (total_len < 1e-9)
    {
        *out_p = smoothed[0];
        return true;
    }
    const double target = f * total_len;
    double acc = 0.0;
    for (size_t i = 0; i + 1U < smoothed.size(); ++i)
    {
        const Eigen::Vector3d seg = smoothed[i + 1U] - smoothed[i];
        const double seg_len = seg.norm();
        if (seg_len < 1e-12)
        {
            continue;
        }
        if (acc + seg_len >= target - 1e-9)
        {
            const double alpha = (target - acc) / seg_len;
            *out_p = smoothed[i] + alpha * seg;
            return true;
        }
        acc += seg_len;
    }
    *out_p = smoothed.back();
    return true;
}

bool str_iequals(const char* a, const char* b)
{
    if (a == nullptr || b == nullptr)
    {
        return false;
    }
    while (*a != '\0' && *b != '\0')
    {
        const unsigned char ca = static_cast<unsigned char>(*a);
        const unsigned char cb = static_cast<unsigned char>(*b);
        if (std::tolower(ca) != std::tolower(cb))
        {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}
}  // namespace

class KeypointToArmTfNode final : public rclcpp::Node
{
public:
    static std::shared_ptr<KeypointToArmTfNode> create()
    {
        auto node = std::shared_ptr<KeypointToArmTfNode>(new KeypointToArmTfNode());
        node->tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(node->tf_buffer_, node, true);
        return node;
    }

private:
    KeypointToArmTfNode()
      : rclcpp::Node("keypoint_to_arm_tf")
      , tf_buffer_(get_clock())
    {
        input_topic_ = declare_parameter<std::string>("input_topic", "/perception/sonar/keypoints");
        source_frame_override_ = declare_parameter<std::string>("source_frame_override", "sensor_link");
        left_arm_frame_ = declare_parameter<std::string>("left_arm_frame", "left_arm_base");
        right_arm_frame_ = declare_parameter<std::string>("right_arm_frame", "right_arm_base");
        tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.5);
        force_source_frame_ = declare_parameter<std::string>("force_source_frame", "");
        tf_use_latest_timestamp_ = declare_parameter<bool>("tf_use_latest_timestamp", true);
        /* 默认 RELIABLE：与 `ros2 topic info -v` 里发布端一致时再改；若为 BEST_EFFORT 发布则设 qos_best_effort:=true。 */
        qos_best_effort_ = declare_parameter<bool>("qos_best_effort", false);
        qos_depth_ = declare_parameter<int>("qos_depth", 10);
        if (qos_depth_ < 1)
        {
            qos_depth_ = 1;
        }

        publish_fused_grasp_topics_ = declare_parameter<bool>("publish_fused_grasp_topics", true);
        fused_orientation_source_ =
            declare_parameter<std::string>("fused_orientation_source", "auto");  // auto | directions | euler_angles
        fused_apply_tcp_frame_correction_ = declare_parameter<bool>("fused_apply_tcp_frame_correction", false);
        fused_grasp_position_mode_ =
            declare_parameter<std::string>("fused_grasp_position_mode", "mean");  // mean | keypoint_index | centerline_arclength
        fused_grasp_keypoint_index_ = declare_parameter<int>("fused_grasp_keypoint_index", 0);
        centerline_dedupe_radius_m_ = declare_parameter<double>("centerline_dedupe_radius_m", 0.0);
        centerline_median_window_ = declare_parameter<int>("centerline_median_window", 1);
        centerline_grasp_arclength_fraction_ = declare_parameter<double>("centerline_grasp_arclength_fraction", 0.5);
        if (centerline_dedupe_radius_m_ < 0.0)
        {
            centerline_dedupe_radius_m_ = 0.0;
        }

        log_each_keypoint_tf_ = declare_parameter<bool>("log_each_keypoint_tf", false);

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

        RCLCPP_INFO(LOGGER,
                    "Keypoints subscription QoS: %s depth=%d (若与发布端不一致可能无回调，可将 qos_best_effort 与对端对齐)",
                    qos_best_effort_ ? "BEST_EFFORT" : "RELIABLE",
                    qos_depth_);

        use_mock_keypoints_ = declare_parameter<bool>("use_mock_keypoints", false);
        mock_frame_id_ = declare_parameter<std::string>("mock_frame_id", "camera");
        mock_kp_x_ = declare_parameter<double>("mock_kp_x", 0.0);
        mock_kp_y_ = declare_parameter<double>("mock_kp_y", 2.9);
        mock_kp_z_ = declare_parameter<double>("mock_kp_z", 0.0);
        mock_direction_x_ = declare_parameter<double>("mock_direction_x", 1.0);
        mock_direction_y_ = declare_parameter<double>("mock_direction_y", 0.0);
        mock_direction_z_ = declare_parameter<double>("mock_direction_z", 0.0);
        mock_period_sec_ = declare_parameter<double>("mock_period_sec", 1.0);
        mock_preset_ = declare_parameter<std::string>("mock_preset", "legacy_single");
        if (mock_period_sec_ < 0.05)
        {
            mock_period_sec_ = 0.05;
        }

        output_grasp_frame_ = declare_parameter<std::string>("output_grasp_frame", "base_link");
        grasp_pose_topic_ =
            declare_parameter<std::string>("grasp_pose_topic", "/manipulator/object_pose_fused");
        grasp_axis_topic_ =
            declare_parameter<std::string>("grasp_axis_topic", "/manipulator/object_axis_fused");
        publish_keypoints_posearray_ = declare_parameter<bool>("publish_keypoints_posearray", true);
        keypoints_posearray_topic_ =
            declare_parameter<std::string>("keypoints_posearray_topic", "/manipulator/keypoints_base_link");

        const double corr_w = declare_parameter<double>("fused_grasp_orientation_correction_w", 1.0);
        const double corr_x = declare_parameter<double>("fused_grasp_orientation_correction_x", 0.0);
        const double corr_y = declare_parameter<double>("fused_grasp_orientation_correction_y", 0.0);
        const double corr_z = declare_parameter<double>("fused_grasp_orientation_correction_z", 0.0);
        fused_orientation_correction_ = Eigen::Quaterniond(corr_w, corr_x, corr_y, corr_z);
        if (fused_orientation_correction_.norm() < 1e-9)
        {
            fused_orientation_correction_ = Eigen::Quaterniond::Identity();
        }
        else
        {
            fused_orientation_correction_.normalize();
        }
        if (std::abs(fused_orientation_correction_.w() - 1.0) > 1e-6 ||
            std::abs(fused_orientation_correction_.x()) > 1e-6 ||
            std::abs(fused_orientation_correction_.y()) > 1e-6 ||
            std::abs(fused_orientation_correction_.z()) > 1e-6)
        {
            RCLCPP_INFO(LOGGER,
                        "fused_grasp_orientation_correction wxyz=(%.6f,%.6f,%.6f,%.6f) left-multiply",
                        fused_orientation_correction_.w(),
                        fused_orientation_correction_.x(),
                        fused_orientation_correction_.y(),
                        fused_orientation_correction_.z());
        }

        fused_grasp_position_z_offset_m_ = declare_parameter<double>("fused_grasp_position_z_offset_m", 0.0);
        if (std::fabs(fused_grasp_position_z_offset_m_) > 1e-9)
        {
            RCLCPP_INFO(LOGGER,
                        "fused_grasp_position_z_offset_m=%.6f in '%s'",
                        fused_grasp_position_z_offset_m_,
                        output_grasp_frame_.c_str());
        }

        if (publish_fused_grasp_topics_)
        {
            pub_grasp_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(grasp_pose_topic_, qos);
            pub_grasp_axis_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(grasp_axis_topic_, qos);
            RCLCPP_INFO(LOGGER,
                        "publish_fused_grasp_topics: pose=%s axis=%s ori_src=%s tcp_corr=%s out_frame=%s",
                        grasp_pose_topic_.c_str(),
                        grasp_axis_topic_.c_str(),
                        fused_orientation_source_.c_str(),
                        fused_apply_tcp_frame_correction_ ? "true" : "false",
                        output_grasp_frame_.c_str());
        }
        if (publish_keypoints_posearray_)
        {
            pub_keypoints_posearray_ = create_publisher<geometry_msgs::msg::PoseArray>(keypoints_posearray_topic_, qos);
            RCLCPP_INFO(LOGGER,
                        "publish_keypoints_posearray: topic=%s frame=%s",
                        keypoints_posearray_topic_.c_str(),
                        output_grasp_frame_.c_str());
        }

        if (use_mock_keypoints_)
        {
            mock_timer_ = create_wall_timer(
                std::chrono::duration<double>(mock_period_sec_),
                std::bind(&KeypointToArmTfNode::onMockTimer, this));
            RCLCPP_WARN(LOGGER,
                        "use_mock_keypoints=true: mock_preset=%s every %.3fs frame=%s",
                        mock_preset_.c_str(),
                        mock_period_sec_,
                        mock_frame_id_.c_str());
        }
        else
        {
            sub_ = create_subscription<sealien_ctrlpilot_msgmanagement::msg::Keypoints>(
                input_topic_,
                qos,
                std::bind(&KeypointToArmTfNode::onKeypoints, this, std::placeholders::_1));
        }

        RCLCPP_INFO(LOGGER,
                    "keypoint_to_arm_tf: mock=%s sub=%s pos_mode=%s fused_publish=%s log_each_kp=%s",
                    use_mock_keypoints_ ? "on" : "off",
                    input_topic_.c_str(),
                    fused_grasp_position_mode_.c_str(),
                    publish_fused_grasp_topics_ ? "true" : "false",
                    log_each_keypoint_tf_ ? "true" : "false");
        if (str_iequals(fused_grasp_position_mode_.c_str(), "centerline_arclength"))
        {
            RCLCPP_INFO(LOGGER,
                        "centerline_arclength: dedupe=%.4fm smooth_win=%d (polyline mean) arclen_frac=%.4f",
                        centerline_dedupe_radius_m_,
                        centerline_median_window_,
                        centerline_grasp_arclength_fraction_);
        }
    }

    void onMockTimer()
    {
        auto msg = std::make_shared<sealien_ctrlpilot_msgmanagement::msg::Keypoints>();
        msg->header.stamp = now();
        msg->header.frame_id = mock_frame_id_;
        msg->has_target = true;
        msg->corner_points[0].x = 0.0f;
        msg->corner_points[0].y = 0.0f;
        msg->corner_points[0].z = 0.0f;
        msg->corner_points[1].x = 0.0f;
        msg->corner_points[1].y = 0.0f;
        msg->corner_points[1].z = 0.0f;
        if (mock_preset_ == "sonar_cable_9")
        {
            msg->keypoints.resize(MOCK_KP_SONAR_CABLE_9.size());
            for (size_t k = 0; k < MOCK_KP_SONAR_CABLE_9.size(); ++k)
            {
                msg->keypoints[k].x = MOCK_KP_SONAR_CABLE_9[k][0];
                msg->keypoints[k].y = MOCK_KP_SONAR_CABLE_9[k][1];
                msg->keypoints[k].z = MOCK_KP_SONAR_CABLE_9[k][2];
            }
        }
        else
        {
            msg->keypoints.resize(1);
            msg->keypoints[0].x = static_cast<float>(mock_kp_x_);
            msg->keypoints[0].y = static_cast<float>(mock_kp_y_);
            msg->keypoints[0].z = static_cast<float>(mock_kp_z_);
        }
        msg->directions[0] = static_cast<float>(mock_direction_x_);
        msg->directions[1] = static_cast<float>(mock_direction_y_);
        msg->directions[2] = static_cast<float>(mock_direction_z_);
        msg->euler_angles[0] = 0.0f;
        msg->euler_angles[1] = 0.0f;
        msg->euler_angles[2] = 0.0f;
        msg->score = 0.0f;
        msg->score_threshold = 0.0f;
        msg->use_score_threshold = false;
        msg->is_available = true;
        processKeypoints(msg);
    }

    void onKeypoints(const sealien_ctrlpilot_msgmanagement::msg::Keypoints::SharedPtr msg)
    {
        processKeypoints(msg);
    }

    bool tryTransformPoint(const std::string& frame_id, const rclcpp::Time& stamp,
                           const geometry_msgs::msg::Point& pt, const std::string& target_frame,
                           geometry_msgs::msg::Point* out_pt, bool log_warnings = true)
    {
        geometry_msgs::msg::PointStamped pt_in;
        if (tf_use_latest_timestamp_)
        {
            pt_in.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        }
        else
        {
            pt_in.header.stamp = stamp;
        }
        pt_in.header.frame_id = frame_id;
        pt_in.point = pt;
        const tf2::Duration timeout = tf2::durationFromSec(tf_timeout_sec_);
        try
        {
            geometry_msgs::msg::PointStamped pt_out = tf_buffer_.transform(pt_in, target_frame, timeout);
            out_pt->x = pt_out.point.x;
            out_pt->y = pt_out.point.y;
            out_pt->z = pt_out.point.z;
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            if (log_warnings)
            {
                RCLCPP_WARN(LOGGER, "TF %s->%s: %s", frame_id.c_str(), target_frame.c_str(), ex.what());
            }
            return false;
        }
    }

    bool tryTransformVector(const std::string& frame_id, const rclcpp::Time& stamp,
                            const geometry_msgs::msg::Vector3& vin, const std::string& target_frame,
                            geometry_msgs::msg::Vector3* vout)
    {
        geometry_msgs::msg::Vector3Stamped v_in;
        if (tf_use_latest_timestamp_)
        {
            v_in.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        }
        else
        {
            v_in.header.stamp = stamp;
        }
        v_in.header.frame_id = frame_id;
        v_in.vector = vin;
        const tf2::Duration timeout = tf2::durationFromSec(tf_timeout_sec_);
        try
        {
            geometry_msgs::msg::Vector3Stamped v_out = tf_buffer_.transform(v_in, target_frame, timeout);
            vout->x = v_out.vector.x;
            vout->y = v_out.vector.y;
            vout->z = v_out.vector.z;
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(LOGGER, "TF vector %s->%s: %s", frame_id.c_str(), target_frame.c_str(), ex.what());
            return false;
        }
    }

    bool lookupRotationQuaternion(const std::string& target_frame, const std::string& source_frame,
                                  const rclcpp::Time& stamp, Eigen::Quaterniond* q_target_source)
    {
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            if (tf_use_latest_timestamp_)
            {
                tf = tf_buffer_.lookupTransform(target_frame, source_frame, rclcpp::Time(0, 0, get_clock()->get_clock_type()),
                    tf2::durationFromSec(tf_timeout_sec_));
            }
            else
            {
                tf = tf_buffer_.lookupTransform(target_frame, source_frame, stamp, tf2::durationFromSec(tf_timeout_sec_));
            }
            Eigen::Quaterniond q(static_cast<double>(tf.transform.rotation.w),
                static_cast<double>(tf.transform.rotation.x),
                static_cast<double>(tf.transform.rotation.y),
                static_cast<double>(tf.transform.rotation.z));
            q.normalize();
            *q_target_source = q;
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(LOGGER,
                *get_clock(),
                2000,
                "lookupTransform %s<- %s : %s",
                target_frame.c_str(),
                source_frame.c_str(),
                ex.what());
            return false;
        }
    }

    void tryPublishFusedGrasp(const rclcpp::Time& stamp_in, const geometry_msgs::msg::Quaternion& q_body,
                              const Eigen::Vector3d& grasp_pos_body, const Eigen::Vector3d& axis_body)
    {
        if (!publish_fused_grasp_topics_ || !pub_grasp_pose_ || !pub_grasp_axis_)
        {
            return;
        }
        geometry_msgs::msg::PoseStamped ps;
        if (tf_use_latest_timestamp_)
        {
            ps.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        }
        else
        {
            ps.header.stamp = stamp_in;
        }
        ps.header.frame_id = left_arm_frame_;
        ps.pose.position.x = grasp_pos_body.x();
        ps.pose.position.y = grasp_pos_body.y();
        ps.pose.position.z = grasp_pos_body.z();
        ps.pose.orientation = applyFusedOrientationCorrection(q_body, fused_orientation_correction_);

        geometry_msgs::msg::Vector3Stamped vs;
        vs.header = ps.header;
        Eigen::Vector3d tu = axis_body;
        const double tn = tu.norm();
        if (tn > 1e-9)
        {
            tu /= tn;
        }
        vs.vector.x = tu.x();
        vs.vector.y = tu.y();
        vs.vector.z = tu.z();

        const tf2::Duration timeout = tf2::durationFromSec(tf_timeout_sec_);
        try
        {
            geometry_msgs::msg::PoseStamped pose_out = tf_buffer_.transform(ps, output_grasp_frame_, timeout);
            pose_out.pose.position.z += fused_grasp_position_z_offset_m_;
            const geometry_msgs::msg::Vector3Stamped axis_out =
                tf_buffer_.transform(vs, output_grasp_frame_, timeout);
            pub_grasp_pose_->publish(pose_out);
            pub_grasp_axis_->publish(axis_out);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(LOGGER,
                *get_clock(),
                2000,
                "object_pose_fused/axis 未发布: PoseStamped/Vector3 从 '%s' 变换到 '%s' 失败: %s (topics %s %s)",
                left_arm_frame_.c_str(),
                output_grasp_frame_.c_str(),
                ex.what(),
                grasp_pose_topic_.c_str(),
                grasp_axis_topic_.c_str());
        }
    }

    bool computeGraspPositionInArmFrame(const std::string& source_frame, const rclcpp::Time& stamp,
                                        const sealien_ctrlpilot_msgmanagement::msg::Keypoints::SharedPtr msg,
                                        Eigen::Vector3d* out_p)
    {
        if (str_iequals(fused_grasp_position_mode_.c_str(), "centerline_arclength"))
        {
            std::vector<Eigen::Vector3d> pts_arm;
            pts_arm.reserve(msg->keypoints.size());
            size_t ki = 0U;
            for (const auto& kp : msg->keypoints)
            {
                geometry_msgs::msg::Point p_arm;
                if (!tryTransformPoint(source_frame, stamp, kp, left_arm_frame_, &p_arm))
                {
                    RCLCPP_WARN_THROTTLE(LOGGER,
                        *get_clock(),
                        2000,
                        "object_pose_fused 未参与发布: grasp_position(centerline_arclength) 在 keypoint[%zu/%zu] "
                        "TF '%s'->'%s' 失败",
                        ki,
                        msg->keypoints.size(),
                        source_frame.c_str(),
                        left_arm_frame_.c_str());
                    return false;
                }
                pts_arm.push_back(pointToEigen(p_arm));
                ++ki;
            }
            if (!centerline_arclength_grasp_position(pts_arm,
                    centerline_dedupe_radius_m_,
                    centerline_median_window_,
                    centerline_grasp_arclength_fraction_,
                    out_p))
            {
                RCLCPP_WARN_THROTTLE(LOGGER,
                    *get_clock(),
                    5000,
                    "object_pose_fused 未参与发布: centerline_arclength_grasp_position 返回 false");
                return false;
            }
            return true;
        }
        if (str_iequals(fused_grasp_position_mode_.c_str(), "keypoint_index"))
        {
            const int n = static_cast<int>(msg->keypoints.size());
            if (n <= 0)
            {
                RCLCPP_WARN_THROTTLE(LOGGER,
                    *get_clock(),
                    5000,
                    "object_pose_fused 未参与发布: grasp_position(keypoint_index) 但 keypoints 为空");
                return false;
            }
            int idx = fused_grasp_keypoint_index_;
            if (idx < 0)
            {
                idx = 0;
            }
            if (idx >= n)
            {
                idx = n - 1;
            }
            geometry_msgs::msg::Point p_arm;
            if (!tryTransformPoint(source_frame, stamp, msg->keypoints[static_cast<size_t>(idx)], left_arm_frame_, &p_arm))
            {
                RCLCPP_WARN_THROTTLE(LOGGER,
                    *get_clock(),
                    2000,
                    "object_pose_fused 未参与发布: grasp_position(keypoint_index=%d) TF '%s'->'%s' 失败",
                    idx,
                    source_frame.c_str(),
                    left_arm_frame_.c_str());
                return false;
            }
            *out_p = pointToEigen(p_arm);
            return true;
        }
        /* mean */
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        size_t cnt = 0;
        size_t ki_mean = 0U;
        for (const auto& kp : msg->keypoints)
        {
            geometry_msgs::msg::Point p_arm;
            if (!tryTransformPoint(source_frame, stamp, kp, left_arm_frame_, &p_arm))
            {
                RCLCPP_WARN_THROTTLE(LOGGER,
                    *get_clock(),
                    2000,
                    "object_pose_fused 未参与发布: grasp_position(mean) 在 keypoint[%zu/%zu] TF '%s'->'%s' 失败",
                    ki_mean,
                    msg->keypoints.size(),
                    source_frame.c_str(),
                    left_arm_frame_.c_str());
                return false;
            }
            sum += pointToEigen(p_arm);
            ++cnt;
            ++ki_mean;
        }
        if (cnt == 0U)
        {
            RCLCPP_WARN_THROTTLE(LOGGER,
                *get_clock(),
                5000,
                "object_pose_fused 未参与发布: grasp_position(mean) 计数为 0");
            return false;
        }
        *out_p = sum / static_cast<double>(cnt);
        return true;
    }

    void publishFusedFromMsgAttitude(const std::string& source_frame, const rclcpp::Time& stamp,
                                       const sealien_ctrlpilot_msgmanagement::msg::Keypoints::SharedPtr msg,
                                       const Eigen::Vector3d& grasp_pos_arm)
    {
        const double dx = static_cast<double>(msg->directions[0]);
        const double dy = static_cast<double>(msg->directions[1]);
        const double dz = static_cast<double>(msg->directions[2]);
        const double dnorm = std::sqrt(dx * dx + dy * dy + dz * dz);

        const bool use_directions_only = str_iequals(fused_orientation_source_.c_str(), "directions");
        const bool use_euler_only = str_iequals(fused_orientation_source_.c_str(), "euler_angles");
        bool pick_directions = false;
        if (use_directions_only)
        {
            pick_directions = dnorm >= kDirectionNormMin;
        }
        else if (!use_euler_only)
        {
            /* auto */
            pick_directions = dnorm >= kDirectionNormMin;
        }

        Eigen::Vector3d axis_arm;
        geometry_msgs::msg::Quaternion q_orient_arm;

        if (pick_directions && dnorm >= kDirectionNormMin)
        {
            geometry_msgs::msg::Vector3 v_src;
            v_src.x = dx;
            v_src.y = dy;
            v_src.z = dz;
            geometry_msgs::msg::Vector3 v_arm;
            if (!tryTransformVector(source_frame, stamp, v_src, left_arm_frame_, &v_arm))
            {
                RCLCPP_WARN_THROTTLE(LOGGER,
                    *get_clock(),
                    2000,
                    "object_pose_fused 未发布: directions 向量 TF '%s'->'%s' 失败",
                    source_frame.c_str(),
                    left_arm_frame_.c_str());
                return;
            }
            const Eigen::Vector3d d_arm(static_cast<double>(v_arm.x), static_cast<double>(v_arm.y),
                static_cast<double>(v_arm.z));
            axis_arm = d_arm;
            q_orient_arm = sideGraspFromDirectionInArmFrame(d_arm, fused_apply_tcp_frame_correction_);
        }
        else
        {
            if (use_directions_only && dnorm < kDirectionNormMin)
            {
                RCLCPP_WARN_THROTTLE(LOGGER,
                    *get_clock(),
                    5000,
                    "fused_orientation_source=directions but ||directions|| too small, skip publish");
                return;
            }
            /* euler_angles or auto fallback: degrees, roll pitch yaw, 内旋 ZYX */
            const double roll = static_cast<double>(msg->euler_angles[0]);
            const double pitch = static_cast<double>(msg->euler_angles[1]);
            const double yaw = static_cast<double>(msg->euler_angles[2]);
            Eigen::Quaterniond q_src = vectorEulerZyxDegToQuat(roll, pitch, yaw);
            Eigen::Quaterniond q_tf;
            if (!lookupRotationQuaternion(left_arm_frame_, source_frame, stamp, &q_tf))
            {
                RCLCPP_WARN_THROTTLE(LOGGER,
                    *get_clock(),
                    2000,
                    "object_pose_fused 未发布: euler 分支 lookupRotation '%s'<-'%s' 失败",
                    left_arm_frame_.c_str(),
                    source_frame.c_str());
                return;
            }
            Eigen::Quaterniond q_arm = q_tf * q_src;
            q_arm.normalize();
            if (q_arm.w() < 0.0)
            {
                q_arm.coeffs() *= -1.0;
            }
            q_orient_arm = eigenQuatToMsg(q_arm);
            /* 轴：缆在源系局部 +Z 旋到基系 */
            const Eigen::Vector3d cable_z_src = q_src * Eigen::Vector3d(0.0, 0.0, 1.0);
            geometry_msgs::msg::Vector3 vs;
            vs.x = cable_z_src.x();
            vs.y = cable_z_src.y();
            vs.z = cable_z_src.z();
            geometry_msgs::msg::Vector3 va;
            if (!tryTransformVector(source_frame, stamp, vs, left_arm_frame_, &va))
            {
                axis_arm = q_arm * Eigen::Vector3d(0.0, 0.0, 1.0);
            }
            else
            {
                axis_arm = Eigen::Vector3d(va.x, va.y, va.z);
            }
        }

        tryPublishFusedGrasp(stamp, q_orient_arm, grasp_pos_arm, axis_arm);
    }

    void tryPublishKeypointsPoseArray(const std::string& source_frame, const rclcpp::Time& stamp,
                                       const sealien_ctrlpilot_msgmanagement::msg::Keypoints::SharedPtr msg)
    {
        if (!publish_keypoints_posearray_ || !pub_keypoints_posearray_)
        {
            return;
        }
        geometry_msgs::msg::PoseArray out;
        out.header.frame_id = output_grasp_frame_;
        if (tf_use_latest_timestamp_)
        {
            out.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        }
        else
        {
            out.header.stamp = stamp;
        }
        out.poses.reserve(msg->keypoints.size());
        size_t fail_count = 0U;
        for (const auto& kp : msg->keypoints)
        {
            geometry_msgs::msg::Point p_out{};
            if (!tryTransformPoint(source_frame, stamp, kp, output_grasp_frame_, &p_out, false))
            {
                ++fail_count;
                continue;
            }
            p_out.z += fused_grasp_position_z_offset_m_;
            geometry_msgs::msg::Pose pose{};
            pose.position = p_out;
            pose.orientation.w = 1.0;
            out.poses.push_back(pose);
        }
        if (fail_count > 0U)
        {
            RCLCPP_WARN_THROTTLE(LOGGER,
                *get_clock(),
                2000,
                "keypoints_posearray: %zu/%zu keypoints TF %s->%s failed (已发布 %zu 点)",
                fail_count,
                msg->keypoints.size(),
                source_frame.c_str(),
                output_grasp_frame_.c_str(),
                out.poses.size());
        }
        if (!out.poses.empty())
        {
            pub_keypoints_posearray_->publish(out);
        }
        else if (!msg->keypoints.empty())
        {
            RCLCPP_WARN_THROTTLE(LOGGER,
                *get_clock(),
                2000,
                "keypoints_posearray 未发布: 全部 %zu 个点 TF '%s'->'%s' 失败（topic=%s）",
                msg->keypoints.size(),
                source_frame.c_str(),
                output_grasp_frame_.c_str(),
                keypoints_posearray_topic_.c_str());
        }
    }

    void processKeypoints(const sealien_ctrlpilot_msgmanagement::msg::Keypoints::SharedPtr msg)
    {
        if (!received_keypoints_once_)
        {
            received_keypoints_once_ = true;
            RCLCPP_DEBUG(LOGGER, "first Keypoints processed (keypoints.size=%zu)", msg->keypoints.size());
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

        if (publish_fused_grasp_topics_)
        {
            Eigen::Vector3d grasp_arm;
            if (computeGraspPositionInArmFrame(frame_id, msg->header.stamp, msg, &grasp_arm))
            {
                publishFusedFromMsgAttitude(frame_id, msg->header.stamp, msg, grasp_arm);
            }
        }
        else
        {
            RCLCPP_DEBUG_THROTTLE(LOGGER,
                *get_clock(),
                10000,
                "publish_fused_grasp_topics=false：不发布 object_pose_fused / axis_fused；仍发布 keypoints_posearray（若开启）");
        }
        tryPublishKeypointsPoseArray(frame_id, msg->header.stamp, msg);

        if (!log_each_keypoint_tf_)
        {
            return;
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

                RCLCPP_DEBUG(LOGGER,
                    "keypoint[%zu] in '%s': (%.6f, %.6f, %.6f)",
                    i,
                    frame_id.c_str(),
                    pt_in.point.x,
                    pt_in.point.y,
                    pt_in.point.z);
                RCLCPP_DEBUG(LOGGER,
                    "  -> '%s': (%.6f, %.6f, %.6f)",
                    left_arm_frame_.c_str(),
                    left_out.point.x,
                    left_out.point.y,
                    left_out.point.z);
                RCLCPP_DEBUG(LOGGER,
                    "  -> '%s': (%.6f, %.6f, %.6f)",
                    right_arm_frame_.c_str(),
                    right_out.point.x,
                    right_out.point.y,
                    right_out.point.z);
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(LOGGER, "TF failed for keypoint[%zu]: %s", i, ex.what());
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
    double mock_direction_x_{1.0};
    double mock_direction_y_{0.0};
    double mock_direction_z_{0.0};
    double mock_period_sec_{1.0};
    std::string mock_preset_{"legacy_single"};

    bool log_each_keypoint_tf_{false};

    bool publish_fused_grasp_topics_{true};
    std::string fused_orientation_source_{"auto"};
    bool fused_apply_tcp_frame_correction_{false};
    std::string fused_grasp_position_mode_{"mean"};
    int fused_grasp_keypoint_index_{0};
    double centerline_dedupe_radius_m_{0.0};
    int centerline_median_window_{1};
    double centerline_grasp_arclength_fraction_{0.5};

    std::string output_grasp_frame_;
    std::string grasp_pose_topic_;
    std::string grasp_axis_topic_;
    bool publish_keypoints_posearray_{true};
    std::string keypoints_posearray_topic_{"/manipulator/keypoints_base_link"};
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_grasp_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_grasp_axis_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_keypoints_posearray_;
    Eigen::Quaterniond fused_orientation_correction_{1.0, 0.0, 0.0, 0.0};
    double fused_grasp_position_z_offset_m_{0.0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(KeypointToArmTfNode::create());
    rclcpp::shutdown();
    return 0;
}

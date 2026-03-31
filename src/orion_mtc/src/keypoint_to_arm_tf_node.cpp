/* 订阅 Keypoints（sealien_ctrlpilot_msgmanagement/msg/Keypoints；默认话题 /perception/sonar/keypoints），
 * 或将 use_mock_keypoints 打开，用定时器注入假数据做离线 TF 测试。
 * centerline_grasp_test：去重 → 中值滤波 → 弧长抓取点 → PCA 切向 → 侧抓姿态；结果经话题发布（默认
 * /manipulator/object_pose_fused），不在此循环打 INFO。调试键位：log_each_keypoint_tf + log level DEBUG。
 * override_all_keypoints_y/z 默认 false，使用消息内真实坐标；仅离线对齐调试时可设 true。 */

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sealien_ctrlpilot_msgmanagement/msg/keypoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <chrono>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("keypoint_to_arm_tf");

/* mock_preset:=sonar_cable_9 时注入的 9 点（源系下 m 级坐标；y 统一 2.9 便于与调试 TF 一致） */
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

/*
 * 侧向抓取：先按与 object_pose 桥接相同的叉乘规则构造 R_legacy（列 x,y,z），再左乘 R_corr，
 * 使切向与 +Z 对齐时等价于绕 X 转 -90°（qx=-√2/2, qw=√2/2），与本节点测试 / gripper_tcp 约定一致。
 * R_corr = Rx(-90°) * Ry(-90°)，满足 R_corr * Ry(+90°) = Rx(-90°)。
 */
geometry_msgs::msg::Quaternion sideGraspQuaternionFromAxis(Eigen::Vector3d a)
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
    Eigen::Matrix3d R_corr;
    R_corr << 0.0, 0.0, -1.0,
        1.0, 0.0, 0.0,
        0.0, -1.0, 0.0;
    Eigen::Matrix3d R = R_corr * R_legacy;
    Eigen::Quaterniond q(R);
    q.normalize();
    geometry_msgs::msg::Quaternion out;
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    out.w = q.w();
    return out;
}

/* 点云主轴方向（最大特征值对应特征向量），用于沿缆线排序 */
Eigen::Vector3d principalDirectionMax(const std::vector<Eigen::Vector3d>& pts, Eigen::Vector3d* mean_out)
{
    const size_t n = pts.size();
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& p : pts)
    {
        mean += p;
    }
    mean /= static_cast<double>(n);
    if (mean_out != nullptr)
    {
        *mean_out = mean;
    }
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : pts)
    {
        Eigen::Vector3d q = p - mean;
        cov += q * q.transpose();
    }
    if (n > 1U)
    {
        cov /= static_cast<double>(n - 1U);
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    Eigen::Vector3d dir = es.eigenvectors().col(2);
    if (dir.norm() > 1e-9)
    {
        dir.normalize();
    }
    else
    {
        dir = Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    return dir;
}

std::vector<Eigen::Vector3d> orderAlongDirection(std::vector<Eigen::Vector3d> pts, const Eigen::Vector3d& dir,
                                                   const Eigen::Vector3d& mean)
{
    std::sort(pts.begin(), pts.end(), [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return (a - mean).dot(dir) < (b - mean).dot(dir);
    });
    return pts;
}

std::vector<Eigen::Vector3d> dedupeOrderedChain(const std::vector<Eigen::Vector3d>& ordered, double radius_m)
{
    std::vector<Eigen::Vector3d> out;
    for (const auto& p : ordered)
    {
        if (out.empty() || (p - out.back()).norm() > radius_m)
        {
            out.push_back(p);
        }
    }
    return out;
}

std::vector<Eigen::Vector3d> medianFilter3(const std::vector<Eigen::Vector3d>& pts, int win)
{
    const int n = static_cast<int>(pts.size());
    if (win < 1)
    {
        return pts;
    }
    const int half = win / 2;
    std::vector<Eigen::Vector3d> out;
    out.reserve(pts.size());
    for (int i = 0; i < n; ++i)
    {
        std::vector<double> cx;
        std::vector<double> cy;
        std::vector<double> cz;
        for (int k = -half; k <= half; ++k)
        {
            const int j = std::clamp(i + k, 0, n - 1);
            cx.push_back(pts[static_cast<size_t>(j)].x());
            cy.push_back(pts[static_cast<size_t>(j)].y());
            cz.push_back(pts[static_cast<size_t>(j)].z());
        }
        std::sort(cx.begin(), cx.end());
        std::sort(cy.begin(), cy.end());
        std::sort(cz.begin(), cz.end());
        const size_t mid = cx.size() / 2;
        out.emplace_back(cx[mid], cy[mid], cz[mid]);
    }
    return out;
}

/* 沿折线弧长比例 pos(0..L) 插值位置与线段索引；fraction 钳位到 [0,1] */
std::pair<Eigen::Vector3d, int> polylinePointAtArcFraction(const std::vector<Eigen::Vector3d>& pl, double fraction)
{
    const int m = static_cast<int>(pl.size());
    if (m <= 0)
    {
        return {Eigen::Vector3d(0.0, 0.0, 0.0), 0};
    }
    if (m == 1)
    {
        return {pl[0], 0};
    }
    double total = 0.0;
    std::vector<double> seg_len(static_cast<size_t>(m - 1));
    for (int i = 0; i < m - 1; ++i)
    {
        const double len = (pl[static_cast<size_t>(i + 1)] - pl[static_cast<size_t>(i)]).norm();
        seg_len[static_cast<size_t>(i)] = len;
        total += len;
    }
    if (total < 1e-12)
    {
        return {pl[0], 0};
    }
    double t = fraction * total;
    if (t <= 0.0)
    {
        return {pl[0], 0};
    }
    if (t >= total)
    {
        return {pl[static_cast<size_t>(m - 1)], m - 2};
    }
    double acc = 0.0;
    for (int i = 0; i < m - 1; ++i)
    {
        const double sl = seg_len[static_cast<size_t>(i)];
        if (acc + sl >= t)
        {
            const double alpha = (t - acc) / sl;
            const Eigen::Vector3d p = (1.0 - alpha) * pl[static_cast<size_t>(i)] + alpha * pl[static_cast<size_t>(i + 1)];
            return {p, i};
        }
        acc += sl;
    }
    return {pl[static_cast<size_t>(m - 1)], m - 2};
}

Eigen::Vector3d localPcaTangent(const std::vector<Eigen::Vector3d>& pl, int center_seg_idx, int half_width,
                                const Eigen::Vector3d& seg_forward_hint)
{
    const int m = static_cast<int>(pl.size());
    const int i0 = std::clamp(center_seg_idx, 0, m - 2);
    int ia = std::max(0, i0 - half_width);
    int ib = std::min(m - 1, i0 + half_width + 1);
    if (ib <= ia)
    {
        return seg_forward_hint.normalized();
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    const int cnt = ib - ia + 1;
    for (int i = ia; i <= ib; ++i)
    {
        mean += pl[static_cast<size_t>(i)];
    }
    mean /= static_cast<double>(cnt);
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (int i = ia; i <= ib; ++i)
    {
        Eigen::Vector3d q = pl[static_cast<size_t>(i)] - mean;
        cov += q * q.transpose();
    }
    if (cnt > 1)
    {
        cov /= static_cast<double>(cnt - 1);
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    Eigen::Vector3d t = es.eigenvectors().col(2);
    if (t.norm() < 1e-9)
    {
        t = seg_forward_hint;
    }
    else
    {
        t.normalize();
    }
    if (t.dot(seg_forward_hint) < 0.0)
    {
        t = -t;
    }
    return t;
}
}  // namespace

class KeypointToArmTfNode final : public rclcpp::Node
{
public:
    /*
     * 工厂：构造节点并挂 TransformListener（传入 shared_from_this 语义下的 node 接口）。
     */
    static std::shared_ptr<KeypointToArmTfNode> create()
    {
        auto node = std::shared_ptr<KeypointToArmTfNode>(new KeypointToArmTfNode());
        /* spin_thread=true：独立线程收 /tf，才能对 buffer 使用带 timeout 的 transform，否则 tf2 报错 */
        node->tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(node->tf_buffer_, node, true);
        return node;
    }

private:
    /*
     * 声明 QoS、帧覆盖、mock 与 TF 查询策略；按 use_mock_keypoints 选择定时器或订阅 Keypoints。
     */
    KeypointToArmTfNode()
      : rclcpp::Node("keypoint_to_arm_tf")
      , tf_buffer_(get_clock())
    {
        input_topic_ = declare_parameter<std::string>("input_topic", "/perception/sonar/keypoints");
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

        centerline_grasp_test_ = declare_parameter<bool>("centerline_grasp_test", true);
        log_each_keypoint_tf_ = declare_parameter<bool>("log_each_keypoint_tf", false);
        dedupe_radius_m_ = declare_parameter<double>("dedupe_radius_m", 0.03);
        median_window_ = declare_parameter<int>("median_window", 3);
        local_fit_half_width_ = declare_parameter<int>("local_fit_half_width", 2);
        grasp_arclength_fraction_ = declare_parameter<double>("grasp_arclength_fraction", 0.5);
        if (grasp_arclength_fraction_ < 0.0)
        {
            grasp_arclength_fraction_ = 0.0;
        }
        if (grasp_arclength_fraction_ > 1.0)
        {
            grasp_arclength_fraction_ = 1.0;
        }
        if (median_window_ < 1)
        {
            median_window_ = 1;
        }
        if (median_window_ % 2 == 0)
        {
            median_window_ += 1;
        }
        if (local_fit_half_width_ < 0)
        {
            local_fit_half_width_ = 0;
        }

        /* 调试：将 keypoint.y / z 统一（接真机可调 false 关闭） */
        override_all_keypoints_y_ = declare_parameter<bool>("override_all_keypoints_y", false);
        override_keypoint_y_value_ = declare_parameter<double>("override_keypoint_y_value", 2.9);
        override_all_keypoints_z_ = declare_parameter<bool>("override_all_keypoints_z", false);
        override_keypoint_z_value_ = declare_parameter<double>("override_keypoint_z_value", 0.0);

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
        mock_kp_y_ = declare_parameter<double>("mock_kp_y", 2.9);
        mock_kp_z_ = declare_parameter<double>("mock_kp_z", 0.0);
        mock_period_sec_ = declare_parameter<double>("mock_period_sec", 1.0);
        mock_preset_ = declare_parameter<std::string>("mock_preset", "legacy_single");
        if (mock_period_sec_ < 0.05)
        {
            mock_period_sec_ = 0.05;
        }

        publish_fused_grasp_topics_ = declare_parameter<bool>("publish_fused_grasp_topics", true);
        output_grasp_frame_ = declare_parameter<std::string>("output_grasp_frame", "base_link");
        grasp_pose_topic_ =
            declare_parameter<std::string>("grasp_pose_topic", "/manipulator/object_pose_fused");
        grasp_axis_topic_ =
            declare_parameter<std::string>("grasp_axis_topic", "/manipulator/object_axis_fused");
        if (publish_fused_grasp_topics_)
        {
            pub_grasp_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(grasp_pose_topic_, qos);
            pub_grasp_axis_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(grasp_axis_topic_, qos);
            RCLCPP_INFO(LOGGER,
                        "publish_fused_grasp_topics: pose=%s axis=%s output_frame=%s",
                        grasp_pose_topic_.c_str(),
                        grasp_axis_topic_.c_str(),
                        output_grasp_frame_.c_str());
        }

        if (use_mock_keypoints_)
        {
            mock_timer_ = create_wall_timer(
                std::chrono::duration<double>(mock_period_sec_),
                std::bind(&KeypointToArmTfNode::onMockTimer, this));
            if (mock_preset_ == "sonar_cable_9")
            {
                RCLCPP_WARN(LOGGER,
                            "use_mock_keypoints=true: NOT subscribing; mock_preset=sonar_cable_9 "
                            "(%zu points) every %.3fs frame=%s",
                            MOCK_KP_SONAR_CABLE_9.size(),
                            mock_period_sec_,
                            mock_frame_id_.c_str());
            }
            else
            {
                if (mock_preset_ != "legacy_single")
                {
                    RCLCPP_ERROR(LOGGER,
                                 "unknown mock_preset='%s', using legacy_single (mock_kp_*)",
                                 mock_preset_.c_str());
                    mock_preset_ = "legacy_single";
                }
                RCLCPP_WARN(LOGGER,
                            "use_mock_keypoints=true: NOT subscribing; mock_preset=legacy_single "
                            "every %.3fs frame=%s pt=(%.6f,%.6f,%.6f)",
                            mock_period_sec_,
                            mock_frame_id_.c_str(),
                            mock_kp_x_,
                            mock_kp_y_,
                            mock_kp_z_);
            }
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
                    "force_frame=%s -> [%s | %s] centerline_test=%s log_each_kp=%s",
                    use_mock_keypoints_ ? "on" : "off",
                    input_topic_.c_str(),
                    qos_best_effort_ ? "best_effort" : "reliable",
                    qos_depth_,
                    tf_use_latest_timestamp_ ? "true" : "false",
                    source_frame_override_.c_str(),
                    force_source_frame_.empty() ? "(use msg header.frame_id)" : force_source_frame_.c_str(),
                    left_arm_frame_.c_str(),
                    right_arm_frame_.c_str(),
                    centerline_grasp_test_ ? "true" : "false",
                    log_each_keypoint_tf_ ? "true" : "false");
        if (override_all_keypoints_y_)
        {
            RCLCPP_WARN(LOGGER,
                        "override_all_keypoints_y=true: all keypoints y <- %.6f (set false for live perception)",
                        override_keypoint_y_value_);
        }
        if (override_all_keypoints_z_)
        {
            RCLCPP_WARN(LOGGER,
                        "override_all_keypoints_z=true: all keypoints z <- %.6f (set false for live perception)",
                        override_keypoint_z_value_);
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
        msg->yaw_degree = 0.0f;
        msg->score = 0.0f;
        msg->score_threshold = 0.0f;
        msg->use_score_threshold = false;
        msg->is_available = true;
        processKeypoints(msg);
    }

    /*
     * 订阅回调：直接转发 processKeypoints。
     */
    void onKeypoints(const sealien_ctrlpilot_msgmanagement::msg::Keypoints::SharedPtr msg)
    {
        processKeypoints(msg);
    }

    bool tryTransformPoint(const std::string& frame_id, const rclcpp::Time& stamp,
                           const geometry_msgs::msg::Point& pt, const std::string& target_frame,
                           geometry_msgs::msg::Point* out_pt)
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
            RCLCPP_WARN(LOGGER, "TF %s->%s: %s", frame_id.c_str(), target_frame.c_str(), ex.what());
            return false;
        }
    }

    /*
     * 将中心线抓取结果发布到 MTC FUSED 话题（变换到 output_grasp_frame_，默认 base_link）。
     */
    void tryPublishFusedGrasp(const rclcpp::Time& stamp_in,
                              const geometry_msgs::msg::Quaternion& q_body,
                              const Eigen::Vector3d& grasp_pos_body,
                              const Eigen::Vector3d& tangent_body)
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
        ps.pose.orientation = q_body;

        geometry_msgs::msg::Vector3Stamped vs;
        vs.header = ps.header;
        Eigen::Vector3d tu = tangent_body;
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
            const geometry_msgs::msg::PoseStamped pose_out =
                tf_buffer_.transform(ps, output_grasp_frame_, timeout);
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
                                 "fused grasp publish: TF %s->%s failed: %s",
                                 left_arm_frame_.c_str(),
                                 output_grasp_frame_.c_str(),
                                 ex.what());
        }
    }

    /*
     * 中心线测试：在 left_arm_frame 下拟合；侧抓位姿由 tryPublishFusedGrasp 发布，供 MTC/UI。
     */
    void runCenterlineGraspTest(const std::string& source_frame, const rclcpp::Time& stamp,
                                const sealien_ctrlpilot_msgmanagement::msg::Keypoints::SharedPtr msg)
    {
        std::vector<Eigen::Vector3d> cloud;
        cloud.reserve(msg->keypoints.size());
        for (const auto& kp : msg->keypoints)
        {
            geometry_msgs::msg::Point p_out;
            if (!tryTransformPoint(source_frame, stamp, kp, left_arm_frame_, &p_out))
            {
                RCLCPP_WARN_THROTTLE(LOGGER, *get_clock(), 2000, "centerline_test: TF failed, abort line fit");
                return;
            }
            cloud.push_back(pointToEigen(p_out));
        }
        if (cloud.size() < 2U)
        {
            RCLCPP_WARN_THROTTLE(LOGGER, *get_clock(), 2000, "centerline_test: need >=2 points after TF");
            return;
        }
        Eigen::Vector3d mean;
        Eigen::Vector3d axis_global = principalDirectionMax(cloud, &mean);
        std::vector<Eigen::Vector3d> ordered = orderAlongDirection(cloud, axis_global, mean);
        std::vector<Eigen::Vector3d> deduped = dedupeOrderedChain(ordered, dedupe_radius_m_);
        if (deduped.size() < 2U)
        {
            RCLCPP_WARN_THROTTLE(LOGGER, *get_clock(), 2000, "centerline_test: dedupe left <2 points");
            return;
        }
        std::vector<Eigen::Vector3d> smoothed = medianFilter3(deduped, median_window_);
        const auto grasp_arc = polylinePointAtArcFraction(smoothed, grasp_arclength_fraction_);
        const Eigen::Vector3d grasp_pos = grasp_arc.first;
        const int seg_idx = grasp_arc.second;
        Eigen::Vector3d seg_hint = smoothed[static_cast<size_t>(seg_idx + 1)] - smoothed[static_cast<size_t>(seg_idx)];
        if (seg_hint.norm() < 1e-9)
        {
            seg_hint = axis_global;
        }
        const Eigen::Vector3d tangent = localPcaTangent(smoothed, seg_idx, local_fit_half_width_, seg_hint);
        const geometry_msgs::msg::Quaternion q = sideGraspQuaternionFromAxis(tangent);
        tryPublishFusedGrasp(stamp, q, grasp_pos, tangent);
    }

    /*
     * 解析源帧 → 将各 keypoint 变到左/右臂基座系并可选 INFO；可选中心线抓取测试。
     */
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

        if (override_all_keypoints_y_ || override_all_keypoints_z_)
        {
            const float y_fix = static_cast<float>(override_keypoint_y_value_);
            const float z_fix = static_cast<float>(override_keypoint_z_value_);
            for (auto& kp : msg->keypoints)
            {
                if (override_all_keypoints_y_)
                {
                    kp.y = y_fix;
                }
                if (override_all_keypoints_z_)
                {
                    kp.z = z_fix;
                }
            }
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

        if (centerline_grasp_test_)
        {
            runCenterlineGraspTest(frame_id, msg->header.stamp, msg);
        }

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
    std::string mock_preset_{"legacy_single"};

    bool centerline_grasp_test_{true};
    bool log_each_keypoint_tf_{false};
    double dedupe_radius_m_{0.03};
    int median_window_{3};
    int local_fit_half_width_{2};
    double grasp_arclength_fraction_{0.5};

    bool override_all_keypoints_y_{false};
    double override_keypoint_y_value_{2.9};
    bool override_all_keypoints_z_{false};
    double override_keypoint_z_value_{0.0};

    bool publish_fused_grasp_topics_{true};
    std::string output_grasp_frame_;
    std::string grasp_pose_topic_;
    std::string grasp_axis_topic_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_grasp_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_grasp_axis_;
};

/*
 * 独立可执行：`keypoint_to_arm_tf` 单线程 spin 工具节点。
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(KeypointToArmTfNode::create());
    rclcpp::shutdown();
    return 0;
}

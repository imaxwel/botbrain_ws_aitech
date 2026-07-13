#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <queue>
#include <cmath>
// #include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <open3d/Open3D.h>

#include "open3d_registration/open3d_registration.h"
#include "open3d_conversions/open3d_conversions.h"

#define PI 3.1415926

namespace
{
bool IsRigidTransform(const Eigen::Matrix4d &transform)
{
    if (!transform.allFinite())
    {
        return false;
    }

    const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    const double orthogonality_error =
        (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm();
    const double determinant_error = std::abs(rotation.determinant() - 1.0);
    const Eigen::Vector4d expected_bottom_row(0.0, 0.0, 0.0, 1.0);
    const double bottom_row_error =
        (transform.row(3).transpose() - expected_bottom_row).norm();

    return orthogonality_error < 1e-2 && determinant_error < 1e-2 &&
           bottom_row_error < 1e-6;
}

double RotationAngleDegrees(const Eigen::Matrix3d &rotation)
{
    const double cosine = std::max(-1.0, std::min(1.0, (rotation.trace() - 1.0) * 0.5));
    return std::acos(cosine) * 180.0 / M_PI;
}

std::string NormalizeFrameId(std::string frame_id)
{
    while (!frame_id.empty() && frame_id.front() == '/')
    {
        frame_id.erase(frame_id.begin());
    }
    return frame_id;
}
} // namespace

class KalmanFilter
{
public:
    KalmanFilter() : processVar_(0.0), estimatedMeasVar_(0.0),
                     posteriEstimate_(0.0), posteriErrorEstimate_(1.0)
    {
    }

    void KalmanFilterInit(double processVar, double estimatedMeasVar, double posteriEstimate = 0.0, double posteriErrorEstimate = 1.0)
    {
        processVar_ = processVar;
        estimatedMeasVar_ = estimatedMeasVar;
        posteriEstimate_ = posteriEstimate;
        posteriErrorEstimate_ = posteriErrorEstimate;
    }
    void inputLatestNoisyMeasurement(double measurement)
    {
        double prioriEstimate = posteriEstimate_;
        double prioriErrorEstimate = posteriErrorEstimate_ + processVar_;

        double denominator = prioriErrorEstimate + estimatedMeasVar_;

        // 防止除零导致 NaN
        if (std::abs(denominator) < 1e-10)
        {
            // 如果分母接近零，直接使用测量值
            posteriEstimate_ = measurement;
            posteriErrorEstimate_ = 1.0;
            return;
        }

        double blendingFactor = prioriErrorEstimate / denominator;
        posteriEstimate_ = prioriEstimate + blendingFactor * (measurement - prioriEstimate);
        posteriErrorEstimate_ = (1 - blendingFactor) * prioriErrorEstimate;
    }

    double getLatestEstimatedMeasurement()
    {
        return posteriEstimate_;
    }

private:
    double processVar_;
    double estimatedMeasVar_;
    double posteriEstimate_;
    double posteriErrorEstimate_;
};

class GloabalLocalization : public rclcpp::Node
{
private:
    /* data */
public:
    GloabalLocalization();
    ~GloabalLocalization();

    /// @brief 初始化定位
    void LocalizationInitialize();

    /// @brief 订阅fast_lio里程计信息
    void CallbackBaselink2Odom(const nav_msgs::msg::Odometry::SharedPtr baselink2odom);
    /// @brief 订阅在baselink下的点云
    void CallbackScan(const sensor_msgs::msg::PointCloud2::SharedPtr scan_in_baselink);

    /// @brief 订阅在初始位姿
    void CallbackInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose);

    void StartLoc();

    void Localization();

    /// @brief 欧拉角转mat3x3
    /// @param euler
    /// @return
    Eigen::Matrix3d Euler2Matrix3d(const Eigen::Vector3d euler);

    /// @brief 获取tf关系到矩阵
    /// @param frame_id
    /// @param child_frame_id
    /// @param matrix
    /// @return
    bool GetTfTransformToMatrix(
        std::string frame_id, std::string child_frame_id, Eigen::Matrix4d &matrix);

    /// @brief compute 3d distance between two points
    /// @param a
    /// @param b
    /// @return 距离值
    double ComputeMotionDis(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

private:
    /// @brief 订阅baselink2odom,即fast_lio的里程计信息
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_baselink2odom_;

    /// @brief 订阅当前帧点云
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_cur_;

    /// @brief 订阅初始位姿
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;

    /// @brief baselink到odom的pose表达
    nav_msgs::msg::Odometry pose_baselink2odom_;

    /// @brief bselink到odom的变换矩阵表达
    Eigen::Matrix4d mat_baselink2odom_;
    // Guarded by lock_mat_odom2map_ so ICP snapshots the pose and its stamp
    // atomically instead of pairing a timestamp from one odometry with the
    // matrix from the next callback.
    double timestamp_pose_odom_seconds_ = 0.0;
    /// @brief odom到map的矩阵
    Eigen::Matrix4d mat_odom2map_;
    Eigen::Matrix4d mat_odom2map_kalman_;
    /// @brief baselink到map = mat_odom2map * mat_baselink2odom
    Eigen::Matrix4d mat_baselink2map_;
    /// @brief initialpose初始位姿
    Eigen::Matrix4d mat_initialpose_;

    std::mutex lock_mat_odom2map_;

    /// @brief baselink和运动中心
    Eigen::Matrix4d mat_baselink2motionlink_;

    /// @brief imulink到baselink
    Eigen::Matrix4d mat_imulink2baselink_;

    /// @brief 初始位姿, x, y, z, roll, pitch, yaw (单位:度degrees)
    std::vector<double> initialpose_;

    /// @brief 原始地图点云
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_ori_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_coarse_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_fine_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_cur_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan_cur_;

    std::queue<open3d::geometry::PointCloud> que_pcd_scan_;
    int queue_maxsize_;
    double voxelsize_coarse_;
    double voxelsize_fine_;

    /// @brief 定位配准fitness(overlap)阈值
    double threshold_fitness_;
    /// @brief 配准fitness(overlap)阈值
    double threshold_fitness_init_;

    std::thread thread_loc_;
    std::mutex lock_scan_;
    double timestamp_scan_seconds_ = 0.0;
    std::atomic<unsigned long long> scan_generation_{0};
    std::string registered_cloud_world_frame_;
    std::atomic<bool> flag_exit_{false};

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_baselink2map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_baselink2map_kalman_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_motionlink2map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom2map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom2map_kalman_;
    rclcpp::Time timestamp_odom_;
    std::mutex lock_timestamp_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan2map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_localization_3d_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_localization_3d_confidence_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_localization_3d_delay_ms_;

    geometry_msgs::msg::PoseStamped localization_3d_;
    std_msgs::msg::Float32 localization_3d_confidence_;
    std_msgs::msg::Float32 localization_3d_delay_ms_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> br_odom2map_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    bool save_scan_;

    /// @brief 定位频率（Hz；保留历史参数名 loc_frequence）
    double loc_frequence_;
    double max_icp_translation_step_;
    double max_icp_rotation_step_deg_;
    double immediate_icp_translation_step_;
    double immediate_icp_rotation_step_deg_;
    double icp_candidate_consistency_translation_;
    double icp_candidate_consistency_rotation_deg_;
    double icp_candidate_max_age_sec_;
    double max_scan_odom_time_skew_sec_;
    double max_icp_inlier_rmse_;
    double min_initialization_fitness_;
    double max_initialization_translation_step_;
    double max_initialization_rotation_step_deg_;
    int large_correction_confirmations_;
    int min_icp_source_points_;
    int min_icp_target_points_;

    /// @brief source点云最大点数量
    int maxpoints_source_ = 50000;
    /// @brief target点云最大点数量
    int maxpoints_target_ = 200000;

    /// @brief 初始化成功标志
    std::atomic<bool> loc_initialized_{false};
    std::atomic<unsigned int> manual_pose_generation_{0};

    /// @brief 当前定位overlap，confidence
    std::atomic<double> loc_fitness_{0.0};

    /// @brief 定位置信度阈值
    double confidence_loc_th_;

    /// 卡尔曼滤波器
    KalmanFilter kf_baselink_x_;
    KalmanFilter kf_baselink_y_;
    KalmanFilter kf_baselink_z_;
    KalmanFilter kalman_filter_odom2map_;

    // 0:kf_processVar 1:kf_estimatedMeasVar
    std::vector<double> kf_param_x_;
    std::vector<double> kf_param_y_;
    std::vector<double> kf_param_z_;

    /// @brief 对odom2map进行kalman滤波
    bool filter_odom2map_ = false;
    double kalman_processVar2_ = 0.0;
    double kalman_estimatedMeasVar2_ = 0.0;

    /// 1202
    /// @brief 上次更新定位时的定位值
    Eigen::Vector3d last_loc_;
    // Eigen::Vector3d cur_loc_;
    /// @brief 更新地图子图的距离,超过则更新地图子图
    double dis_updatemap_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

GloabalLocalization::GloabalLocalization() : Node("global_loc_node"),
                                             tf_buffer_(this->get_clock()),
                                             tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
{
    flag_exit_.store(false);
    loc_initialized_.store(false);
    mat_baselink2odom_ = Eigen::Matrix4d::Identity();
    mat_odom2map_ = Eigen::Matrix4d::Identity();
    mat_odom2map_kalman_ = Eigen::Matrix4d::Identity();
    mat_baselink2map_ = Eigen::Matrix4d::Identity();
    mat_initialpose_ = Eigen::Matrix4d::Identity();
    // Default-initialize TF-derived matrices to Identity so that a failed
    // lookupTransform (e.g. static TF not yet received at startup) does not
    // leave them at zero and produce NaN through `.inverse()` downstream.
    mat_baselink2motionlink_ = Eigen::Matrix4d::Identity();
    mat_imulink2baselink_ = Eigen::Matrix4d::Identity();
    last_loc_ = Eigen::Vector3d(0, 0, -5000);

    pcd_map_ori_.reset(new open3d::geometry::PointCloud);
    pcd_map_coarse_.reset(new open3d::geometry::PointCloud);
    pcd_map_cur_.reset(new open3d::geometry::PointCloud);
    pcd_scan_cur_.reset(new open3d::geometry::PointCloud);
    pcd_map_fine_.reset(new open3d::geometry::PointCloud);
    queue_maxsize_ = 1;

    pub_baselink2map_ = this->create_publisher<nav_msgs::msg::Odometry>("baselink2map", 100000);
    pub_baselink2map_kalman_ = this->create_publisher<nav_msgs::msg::Odometry>("baselink2map_kalman", 100000);
    pub_motionlink2map_ = this->create_publisher<nav_msgs::msg::Odometry>("motionlink2map", 100000);
    pub_odom2map_ = this->create_publisher<nav_msgs::msg::Odometry>("odom2map", 100000);
    pub_odom2map_kalman_ = this->create_publisher<nav_msgs::msg::Odometry>("odom2map_kalman", 100000);

    pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcd_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    pub_submap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("submap", 1);
    pub_scan2map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan2map", 1);
    pub_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan", 1);
    pub_localization_3d_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("localization_3d", 1);
    pub_localization_3d_confidence_ = this->create_publisher<std_msgs::msg::Float32>("localization_3d_confidence", 1);
    pub_localization_3d_delay_ms_ = this->create_publisher<std_msgs::msg::Float32>("localization_3d_delay_ms", 1);

    loc_frequence_ = 2.0; //
    loc_fitness_ = 0.0;
    // 注册回调函数
    sub_baselink2odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "Odometry_loc", 50, std::bind(&GloabalLocalization::CallbackBaselink2Odom, this, std::placeholders::_1));
    sub_scan_cur_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_registered_1", 50, std::bind(&GloabalLocalization::CallbackScan, this, std::placeholders::_1));
    sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 50, std::bind(&GloabalLocalization::CallbackInitialPose, this, std::placeholders::_1));

    pose_baselink2odom_ = nav_msgs::msg::Odometry();
    pose_baselink2odom_.header.frame_id = "odom";
    pose_baselink2odom_.child_frame_id = "base_link";
    // geometry_msgs的Quaternion会被初始化为0,0,0,0,而不是正确的0,0,0,1
    pose_baselink2odom_.pose.pose.orientation.w = 1;
    RCLCPP_INFO(this->get_logger(), "pose baselink2odom:\nx: %f, y: %f, z: %f, qx: %f, \
                            qy: %f, qz: %f, qw: %f",
                pose_baselink2odom_.pose.pose.position.x,
                pose_baselink2odom_.pose.pose.position.y,
                pose_baselink2odom_.pose.pose.position.z,
                pose_baselink2odom_.pose.pose.orientation.x,
                pose_baselink2odom_.pose.pose.orientation.y,
                pose_baselink2odom_.pose.pose.orientation.z,
                pose_baselink2odom_.pose.pose.orientation.w);

    // 队列最大数量
    this->declare_parameter<int>("pcd_queue_maxsize", 1);
    this->declare_parameter<std::string>("registered_cloud_world_frame", "camera_init");
    this->declare_parameter<bool>("save_scan", false);
    /// 最大点数量限制
    this->declare_parameter<int>("maxpoints_source", 50000);
    this->declare_parameter<int>("maxpoints_target", 200000);

    // 定位间隔时间
    // Localization update rate in Hz (kept under the legacy parameter name).
    this->declare_parameter<double>("loc_frequence", 2.0);
    this->declare_parameter<double>("max_icp_translation_step", 1.0);
    this->declare_parameter<double>("max_icp_rotation_step_deg", 15.0);
    this->declare_parameter<double>("immediate_icp_translation_step", 0.10);
    this->declare_parameter<double>("immediate_icp_rotation_step_deg", 2.0);
    this->declare_parameter<int>("large_correction_confirmations", 2);
    this->declare_parameter<double>("icp_candidate_consistency_translation", 0.20);
    this->declare_parameter<double>("icp_candidate_consistency_rotation_deg", 4.0);
    this->declare_parameter<double>("icp_candidate_max_age_sec", 1.0);
    this->declare_parameter<double>("max_scan_odom_time_skew_sec", 0.03);
    this->declare_parameter<double>("max_icp_inlier_rmse", 0.30);
    this->declare_parameter<double>("min_initialization_fitness", 0.20);
    this->declare_parameter<double>("max_initialization_translation_step", 2.0);
    this->declare_parameter<double>("max_initialization_rotation_step_deg", 45.0);
    this->declare_parameter<int>("min_icp_source_points", 100);
    this->declare_parameter<int>("min_icp_target_points", 1000);

    /// 定位阈值
    this->declare_parameter<double>("confidence_loc_th", 0.6);

    /// 卡尔曼参数
    this->declare_parameter<std::vector<double>>("kf_baselink2map_x", std::vector<double>(2));
    this->declare_parameter<std::vector<double>>("kf_baselink2map_y", std::vector<double>(2));
    this->declare_parameter<std::vector<double>>("kf_baselink2map_z", std::vector<double>(2));

    this->declare_parameter<bool>("filter_odom2map", false);
    this->declare_parameter<double>("kalman_processVar2", 0.02);
    this->declare_parameter<double>("kalman_estimatedMeasVar2", 0.04);
    // voxelsize
    this->declare_parameter<double>("voxelsize_coarse", 0.2);
    this->declare_parameter<double>("voxelsize_fine", 0.05);
    this->declare_parameter<double>("threshold_fitness_init", 0.9);
    this->declare_parameter<double>("threshold_fitness", 0.9);
    this->declare_parameter<std::vector<double>>("initialpose", std::vector<double>());
    this->declare_parameter<double>("dis_updatemap", 1);

    this->get_parameter("pcd_queue_maxsize", queue_maxsize_);
    this->get_parameter("registered_cloud_world_frame", registered_cloud_world_frame_);
    registered_cloud_world_frame_ = NormalizeFrameId(registered_cloud_world_frame_);
    if (registered_cloud_world_frame_.empty())
    {
        RCLCPP_WARN(this->get_logger(),
                    "registered_cloud_world_frame is empty; forcing camera_init");
        registered_cloud_world_frame_ = "camera_init";
    }
    this->get_parameter("save_scan", save_scan_);
    this->get_parameter("maxpoints_source", maxpoints_source_);
    this->get_parameter("maxpoints_target", maxpoints_target_);
    this->get_parameter("loc_frequence", loc_frequence_);
    this->get_parameter("max_icp_translation_step", max_icp_translation_step_);
    this->get_parameter("max_icp_rotation_step_deg", max_icp_rotation_step_deg_);
    this->get_parameter("immediate_icp_translation_step", immediate_icp_translation_step_);
    this->get_parameter("immediate_icp_rotation_step_deg", immediate_icp_rotation_step_deg_);
    this->get_parameter("large_correction_confirmations", large_correction_confirmations_);
    this->get_parameter("icp_candidate_consistency_translation", icp_candidate_consistency_translation_);
    this->get_parameter("icp_candidate_consistency_rotation_deg", icp_candidate_consistency_rotation_deg_);
    this->get_parameter("icp_candidate_max_age_sec", icp_candidate_max_age_sec_);
    this->get_parameter("max_scan_odom_time_skew_sec", max_scan_odom_time_skew_sec_);
    this->get_parameter("max_icp_inlier_rmse", max_icp_inlier_rmse_);
    this->get_parameter("min_initialization_fitness", min_initialization_fitness_);
    this->get_parameter("max_initialization_translation_step", max_initialization_translation_step_);
    this->get_parameter("max_initialization_rotation_step_deg", max_initialization_rotation_step_deg_);
    this->get_parameter("min_icp_source_points", min_icp_source_points_);
    this->get_parameter("min_icp_target_points", min_icp_target_points_);
    this->get_parameter("confidence_loc_th", confidence_loc_th_);
    this->get_parameter("kf_baselink2map_x", kf_param_x_);
    this->get_parameter("kf_baselink2map_y", kf_param_y_);
    this->get_parameter("kf_baselink2map_z", kf_param_z_);
    this->get_parameter("filter_odom2map", filter_odom2map_);
    this->get_parameter("kalman_processVar2", kalman_processVar2_);
    this->get_parameter("kalman_estimatedMeasVar2", kalman_estimatedMeasVar2_);

    RCLCPP_INFO(this->get_logger(), "Kalman filter parameters:");
    RCLCPP_INFO(this->get_logger(), "  kf_x: [%.6f, %.6f], size: %zu",
                kf_param_x_.size() >= 1 ? kf_param_x_[0] : 0.0,
                kf_param_x_.size() >= 2 ? kf_param_x_[1] : 0.0,
                kf_param_x_.size());
    RCLCPP_INFO(this->get_logger(), "  kf_y: [%.6f, %.6f], size: %zu",
                kf_param_y_.size() >= 1 ? kf_param_y_[0] : 0.0,
                kf_param_y_.size() >= 2 ? kf_param_y_[1] : 0.0,
                kf_param_y_.size());
    RCLCPP_INFO(this->get_logger(), "  kf_z: [%.6f, %.6f], size: %zu",
                kf_param_z_.size() >= 1 ? kf_param_z_[0] : 0.0,
                kf_param_z_.size() >= 2 ? kf_param_z_[1] : 0.0,
                kf_param_z_.size());
    RCLCPP_INFO(this->get_logger(), "  filter_odom2map: %s", filter_odom2map_ ? "true" : "false");
    this->get_parameter("voxelsize_coarse", voxelsize_coarse_);
    this->get_parameter("voxelsize_fine", voxelsize_fine_);
    this->get_parameter("threshold_fitness_init", threshold_fitness_init_);
    this->get_parameter("threshold_fitness", threshold_fitness_);
    this->get_parameter("initialpose", initialpose_);
    this->get_parameter("dis_updatemap", dis_updatemap_);

    if (initialpose_.size() != 6)
    {
        RCLCPP_WARN(this->get_logger(),
                    "initialpose must contain [x,y,z,roll,pitch,yaw]; using zeros instead");
        initialpose_ = std::vector<double>(6, 0.0);
    }
    for (auto i : initialpose_)
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    mat_initialpose_.block<3, 3>(0, 0) = Euler2Matrix3d(Eigen::Vector3d(initialpose_[3], initialpose_[4], initialpose_[5]));
    mat_initialpose_.block<3, 1>(0, 3) = Eigen::Vector3d(initialpose_[0], initialpose_[1], initialpose_[2]);

    if (loc_frequence_ <= 0.0)
    {
        RCLCPP_WARN(this->get_logger(), "loc_frequence must be > 0 Hz; using 2.0 Hz");
        loc_frequence_ = 2.0;
    }
    queue_maxsize_ = std::max(queue_maxsize_, 1);
    min_icp_source_points_ = std::max(min_icp_source_points_, 1);
    min_icp_target_points_ = std::max(min_icp_target_points_, 1);
    max_icp_translation_step_ = std::max(max_icp_translation_step_, 0.01);
    max_icp_rotation_step_deg_ = std::max(max_icp_rotation_step_deg_, 0.1);
    immediate_icp_translation_step_ = std::max(
        0.0, std::min(immediate_icp_translation_step_, max_icp_translation_step_));
    immediate_icp_rotation_step_deg_ = std::max(
        0.0, std::min(immediate_icp_rotation_step_deg_, max_icp_rotation_step_deg_));
    large_correction_confirmations_ = std::max(large_correction_confirmations_, 1);
    icp_candidate_consistency_translation_ = std::max(icp_candidate_consistency_translation_, 0.01);
    icp_candidate_consistency_rotation_deg_ = std::max(icp_candidate_consistency_rotation_deg_, 0.1);
    icp_candidate_max_age_sec_ = std::max(icp_candidate_max_age_sec_, 0.1);
    max_scan_odom_time_skew_sec_ = std::max(max_scan_odom_time_skew_sec_, 0.001);
    max_icp_inlier_rmse_ = std::max(max_icp_inlier_rmse_, 0.01);
    min_initialization_fitness_ = std::max(
        std::max(0.0, std::min(1.0, threshold_fitness_init_)),
        std::max(0.0, std::min(1.0, min_initialization_fitness_)));
    max_initialization_translation_step_ = std::max(max_initialization_translation_step_, 0.01);
    max_initialization_rotation_step_deg_ = std::max(max_initialization_rotation_step_deg_, 0.1);
    RCLCPP_INFO(this->get_logger(), "Registered cloud world frame: %s",
                registered_cloud_world_frame_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "ICP %.2f Hz (%.1f ms), queue=%d, points=%d/%d, immediate<=%.2fm/%.1fdeg, max<=%.2fm/%.1fdeg, confirmations=%d within %.2fs, stamp_skew<=%.3fs, rmse<=%.2f",
                loc_frequence_, 1000.0 / loc_frequence_, queue_maxsize_,
                min_icp_source_points_, min_icp_target_points_,
                immediate_icp_translation_step_, immediate_icp_rotation_step_deg_,
                max_icp_translation_step_, max_icp_rotation_step_deg_,
                large_correction_confirmations_, icp_candidate_max_age_sec_,
                max_scan_odom_time_skew_sec_, max_icp_inlier_rmse_);

    // 读取地图
    std::string path_map = "";
    this->declare_parameter<std::string>("path_map", "");
    this->get_parameter("path_map", path_map);
    open3d::io::ReadPointCloud(path_map, *pcd_map_ori_);
    if (pcd_map_ori_ == nullptr || pcd_map_ori_->IsEmpty())
    {
        RCLCPP_ERROR(this->get_logger(), "read map from path: %s failed", path_map.c_str());
        rclcpp::shutdown();
    }

    if (!pcd_map_ori_->HasColors())
    {
        pcd_map_ori_->PaintUniformColor({1, 0, 0});
    }
    // pcd_map_ori_->PaintUniformColor({1, 0, 0});

    pcd_map_coarse_ = pcd_map_ori_->VoxelDownSample(voxelsize_coarse_);
    pcd_map_coarse_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxelsize_coarse_ * 2, 30));

    /// publish map, 用粗地图可视化，减少资源占用
    sensor_msgs::msg::PointCloud2 pc2_map;
    open3d_conversions::open3dToRos(*pcd_map_coarse_, pc2_map);
    pc2_map.header.frame_id = "map";
    pc2_map.header.stamp = this->now();
    pub_map_->publish(pc2_map);

    pcd_map_fine_ = pcd_map_ori_->VoxelDownSample(voxelsize_fine_);
    pcd_map_fine_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxelsize_fine_ * 2, 30));

    GetTfTransformToMatrix("base_link", "imu_link", mat_imulink2baselink_);
    std::cout << "mat_imulink2baselink_:\n"
              << mat_imulink2baselink_ << std::endl;

    GetTfTransformToMatrix("motion_link", "base_link", mat_baselink2motionlink_);
    std::cout << "mat_baselink2motionlink_:\n"
              << mat_baselink2motionlink_ << std::endl;

    RCLCPP_WARN(this->get_logger(), "initialize finished");

    br_odom2map_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    StartLoc();
}

GloabalLocalization::~GloabalLocalization()
{
    flag_exit_.store(true);
    if (thread_loc_.joinable())
    {
        thread_loc_.join();
    }
}

Eigen::Matrix3d GloabalLocalization::Euler2Matrix3d(const Eigen::Vector3d euler)
{
    Eigen::Matrix3d mat3d;
    // convert degrees to radians
    auto eulerAngle = euler / 180 * M_PI;
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitZ()));
    mat3d = rollAngle * pitchAngle * yawAngle;
    return mat3d;
}
bool GloabalLocalization::GetTfTransformToMatrix(std::string frame_id, std::string child_frame_id, Eigen::Matrix4d &matrix)
{
    // 获取pose
    geometry_msgs::msg::TransformStamped pose_;
    try
    {
        // Wait up to 3s for the static TF chain to populate the buffer.
        // Without this, namespaced static_transform_publishers that haven't
        // yet delivered their transient_local message cause lookup failure,
        // leaving `matrix` at its caller-supplied default (Identity).
        pose_ = tf_buffer_.lookupTransform(frame_id, child_frame_id, rclcpp::Time(0),
                                           rclcpp::Duration::from_seconds(3.0));
    }
    catch (tf2::TransformException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[GetTransformMatrix]: %s", e.what());
        return false;
    }

    Eigen::Vector3d translation = Eigen::Vector3d(pose_.transform.translation.x, pose_.transform.translation.y, pose_.transform.translation.z);
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();

    quat = Eigen::Quaterniond(pose_.transform.rotation.w,
                              pose_.transform.rotation.x,
                              pose_.transform.rotation.y,
                              pose_.transform.rotation.z);
    if (!translation.allFinite() || !quat.coeffs().allFinite() || quat.norm() < 1e-9)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "[GetTransformMatrix]: received non-finite translation or invalid quaternion for %s <- %s",
                     frame_id.c_str(), child_frame_id.c_str());
        return false;
    }
    quat.normalize();
    Eigen::Matrix3d rotation = quat.matrix();

    matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = rotation;
    matrix.matrix().block<3, 1>(0, 3) = translation;
    return true;
}

void GloabalLocalization::CallbackBaselink2Odom(const nav_msgs::msg::Odometry::SharedPtr baselink2odom)
{
    Eigen::Isometry3d mat_current = Eigen::Isometry3d::Identity();
    tf2::fromMsg(baselink2odom->pose.pose, mat_current);
    const Eigen::Matrix4d mat_imulink2odom = mat_current.matrix();

    // Never allow an invalid FAST-LIO pose to enter the map->odom state.
    if (!IsRigidTransform(mat_imulink2odom))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Skipping odometry callback: upstream pose is not a finite rigid transform");
        return;
    }

    const Eigen::Matrix4d mat_baselink2odom =
        mat_imulink2odom * mat_imulink2baselink_.inverse();
    if (!IsRigidTransform(mat_baselink2odom))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Skipping odometry callback: invalid base_link->odom transform");
        return;
    }

    Eigen::Matrix4d mat_odom2map = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d mat_baselink2map = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d mat_odom2map_kalman = Eigen::Matrix4d::Identity();
    {
        std::lock_guard<std::mutex> pose_guard(lock_mat_odom2map_);
        if (!IsRigidTransform(mat_odom2map_))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Skipping odometry callback: current map->odom transform is invalid");
            return;
        }

        mat_odom2map = mat_odom2map_;
        mat_baselink2map = mat_odom2map * mat_baselink2odom;
        mat_odom2map_kalman = mat_odom2map_kalman_;
        if (!IsRigidTransform(mat_baselink2map))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Skipping odometry callback: computed base_link->map transform is invalid");
            return;
        }

        // Commit pose and derived map pose atomically. The timestamp is
        // published only after this complete validated snapshot is available.
        mat_baselink2odom_ = mat_baselink2odom;
        timestamp_pose_odom_seconds_ = rclcpp::Time(baselink2odom->header.stamp).seconds();
        mat_baselink2map_ = mat_baselink2map;
    }
    {
        std::lock_guard<std::mutex> timestamp_guard(lock_timestamp_);
        timestamp_odom_ = baselink2odom->header.stamp;
    }

    Eigen::Isometry3d isometry_baselink2map = Eigen::Isometry3d::Identity();
    isometry_baselink2map.matrix() = mat_baselink2map;
    nav_msgs::msg::Odometry baselink2map;
    baselink2map.pose.pose = tf2::toMsg(isometry_baselink2map);
    baselink2map.header.frame_id = "map";
    baselink2map.child_frame_id = "base_link";
    baselink2map.header.stamp = baselink2odom->header.stamp;
    pub_baselink2map_->publish(baselink2map);

    Eigen::Isometry3d isometry_odom2map = Eigen::Isometry3d::Identity();
    isometry_odom2map.matrix() = mat_odom2map;
    nav_msgs::msg::Odometry odom2map;
    odom2map.pose.pose = tf2::toMsg(isometry_odom2map);
    odom2map.header.frame_id = "map";
    odom2map.child_frame_id = "odom";
    odom2map.header.stamp = baselink2odom->header.stamp;
    pub_odom2map_->publish(odom2map);

    geometry_msgs::msg::TransformStamped transform_odom2map;
    transform_odom2map.header.frame_id = "map";
    transform_odom2map.child_frame_id = "odom";
    transform_odom2map.header.stamp = baselink2odom->header.stamp;
    transform_odom2map.transform.translation.x = odom2map.pose.pose.position.x;
    transform_odom2map.transform.translation.y = odom2map.pose.pose.position.y;
    transform_odom2map.transform.translation.z = odom2map.pose.pose.position.z;
    transform_odom2map.transform.rotation = odom2map.pose.pose.orientation;
    br_odom2map_->sendTransform(transform_odom2map);

    if (!loc_initialized_.load())
    {
        return;
    }

    Eigen::Matrix4d mat_baselink2map_filtered = mat_baselink2map;
    if (filter_odom2map_)
    {
        if (!IsRigidTransform(mat_odom2map_kalman))
        {
            mat_odom2map_kalman = mat_odom2map;
        }
        mat_baselink2map_filtered = mat_odom2map_kalman * mat_baselink2odom;

        Eigen::Isometry3d isometry_odom2map_kalman = Eigen::Isometry3d::Identity();
        isometry_odom2map_kalman.matrix() = mat_odom2map_kalman;
        nav_msgs::msg::Odometry odom2map_kalman;
        odom2map_kalman.pose.pose = tf2::toMsg(isometry_odom2map_kalman);
        odom2map_kalman.header.frame_id = "map";
        odom2map_kalman.child_frame_id = "odom_kalman";
        odom2map_kalman.header.stamp = baselink2odom->header.stamp;
        pub_odom2map_kalman_->publish(odom2map_kalman);
    }

    const double input_x = mat_baselink2map_filtered(0, 3);
    const double input_y = mat_baselink2map_filtered(1, 3);
    const double input_z = mat_baselink2map_filtered(2, 3);
    kf_baselink_x_.inputLatestNoisyMeasurement(input_x);
    kf_baselink_y_.inputLatestNoisyMeasurement(input_y);
    kf_baselink_z_.inputLatestNoisyMeasurement(input_z);

    const double filtered_z = kf_baselink_z_.getLatestEstimatedMeasurement();
    if (std::isfinite(filtered_z))
    {
        mat_baselink2map_filtered(2, 3) = filtered_z;
    }
    else
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Kalman filter returned a non-finite Z value; using unfiltered Z");
        mat_baselink2map_filtered(2, 3) = input_z;
    }

    if (!IsRigidTransform(mat_baselink2map_filtered))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Skipping filtered localization output: invalid transform");
        return;
    }

    Eigen::Isometry3d isometry_baselink2map_filtered = Eigen::Isometry3d::Identity();
    isometry_baselink2map_filtered.matrix() = mat_baselink2map_filtered;
    nav_msgs::msg::Odometry baselink2map_kalman;
    baselink2map_kalman.pose.pose = tf2::toMsg(isometry_baselink2map_filtered);
    baselink2map_kalman.header.frame_id = "map";
    baselink2map_kalman.header.stamp = baselink2odom->header.stamp;
    pub_baselink2map_kalman_->publish(baselink2map_kalman);

    const Eigen::Matrix4d mat_motionlink2map =
        mat_baselink2map_filtered * mat_baselink2motionlink_.inverse();
    if (!IsRigidTransform(mat_motionlink2map))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Skipping map->motion_link output: invalid transform");
        return;
    }

    Eigen::Isometry3d isometry_motionlink2map = Eigen::Isometry3d::Identity();
    isometry_motionlink2map.matrix() = mat_motionlink2map;
    nav_msgs::msg::Odometry motionlink2map;
    motionlink2map.pose.pose = tf2::toMsg(isometry_motionlink2map);
    motionlink2map.header.frame_id = "map";
    motionlink2map.header.stamp = baselink2odom->header.stamp;
    pub_motionlink2map_->publish(motionlink2map);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "motion_link";
    transform.header.stamp = baselink2odom->header.stamp;
    transform.transform.translation.x = motionlink2map.pose.pose.position.x;
    transform.transform.translation.y = motionlink2map.pose.pose.position.y;
    transform.transform.translation.z = motionlink2map.pose.pose.position.z;
    transform.transform.rotation = motionlink2map.pose.pose.orientation;

    auto &rotation = transform.transform.rotation;
    const double quaternion_norm = std::sqrt(
        rotation.x * rotation.x + rotation.y * rotation.y +
        rotation.z * rotation.z + rotation.w * rotation.w);
    if (std::isfinite(quaternion_norm) && quaternion_norm > 1e-9)
    {
        rotation.x /= quaternion_norm;
        rotation.y /= quaternion_norm;
        rotation.z /= quaternion_norm;
        rotation.w /= quaternion_norm;
        br_odom2map_->sendTransform(transform);
    }
    else
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Skipping map->motion_link TF: invalid quaternion");
    }

    localization_3d_confidence_.data = loc_fitness_.load();
    pub_localization_3d_confidence_->publish(localization_3d_confidence_);
    localization_3d_delay_ms_.data =
        (this->now() - baselink2odom->header.stamp).seconds() * 1000.0;
    pub_localization_3d_delay_ms_->publish(localization_3d_delay_ms_);
    localization_3d_.header.frame_id = "map";
    localization_3d_.header.stamp = baselink2odom->header.stamp;
    localization_3d_.pose = motionlink2map.pose.pose;
    pub_localization_3d_->publish(localization_3d_);
}

void GloabalLocalization::CallbackScan(
    const sensor_msgs::msg::PointCloud2::SharedPtr scan_in_baselink)
{
    const std::string incoming_frame = NormalizeFrameId(scan_in_baselink->header.frame_id);
    if (registered_cloud_world_frame_.empty() ||
        incoming_frame != registered_cloud_world_frame_)
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Rejecting cloud_registered_1 frame '%s': Open3D requires world-frame cloud '%s'; never remap cloud_registered_body_1 here",
            scan_in_baselink->header.frame_id.c_str(),
            registered_cloud_world_frame_.c_str());
        loc_fitness_.store(0.0);
        return;
    }

    open3d::geometry::PointCloud pcd_received;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr const_scan_ptr = scan_in_baselink;
    open3d_conversions::rosToOpen3d(const_scan_ptr, pcd_received);

    if (pcd_received.IsEmpty())
    {
        return;
    }

    // Keep one coherent, newest-first window. The old implementation built the
    // aggregate before pushing the newest scan and modified the queue outside
    // the mutex, so ICP could consume stale data and race with this callback.
    std::lock_guard<std::mutex> scan_guard(lock_scan_);
    que_pcd_scan_.push(std::move(pcd_received));
    while (que_pcd_scan_.size() > static_cast<size_t>(queue_maxsize_))
    {
        que_pcd_scan_.pop();
    }

    pcd_scan_cur_->Clear();
    std::queue<open3d::geometry::PointCloud> queue_copy = que_pcd_scan_;
    while (!queue_copy.empty())
    {
        *pcd_scan_cur_ += queue_copy.front();
        queue_copy.pop();
    }
    // cloud_registered_1 and Odometry_loc are emitted from the same accepted
    // FAST-LIO scan. Preserve the cloud stamp so the ICP thread can pair them
    // instead of accidentally combining scan N with odometry N+1 during the
    // publish-order window (odometry is published before the cloud).
    timestamp_scan_seconds_ = rclcpp::Time(scan_in_baselink->header.stamp).seconds();
    // ICP confirmations must be based on distinct incoming cloud windows, not
    // repeated processing of the same scan while only odometry timestamps move.
    scan_generation_.fetch_add(1);
}

void GloabalLocalization::LocalizationInitialize()
{
    auto map_fine_crop = std::make_shared<open3d::geometry::PointCloud>();
    auto pcd_scan = std::make_shared<open3d::geometry::PointCloud>();
    auto source = std::make_shared<open3d::geometry::PointCloud>();
    auto target = std::make_shared<open3d::geometry::PointCloud>();
    auto obb_map = std::make_shared<open3d::geometry::OrientedBoundingBox>();
    auto obb_scan = std::make_shared<open3d::geometry::OrientedBoundingBox>();
    obb_map->extent_ = Eigen::Vector3d(60, 60, 40);
    obb_scan->extent_ = Eigen::Vector3d(60, 60, 40);

    int consecutive_successes = 0;
    Eigen::Matrix4d pending_initialization_candidate = Eigen::Matrix4d::Identity();
    auto pending_initialization_time = std::chrono::steady_clock::time_point::min();
    unsigned long long last_processed_scan_generation = 0;
    while (rclcpp::ok() && !flag_exit_.load())
    {
        const auto loc_start = std::chrono::steady_clock::now();
        unsigned long long current_scan_generation = 0;
        double current_scan_stamp = 0.0;
        {
            std::lock_guard<std::mutex> scan_guard(lock_scan_);
            current_scan_generation = scan_generation_.load();
            current_scan_stamp = timestamp_scan_seconds_;
            if (pcd_scan_cur_->IsEmpty())
            {
                pcd_scan->Clear();
            }
            else
            {
                *pcd_scan = *pcd_scan_cur_;
            }
        }
        if (pcd_scan->IsEmpty() ||
            current_scan_generation == last_processed_scan_generation)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        double current_odom_stamp = 0.0;
        Eigen::Matrix4d mat_baselink2odom_cur = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d mat_baselink2map_cur = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d current_odom2map = Eigen::Matrix4d::Identity();
        {
            std::lock_guard<std::mutex> pose_guard(lock_mat_odom2map_);
            mat_baselink2odom_cur = mat_baselink2odom_;
            mat_baselink2map_cur = mat_odom2map_ * mat_baselink2odom_;
            current_odom2map = mat_odom2map_;
            current_odom_stamp = timestamp_pose_odom_seconds_;
        }
        const double scan_odom_skew = std::fabs(current_scan_stamp - current_odom_stamp);
        if (!std::isfinite(current_scan_stamp) || !std::isfinite(current_odom_stamp) ||
            current_scan_stamp <= 0.0 || current_odom_stamp <= 0.0 ||
            scan_odom_skew > max_scan_odom_time_skew_sec_)
        {
            loc_fitness_.store(0.0);
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "LocalizationInitialize: waiting for matching cloud/odometry stamps (cloud=%.6f odom=%.6f skew=%.4fs max=%.4fs)",
                current_scan_stamp, current_odom_stamp, scan_odom_skew,
                max_scan_odom_time_skew_sec_);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        last_processed_scan_generation = current_scan_generation;
        const unsigned int iteration_manual_pose_generation =
            manual_pose_generation_.load();
        if (!IsRigidTransform(mat_baselink2odom_cur) ||
            !IsRigidTransform(mat_baselink2map_cur) ||
            !IsRigidTransform(current_odom2map))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "LocalizationInitialize: invalid pose snapshot");
            loc_fitness_.store(0.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        obb_map->center_ = mat_baselink2map_cur.block<3, 1>(0, 3);
        obb_map->R_ = mat_baselink2map_cur.block<3, 3>(0, 0);
        obb_scan->center_ = mat_baselink2odom_cur.block<3, 1>(0, 3);
        obb_scan->R_ = mat_baselink2odom_cur.block<3, 3>(0, 0);
        *map_fine_crop = *pcd_map_fine_->Crop(*obb_map);
        *target = *map_fine_crop;
        if (target->points_.size() > static_cast<size_t>(maxpoints_target_))
        {
            target = target->RandomDownSample(
                static_cast<double>(maxpoints_target_) / target->points_.size());
        }
        source = pcd_scan->Crop(*obb_scan);
        if (source->points_.size() > static_cast<size_t>(maxpoints_source_))
        {
            source = source->RandomDownSample(
                static_cast<double>(maxpoints_source_) / source->points_.size());
        }

        if (source->points_.size() < static_cast<size_t>(min_icp_source_points_) ||
            target->points_.size() < static_cast<size_t>(min_icp_target_points_))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "LocalizationInitialize: insufficient ICP points (source=%zu/%d target=%zu/%d)",
                source->points_.size(), min_icp_source_points_,
                target->points_.size(), min_icp_target_points_);
            loc_fitness_.store(0.0);
            consecutive_successes = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        source->Transform(current_odom2map);
        const Eigen::Matrix4d correction = pcd_tools::RegistrationMultiScaleIcp(
            source, target, voxelsize_fine_, 1, {1, 2, 3});
        const Eigen::Matrix4d candidate_odom2map = correction * current_odom2map;
        source->Transform(correction);
        // Evaluate with the same fine-scale correspondence radius used by
        // normal ICP. The previous 3x radius inflated fitness in corridors by
        // counting visibly displaced parallel walls as inliers.
        const auto evaluation = open3d::pipelines::registration::EvaluateRegistration(
            *source, *target, voxelsize_fine_ * 2);
        const double fitness = evaluation.fitness_;
        const double inlier_rmse = evaluation.inlier_rmse_;
        const double translation_step = correction.block<3, 1>(0, 3).norm();
        const double rotation_step_deg = RotationAngleDegrees(
            correction.block<3, 3>(0, 0));
        const bool valid_result =
            IsRigidTransform(correction) && IsRigidTransform(candidate_odom2map) &&
            std::isfinite(fitness) && std::isfinite(inlier_rmse) &&
            std::isfinite(translation_step) && std::isfinite(rotation_step_deg);
        const bool safe_initialization_step =
            valid_result && fitness >= min_initialization_fitness_ &&
            inlier_rmse <= max_icp_inlier_rmse_ &&
            translation_step <= max_initialization_translation_step_ &&
            rotation_step_deg <= max_initialization_rotation_step_deg_;

        if (!safe_initialization_step)
        {
            loc_fitness_.store(0.0);
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "LocalizationInitialize: rejecting ICP fitness=%.3f rmse=%.3f step=%.3fm/%.2fdeg (min fitness %.3f, max rmse %.3f, max step %.2fm/%.1fdeg)",
                fitness, inlier_rmse, translation_step, rotation_step_deg,
                min_initialization_fitness_, max_icp_inlier_rmse_,
                max_initialization_translation_step_, max_initialization_rotation_step_deg_);
            consecutive_successes = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        const auto candidate_time = std::chrono::steady_clock::now();
        const bool pending_is_fresh =
            consecutive_successes > 0 &&
            pending_initialization_time != std::chrono::steady_clock::time_point::min() &&
            std::chrono::duration<double>(candidate_time - pending_initialization_time).count() <=
                icp_candidate_max_age_sec_;
        bool consistent_with_pending = false;
        if (pending_is_fresh && IsRigidTransform(pending_initialization_candidate))
        {
            const Eigen::Matrix4d candidate_delta =
                pending_initialization_candidate.inverse() * candidate_odom2map;
            consistent_with_pending =
                IsRigidTransform(candidate_delta) &&
                candidate_delta.block<3, 1>(0, 3).norm() <=
                    icp_candidate_consistency_translation_ &&
                RotationAngleDegrees(candidate_delta.block<3, 3>(0, 0)) <=
                    icp_candidate_consistency_rotation_deg_;
        }

        if (consistent_with_pending)
        {
            ++consecutive_successes;
        }
        else
        {
            pending_initialization_candidate = candidate_odom2map;
            consecutive_successes = 1;
        }
        pending_initialization_time = candidate_time;

        if (manual_pose_generation_.load() != iteration_manual_pose_generation)
        {
            // A user-provided /initialpose arrived while ICP was running. Never
            // let a result computed from the old map->odom snapshot become a
            // confirmation candidate for the new manual pose.
            loc_fitness_.store(0.0);
            consecutive_successes = 0;
            pending_initialization_candidate = Eigen::Matrix4d::Identity();
            pending_initialization_time = std::chrono::steady_clock::time_point::min();
            RCLCPP_WARN(this->get_logger(),
                        "LocalizationInitialize: discarding stale ICP result after manual relocalization");
            continue;
        }

        // Do not walk map->odom one frame at a time during initialization. Two
        // independent scan windows must estimate the same absolute candidate
        // within a bounded time before the candidate is committed atomically.
        if (consecutive_successes >= std::max(2, large_correction_confirmations_))
        {
            bool stale_after_manual_pose = false;
            {
                std::lock_guard<std::mutex> pose_guard(lock_mat_odom2map_);
                stale_after_manual_pose =
                    manual_pose_generation_.load() != iteration_manual_pose_generation;
                if (!stale_after_manual_pose)
                {
                    mat_odom2map_ = pending_initialization_candidate;
                    mat_baselink2map_ = mat_odom2map_ * mat_baselink2odom_;
                }
            }
            if (stale_after_manual_pose)
            {
                loc_fitness_.store(0.0);
                consecutive_successes = 0;
                pending_initialization_candidate = Eigen::Matrix4d::Identity();
                pending_initialization_time = std::chrono::steady_clock::time_point::min();
                RCLCPP_WARN(this->get_logger(),
                            "LocalizationInitialize: discarding stale confirmed ICP after manual relocalization");
                continue;
            }

            loc_fitness_.store(fitness);
            const double elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - loc_start).count();
            RCLCPP_INFO(this->get_logger(),
                        "Localization initialization succeeded: fitness=%.3f, consistent confirmations=%d, last iteration=%.1f ms",
                        fitness, consecutive_successes, elapsed_ms);
            return;
        }

        loc_fitness_.store(0.0);
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "LocalizationInitialize: holding consistent candidate (%d/%d), fitness=%.3f rmse=%.3f",
            consecutive_successes, std::max(2, large_correction_confirmations_),
            fitness, inlier_rmse);
    }
}

void GloabalLocalization::Localization()
{
    RCLCPP_INFO(this->get_logger(), "Waiting for Odometry_loc");
    while (rclcpp::ok() && !flag_exit_.load())
    {
        double stamp_seconds = 0.0;
        {
            std::lock_guard<std::mutex> timestamp_guard(lock_timestamp_);
            stamp_seconds = timestamp_odom_.seconds();
        }
        if (stamp_seconds != 0.0)
        {
            break;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Waiting for Odometry_loc...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for cloud_registered_1");
    while (rclcpp::ok() && !flag_exit_.load())
    {
        bool has_scan = false;
        {
            std::lock_guard<std::mutex> scan_guard(lock_scan_);
            has_scan = !pcd_scan_cur_->IsEmpty();
        }
        if (has_scan)
        {
            break;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Waiting for cloud_registered_1...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!rclcpp::ok() || flag_exit_.load())
    {
        return;
    }

    {
        std::lock_guard<std::mutex> pose_guard(lock_mat_odom2map_);
        mat_odom2map_ = mat_initialpose_;
        mat_baselink2map_ = mat_odom2map_ * mat_baselink2odom_;
    }
    LocalizationInitialize();
    if (!rclcpp::ok() || flag_exit_.load())
    {
        return;
    }

    Eigen::Matrix4d init_baselink2map = Eigen::Matrix4d::Identity();
    {
        std::lock_guard<std::mutex> pose_guard(lock_mat_odom2map_);
        init_baselink2map = mat_odom2map_ * mat_baselink2odom_;
        mat_baselink2map_ = init_baselink2map;
        mat_odom2map_kalman_ = mat_odom2map_;
    }
    const double init_x = init_baselink2map(0, 3);
    const double init_y = init_baselink2map(1, 3);
    const double init_z = init_baselink2map(2, 3);
    if (kf_param_x_.size() >= 2 && kf_param_y_.size() >= 2 && kf_param_z_.size() >= 2)
    {
        kf_baselink_x_.KalmanFilterInit(kf_param_x_[0], kf_param_x_[1], init_x, 1);
        kf_baselink_y_.KalmanFilterInit(kf_param_y_[0], kf_param_y_[1], init_y, 1);
        kf_baselink_z_.KalmanFilterInit(kf_param_z_[0], kf_param_z_[1], init_z, 1);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid Kalman filter parameters");
    }
    kalman_filter_odom2map_.KalmanFilterInit(
        kalman_processVar2_, kalman_estimatedMeasVar2_, init_z, 1);
    loc_initialized_.store(true);

    auto pcd_scan = std::make_shared<open3d::geometry::PointCloud>();
    auto source = std::make_shared<open3d::geometry::PointCloud>();
    auto target = std::make_shared<open3d::geometry::PointCloud>();
    auto map_fine_crop = std::make_shared<open3d::geometry::PointCloud>();
    auto obb_map = std::make_shared<open3d::geometry::OrientedBoundingBox>();
    auto obb_scan = std::make_shared<open3d::geometry::OrientedBoundingBox>();
    obb_map->extent_ = Eigen::Vector3d(60, 60, 40);
    obb_scan->extent_ = Eigen::Vector3d(60, 60, 40);

    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / loc_frequence_));
    auto next_run = std::chrono::steady_clock::now();
    unsigned long long last_processed_scan_generation = 0;
    int scan_count = 0;
    int pending_large_correction_count = 0;
    Eigen::Matrix4d pending_large_candidate = Eigen::Matrix4d::Identity();
    auto pending_large_candidate_time = std::chrono::steady_clock::time_point::min();
    unsigned int observed_manual_pose_generation = manual_pose_generation_.load();
    const std::string save_path = "/tmp/open3d_loc_scan_";

    while (rclcpp::ok() && !flag_exit_.load())
    {
        std::this_thread::sleep_until(next_run);
        next_run += period;
        const auto now_steady = std::chrono::steady_clock::now();
        if (now_steady > next_run + period)
        {
            next_run = now_steady + period;
        }

        const unsigned int current_manual_pose_generation =
            manual_pose_generation_.load();
        if (current_manual_pose_generation != observed_manual_pose_generation)
        {
            observed_manual_pose_generation = current_manual_pose_generation;
            pending_large_correction_count = 0;
            pending_large_candidate = Eigen::Matrix4d::Identity();
            pending_large_candidate_time = std::chrono::steady_clock::time_point::min();
            map_fine_crop->Clear();
            last_loc_ = Eigen::Vector3d(0.0, 0.0, -5000.0);
            RCLCPP_INFO(this->get_logger(),
                        "Manual pose reset detected: cleared ICP candidate history and submap cache");
        }

        const auto loc_start = std::chrono::steady_clock::now();

        unsigned long long current_scan_generation = 0;
        double current_scan_stamp = 0.0;
        {
            std::lock_guard<std::mutex> scan_guard(lock_scan_);
            current_scan_generation = scan_generation_.load();
            current_scan_stamp = timestamp_scan_seconds_;
            if (pcd_scan_cur_->IsEmpty())
            {
                pcd_scan->Clear();
            }
            else
            {
                *pcd_scan = *pcd_scan_cur_;
            }
        }
        if (pcd_scan->IsEmpty() ||
            current_scan_generation == last_processed_scan_generation)
        {
            loc_fitness_.store(0.0);
            continue;
        }

        double current_odom_stamp = 0.0;
        Eigen::Matrix4d mat_baselink2odom_cur = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d mat_baselink2map_cur = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d current_odom2map = Eigen::Matrix4d::Identity();
        {
            std::lock_guard<std::mutex> pose_guard(lock_mat_odom2map_);
            mat_baselink2odom_cur = mat_baselink2odom_;
            current_odom2map = mat_odom2map_;
            mat_baselink2map_cur = current_odom2map * mat_baselink2odom_cur;
            current_odom_stamp = timestamp_pose_odom_seconds_;
            if (filter_odom2map_)
            {
                kalman_filter_odom2map_.inputLatestNoisyMeasurement(current_odom2map(2, 3));
                mat_odom2map_kalman_ = current_odom2map;
                mat_odom2map_kalman_(2, 3) =
                    kalman_filter_odom2map_.getLatestEstimatedMeasurement();
            }
        }
        const double scan_odom_skew = std::fabs(current_scan_stamp - current_odom_stamp);
        if (!std::isfinite(current_scan_stamp) || !std::isfinite(current_odom_stamp) ||
            current_scan_stamp <= 0.0 || current_odom_stamp <= 0.0 ||
            scan_odom_skew > max_scan_odom_time_skew_sec_)
        {
            loc_fitness_.store(0.0);
            pending_large_correction_count = 0;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Skipping ICP until cloud/odometry stamps match (cloud=%.6f odom=%.6f skew=%.4fs max=%.4fs)",
                current_scan_stamp, current_odom_stamp, scan_odom_skew,
                max_scan_odom_time_skew_sec_);
            continue;
        }
        last_processed_scan_generation = current_scan_generation;
        const unsigned int iteration_manual_pose_generation =
            manual_pose_generation_.load();
        if (!IsRigidTransform(mat_baselink2odom_cur) ||
            !IsRigidTransform(mat_baselink2map_cur) ||
            !IsRigidTransform(current_odom2map))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Skipping ICP: invalid pose snapshot");
            loc_fitness_.store(0.0);
            pending_large_correction_count = 0;
            continue;
        }

        const Eigen::Vector3d current_location =
            mat_baselink2map_cur.block<3, 1>(0, 3);
        if (map_fine_crop->IsEmpty() ||
            ComputeMotionDis(last_loc_, current_location) > dis_updatemap_)
        {
            last_loc_ = current_location;
            obb_map->center_ = current_location;
            obb_map->R_ = mat_baselink2map_cur.block<3, 3>(0, 0);
            *map_fine_crop = *pcd_map_fine_->Crop(*obb_map);
        }

        obb_scan->center_ = mat_baselink2odom_cur.block<3, 1>(0, 3);
        obb_scan->R_ = mat_baselink2odom_cur.block<3, 3>(0, 0);
        *target = *map_fine_crop;
        if (target->points_.size() > static_cast<size_t>(maxpoints_target_))
        {
            target = target->RandomDownSample(
                static_cast<double>(maxpoints_target_) / target->points_.size());
        }
        source = pcd_scan->Crop(*obb_scan);
        source = source->VoxelDownSample(voxelsize_fine_);
        if (source->points_.size() > static_cast<size_t>(maxpoints_source_))
        {
            source = source->RandomDownSample(
                static_cast<double>(maxpoints_source_) / source->points_.size());
        }

        if (source->points_.size() < static_cast<size_t>(min_icp_source_points_) ||
            target->points_.size() < static_cast<size_t>(min_icp_target_points_))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Skipping ICP: insufficient points (source=%zu/%d target=%zu/%d)",
                source->points_.size(), min_icp_source_points_,
                target->points_.size(), min_icp_target_points_);
            loc_fitness_.store(0.0);
            pending_large_correction_count = 0;
            continue;
        }

        const auto registration = pcd_tools::RegistrationIcp(
            source, target, voxelsize_fine_ * 2, current_odom2map, 1);
        const Eigen::Matrix4d correction = registration.transformation_;
        const Eigen::Matrix4d candidate_odom2map = correction * current_odom2map;
        // Score with the same correspondence radius used by ICP itself. A
        // wider 4x radius made parallel corridor walls look valid even after a
        // large lateral/yaw mismatch, so a bad correction could accumulate.
        const auto evaluation = open3d::pipelines::registration::EvaluateRegistration(
            *source, *target, voxelsize_fine_ * 2, candidate_odom2map);
        const double fitness = evaluation.fitness_;
        const double inlier_rmse = evaluation.inlier_rmse_;

        const double translation_step = correction.block<3, 1>(0, 3).norm();
        const double rotation_step_deg = RotationAngleDegrees(
            correction.block<3, 3>(0, 0));
        const bool valid_result =
            IsRigidTransform(correction) && IsRigidTransform(candidate_odom2map) &&
            std::isfinite(fitness) && std::isfinite(inlier_rmse) &&
            std::isfinite(translation_step) && std::isfinite(rotation_step_deg);
        const bool fitness_ok = fitness > threshold_fitness_ &&
                                inlier_rmse <= max_icp_inlier_rmse_;
        const bool within_step_gate =
            translation_step <= max_icp_translation_step_ &&
            rotation_step_deg <= max_icp_rotation_step_deg_;
        const bool immediate_step =
            translation_step <= immediate_icp_translation_step_ &&
            rotation_step_deg <= immediate_icp_rotation_step_deg_;

        bool large_step_confirmed = false;
        if (valid_result && fitness_ok && within_step_gate && !immediate_step)
        {
            const auto candidate_time = std::chrono::steady_clock::now();
            const bool pending_is_fresh =
                pending_large_correction_count > 0 &&
                pending_large_candidate_time != std::chrono::steady_clock::time_point::min() &&
                std::chrono::duration<double>(candidate_time - pending_large_candidate_time).count() <=
                    icp_candidate_max_age_sec_;
            bool consistent_with_pending = false;
            if (pending_is_fresh && IsRigidTransform(pending_large_candidate))
            {
                const Eigen::Matrix4d candidate_delta =
                    pending_large_candidate.inverse() * candidate_odom2map;
                consistent_with_pending =
                    IsRigidTransform(candidate_delta) &&
                    candidate_delta.block<3, 1>(0, 3).norm() <=
                        icp_candidate_consistency_translation_ &&
                    RotationAngleDegrees(candidate_delta.block<3, 3>(0, 0)) <=
                        icp_candidate_consistency_rotation_deg_;
            }

            if (consistent_with_pending)
            {
                ++pending_large_correction_count;
            }
            else
            {
                pending_large_candidate = candidate_odom2map;
                pending_large_correction_count = 1;
            }
            pending_large_candidate_time = candidate_time;
            large_step_confirmed =
                pending_large_correction_count >= large_correction_confirmations_;
        }
        else
        {
            pending_large_correction_count = 0;
        }

        bool accepted =
            valid_result && fitness_ok && within_step_gate &&
            (immediate_step || large_step_confirmed);
        bool stale_after_manual_pose = false;

        if (accepted)
        {
            std::lock_guard<std::mutex> pose_guard(lock_mat_odom2map_);
            stale_after_manual_pose =
                manual_pose_generation_.load() != iteration_manual_pose_generation;
            if (!stale_after_manual_pose)
            {
                mat_odom2map_ = candidate_odom2map;
                mat_baselink2map_ = mat_odom2map_ * mat_baselink2odom_;
            }
        }
        if (stale_after_manual_pose)
        {
            accepted = false;
            observed_manual_pose_generation = manual_pose_generation_.load();
            pending_large_correction_count = 0;
            pending_large_candidate = Eigen::Matrix4d::Identity();
            pending_large_candidate_time = std::chrono::steady_clock::time_point::min();
            map_fine_crop->Clear();
            last_loc_ = Eigen::Vector3d(0.0, 0.0, -5000.0);
            RCLCPP_WARN(this->get_logger(),
                        "Discarding stale ICP result after manual relocalization");
        }
        else if (accepted)
        {
            pending_large_correction_count = 0;
        }
        loc_fitness_.store(accepted ? fitness : 0.0);

        if (stale_after_manual_pose || accepted)
        {
            // Accepted corrections need no warning. A stale result was already
            // reported above and must not fall through to a misleading gate log.
        }
        else if (!valid_result)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Rejecting ICP: non-finite or non-rigid result");
        }
        else if (!within_step_gate)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Rejecting ICP jump: correction %.3f m / %.2f deg exceeds %.3f m / %.2f deg",
                translation_step, rotation_step_deg,
                max_icp_translation_step_, max_icp_rotation_step_deg_);
        }
        else if (!fitness_ok)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Rejecting ICP quality: fitness %.3f (min %.3f), rmse %.3f (max %.3f)",
                fitness, threshold_fitness_, inlier_rmse, max_icp_inlier_rmse_);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Holding large ICP correction %.3f m / %.2f deg for temporal confirmation (%d/%d)",
                translation_step, rotation_step_deg,
                pending_large_correction_count, large_correction_confirmations_);
        }

        if (save_scan_ && accepted)
        {
            auto pcd_scan2map = std::make_shared<open3d::geometry::PointCloud>();
            *pcd_scan2map = *source;
            pcd_scan2map->Transform(candidate_odom2map);
            open3d::io::WritePointCloud(
                save_path + std::to_string(scan_count++) + ".ply", *pcd_scan2map);
        }

        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - loc_start).count();
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "ICP: accepted=%s fitness=%.3f rmse=%.3f correction=%.3f m/%.2f deg cost=%.1f ms",
            accepted ? "true" : "false", fitness, inlier_rmse,
            translation_step, rotation_step_deg, elapsed_ms);
    }
}

void GloabalLocalization::StartLoc()
{
    thread_loc_ = std::thread(&GloabalLocalization::Localization, this);
}

void GloabalLocalization::CallbackInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose)
{
    Eigen::Quaterniond rotation_q(
        initialpose->pose.pose.orientation.w,
        initialpose->pose.pose.orientation.x,
        initialpose->pose.pose.orientation.y,
        initialpose->pose.pose.orientation.z);
    if (!rotation_q.coeffs().allFinite() || rotation_q.norm() < 1e-9)
    {
        RCLCPP_WARN(this->get_logger(), "Rejecting initial pose: invalid quaternion");
        return;
    }
    rotation_q.normalize();

    // /initialpose describes the robot (base_link) pose in map, not map->odom.
    // Convert map_T_base into map_T_odom using the latest odom_T_base. Directly
    // assigning map_T_base to map_T_odom works only while odometry is identity
    // and causes a large jump when the operator relocalizes after driving.
    Eigen::Matrix4d requested_baselink2map = Eigen::Matrix4d::Identity();
    requested_baselink2map.block<3, 3>(0, 0) = rotation_q.matrix();
    requested_baselink2map.block<3, 1>(0, 3) = Eigen::Vector3d(
        initialpose->pose.pose.position.x,
        initialpose->pose.pose.position.y,
        initialpose->pose.pose.position.z);
    if (!IsRigidTransform(requested_baselink2map))
    {
        RCLCPP_WARN(this->get_logger(), "Rejecting initial pose: non-finite transform");
        return;
    }

    Eigen::Matrix4d new_odom2map = Eigen::Matrix4d::Identity();
    {
        std::lock_guard<std::mutex> pose_guard(lock_mat_odom2map_);
        if (!IsRigidTransform(mat_baselink2odom_))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Rejecting initial pose: latest odometry transform is invalid");
            return;
        }
        new_odom2map = requested_baselink2map * mat_baselink2odom_.inverse();
        if (!IsRigidTransform(new_odom2map))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Rejecting initial pose: computed map->odom is invalid");
            return;
        }
        mat_initialpose_ = new_odom2map;
        mat_odom2map_ = new_odom2map;
        mat_odom2map_kalman_ = new_odom2map;
        mat_baselink2map_ = requested_baselink2map;
        // Bump the generation under the same pose lock. ICP writers can then
        // atomically prove that their snapshot was not superseded by this pose.
        manual_pose_generation_.fetch_add(1);
    }
    loc_fitness_.store(0.0);
    RCLCPP_WARN(this->get_logger(),
                "Manual relocalization applied: map->odom recomputed from map->base_link and current odometry");
}

double GloabalLocalization::ComputeMotionDis(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
    return std::sqrt(std::pow(a.x() - b.x(), 2) + std::pow(a.y() - b.y(), 2) + std::pow(a.z() - b.z(), 2));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GloabalLocalization>();

    // 使用多线程执行器，可以指定线程数
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
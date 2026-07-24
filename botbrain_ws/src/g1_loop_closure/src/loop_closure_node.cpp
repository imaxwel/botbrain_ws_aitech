#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "g1_loop_closure/pose_graph.hpp"

namespace
{

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using g1_loop_closure::Between;
using g1_loop_closure::Compose;
using g1_loop_closure::kPi;
using g1_loop_closure::Pose2;
using g1_loop_closure::WrapAngle;

std::string NormalizeFrame(std::string frame)
{
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  return frame;
}

geometry_msgs::msg::Pose ToRosPose(const Pose2 &pose, const double z)
{
  geometry_msgs::msg::Pose message;
  message.position.x = pose.x;
  message.position.y = pose.y;
  message.position.z = z;
  message.orientation.z = std::sin(pose.yaw * 0.5);
  message.orientation.w = std::cos(pose.yaw * 0.5);
  return message;
}

}  // namespace

class LoopClosureNode final : public rclcpp::Node
{
public:
  LoopClosureNode()
  : Node("loop_closure")
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered_1");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry_loc");
    world_frame_ = NormalizeFrame(declare_parameter<std::string>("world_frame", "camera_init"));
    odom_sync_tolerance_sec_ = declare_parameter<double>("odom_sync_tolerance_sec", 0.035);
    odom_history_size_ = static_cast<size_t>(std::max<int64_t>(
        20, declare_parameter<int>("odom_history_size", 300)));

    keyframe_distance_m_ = declare_parameter<double>("keyframe_distance_m", 1.5);
    keyframe_yaw_rad_ = declare_parameter<double>("keyframe_yaw_deg", 15.0) * kPi / 180.0;
    keyframe_time_sec_ = declare_parameter<double>("keyframe_time_sec", 2.0);
    keyframe_voxel_m_ = declare_parameter<double>("keyframe_voxel_m", 0.20);
    keyframe_max_points_ = static_cast<size_t>(std::max<int64_t>(
        200, declare_parameter<int>("keyframe_max_points", 5000)));
    max_keyframes_ = static_cast<size_t>(std::max<int64_t>(
        20, declare_parameter<int>("max_keyframes", 800)));
    query_interval_sec_ = declare_parameter<double>("query_interval_sec", 5.0);
    min_loop_time_sec_ = declare_parameter<double>("min_loop_time_sec", 30.0);
    min_loop_path_distance_m_ = declare_parameter<double>("min_loop_path_distance_m", 12.0);

    descriptor_rings_ = static_cast<int>(std::max<int64_t>(
        4, declare_parameter<int>("descriptor_rings", 20)));
    descriptor_sectors_ = static_cast<int>(std::max<int64_t>(
        12, declare_parameter<int>("descriptor_sectors", 60)));
    descriptor_max_range_m_ = declare_parameter<double>("descriptor_max_range_m", 25.0);
    descriptor_min_similarity_ = declare_parameter<double>("descriptor_min_similarity", 0.82);

    icp_max_correspondence_m_ = declare_parameter<double>("icp_max_correspondence_m", 0.80);
    icp_max_iterations_ = static_cast<int>(std::max<int64_t>(
        5, declare_parameter<int>("icp_max_iterations", 60)));
    icp_max_fitness_m_ = declare_parameter<double>("icp_max_fitness_m", 0.18);
    icp_min_inlier_ratio_ = declare_parameter<double>("icp_min_inlier_ratio", 0.35);
    icp_max_translation_m_ = declare_parameter<double>("icp_max_translation_m", 8.0);
    icp_max_z_translation_m_ = declare_parameter<double>("icp_max_z_translation_m", 0.40);
    icp_max_yaw_rad_ = declare_parameter<double>("icp_max_yaw_deg", 50.0) * kPi / 180.0;
    icp_max_roll_pitch_rad_ =
      declare_parameter<double>("icp_max_roll_pitch_deg", 15.0) * kPi / 180.0;

    // Phase 1 is intentionally the default: it only observes, retrieves and
    // verifies loop candidates.  Phase 2 is explicitly enabled after those
    // observations have been reviewed, and still remains diagnostic-only.
    enable_pose_graph_ = declare_parameter<bool>("enable_pose_graph", false);
    optimizer_iterations_ = static_cast<int>(std::max<int64_t>(
        1, declare_parameter<int>("optimizer_iterations", 8)));
    odom_edge_xy_weight_ = declare_parameter<double>("odom_edge_xy_weight", 1.0);
    odom_edge_yaw_weight_ = declare_parameter<double>("odom_edge_yaw_weight", 2.0);
    loop_edge_xy_weight_ = declare_parameter<double>("loop_edge_xy_weight", 8.0);
    loop_edge_yaw_weight_ = declare_parameter<double>("loop_edge_yaw_weight", 10.0);
    map_preview_max_points_ = static_cast<size_t>(std::max<int64_t>(
        10000, declare_parameter<int>("map_preview_max_points", 250000)));
    export_optimized_map_path_ = declare_parameter<std::string>("export_optimized_map_path", "");

    const auto cloud_qos = rclcpp::SensorDataQoS().keep_last(1);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, cloud_qos,
      std::bind(&LoopClosureNode::CloudCallback, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(100)).reliable(),
      std::bind(&LoopClosureNode::OdomCallback, this, std::placeholders::_1));

    keyframe_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/loop_closure/keyframe_path", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    optimized_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/loop_closure/optimized_path", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    candidate_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/loop_closure/candidate_poses", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/loop_closure/markers", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    map_preview_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/loop_closure/optimized_map_preview", rclcpp::SensorDataQoS().keep_last(1));
    diagnostic_publisher_ = create_publisher<std_msgs::msg::String>(
      "/loop_closure/diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    export_service_ = create_service<std_srvs::srv::Trigger>(
      "/loop_closure/export_optimized_map",
      std::bind(
        &LoopClosureNode::ExportOptimizedMap, this,
        std::placeholders::_1, std::placeholders::_2));
    parameter_callback_ = add_on_set_parameters_callback(
      std::bind(&LoopClosureNode::HandleParameterUpdate, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Loop-closure observer started: cloud=%s odom=%s phase=%s. It never publishes TF "
      "or modifies FAST-LIO odometry/maps.",
      cloud_topic_.c_str(), odom_topic_.c_str(),
      enable_pose_graph_ ? "2-diagnostic-pose-graph" : "1-observation-only");
  }

private:
  struct OdomSample
  {
    rclcpp::Time stamp;
    Pose2 pose;
    Eigen::Isometry3d pose_3d{Eigen::Isometry3d::Identity()};
  };

  struct Keyframe
  {
    size_t id{0};
    rclcpp::Time stamp;
    Pose2 raw_pose;
    double raw_z{0.0};
    double path_distance{0.0};
    Cloud::Ptr local_cloud{new Cloud};
    std::vector<float> descriptor;
  };

  using Edge = g1_loop_closure::PoseGraphEdge;

  struct DescriptorMatch
  {
    size_t keyframe_index{0};
    double similarity{-1.0};
    int yaw_shift{0};
  };

  struct CandidateState
  {
    bool valid{false};
    bool accepted{false};
    size_t from{0};
    size_t to{0};
    double descriptor_similarity{0.0};
    double icp_fitness{std::numeric_limits<double>::infinity()};
    double inlier_ratio{0.0};
  };

  rcl_interfaces::msg::SetParametersResult HandleParameterUpdate(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "unchanged";
    for (const auto &parameter : parameters) {
      if (parameter.get_name() == "export_optimized_map_path") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          result.successful = false;
          result.reason = "export_optimized_map_path must be a string";
          return result;
        }
        export_optimized_map_path_ = parameter.as_string();
        result.reason = export_optimized_map_path_.empty() ?
          "optimized-map export disabled" : "optimized-map export path updated";
        continue;
      }
      if (parameter.get_name() != "enable_pose_graph") {
        continue;
      }
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "enable_pose_graph must be boolean";
        return result;
      }
      const bool requested = parameter.as_bool();
      if (!requested && enable_pose_graph_) {
        result.successful = false;
        result.reason =
          "disabling an active diagnostic graph is not supported; restart for phase 1";
        return result;
      }
      if (requested && !enable_pose_graph_) {
        enable_pose_graph_ = true;
        for (; active_loop_edge_count_ < accepted_loop_edges_.size(); ++active_loop_edge_count_) {
          edges_.push_back(accepted_loop_edges_[active_loop_edge_count_]);
        }
        RCLCPP_INFO(
          get_logger(),
          "Loop closure phase 2 enabled with %zu accepted constraint(s); optimizing "
          "diagnostic graph only (FAST-LIO TF/odometry remain untouched)",
          accepted_loop_edges_.size());
        OptimizePoseGraph();
        PublishPathsAndMarkers();
        PublishOptimizedMapPreview();
        result.reason = "phase 2 diagnostic pose graph enabled";
      }
    }
    return result;
  }

  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto &orientation = message->pose.pose.orientation;
    Eigen::Quaterniond quaternion(
      orientation.w, orientation.x, orientation.y, orientation.z);
    const auto &position = message->pose.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !quaternion.coeffs().allFinite() ||
      quaternion.norm() < 1e-8)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring non-finite FAST-LIO odometry for loop closure");
      return;
    }
    quaternion.normalize();
    OdomSample sample;
    sample.stamp = rclcpp::Time(message->header.stamp);
    sample.pose = {position.x, position.y,
      std::atan2(2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y()),
        1.0 - 2.0 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z()))};
    sample.pose_3d.linear() = quaternion.toRotationMatrix();
    sample.pose_3d.translation() = Eigen::Vector3d(position.x, position.y, position.z);

    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_history_.push_back(std::move(sample));
    while (odom_history_.size() > odom_history_size_) {
      odom_history_.pop_front();
    }
  }

  std::optional<OdomSample> FindOdom(const rclcpp::Time &stamp)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (odom_history_.empty()) {
      return std::nullopt;
    }
    const OdomSample *best = nullptr;
    double best_difference = std::numeric_limits<double>::infinity();
    for (const auto &sample : odom_history_) {
      const double difference = std::fabs((sample.stamp - stamp).seconds());
      if (difference < best_difference) {
        best_difference = difference;
        best = &sample;
      }
    }
    if (best == nullptr || best_difference > odom_sync_tolerance_sec_) {
      return std::nullopt;
    }
    return *best;
  }

  Cloud::Ptr Downsample(const Cloud::ConstPtr &input) const
  {
    auto output = std::make_shared<Cloud>();
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(
      static_cast<float>(keyframe_voxel_m_), static_cast<float>(keyframe_voxel_m_),
      static_cast<float>(keyframe_voxel_m_));
    voxel.setInputCloud(input);
    voxel.filter(*output);
    if (output->size() <= keyframe_max_points_) {
      return output;
    }
    auto bounded = std::make_shared<Cloud>();
    bounded->reserve(keyframe_max_points_);
    const size_t stride = (output->size() + keyframe_max_points_ - 1) / keyframe_max_points_;
    for (size_t index = 0; index < output->size(); index += stride) {
      bounded->push_back((*output)[index]);
    }
    bounded->width = static_cast<uint32_t>(bounded->size());
    bounded->height = 1;
    bounded->is_dense = output->is_dense;
    return bounded;
  }

  std::vector<float> BuildDescriptor(const Cloud::ConstPtr &cloud) const
  {
    const size_t cells = static_cast<size_t>(descriptor_rings_ * descriptor_sectors_);
    std::vector<float> height(cells, 0.0F);
    std::vector<float> density(cells, 0.0F);
    for (const auto &point : cloud->points) {
      if (!pcl::isFinite(point)) {
        continue;
      }
      const double radius = std::hypot(point.x, point.y);
      if (radius < 0.2 || radius >= descriptor_max_range_m_) {
        continue;
      }
      const int ring = std::min(
        descriptor_rings_ - 1,
        static_cast<int>(radius / descriptor_max_range_m_ * descriptor_rings_));
      const double angle = std::atan2(point.y, point.x) + kPi;
      const int sector = std::min(
        descriptor_sectors_ - 1,
        static_cast<int>(angle / (2.0 * kPi) * descriptor_sectors_));
      const size_t index = static_cast<size_t>(ring * descriptor_sectors_ + sector);
      // Height and density together are more discriminative than height alone
      // in a long corridor with repeated vertical walls.
      height[index] = std::max(height[index], std::max(0.0F, point.z + 1.5F));
      density[index] = std::min(1.0F, density[index] + 0.10F);
    }
    std::vector<float> descriptor;
    descriptor.reserve(cells * 2);
    descriptor.insert(descriptor.end(), height.begin(), height.end());
    descriptor.insert(descriptor.end(), density.begin(), density.end());
    return descriptor;
  }

  std::pair<double, int> DescriptorSimilarity(
    const std::vector<float> &first, const std::vector<float> &second) const
  {
    const size_t cells = static_cast<size_t>(descriptor_rings_ * descriptor_sectors_);
    if (first.size() != cells * 2 || second.size() != cells * 2) {
      return {-1.0, 0};
    }
    double best_score = -1.0;
    int best_shift = 0;
    for (int shift = 0; shift < descriptor_sectors_; ++shift) {
      double dot = 0.0;
      double first_norm = 0.0;
      double second_norm = 0.0;
      for (int ring = 0; ring < descriptor_rings_; ++ring) {
        for (int sector = 0; sector < descriptor_sectors_; ++sector) {
          const size_t lhs = static_cast<size_t>(ring * descriptor_sectors_ + sector);
          const size_t rhs = static_cast<size_t>(
            ring * descriptor_sectors_ + (sector + shift) % descriptor_sectors_);
          for (size_t channel = 0; channel < 2; ++channel) {
            const double weight = channel == 0 ? 1.0 : 0.6;
            const double a = first[channel * cells + lhs] * weight;
            const double b = second[channel * cells + rhs] * weight;
            dot += a * b;
            first_norm += a * a;
            second_norm += b * b;
          }
        }
      }
      const double score = dot / std::sqrt(std::max(1e-12, first_norm * second_norm));
      if (score > best_score) {
        best_score = score;
        best_shift = shift;
      }
    }
    return {best_score, best_shift};
  }

  std::optional<DescriptorMatch> FindLoopCandidate(const Keyframe &current) const
  {
    if (keyframes_.size() < 3) {
      return std::nullopt;
    }
    DescriptorMatch best;
    for (const auto &candidate : keyframes_) {
      if (candidate.id == current.id ||
        (current.stamp - candidate.stamp).seconds() < min_loop_time_sec_ ||
        current.path_distance - candidate.path_distance < min_loop_path_distance_m_)
      {
        continue;
      }
      const auto [similarity, yaw_shift] = DescriptorSimilarity(
          current.descriptor, candidate.descriptor);
      if (similarity > best.similarity) {
        best = {candidate.id, similarity, yaw_shift};
      }
    }
    if (best.similarity < descriptor_min_similarity_) {
      return std::nullopt;
    }
    return best;
  }

  double ComputeInlierRatio(
    const Cloud::ConstPtr &source, const Cloud::ConstPtr &target,
    const Eigen::Matrix4f &transform) const
  {
    if (source->empty() || target->empty()) {
      return 0.0;
    }
    Cloud transformed;
    pcl::transformPointCloud(*source, transformed, transform);
    pcl::KdTreeFLANN<Point> tree;
    tree.setInputCloud(target);
    size_t inliers = 0;
    const float max_distance_sq = static_cast<float>(
      icp_max_correspondence_m_ * icp_max_correspondence_m_);
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    for (const auto &point : transformed.points) {
      if (tree.nearestKSearch(point, 1, indices, distances) > 0 && distances[0] <= max_distance_sq) {
        ++inliers;
      }
    }
    return static_cast<double>(inliers) / static_cast<double>(transformed.size());
  }

  bool VerifyLoop(
    const Keyframe &candidate, const Keyframe &current, const DescriptorMatch &match,
    Pose2 &measurement, double &fitness, double &inlier_ratio)
  {
    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setInputSource(current.local_cloud);
    icp.setInputTarget(candidate.local_cloud);
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_m_);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);

    const Pose2 odometry_guess = Between(candidate.raw_pose, current.raw_pose);
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    const double descriptor_yaw =
      -2.0 * kPi * static_cast<double>(match.yaw_shift) / descriptor_sectors_;
    const double yaw_guess = WrapAngle(odometry_guess.yaw + descriptor_yaw);
    initial_guess(0, 0) = static_cast<float>(std::cos(yaw_guess));
    initial_guess(0, 1) = static_cast<float>(-std::sin(yaw_guess));
    initial_guess(1, 0) = static_cast<float>(std::sin(yaw_guess));
    initial_guess(1, 1) = static_cast<float>(std::cos(yaw_guess));
    initial_guess(0, 3) = static_cast<float>(odometry_guess.x);
    initial_guess(1, 3) = static_cast<float>(odometry_guess.y);

    Cloud aligned;
    icp.align(aligned, initial_guess);
    if (!icp.hasConverged()) {
      fitness = std::numeric_limits<double>::infinity();
      inlier_ratio = 0.0;
      return false;
    }
    const Eigen::Matrix4f transform = icp.getFinalTransformation();
    if (!transform.allFinite()) {
      fitness = std::numeric_limits<double>::infinity();
      inlier_ratio = 0.0;
      return false;
    }
    fitness = icp.getFitnessScore(icp_max_correspondence_m_);
    inlier_ratio = ComputeInlierRatio(current.local_cloud, candidate.local_cloud, transform);
    measurement = {
      transform(0, 3), transform(1, 3),
      std::atan2(transform(1, 0), transform(0, 0)),
    };
    const double translation = std::hypot(measurement.x, measurement.y);
    const double z_translation = std::fabs(transform(2, 3));
    const double roll = std::atan2(transform(2, 1), transform(2, 2));
    const double pitch = std::asin(std::clamp(-transform(2, 0), -1.0F, 1.0F));
    return std::isfinite(fitness) && fitness <= icp_max_fitness_m_ &&
      inlier_ratio >= icp_min_inlier_ratio_ &&
      translation <= icp_max_translation_m_ &&
      z_translation <= icp_max_z_translation_m_ &&
      std::fabs(measurement.yaw) <= icp_max_yaw_rad_ &&
      std::fabs(roll) <= icp_max_roll_pitch_rad_ &&
      std::fabs(pitch) <= icp_max_roll_pitch_rad_;
  }

  bool ShouldCreateKeyframe(const OdomSample &odom, const rclcpp::Time &stamp) const
  {
    if (keyframes_.empty()) {
      return true;
    }
    const auto &last = keyframes_.back();
    const double distance = std::hypot(
      odom.pose.x - last.raw_pose.x, odom.pose.y - last.raw_pose.y);
    const double yaw = std::fabs(WrapAngle(odom.pose.yaw - last.raw_pose.yaw));
    return distance >= keyframe_distance_m_ || yaw >= keyframe_yaw_rad_ ||
      (stamp - last.stamp).seconds() >= keyframe_time_sec_;
  }

  void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (NormalizeFrame(message->header.frame_id) != world_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring %s frame '%s'; loop closure needs FAST-LIO world cloud in '%s'",
        cloud_topic_.c_str(), message->header.frame_id.c_str(), world_frame_.c_str());
      return;
    }
    const rclcpp::Time stamp(message->header.stamp);
    const auto odom = FindOdom(stamp);
    if (!odom) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for Odometry_loc paired with loop-closure cloud");
      return;
    }
    if (!ShouldCreateKeyframe(*odom, stamp)) {
      return;
    }
    if (keyframes_.size() >= max_keyframes_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Loop closure reached max_keyframes=%zu; keeping the existing diagnostic graph", max_keyframes_);
      return;
    }

    auto world_cloud = std::make_shared<Cloud>();
    pcl::fromROSMsg(*message, *world_cloud);
    if (world_cloud->empty()) {
      return;
    }
    const Eigen::Matrix4f world_to_body = odom->pose_3d.inverse().matrix().cast<float>();
    auto local_cloud = std::make_shared<Cloud>();
    pcl::transformPointCloud(*world_cloud, *local_cloud, world_to_body);
    local_cloud = Downsample(local_cloud);
    if (local_cloud->size() < 100) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping sparse loop-closure keyframe (%zu points)", local_cloud->size());
      return;
    }

    Keyframe keyframe;
    keyframe.id = keyframes_.size();
    keyframe.stamp = stamp;
    keyframe.raw_pose = odom->pose;
    keyframe.raw_z = odom->pose_3d.translation().z();
    keyframe.path_distance = keyframes_.empty() ? 0.0 :
      keyframes_.back().path_distance + std::hypot(
      odom->pose.x - keyframes_.back().raw_pose.x,
      odom->pose.y - keyframes_.back().raw_pose.y);
    keyframe.local_cloud = std::move(local_cloud);
    keyframe.descriptor = BuildDescriptor(keyframe.local_cloud);

    if (!keyframes_.empty()) {
      edges_.push_back({
        keyframes_.back().id, keyframe.id,
        Between(keyframes_.back().raw_pose, keyframe.raw_pose),
        odom_edge_xy_weight_, odom_edge_yaw_weight_, false});
      optimized_poses_.push_back(Compose(
          optimized_poses_.back(), Between(keyframes_.back().raw_pose, keyframe.raw_pose)));
    } else {
      optimized_poses_.push_back(keyframe.raw_pose);
    }
    keyframes_.push_back(std::move(keyframe));
    PublishPathsAndMarkers();

    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_query_time_).count() < query_interval_sec_) {
      return;
    }
    last_query_time_ = now;
    const Keyframe &current = keyframes_.back();
    const auto match = FindLoopCandidate(current);
    if (!match) {
      PublishDiagnostic("no_descriptor_candidate", std::nullopt);
      return;
    }

    const Keyframe &candidate = keyframes_.at(match->keyframe_index);
    Pose2 measurement;
    double fitness = std::numeric_limits<double>::infinity();
    double inlier_ratio = 0.0;
    const bool accepted = VerifyLoop(candidate, current, *match, measurement, fitness, inlier_ratio);
    last_candidate_ = {
      true, accepted, candidate.id, current.id, match->similarity, fitness, inlier_ratio};
    PublishCandidatePoses(candidate, current);

    if (accepted) {
      const Edge loop_edge{
        candidate.id, current.id, measurement,
        loop_edge_xy_weight_, loop_edge_yaw_weight_, true};
      accepted_loop_edges_.push_back(loop_edge);
      RCLCPP_INFO(
        get_logger(),
        "LOOP ACCEPTED candidate=%zu current=%zu descriptor=%.3f fitness=%.3f inliers=%.3f "
        "constraint=(%.2f, %.2f, %.1fdeg)%s",
        candidate.id, current.id, match->similarity, fitness, inlier_ratio,
        measurement.x, measurement.y, measurement.yaw * 180.0 / kPi,
        enable_pose_graph_ ? "; optimizing diagnostic SE2 graph only" :
        "; recorded for phase 1 observation only");
      if (enable_pose_graph_) {
        edges_.push_back(loop_edge);
        ++active_loop_edge_count_;
        OptimizePoseGraph();
        PublishOptimizedMapPreview();
      }
    } else {
      RCLCPP_INFO(
        get_logger(),
        "LOOP REJECTED candidate=%zu current=%zu descriptor=%.3f fitness=%.3f inliers=%.3f",
        candidate.id, current.id, match->similarity, fitness, inlier_ratio);
    }
    PublishPathsAndMarkers();
    PublishDiagnostic(accepted ? "accepted" : "rejected_icp", last_candidate_);
  }

  void OptimizePoseGraph()
  {
    if (keyframes_.size() < 3 || optimized_poses_.size() != keyframes_.size()) {
      return;
    }
    std::string failure_reason;
    if (!g1_loop_closure::OptimizePoseGraph(
        optimized_poses_, edges_, optimizer_iterations_, &failure_reason))
    {
      RCLCPP_ERROR(
        get_logger(), "Loop pose-graph optimization failed: %s; preserving the last diagnostic trajectory",
        failure_reason.c_str());
    }
  }

  Cloud::Ptr BuildOptimizedMapPreview() const
  {
    auto preview = std::make_shared<Cloud>();
    if (keyframes_.empty() || optimized_poses_.size() != keyframes_.size()) {
      return preview;
    }
    const size_t per_keyframe = std::max<size_t>(
      1, map_preview_max_points_ / keyframes_.size());
    for (size_t index = 0; index < keyframes_.size(); ++index) {
      const auto &keyframe = keyframes_[index];
      const auto &pose = optimized_poses_[index];
      const size_t stride = std::max<size_t>(1, keyframe.local_cloud->size() / per_keyframe);
      const float c = static_cast<float>(std::cos(pose.yaw));
      const float s = static_cast<float>(std::sin(pose.yaw));
      for (size_t point_index = 0; point_index < keyframe.local_cloud->size(); point_index += stride) {
        const auto &point = (*keyframe.local_cloud)[point_index];
        preview->push_back({
          c * point.x - s * point.y + static_cast<float>(pose.x),
          s * point.x + c * point.y + static_cast<float>(pose.y),
          point.z + static_cast<float>(keyframe.raw_z)});
        if (preview->size() >= map_preview_max_points_) {
          break;
        }
      }
      if (preview->size() >= map_preview_max_points_) {
        break;
      }
    }
    preview->width = static_cast<uint32_t>(preview->size());
    preview->height = 1;
    preview->is_dense = true;
    return preview;
  }

  void PublishOptimizedMapPreview() const
  {
    const auto preview = BuildOptimizedMapPreview();
    if (preview->empty()) {
      return;
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*preview, message);
    message.header.frame_id = world_frame_;
    message.header.stamp = now();
    map_preview_publisher_->publish(message);
  }

  void PublishCandidatePoses(const Keyframe &candidate, const Keyframe &current) const
  {
    geometry_msgs::msg::PoseArray message;
    message.header.frame_id = world_frame_;
    message.header.stamp = now();
    message.poses.push_back(ToRosPose(candidate.raw_pose, candidate.raw_z));
    message.poses.push_back(ToRosPose(current.raw_pose, current.raw_z));
    candidate_publisher_->publish(message);
  }

  void PublishPathsAndMarkers() const
  {
    nav_msgs::msg::Path raw_path;
    nav_msgs::msg::Path optimized_path;
    raw_path.header.frame_id = world_frame_;
    optimized_path.header.frame_id = world_frame_;
    raw_path.header.stamp = now();
    optimized_path.header.stamp = raw_path.header.stamp;
    for (size_t index = 0; index < keyframes_.size(); ++index) {
      geometry_msgs::msg::PoseStamped raw_pose;
      raw_pose.header = raw_path.header;
      raw_pose.pose = ToRosPose(keyframes_[index].raw_pose, keyframes_[index].raw_z);
      raw_path.poses.push_back(raw_pose);
      geometry_msgs::msg::PoseStamped optimized_pose;
      optimized_pose.header = optimized_path.header;
      optimized_pose.pose = ToRosPose(optimized_poses_[index], keyframes_[index].raw_z);
      optimized_path.poses.push_back(optimized_pose);
    }
    keyframe_path_publisher_->publish(raw_path);
    if (enable_pose_graph_) {
      optimized_path_publisher_->publish(optimized_path);
    }

    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    visualization_msgs::msg::Marker keyframes;
    keyframes.header = raw_path.header;
    keyframes.ns = "loop_keyframes";
    keyframes.id = 0;
    keyframes.type = visualization_msgs::msg::Marker::POINTS;
    keyframes.action = visualization_msgs::msg::Marker::ADD;
    keyframes.scale.x = 0.12;
    keyframes.scale.y = 0.12;
    keyframes.color.r = 0.1F;
    keyframes.color.g = 0.9F;
    keyframes.color.b = 1.0F;
    keyframes.color.a = 0.9F;
    for (const auto &keyframe : keyframes_) {
      geometry_msgs::msg::Point point;
      point.x = keyframe.raw_pose.x;
      point.y = keyframe.raw_pose.y;
      point.z = keyframe.raw_z;
      keyframes.points.push_back(point);
    }
    array.markers.push_back(keyframes);

    visualization_msgs::msg::Marker loops;
    loops.header = raw_path.header;
    loops.ns = "accepted_loop_constraints";
    loops.id = 1;
    loops.type = visualization_msgs::msg::Marker::LINE_LIST;
    loops.action = visualization_msgs::msg::Marker::ADD;
    loops.scale.x = 0.05;
    loops.color.g = 1.0F;
    loops.color.a = 0.95F;
    for (const auto &edge : accepted_loop_edges_) {
      const auto &from = keyframes_.at(edge.from);
      const auto &to = keyframes_.at(edge.to);
      geometry_msgs::msg::Point first;
      first.x = from.raw_pose.x;
      first.y = from.raw_pose.y;
      first.z = from.raw_z;
      geometry_msgs::msg::Point second;
      second.x = to.raw_pose.x;
      second.y = to.raw_pose.y;
      second.z = to.raw_z;
      loops.points.push_back(first);
      loops.points.push_back(second);
    }
    array.markers.push_back(loops);

    if (last_candidate_.valid) {
      visualization_msgs::msg::Marker candidate;
      candidate.header = raw_path.header;
      candidate.ns = "last_loop_candidate";
      candidate.id = 2;
      candidate.type = visualization_msgs::msg::Marker::LINE_LIST;
      candidate.action = visualization_msgs::msg::Marker::ADD;
      candidate.scale.x = 0.08;
      candidate.color.r = last_candidate_.accepted ? 0.0F : 1.0F;
      candidate.color.g = last_candidate_.accepted ? 1.0F : 0.7F;
      candidate.color.b = last_candidate_.accepted ? 0.0F : 0.0F;
      candidate.color.a = 0.95F;
      const auto &from = keyframes_.at(last_candidate_.from);
      const auto &to = keyframes_.at(last_candidate_.to);
      geometry_msgs::msg::Point first;
      first.x = from.raw_pose.x;
      first.y = from.raw_pose.y;
      first.z = from.raw_z;
      geometry_msgs::msg::Point second;
      second.x = to.raw_pose.x;
      second.y = to.raw_pose.y;
      second.z = to.raw_z;
      candidate.points.push_back(first);
      candidate.points.push_back(second);
      array.markers.push_back(candidate);
    }
    marker_publisher_->publish(array);
  }

  void PublishDiagnostic(
    const std::string &event, const std::optional<CandidateState> &candidate) const
  {
    std_msgs::msg::String message;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "{\"event\":\"" << event << "\",\"keyframes\":" << keyframes_.size()
           << ",\"graph_edges\":" << edges_.size()
           << ",\"accepted_loops\":" << accepted_loop_edges_.size()
           << ",\"phase\":" << (enable_pose_graph_ ? 2 : 1);
    if (candidate) {
      stream << ",\"candidate\":" << candidate->from
             << ",\"current\":" << candidate->to
             << ",\"descriptor\":" << candidate->descriptor_similarity
             << ",\"fitness\":" << candidate->icp_fitness
             << ",\"inlier_ratio\":" << candidate->inlier_ratio
             << ",\"accepted\":" << (candidate->accepted ? "true" : "false");
    }
    stream << "}";
    message.data = stream.str();
    diagnostic_publisher_->publish(message);
  }

  void ExportOptimizedMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (export_optimized_map_path_.empty()) {
      response->success = false;
      response->message =
        "export_optimized_map_path is empty; no map file is written automatically";
      return;
    }
    const auto preview = BuildOptimizedMapPreview();
    if (preview->empty()) {
      response->success = false;
      response->message = "no optimized keyframe map available";
      return;
    }
    const int result = pcl::io::savePCDFileBinary(export_optimized_map_path_, *preview);
    response->success = result == 0;
    response->message = response->success ?
      "wrote diagnostic optimized map: " + export_optimized_map_path_ :
      "failed to write: " + export_optimized_map_path_;
  }

  std::string cloud_topic_;
  std::string odom_topic_;
  std::string world_frame_;
  double odom_sync_tolerance_sec_{0.035};
  size_t odom_history_size_{300};
  double keyframe_distance_m_{1.5};
  double keyframe_yaw_rad_{15.0 * kPi / 180.0};
  double keyframe_time_sec_{2.0};
  double keyframe_voxel_m_{0.2};
  size_t keyframe_max_points_{5000};
  size_t max_keyframes_{800};
  double query_interval_sec_{5.0};
  double min_loop_time_sec_{30.0};
  double min_loop_path_distance_m_{12.0};
  int descriptor_rings_{20};
  int descriptor_sectors_{60};
  double descriptor_max_range_m_{25.0};
  double descriptor_min_similarity_{0.82};
  double icp_max_correspondence_m_{0.8};
  int icp_max_iterations_{60};
  double icp_max_fitness_m_{0.18};
  double icp_min_inlier_ratio_{0.35};
  double icp_max_translation_m_{8.0};
  double icp_max_z_translation_m_{0.40};
  double icp_max_yaw_rad_{50.0 * kPi / 180.0};
  double icp_max_roll_pitch_rad_{15.0 * kPi / 180.0};
  bool enable_pose_graph_{false};
  int optimizer_iterations_{8};
  double odom_edge_xy_weight_{1.0};
  double odom_edge_yaw_weight_{2.0};
  double loop_edge_xy_weight_{8.0};
  double loop_edge_yaw_weight_{10.0};
  size_t map_preview_max_points_{250000};
  std::string export_optimized_map_path_;

  std::mutex odom_mutex_;
  std::deque<OdomSample> odom_history_;
  std::vector<Keyframe> keyframes_;
  std::vector<Pose2> optimized_poses_;
  std::vector<Edge> edges_;
  std::vector<Edge> accepted_loop_edges_;
  size_t active_loop_edge_count_{0};
  CandidateState last_candidate_;
  std::chrono::steady_clock::time_point last_query_time_{};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr keyframe_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr candidate_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_preview_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr export_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoopClosureNode>());
  rclcpp::shutdown();
  return 0;
}

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

class VisualDetectionTester : public rclcpp::Node {
public:
  VisualDetectionTester()
  : Node("visual_detection_tester"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {
    this->declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
    this->declare_parameter<std::string>("depth_topic", "/camera/aligned_depth_to_color/image_raw");
    this->declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    this->declare_parameter<std::string>("display_topic", "");
    this->declare_parameter<std::string>("robot_frame", "pelvis");

    this->get_parameter("rgb_topic", rgb_topic_);
    this->get_parameter("depth_topic", depth_topic_);
    this->get_parameter("camera_info_topic", camera_info_topic_);
    this->get_parameter("display_topic", display_topic_);
    this->get_parameter("robot_frame", robot_frame_);

    tf_buffer_.setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()));

    rgb_sub_.subscribe(this, rgb_topic_, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
    info_sub_.subscribe(this, camera_info_topic_, rmw_qos_profile_sensor_data);

    if (!display_topic_.empty()) {
      display_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        display_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&VisualDetectionTester::displayCallback, this, std::placeholders::_1));
    }

    sync_.reset(new Sync(SyncPolicy(10), rgb_sub_, depth_sub_, info_sub_));
    sync_->registerCallback(std::bind(
      &VisualDetectionTester::imageCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3));

    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(window_name_, &VisualDetectionTester::mouseCallbackThunk, this);

    RCLCPP_INFO(
      this->get_logger(),
      "Visual detection tester ready. robot_frame='%s', rgb_topic='%s', depth_topic='%s', display_topic='%s'",
      robot_frame_.c_str(), rgb_topic_.c_str(), depth_topic_.c_str(), display_topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Left click the image window to print the clicked pixel in frame '%s'. Press 'q' in the image window to quit.",
      robot_frame_.c_str());
  }

  ~VisualDetectionTester() override {
    cv::destroyWindow(window_name_);
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  static void mouseCallbackThunk(int event, int x, int y, int flags, void * userdata) {
    if (userdata == nullptr) {
      return;
    }
    static_cast<VisualDetectionTester *>(userdata)->mouseCallback(event, x, y, flags);
  }

  void mouseCallback(int event, int x, int y, int /*flags*/) {
    if (event != cv::EVENT_LBUTTONDOWN) {
      return;
    }

    cv::Mat depth_snapshot;
    sensor_msgs::msg::CameraInfo info_snapshot;
    std_msgs::msg::Header header_snapshot;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (latest_depth_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No synchronized image/depth data received yet.");
        return;
      }
      if (x < 0 || y < 0 || x >= latest_depth_.cols || y >= latest_depth_.rows) {
        RCLCPP_WARN(this->get_logger(), "Clicked pixel (%d, %d) is out of bounds.", x, y);
        return;
      }

      clicked_pixel_ = cv::Point(x, y);
      has_clicked_pixel_ = true;
      depth_snapshot = latest_depth_.clone();
      info_snapshot = latest_info_;
      header_snapshot = latest_header_;
    }

    const uint16_t z_raw = depth_snapshot.at<uint16_t>(y, x);
    const float z = static_cast<float>(z_raw) * 0.001f;
    if (!std::isfinite(z) || z <= 0.0f || z > 3.0f) {
      RCLCPP_WARN(
        this->get_logger(),
        "Clicked pixel (%d, %d) has invalid depth: raw=%u",
        x,
        y,
        z_raw);
      return;
    }

    const float fx = info_snapshot.k[0];
    const float fy = info_snapshot.k[4];
    const float cx = info_snapshot.k[2];
    const float cy = info_snapshot.k[5];
    if (fx == 0.0f || fy == 0.0f) {
      RCLCPP_ERROR(this->get_logger(), "Camera intrinsics fx or fy is zero.");
      return;
    }

    geometry_msgs::msg::PoseStamped pose_in;
    pose_in.header = header_snapshot;
    pose_in.pose.orientation.w = 1.0;
    pose_in.pose.position.x = (static_cast<float>(x) - cx) * z / fx;
    pose_in.pose.position.y = (static_cast<float>(y) - cy) * z / fy;
    pose_in.pose.position.z = z;

    try {
      const geometry_msgs::msg::PoseStamped pose_out =
        tf_buffer_.transform(pose_in, robot_frame_, tf2::durationFromSec(0.2));

      RCLCPP_INFO(
        this->get_logger(),
        "[click] pixel=(%d, %d) depth=%.4f source_frame='%s' -> robot_frame='%s' position=(%.4f, %.4f, %.4f)",
        x,
        y,
        z,
        pose_in.header.frame_id.c_str(),
        robot_frame_.c_str(),
        pose_out.pose.position.x,
        pose_out.pose.position.y,
        pose_out.pose.position.z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to transform clicked pixel from '%s' to '%s': %s",
        pose_in.header.frame_id.c_str(),
        robot_frame_.c_str(),
        ex.what());
    }
  }

  void displayCallback(sensor_msgs::msg::Image::ConstSharedPtr display_msg) {
    if (!display_msg) {
      return;
    }

    try {
      const cv::Mat display = cv_bridge::toCvShare(display_msg, "bgr8")->image;
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_display_ = display.clone();
      latest_display_header_ = display_msg->header;
    } catch (const std::exception & ex) {
      RCLCPP_WARN(this->get_logger(), "Failed to decode display image on '%s': %s", display_topic_.c_str(), ex.what());
    }
  }

  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg) {
    if (!rgb_msg || !depth_msg || !info_msg) {
      RCLCPP_ERROR(this->get_logger(), "Received null image/depth/camera_info message.");
      return;
    }

    try {
      const cv::Mat rgb = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
      const cv::Mat depth = cv_bridge::toCvShare(depth_msg)->image;

      cv::Mat display = rgb.clone();
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_depth_ = depth.clone();
        latest_info_ = *info_msg;
        latest_header_ = rgb_msg->header;

        const bool has_matching_display =
          !latest_display_.empty() &&
          latest_display_.rows == rgb.rows &&
          latest_display_.cols == rgb.cols &&
          latest_display_header_.stamp == rgb_msg->header.stamp;
        if (has_matching_display) {
          display = latest_display_.clone();
        }

        if (has_clicked_pixel_) {
          cv::circle(display, clicked_pixel_, 5, cv::Scalar(0, 255, 255), 2);
          cv::putText(
            display,
            "click",
            clicked_pixel_ + cv::Point(8, -8),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0, 255, 255),
            1,
            cv::LINE_AA);
        }
      }

      cv::imshow(window_name_, display);
      const int key = cv::waitKey(1);
      if (key == 'q' || key == 'Q') {
        RCLCPP_INFO(this->get_logger(), "Received 'q', shutting down visual_detection_tester.");
        rclcpp::shutdown();
      }
    } catch (const cv::Exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "OpenCV exception in imageCallback: %s", ex.what());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Standard exception in imageCallback: %s", ex.what());
    }
  }

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  std::shared_ptr<Sync> sync_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr display_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string rgb_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string display_topic_;
  std::string robot_frame_;

  std::mutex data_mutex_;
  cv::Mat latest_depth_;
  cv::Mat latest_display_;
  sensor_msgs::msg::CameraInfo latest_info_;
  std_msgs::msg::Header latest_header_;
  std_msgs::msg::Header latest_display_header_;
  cv::Point clicked_pixel_{0, 0};
  bool has_clicked_pixel_ = false;

  const std::string window_name_ = "visual_detection_click_tester";
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualDetectionTester>());
  rclcpp::shutdown();
  return 0;
}

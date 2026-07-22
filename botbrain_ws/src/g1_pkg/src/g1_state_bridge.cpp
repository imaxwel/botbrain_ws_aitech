#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace
{
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using SportModeState = unitree_go::msg::dds_::SportModeState_;

double yaw_from_quaternion(const std::array<float, 4> & quaternion)
{
  const double w = quaternion[0];
  const double x = quaternion[1];
  const double y = quaternion[2];
  const double z = quaternion[3];
  return std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
}
}  // namespace

class G1StateBridge : public rclcpp_lifecycle::LifecycleNode
{
public:
  G1StateBridge()
  : rclcpp_lifecycle::LifecycleNode("robot_read_node")
  {
    this->declare_parameter<std::string>("prefix", "");
    this->declare_parameter<std::string>("network_interface", "enP8p1s0");
    this->declare_parameter<std::string>("velocity_frame", "odom");
    this->declare_parameter<std::string>(
      "unitree_sport_state_topic", "rt/odommodestate");
    RCLCPP_INFO(
      this->get_logger(),
      "Unitree native state bridge created in unconfigured state");
  }

  ~G1StateBridge() override
  {
    close_native_subscription();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    prefix_ = this->get_parameter("prefix").as_string();
    network_interface_ = this->get_parameter("network_interface").as_string();
    velocity_frame_ = this->get_parameter("velocity_frame").as_string();
    sport_state_topic_ =
      this->get_parameter("unitree_sport_state_topic").as_string();

    if (network_interface_.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "network_interface is empty; refusing Unitree DDS auto-selection");
      return CallbackReturn::FAILURE;
    }
    if (velocity_frame_ != "odom" && velocity_frame_ != "body") {
      RCLCPP_ERROR(
        this->get_logger(),
        "velocity_frame must be either 'odom' or 'body'");
      return CallbackReturn::FAILURE;
    }

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 20);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 20);
    imu_temp_pub_ = this->create_publisher<std_msgs::msg::Float32>("imu_temp", 10);

    if (!unitree_factory_initialized_) {
      try {
        RCLCPP_INFO(
          this->get_logger(),
          "Initializing Unitree DDS domain 0 on interface '%s'",
          network_interface_.c_str());
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);
        unitree_factory_initialized_ = true;
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          this->get_logger(), "Unitree ChannelFactory initialization failed: %s",
          error.what());
        return CallbackReturn::FAILURE;
      }
    }

    last_state_time_ = std::chrono::steady_clock::time_point{};
    watchdog_ = this->create_wall_timer(
      std::chrono::seconds(2), [this]() {check_state_liveness();});
    RCLCPP_INFO(
      this->get_logger(),
      "Configured native Unitree state bridge: topic=%s prefix=%s",
      sport_state_topic_.c_str(), prefix_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    odom_pub_->on_activate();
    imu_pub_->on_activate();
    imu_temp_pub_->on_activate();

    try {
      sport_state_sub_ = std::make_shared<
        unitree::robot::ChannelSubscriber<SportModeState>>(sport_state_topic_);
      sport_state_sub_->InitChannel(
        [this](const void * message) {sport_state_callback(message);}, 1);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        this->get_logger(), "Unitree SportModeState subscription failed: %s",
        error.what());
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Native Unitree state bridge active; waiting for %s on %s",
      sport_state_topic_.c_str(), network_interface_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    close_native_subscription();
    odom_pub_->on_deactivate();
    imu_pub_->on_deactivate();
    imu_temp_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    close_native_subscription();
    watchdog_.reset();
    odom_pub_.reset();
    imu_pub_.reset();
    imu_temp_pub_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  void close_native_subscription()
  {
    if (sport_state_sub_) {
      sport_state_sub_->CloseChannel();
      sport_state_sub_.reset();
    }
  }

  void check_state_liveness()
  {
    if (!sport_state_sub_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (last_state_time_ == std::chrono::steady_clock::time_point{} ||
      now - last_state_time_ > std::chrono::seconds(2))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "No Unitree SportModeState received on %s via %s; navigation odometry is unavailable",
        sport_state_topic_.c_str(), network_interface_.c_str());
    }
  }

  void sport_state_callback(const void * raw_message)
  {
    if (raw_message == nullptr || !odom_pub_ || !odom_pub_->is_activated()) {
      return;
    }
    const auto & state = *static_cast<const SportModeState *>(raw_message);
    last_state_time_ = std::chrono::steady_clock::now();
    const auto stamp = this->get_clock()->now();
    const auto & quaternion = state.imu_state().quaternion();
    const double yaw = yaw_from_quaternion(quaternion);
    const double half_yaw = yaw * 0.5;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = prefix_ + "odom";
    odom.child_frame_id = prefix_ + "base_footprint";
    odom.pose.pose.position.x = state.position()[0];
    odom.pose.pose.position.y = state.position()[1];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.z = std::sin(half_yaw);
    odom.pose.pose.orientation.w = std::cos(half_yaw);

    double velocity_x = state.velocity()[0];
    double velocity_y = state.velocity()[1];
    if (velocity_frame_ == "odom") {
      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      const double body_x = cos_yaw * velocity_x + sin_yaw * velocity_y;
      const double body_y = -sin_yaw * velocity_x + cos_yaw * velocity_y;
      velocity_x = body_x;
      velocity_y = body_y;
    }
    odom.twist.twist.linear.x = velocity_x;
    odom.twist.twist.linear.y = velocity_y;
    odom.twist.twist.angular.z = state.yaw_speed();
    odom.twist.covariance[0] = 0.05;
    odom.twist.covariance[7] = 0.05;
    odom.twist.covariance[35] = 0.10;
    odom_pub_->publish(odom);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = prefix_ + "imu";
    imu.orientation.w = quaternion[0];
    imu.orientation.x = quaternion[1];
    imu.orientation.y = quaternion[2];
    imu.orientation.z = quaternion[3];
    imu.angular_velocity.x = state.imu_state().gyroscope()[0];
    imu.angular_velocity.y = state.imu_state().gyroscope()[1];
    imu.angular_velocity.z = state.imu_state().gyroscope()[2];
    imu.linear_acceleration.x = state.imu_state().accelerometer()[0];
    imu.linear_acceleration.y = state.imu_state().accelerometer()[1];
    imu.linear_acceleration.z = state.imu_state().accelerometer()[2];
    imu_pub_->publish(imu);

    std_msgs::msg::Float32 temperature;
    temperature.data = static_cast<float>(state.imu_state().temperature());
    imu_temp_pub_->publish(temperature);

    if (!received_first_state_) {
      received_first_state_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Receiving native Unitree state; /%sodom now carries real planar twist",
        prefix_.c_str());
    }
  }

  std::string prefix_;
  std::string network_interface_;
  std::string velocity_frame_;
  std::string sport_state_topic_;
  bool received_first_state_{false};
  bool unitree_factory_initialized_{false};
  std::chrono::steady_clock::time_point last_state_time_{};

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr imu_temp_pub_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<SportModeState>> sport_state_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<G1StateBridge>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

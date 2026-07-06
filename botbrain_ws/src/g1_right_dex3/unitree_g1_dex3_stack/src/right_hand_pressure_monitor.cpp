#include "rclcpp/rclcpp.hpp"
#include <unitree_hg/msg/hand_state.hpp>

#include <sstream>
#include <string>
#include <vector>

class RightHandPressureMonitor : public rclcpp::Node {
public:
  RightHandPressureMonitor() : Node("right_hand_pressure_monitor") {
    this->declare_parameter<std::string>("state_topic", "/lf/dex3/right/state");
    this->get_parameter("state_topic", state_topic_);

    sub_ = this->create_subscription<unitree_hg::msg::HandState>(
      state_topic_,
      10,
      std::bind(&RightHandPressureMonitor::stateCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RightHandPressureMonitor::printLatestPressure, this));

    RCLCPP_INFO(this->get_logger(), "Right-hand pressure monitor started. Subscribing to: %s", state_topic_.c_str());
  }

private:
  void stateCallback(const unitree_hg::msg::HandState::SharedPtr msg) {
    latest_pressure_groups_.clear();
    latest_pressure_groups_.reserve(msg->press_sensor_state.size());

    for (const auto &press_state : msg->press_sensor_state) {
      latest_pressure_groups_.emplace_back(press_state.pressure.begin(), press_state.pressure.end());
    }

    has_data_ = true;
  }

  void printLatestPressure() {
    if (!has_data_) {
      RCLCPP_INFO(this->get_logger(), "[right pressure] Waiting for data...");
      return;
    }

    std::ostringstream oss;
    oss << "[right pressure] groups=" << latest_pressure_groups_.size();

    for (size_t group_idx = 0; group_idx < latest_pressure_groups_.size(); ++group_idx) {
      const auto &group = latest_pressure_groups_[group_idx];
      oss << " | g" << group_idx << ":";
      if (group.empty()) {
        oss << "[]";
        continue;
      }

      oss << "[";
      for (size_t i = 0; i < group.size(); ++i) {
        oss << group[i];
        if (i + 1 < group.size()) {
          oss << ",";
        }
      }
      oss << "]";
    }

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }

  std::string state_topic_;
  bool has_data_{false};
  std::vector<std::vector<float>> latest_pressure_groups_;

  rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RightHandPressureMonitor>());
  rclcpp::shutdown();
  return 0;
}

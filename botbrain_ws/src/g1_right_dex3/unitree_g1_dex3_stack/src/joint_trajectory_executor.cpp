#include "rclcpp/rclcpp.hpp"

#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/motor_cmd.hpp>

#include <unitree_hg/msg/low_state.hpp>
#include <unitree_hg/msg/motor_state.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/empty.hpp>


#include <array>
#include <algorithm>
#include <vector>
#include <chrono>
#include <map>
#include <sstream>
#include <string>
#include <string_view>
#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>

#include <atomic>
#include <csignal>

#include <g1_dex3_joint_defs.hpp>

using namespace std::chrono_literals;

// Set by SIGINT/SIGTERM handler so (a) main()'s polling loop can exit
// cleanly even from idle state, and (b) trajectoryCallback's waypoint
// loop can break out at the next iteration and fall through to the
// existing end-of-trajectory ramp for a smooth in-flight release. The
// originally-planned separate 3-second ramp from main() was removed in
// Plan 01-06 because it stole control authority back from the body
// controller and jerked the arm to q=0. See Plans 01-04 and 01-06.
static std::atomic<bool> g_shutdown_requested{false};
static void executor_signal_handler(int /*sig*/) {
  g_shutdown_requested.store(true);
}

// Struct to hold joint limits
struct JointLimits {
  double lower;
  double upper;
  double velocity;
  double effort;
};

namespace {

// Plan 04-01: consumed by Plan 04-02 validation and Plan 04-03 publish-loop reasoning; indices and names must stay in sync.
static constexpr std::array<JointIndex, 7> kRightArmJointIndices = {
  kRightShoulderPitch,
  kRightShoulderRoll,
  kRightShoulderYaw,
  kRightElbow,
  kRightWristRoll,
  kRightWristPitch,
  kRightWristYaw,
};

static constexpr std::array<std::string_view, 7> kRightArmJointNames = {
  "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint",
  "right_elbow_joint",
  "right_wrist_roll_joint",
  "right_wrist_pitch_joint",
  "right_wrist_yaw_joint",
};

inline bool isRightArmJoint(std::string_view name) {
  for (const auto& right_arm_name : kRightArmJointNames) {
    if (name == right_arm_name) {
      return true;
    }
  }
  return false;
}

}  // namespace

class JointTrajectoryExecutor : public rclcpp::Node {
public:
  JointTrajectoryExecutor()
  : Node("joint_trajectory_executor")
  {
    RCLCPP_INFO(this->get_logger(), "Joint Trajectory Executor Node Initialized");
    this->declare_parameter("auto_return_to_standing", true);
    this->get_parameter("auto_return_to_standing", auto_return_to_standing_);
    RCLCPP_INFO(this->get_logger(), "auto_return_to_standing=%s",
      auto_return_to_standing_ ? "true" : "false");
    this->declare_parameter("hold_indefinitely", false);
    this->get_parameter("hold_indefinitely", hold_indefinitely_);
    RCLCPP_INFO(this->get_logger(), "hold_indefinitely=%s",
      hold_indefinitely_ ? "true" : "false");

    rclcpp::QoS qos_profile(10);
    qos_profile.best_effort();

    cmd_pub_ = this->create_publisher<unitree_hg::msg::LowCmd>("/arm_sdk", qos_profile);

    traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_targets", 10,
      std::bind(&JointTrajectoryExecutor::trajectoryCallback, this, std::placeholders::_1));

    lowstate_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
      "/lf/lowstate", 10,
      std::bind(&JointTrajectoryExecutor::lowstateCallback, this, std::placeholders::_1));

    return_to_standing_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/executor/return_to_standing", 10,
      std::bind(&JointTrajectoryExecutor::returnToStandingCallback, this, std::placeholders::_1));

    // Load URDF and parse joint limits
    std::string urdf_xml;
        auto client = this->create_client<rcl_interfaces::srv::GetParameters>("/robot_state_publisher/get_parameters");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /robot_state_publisher service...");
    }
    
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("robot_description");

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      if (response->values.size() == 1 && response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
        urdf_xml = response->values[0].string_value;
      } else {
        RCLCPP_FATAL(this->get_logger(), "robot_description not found in /robot_state_publisher");
        rclcpp::shutdown();
        return;
      }
    } else {
      RCLCPP_FATAL(this->get_logger(), "Failed to connect to /robot_state_publisher/get_parameters service");
      rclcpp::shutdown();
      return;
    }

    if (!urdf_xml.empty()) {
      urdf::Model urdf_model;
      if (urdf_model.initString(urdf_xml)) {
        for (const auto& joint_pair : urdf_model.joints_) {
          const auto& joint = joint_pair.second;
          if (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::PRISMATIC) continue;
          if (!joint->limits) continue;
          JointLimits lim;
          lim.lower = joint->limits->lower;
          lim.upper = joint->limits->upper;
          lim.velocity = joint->limits->velocity;
          lim.effort = joint->limits->effort;
          joint_limits_[joint->name] = lim;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded joint limits from URDF");

        // Build KDL tree and right-arm chain for gravity compensation
        KDL::Tree kdl_tree;
        if (kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
          if (kdl_tree.getChain("torso_link", "right_wrist_yaw_link", kdl_chain_right_)) {
            // Gravity in torso_link frame: Z is up
            gravity_solver_ = std::make_unique<KDL::ChainDynParam>(
                kdl_chain_right_, KDL::Vector(0.0, 0.0, -9.81));
            gravity_enabled_ = true;
            RCLCPP_INFO(this->get_logger(),
                "KDL gravity compensation enabled: %u segments, %u joints",
                kdl_chain_right_.getNrOfSegments(), kdl_chain_right_.getNrOfJoints());
          } else {
            RCLCPP_WARN(this->get_logger(),
                "Failed to get KDL chain torso_link->right_wrist_yaw_link; gravity comp disabled");
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to build KDL tree; gravity comp disabled");
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to parse URDF for joint limits");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "robot_description parameter is empty, joint limits not loaded");
    }
  }

  ~JointTrajectoryExecutor() override {
    RCLCPP_INFO(this->get_logger(), "Shutting down Joint Trajectory Executor Node");

    // Reaffirm body-controller authority on shutdown. In the normal
    // SIGINT/SIGTERM path, trajectoryCallback's own end-of-trajectory ramp
    // (or the new break-out path added in Plan 01-06) has already left
    // kNotUsedJoint.q at 0.0, so this is a redundant-but-harmless final
    // assertion. On a forced exit (SIGKILL, std::terminate) this destructor
    // may not run at all; arm_sdk's own timeout takes over in that case.
    unitree_hg::msg::LowCmd final_cmd;
    final_cmd.motor_cmd[JointIndex::kNotUsedJoint].q = 0.0f;
    cmd_pub_->publish(final_cmd);
  }

  void executeShutdownRampIfNeeded() {
    if (auto_return_to_standing_) {
      return;
    }
    if (ramp_to_standing_executed_.load()) {
      return;
    }
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(this->get_logger(), "Shutdown ramp skipped because executor is busy");
      return;
    }
    if (initial_standing_pose_ready_ && !latest_joint_positions_.empty()) {
      RCLCPP_WARN(this->get_logger(), "shutdown ramp triggered");
      executeRampToStanding(latest_joint_positions_);
    }
    busy_.store(false);
  }

private:
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr return_to_standing_sub_;
  std::map<std::string, JointLimits> joint_limits_;
  std::vector<float> latest_joint_positions_;
  std::vector<float> initial_standing_pose_;
  bool initial_standing_pose_ready_ = false;
  bool auto_return_to_standing_ = true;
  bool hold_indefinitely_ = false;
  std::atomic<bool> busy_{false};
  std::atomic<bool> ramp_to_standing_executed_{false};

  // KDL gravity compensation
  KDL::Chain kdl_chain_right_;
  std::unique_ptr<KDL::ChainDynParam> gravity_solver_;
  bool gravity_enabled_ = false;

  // KDL gravity torque correction (from calibrate_kdl_tau.py)
  // Wrist joints (4-6): scale=1 since KDL gives near-zero; only bias compensates
  const std::array<float, 7> gravity_scale_ = {1.5761f, 1.6540f, 2.1793f, 2.5543f, 1.0f, 1.0f, 1.0f};
  const std::array<float, 7> gravity_bias_  = {-0.0400f, 0.4672f, 0.1232f, 0.2478f, -0.0832f, 0.0251f, 0.0624f};

  // Compute gravity torques for 7 right-arm joints given their positions.
  std::vector<float> computeGravityTorques(const std::array<float, 7>& q_right_arm) {
    std::vector<float> torques(7, 0.0f);
    if (!gravity_enabled_) return torques;
    unsigned int nj = kdl_chain_right_.getNrOfJoints();
    KDL::JntArray q_kdl(nj);
    for (unsigned int i = 0; i < nj && i < 7; ++i) {
      q_kdl(i) = static_cast<double>(q_right_arm[i]);
    }
    KDL::JntArray gravity_torques(nj);
    if (gravity_solver_->JntToGravity(q_kdl, gravity_torques) == 0) {
      for (unsigned int i = 0; i < nj && i < 7; ++i) {
        torques[i] = gravity_scale_[i] * static_cast<float>(gravity_torques(i)) + gravity_bias_[i];
      }
    }
    return torques;
  }

  void executeRampToStanding(const std::vector<float>& ramp_start_positions) {
    if (!initial_standing_pose_ready_) {
      RCLCPP_WARN(this->get_logger(), "Cannot ramp to standing: initial standing pose not ready");
      return;
    }

    const double interp_duration = 3.0;
    const int interp_steps = 750;
    auto sleep_ns = std::chrono::nanoseconds(static_cast<int64_t>((interp_duration / interp_steps) * 1e9));
    for (int step = 0; step <= interp_steps; ++step) {
      double t = static_cast<double>(step) / interp_steps;
      double value = (1.0 - t) * 1.0 + t * 0.0;
      unitree_hg::msg::LowCmd final_cmd;
      final_cmd.motor_cmd[JointIndex::kNotUsedJoint].q = static_cast<float>(value);
      for (const auto& pair : joint_name_to_index) {
        size_t idx = pair.second;
        final_cmd.motor_cmd[idx].mode = 1;
        if (ramp_start_positions.size() > idx && initial_standing_pose_.size() > idx) {
          final_cmd.motor_cmd[idx].q = static_cast<float>(
            (1.0 - t) * ramp_start_positions[idx] + t * initial_standing_pose_[idx]);
        } else if (latest_joint_positions_.size() > idx) {
          final_cmd.motor_cmd[idx].q = latest_joint_positions_[idx];
        } else {
          final_cmd.motor_cmd[idx].q = 0.0f;
        }
        final_cmd.motor_cmd[idx].dq = 0.f;
        bool is_wrist = (pair.first.find("wrist") != std::string::npos);
        final_cmd.motor_cmd[idx].kp = is_wrist ? 40.0f : 100.0f;
        final_cmd.motor_cmd[idx].kd = 5.0f;
        final_cmd.motor_cmd[idx].tau = 0.f;
      }
      if (gravity_enabled_) {
        std::array<float, 7> q_ra;
        for (size_t k = 0; k < 7; ++k) {
          q_ra[k] = final_cmd.motor_cmd[kRightArmJointIndices[k]].q;
        }
        auto grav_tau = computeGravityTorques(q_ra);
        for (size_t k = 0; k < 7; ++k) {
          final_cmd.motor_cmd[kRightArmJointIndices[k]].tau = grav_tau[k];
        }
      }
      cmd_pub_->publish(final_cmd);
      rclcpp::sleep_for(sleep_ns);
    }
    ramp_to_standing_executed_.store(true);
  }

  void returnToStandingCallback(const std_msgs::msg::Empty::SharedPtr) {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(this->get_logger(), "return_to_standing ignored because executor is busy");
      return;
    }
    if (latest_joint_positions_.empty()) {
      RCLCPP_WARN(this->get_logger(), "return_to_standing ignored: no lowstate received yet");
      busy_.store(false);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "return_to_standing requested");
    executeRampToStanding(latest_joint_positions_);
    busy_.store(false);
  }

  void lowstateCallback(const unitree_hg::msg::LowState::SharedPtr msg) {
    // Store latest positions for all arm joints
    latest_joint_positions_.resize(joint_name_to_index.size(), 0.0f);
    for (const auto& pair : joint_name_to_index) {
      size_t idx = pair.second;
      if (idx < msg->motor_state.size()) {
        latest_joint_positions_[idx] = msg->motor_state[idx].q;
      }
    }
    if (!initial_standing_pose_ready_) {
      initial_standing_pose_ = latest_joint_positions_;
      initial_standing_pose_ready_ = true;
      RCLCPP_INFO(this->get_logger(), "Cached initial standing pose from first lowstate frame");
    }
  }

  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(this->get_logger(), "Trajectory ignored because executor is busy");
      return;
    }
    // Find out which hand to use based on the joint names
    if (msg->joint_names.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Received empty joint names in trajectory message");
      busy_.store(false);
      return;
    }

    // Plan 04-02 D-03 stage 1: foreign-joint WARN + strip.
    std::vector<size_t> right_arm_columns_in_msg;
    right_arm_columns_in_msg.reserve(kRightArmJointNames.size());
    std::vector<std::string> filtered_joint_names;
    filtered_joint_names.reserve(kRightArmJointNames.size());
    std::vector<std::string> foreign_names;
    for (size_t k = 0; k < msg->joint_names.size(); ++k) {
      const auto& joint_name = msg->joint_names[k];
      if (isRightArmJoint(joint_name)) {
        right_arm_columns_in_msg.push_back(k);
        filtered_joint_names.push_back(joint_name);
      } else {
        foreign_names.push_back(joint_name);
      }
    }
    if (!foreign_names.empty()) {
      std::ostringstream foreign_oss;
      for (size_t k = 0; k < foreign_names.size(); ++k) {
        if (k > 0) foreign_oss << ", ";
        foreign_oss << foreign_names[k];
      }
      RCLCPP_WARN(this->get_logger(),
        "Trajectory contains %zu foreign (non-right-arm) joint(s): %s — stripping these columns and proceeding with the right-arm subset.",
        foreign_names.size(), foreign_oss.str().c_str());
    }

    // Plan 04-02 D-03 stage 2: completeness ERROR.
    std::vector<std::string> missing;
    for (const auto& right_arm_name : kRightArmJointNames) {
      bool found = false;
      for (const auto& filtered_name : filtered_joint_names) {
        if (filtered_name == right_arm_name) {
          found = true;
          break;
        }
      }
      if (!found) {
        missing.emplace_back(right_arm_name);
      }
    }
    if (!missing.empty()) {
      std::ostringstream missing_oss;
      for (size_t k = 0; k < missing.size(); ++k) {
        if (k > 0) missing_oss << ", ";
        missing_oss << missing[k];
      }
      RCLCPP_ERROR(this->get_logger(),
        "Trajectory missing %zu right-arm joint(s): %s — rejecting trajectory; no LowCmd published.",
        missing.size(), missing_oss.str().c_str());
      busy_.store(false);
      return;
    }


    // snapshot standing pose at callback entry — target for end-of-trajectory ramp
    std::vector<float> standing_pose;
    if (!latest_joint_positions_.empty()) {
      standing_pose = latest_joint_positions_;
    }

    auto start_time = this->now();

    auto publish_command_for_positions = [&](const std::vector<double>& positions) {
      unitree_hg::msg::LowCmd cmd_msg;

      cmd_msg.motor_cmd[JointIndex::kNotUsedJoint].q = 1.0f; // Full motion-sdk authority (matches free_arm_demo.py)
      // Fill all joints with latest state first
      // Plan 04-03 D-06 Option A: all 28 body joints locked with kp=60; non-right-arm joints held at latest_joint_positions_. Trajectory column override below drives right-arm q. Matches free_arm_demo.py coexistence pattern.
      for (const auto& pair : joint_name_to_index) {
        size_t idx = pair.second;
        // Plan 01-10: enable PD control mode (was implicitly mode=0 before,
        // which caused arm_sdk to ignore q/kp/kd; reference: right_arm_mode.py).
        cmd_msg.motor_cmd[idx].mode = 1;
        if (latest_joint_positions_.size() > idx) {
          cmd_msg.motor_cmd[idx].q = latest_joint_positions_[idx];
        } else {
          cmd_msg.motor_cmd[idx].q = 0.0f;
        }
        cmd_msg.motor_cmd[idx].dq = 0.f;
        bool is_wrist = (pair.first.find("wrist") != std::string::npos);
        cmd_msg.motor_cmd[idx].kp = is_wrist ? 40.0f : 100.0f;
        cmd_msg.motor_cmd[idx].kd = 5.0f;
        cmd_msg.motor_cmd[idx].tau = 0.f;
      }
      // Plan 04-02 D-05: walk only the right-arm columns from the original msg; map lookups are guarded by stage 2 above.
      for (size_t k = 0; k < right_arm_columns_in_msg.size(); ++k) {
        const size_t j = right_arm_columns_in_msg[k];
        auto target_joint_name = msg->joint_names[j];
        auto target_index = joint_name_to_index.at(target_joint_name);
        auto target_position = positions[j];
        auto lim = joint_limits_.at(target_joint_name);
        target_position = std::min(std::max(target_position, lim.lower), lim.upper);
        cmd_msg.motor_cmd[target_index].q = target_position;
      }

      // Gravity compensation: KDL feedforward torques for right-arm joints
      if (gravity_enabled_) {
        std::array<float, 7> q_ra;
        for (size_t k = 0; k < 7; ++k) {
          q_ra[k] = cmd_msg.motor_cmd[kRightArmJointIndices[k]].q;
        }
        auto grav_tau = computeGravityTorques(q_ra);
        for (size_t k = 0; k < 7; ++k) {
          cmd_msg.motor_cmd[kRightArmJointIndices[k]].tau = grav_tau[k];
        }
      }

      cmd_pub_->publish(cmd_msg);
    };

    auto sleep_until = [&](const rclcpp::Time& scheduled_time) {
      rclcpp::Duration wait_time = scheduled_time - this->now();
      if (wait_time > rclcpp::Duration::from_seconds(0)) {
        rclcpp::sleep_for(std::chrono::nanoseconds(wait_time.nanoseconds()));
      }
    };

    const int64_t trajectory_publish_period_ns = 4'000'000;

    if (!msg->points.empty() && !g_shutdown_requested.load()) {
      const auto& first_point = msg->points.front();
      sleep_until(start_time + first_point.time_from_start);
      publish_command_for_positions(first_point.positions);
    }

    // Plan 01-06: honor SIGINT/SIGTERM mid-trajectory by breaking out of
    // the waypoint loop at the next iteration. The end-of-trajectory ramp
    // then runs from the current trajectory point, producing a smooth release.
    for (size_t i = 1; i < msg->points.size() && !g_shutdown_requested.load(); ++i) {
      const auto& previous_point = msg->points[i - 1];
      const auto& point = msg->points[i];
      rclcpp::Time segment_start_time = start_time + previous_point.time_from_start;
      rclcpp::Time segment_end_time = start_time + point.time_from_start;
      const int64_t segment_duration_ns = (segment_end_time - segment_start_time).nanoseconds();

      if (segment_duration_ns > 0) {
        for (int64_t elapsed_ns = trajectory_publish_period_ns;
             elapsed_ns < segment_duration_ns && !g_shutdown_requested.load();
             elapsed_ns += trajectory_publish_period_ns)
        {
          const double t = static_cast<double>(elapsed_ns) / static_cast<double>(segment_duration_ns);
          std::vector<double> interpolated_positions = point.positions;
          for (size_t j = 0; j < interpolated_positions.size() && j < previous_point.positions.size(); ++j) {
            interpolated_positions[j] =
              (1.0 - t) * previous_point.positions[j] + t * point.positions[j];
          }
          sleep_until(segment_start_time + rclcpp::Duration(std::chrono::nanoseconds(elapsed_ns)));
          publish_command_for_positions(interpolated_positions);
        }
      }

      if (!g_shutdown_requested.load()) {
        sleep_until(segment_end_time);
        publish_command_for_positions(point.positions);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory %zu waypoints executed; holding stiff at end-point.", msg->points.size());

    // Plan 01-12: capture trajectory end-point from the last waypoint.
    // latest_joint_positions_ is STALE here — single-threaded executor
    // blocks lowstateCallback for the entire trajectoryCallback, so it
    // still holds the standing pose from callback entry. Using it in
    // the hold loop or ramp was commanding the arm back to standing.
    std::vector<float> trajectory_endpoint;
    if (!standing_pose.empty()) {
      trajectory_endpoint = standing_pose;  // baseline for non-trajectory joints
    } else {
      trajectory_endpoint.resize(29, 0.0f);
    }
    if (!msg->points.empty()) {
      const auto& last_point = msg->points.back();
      for (size_t j = 0; j < last_point.positions.size() && j < msg->joint_names.size(); ++j) {
        auto it = joint_name_to_index.find(msg->joint_names[j]);
        if (it != joint_name_to_index.end()) {
          size_t idx = it->second;
          if (idx < trajectory_endpoint.size()) {
            auto lim = joint_limits_.at(msg->joint_names[j]);
            double pos = std::min(std::max(last_point.positions[j], lim.lower), lim.upper);
            trajectory_endpoint[idx] = static_cast<float>(pos);
          }
        }
      }
    }

    // Hold-publish loop at 250 Hz — keeps publishing endpoint q so the arm
    // stays at the trajectory end-point.  When hold_indefinitely_=false the
    // loop runs for 1 s then falls through to the ramp/return logic.  When
    // hold_indefinitely_=true the loop runs until SIGINT/SIGTERM, then ramps
    // back to standing.

    {
      const int hold_steps = hold_indefinitely_ ? std::numeric_limits<int>::max() : 250;
      const auto hold_sleep_ns = std::chrono::nanoseconds(4'000'000); // 4 ms
      for (int s = 0; s < hold_steps && !g_shutdown_requested.load(); ++s) {
        unitree_hg::msg::LowCmd hold_cmd;
        hold_cmd.motor_cmd[JointIndex::kNotUsedJoint].q = 1.0f; // full motion-sdk authority
        // Plan 04-03 D-06 Option A: all 28 body joints locked with kp=60 at trajectory end-point.
        for (const auto& pair : joint_name_to_index) {
          size_t idx = pair.second;
          hold_cmd.motor_cmd[idx].mode = 1;
          if (trajectory_endpoint.size() > idx) {
            hold_cmd.motor_cmd[idx].q = trajectory_endpoint[idx];
          } else {
            hold_cmd.motor_cmd[idx].q = 0.0f;
          }
          hold_cmd.motor_cmd[idx].dq = 0.f;
          bool is_wrist = (pair.first.find("wrist") != std::string::npos);
          hold_cmd.motor_cmd[idx].kp = is_wrist ? 40.0f : 100.0f;
          hold_cmd.motor_cmd[idx].kd = 5.0f;
          hold_cmd.motor_cmd[idx].tau = 0.f;
        }
        // Gravity compensation for hold
        if (gravity_enabled_) {
          std::array<float, 7> q_ra;
          for (size_t k = 0; k < 7; ++k) {
            q_ra[k] = hold_cmd.motor_cmd[kRightArmJointIndices[k]].q;
          }
          auto grav_tau = computeGravityTorques(q_ra);
          for (size_t k = 0; k < 7; ++k) {
            hold_cmd.motor_cmd[kRightArmJointIndices[k]].tau = grav_tau[k];
          }
        }
        cmd_pub_->publish(hold_cmd);
        rclcpp::sleep_for(hold_sleep_ns);
      }
    }
    if (auto_return_to_standing_ || g_shutdown_requested.load()) {
      RCLCPP_INFO(this->get_logger(), "Trajectory execution complete, ramping back to standing pose.");
      executeRampToStanding(trajectory_endpoint);
    } else {
      RCLCPP_INFO(this->get_logger(), "auto_return_to_standing=false, holding at trajectory endpoint");
    }
    busy_.store(false);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Replace rclcpp's default SIGINT handler so we can perform a graceful
  // arm-control release before the publisher is torn down. See Plan 01-04.
  rclcpp::uninstall_signal_handlers();
  std::signal(SIGINT, executor_signal_handler);
  std::signal(SIGTERM, executor_signal_handler);

  auto node = std::make_shared<JointTrajectoryExecutor>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  while (rclcpp::ok() && !g_shutdown_requested.load()) {
    exec.spin_some(std::chrono::milliseconds(50));
  }
  node->executeShutdownRampIfNeeded();

  // SIGINT/SIGTERM received (or rclcpp::ok went false). The trajectory-end
  // ramp inside trajectoryCallback (line ~249) is the only graceful release
  // we need; if a callback was running, its ramp has already executed by
  // the time we get here. The destructor's instantaneous q=0.0 publish is
  // a harmless reaffirmation in idle state. See Plan 01-06.
  rclcpp::shutdown();
  return 0;
}

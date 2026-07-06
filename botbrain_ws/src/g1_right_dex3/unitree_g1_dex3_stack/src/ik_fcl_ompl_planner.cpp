#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <trac_ik/trac_ik.hpp>
#include <fcl/fcl.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathSimplifier.h>
#include <std_msgs/msg/string.hpp>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <resource_retriever/retriever.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <map>
#include <set>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <random>
#include <sstream>
#include <iomanip>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using std::placeholders::_1;

class IKFCLPlannerNode : public rclcpp::Node
{
public:
    IKFCLPlannerNode() : Node("ik_fcl_ompl_planner"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
    {
        // Real initialization deferred to init() — TRAC-IK construction calls
        // shared_from_this(), which only becomes valid AFTER make_shared returns.
    }

    void init()
    {
        // Parameterization
        this->declare_parameter("velocity_scale", 0.05);
        this->declare_parameter("min_time_step", 0.02);
        this->declare_parameter("planning_timeout", 1.0);
        this->declare_parameter("base_link", "torso_link");
        this->declare_parameter("right_tip", "right_tcp_link");
        this->declare_parameter("goal_pose_topic", "/goal_pose");
        this->declare_parameter("planner_type", "RRTConnect");
        this->declare_parameter("collision_skip_pairs", std::vector<std::string>{});
        this->declare_parameter("collision_detection_enabled", true);
        this->declare_parameter("simplify_method", std::string("simple"));
        this->declare_parameter("simplify_timeout", 0.5);
        this->declare_parameter("simplify_max_steps", 100);
        this->declare_parameter("simplify_max_empty_steps", 50);
        this->declare_parameter("fallback_enabled", true);
        this->declare_parameter("fallback_max_retraction", 0.05);
        this->declare_parameter("fallback_step", 0.005);
        this->declare_parameter("ik_timeout", 0.1);
        this->declare_parameter("ik_random_seed_tries", 3);
        this->declare_parameter("fallback_total_timeout_s", 2.0);
        this->declare_parameter("fallback_ik_only", false);

        // Fetch robot_description from global parameter server
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
        if (urdf_xml.empty()) {
            RCLCPP_FATAL(this->get_logger(), "robot_description parameter is missing or empty. Cannot continue.");
            rclcpp::shutdown();
            return;
        }

        this->get_parameter("velocity_scale", velocity_scale_);
        this->get_parameter("min_time_step", min_time_step_);
        this->get_parameter("planning_timeout", planning_timeout_);
        this->get_parameter("base_link", base_link_);
        this->get_parameter("right_tip", right_tip_);
        this->get_parameter("goal_pose_topic", goal_pose_topic_);
        this->get_parameter("planner_type", planner_type_);
        this->get_parameter("collision_skip_pairs", collision_skip_pairs_);
        this->get_parameter("collision_detection_enabled", collision_detection_enabled_);
        this->get_parameter("simplify_method", simplify_method_);
        this->get_parameter("simplify_timeout", simplify_timeout_);
        this->get_parameter("simplify_max_steps", simplify_max_steps_);
        this->get_parameter("simplify_max_empty_steps", simplify_max_empty_steps_);
        this->get_parameter("fallback_enabled", fallback_enabled_);
        this->get_parameter("fallback_max_retraction", fallback_max_retraction_);
        this->get_parameter("fallback_step", fallback_step_);
        this->get_parameter("ik_timeout", ik_timeout_);
        this->get_parameter("ik_random_seed_tries", ik_random_seed_tries_);
        this->get_parameter("fallback_total_timeout_s", fallback_total_timeout_s_);
        this->get_parameter("fallback_ik_only", fallback_ik_only_);

        if (!urdf_model.initString(urdf_xml)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to parse URDF");
            rclcpp::shutdown();
            return;
        }

        if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create KDL tree");
            rclcpp::shutdown();
            return;
        }
        if (!kdl_tree.getChain(base_link_, right_tip_, kdl_chain_right)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to extract KDL chain for right arm");
            rclcpp::shutdown();
            return;
        }

        // TCP offset override: allow runtime adjustment of the last fixed segment
        this->declare_parameter("tcp_offset_x", 0.175);
        double tcp_offset_x = this->get_parameter("tcp_offset_x").as_double();
        {
            unsigned int n_seg = kdl_chain_right.getNrOfSegments();
            if (n_seg > 0 && kdl_chain_right.getSegment(n_seg - 1).getJoint().getType() == KDL::Joint::None) {
                KDL::Chain new_chain;
                for (unsigned int i = 0; i < n_seg - 1; ++i) {
                    new_chain.addSegment(kdl_chain_right.getSegment(i));
                }
                new_chain.addSegment(KDL::Segment(
                    kdl_chain_right.getSegment(n_seg - 1).getName(),
                    KDL::Joint(KDL::Joint::None),
                    KDL::Frame(KDL::Vector(tcp_offset_x, 0.0, 0.0))
                ));
                kdl_chain_right = new_chain;
                RCLCPP_INFO(this->get_logger(), "TCP offset overridden to %.4f m", tcp_offset_x);
            }
        }

        // Phase 8: adaptive end-effector orientation toggle (D-09, D-10).
        this->declare_parameter("adaptive_orientation_enabled", true);
        this->get_parameter("adaptive_orientation_enabled", adaptive_orientation_enabled_);
        RCLCPP_INFO(this->get_logger(),
            "adaptive_orientation_enabled = %s",
            adaptive_orientation_enabled_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(),
            "collision_detection_enabled = %s",
            collision_detection_enabled_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(),
            "fallback: enabled=%s, max_retraction=%.4f m, step=%.4f m",
            fallback_enabled_ ? "true" : "false",
            fallback_max_retraction_, fallback_step_);
        RCLCPP_INFO(this->get_logger(),
            "ik: timeout=%.3f s, random_seed_tries=%d",
            ik_timeout_, ik_random_seed_tries_);
        RCLCPP_INFO(this->get_logger(),
            "press fallback: total_timeout=%.3f s, ik_only=%s",
            fallback_total_timeout_s_, fallback_ik_only_ ? "true" : "false");

        // Parse joint limits from URDF for OMPL bounds
        for (const auto& joint_pair : urdf_model.joints_) {
            const auto& joint = joint_pair.second;
            if (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::PRISMATIC && joint->type != urdf::Joint::FIXED) continue;
            if (!joint->limits) continue;
            joint_limits_[joint->name] = std::make_pair(joint->limits->lower, joint->limits->upper);
            if (joint->type != urdf::Joint::FIXED && joint->limits->velocity > 0.0) {
                velocity_limits_[joint->name] = joint->limits->velocity;
            }
        }

        // Build joint limits array for right arm
        KDL::JntArray right_lower(kdl_chain_right.getNrOfJoints()), right_upper(kdl_chain_right.getNrOfJoints());
        size_t idx = 0;
        for (unsigned int i = 0; i < kdl_chain_right.getNrOfSegments(); ++i) {
            const auto& joint = kdl_chain_right.getSegment(i).getJoint();
            if (joint.getType() != KDL::Joint::None) {
                auto lim = joint_limits_.find(joint.getName());
                if (lim != joint_limits_.end()) {
                    right_lower(idx) = lim->second.first;
                    right_upper(idx) = lim->second.second;
                } else {
                    right_lower(idx) = -3.14;
                    right_upper(idx) = 3.14;
                }
                ++idx;
            }
        }

        // Auto-derive adjacent-link skip pairs from the KDL chain so that
        // physically-connected (and therefore geometrically-overlapping)
        // links are not flagged as self-collisions. Order in the pair string
        // does not matter -- isInCollision matches both "a:b" and "b:a".
        // See Plan 01-05.
        {
            std::string prev = base_link_;
            for (unsigned int i = 0; i < kdl_chain_right.getNrOfSegments(); ++i) {
                const std::string& cur = kdl_chain_right.getSegment(i).getName();
                collision_skip_pairs_.push_back(prev + ":" + cur);
                prev = cur;
            }
        }

        // Use longer timeout and error tolerance for TRAC-IK
        double ik_timeout = std::max(0.001, ik_timeout_); // seconds
        double ik_tol = 1e-5;  // Tolerance for IK solutions
        TRAC_IK::SolveType solve_type = TRAC_IK::Distance;  // Distance, Manip1, Manip2, Speed
        ik_right = std::make_shared<TRAC_IK::TRAC_IK>(shared_from_this(), kdl_chain_right, right_lower, right_upper, ik_timeout, ik_tol, solve_type);

        assert(right_lower.rows() == kdl_chain_right.getNrOfJoints());
        assert(right_upper.rows() == kdl_chain_right.getNrOfJoints());

        fk_right_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_right);

        // Phase 8: cache the right shoulder origin in base_link_ (torso_link) once.
        // Per D-05, the shoulder reference is the right_shoulder_pitch_link origin;
        // per D-06, compute via URDF/KDL FK with all-zero joints (no runtime TF lookup).
        // KDL is 1-based on segmentNr — segmentNr=1 returns the frame at the end of
        // segment 0 (the segment created by right_shoulder_pitch_joint), whose origin
        // is invariant of joint state because revolute-joint angle does not move
        // the child link's origin.
        {
            KDL::JntArray zero_jnt(kdl_chain_right.getNrOfJoints());
            KDL::Frame shoulder_frame;
            if (fk_right_solver->JntToCart(zero_jnt, shoulder_frame, 1) < 0) {
                RCLCPP_FATAL(this->get_logger(),
                    "Failed to compute shoulder origin via FK on segment 1 of right-arm chain");
                rclcpp::shutdown();
                return;
            }
            right_shoulder_pos_in_base_ = shoulder_frame.p;
            RCLCPP_INFO(this->get_logger(),
                "Right shoulder reference point in '%s': [%.4f, %.4f, %.4f]",
                base_link_.c_str(),
                right_shoulder_pos_in_base_.x(),
                right_shoulder_pos_in_base_.y(),
                right_shoulder_pos_in_base_.z());
        }

        // Collect link names from the planning chain to filter collision objects
        chain_link_names_.insert(base_link_);
        for (unsigned int i = 0; i < kdl_chain_right.getNrOfSegments(); ++i) {
            chain_link_names_.insert(kdl_chain_right.getSegment(i).getName());
        }

        if (collision_detection_enabled_) {
            buildCollisionObjects();
        }

        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&IKFCLPlannerNode::goalPoseCallback, this, _1));
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_targets", 10);
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&IKFCLPlannerNode::jointStateCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "IKFCLPlannerNode initialized with %zu joints", joint_limits_.size());
        RCLCPP_INFO(this->get_logger(), "Using base link: %s, right tip: %s",
                    base_link_.c_str(), right_tip_.c_str());
        RCLCPP_INFO(this->get_logger(), "Planning timeout: %.2f seconds, velocity_scale: %.2f, min_time_step: %.2f",
                    planning_timeout_, velocity_scale_, min_time_step_);
        RCLCPP_INFO(this->get_logger(), "Collision skip pairs: %zu", collision_skip_pairs_.size());
        for (const auto& pair : collision_skip_pairs_) {
            RCLCPP_INFO(this->get_logger(), "Skipping collision check for pair: %s", pair.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Planner type: %s", planner_type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Simplification: method=%s, timeout=%.2f seconds, max_steps=%d, max_empty_steps=%d",
                    simplify_method_.c_str(), simplify_timeout_, simplify_max_steps_, simplify_max_empty_steps_);
        RCLCPP_INFO(this->get_logger(), "Right arm: base_link = %s, tip_link = %s", base_link_.c_str(), right_tip_.c_str());
    }

private:
    urdf::Model urdf_model;
    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain_right;
    std::shared_ptr<TRAC_IK::TRAC_IK> ik_right;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_right_solver;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::vector<std::string> joint_names_;
    std::vector<double> latest_joint_positions_;

    struct LinkCollision {
        std::shared_ptr<fcl::CollisionGeometryd> geometry;
        std::shared_ptr<fcl::CollisionObjectd> object;
        std::string segment_name;
        Eigen::Isometry3d local_transform;
    };

    std::map<std::string, LinkCollision> link_collisions;
    std::set<std::string> chain_link_names_;

    // Add joint_limits_ as a member variable
    std::map<std::string, std::pair<double, double>> joint_limits_;
    std::map<std::string, double> velocity_limits_;

    // Parameters
    double velocity_scale_ = 0.05;
    double min_time_step_ = 0.02;
    double planning_timeout_ = 1.0;
    std::string base_link_ = "pelvis";
    std::string right_tip_ = "right_hand_palm_link";
    std::string goal_pose_topic_ = "/goal_pose";
    std::string planner_type_ = "RRTConnect";
    std::vector<std::string> collision_skip_pairs_;
    bool collision_detection_enabled_ = true;
    std::string log_level_ = "info";
    std::string simplify_method_ = "simple";
    double simplify_timeout_ = 0.5;
    int simplify_max_steps_ = 100;
    int simplify_max_empty_steps_ = 50;
    bool fallback_enabled_ = true;
    double fallback_max_retraction_ = 0.05;
    double fallback_step_ = 0.005;
    double ik_timeout_ = 0.1;
    int ik_random_seed_tries_ = 3;
    double fallback_total_timeout_s_ = 2.0;

    struct GoalCandidate {
        std::string label;
        KDL::Vector position;
        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 1.0;
        double retraction = 0.0;
        int retraction_step = 0;
        bool fallback_candidate = false;
    };

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Phase 8: adaptive end-effector orientation (ORI-01)
    KDL::Vector right_shoulder_pos_in_base_;
    bool adaptive_orientation_enabled_ = true;
    bool fallback_ik_only_ = false;

    enum class AdaptiveOrientationStatus {
        OK,
        TARGET_TOO_CLOSE_TO_SHOULDER,
    };

    // Returns OK and writes out_q*/out_dir_normalized on success; returns
    // TARGET_TOO_CLOSE_TO_SHOULDER when target is within kMinTargetDistance
    // of the cached shoulder reference. Honors D-01..D-04, D-08 of CONTEXT.md.
    AdaptiveOrientationStatus computeAdaptiveOrientation(
        const KDL::Vector& target_in_base,
        double& out_qx, double& out_qy, double& out_qz, double& out_qw,
        KDL::Vector& out_dir_normalized) const
    {
        constexpr double kMinTargetDistance     = 0.05;   // m  (PIT-03, D-08)
        constexpr double kParallelDotThreshold  = 0.95;   // dimensionless  (PIT-02, D-03)

        KDL::Vector d = target_in_base - right_shoulder_pos_in_base_;
        const double d_norm = d.Norm();
        if (d_norm < kMinTargetDistance) {
            return AdaptiveOrientationStatus::TARGET_TOO_CLOSE_TO_SHOULDER;
        }

        const KDL::Vector x_axis = d / d_norm;                  // D-01: TCP +X = shoulder→target

        KDL::Vector up(0.0, 0.0, 1.0);                          // D-02: torso +Z primary up
        if (std::abs(KDL::dot(x_axis, up)) > kParallelDotThreshold) {
            up = KDL::Vector(0.0, 1.0, 0.0);                    // D-03: +Y fallback
        }

        KDL::Vector y_axis = up * x_axis;                       // KDL operator* on Vector = cross
        y_axis.Normalize();
        KDL::Vector z_axis = x_axis * y_axis;                   // already unit, right-handed

        KDL::Rotation R(x_axis, y_axis, z_axis);                // column-vector ctor
        R.GetQuaternion(out_qx, out_qy, out_qz, out_qw);

        out_dir_normalized = x_axis;
        return AdaptiveOrientationStatus::OK;
    }

    void buildCollisionObjects()
    {
        for (const auto& link_pair : urdf_model.links_) {
            auto link = link_pair.second;
            if (!link->collision || !link->collision->geometry) continue;
            if (chain_link_names_.find(link->name) == chain_link_names_.end()) continue;

            std::shared_ptr<fcl::CollisionGeometryd> geom;
            if (link->collision->geometry->type == urdf::Geometry::BOX) {
                urdf::Box* box = dynamic_cast<urdf::Box*>(link->collision->geometry.get());
                geom = std::make_shared<fcl::Boxd>(box->dim.x, box->dim.y, box->dim.z);
                RCLCPP_DEBUG(this->get_logger(), "Collision geometry for link %s: BOX [%.3f, %.3f, %.3f]", link->name.c_str(), box->dim.x, box->dim.y, box->dim.z);
            }
            else if (link->collision->geometry->type == urdf::Geometry::CYLINDER) {
                urdf::Cylinder* cyl = dynamic_cast<urdf::Cylinder*>(link->collision->geometry.get());
                geom = std::make_shared<fcl::Cylinderd>(cyl->radius, cyl->length);
                RCLCPP_DEBUG(this->get_logger(), "Collision geometry for link %s: CYLINDER [radius=%.3f, length=%.3f]", link->name.c_str(), cyl->radius, cyl->length);
            }
            else if (link->collision->geometry->type == urdf::Geometry::MESH) {
                urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(link->collision->geometry.get());
                if (!mesh) {
                    RCLCPP_WARN(this->get_logger(), "Failed to cast mesh geometry for link %s", link->name.c_str());
                    continue;
                }
                std::string mesh_filename = mesh->filename;
                double scale_x = mesh->scale.x;
                double scale_y = mesh->scale.y;
                double scale_z = mesh->scale.z;
                RCLCPP_DEBUG(this->get_logger(), "Attempting to load mesh for link %s: %s", link->name.c_str(), mesh_filename.c_str());
                shapes::Mesh* shape_mesh = shapes::createMeshFromResource(mesh_filename, Eigen::Vector3d(scale_x, scale_y, scale_z));
                if (!shape_mesh || shape_mesh->vertex_count == 0) {
                    RCLCPP_WARN(this->get_logger(), "Mesh for link %s is empty or failed to load (resource: %s)", link->name.c_str(), mesh_filename.c_str());
                    continue;
                }
                // Convert to FCL BVHModel
                auto bvh = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
                std::vector<fcl::Vector3d> vertices;
                for (unsigned int i = 0; i < shape_mesh->vertex_count; ++i) {
                    vertices.emplace_back(
                        shape_mesh->vertices[3*i+0],
                        shape_mesh->vertices[3*i+1],
                        shape_mesh->vertices[3*i+2]);
                }
                bvh->beginModel();
                for (unsigned int i = 0; i < shape_mesh->triangle_count; ++i) {
                    unsigned int idx0 = shape_mesh->triangles[3*i+0];
                    unsigned int idx1 = shape_mesh->triangles[3*i+1];
                    unsigned int idx2 = shape_mesh->triangles[3*i+2];
                    bvh->addTriangle(vertices[idx0], vertices[idx1], vertices[idx2]);
                }
                bvh->endModel();
                delete shape_mesh;
                geom = bvh;
                RCLCPP_DEBUG(this->get_logger(), "Collision geometry for link %s: MESH [%s] (scaled %.3f, %.3f, %.3f)", link->name.c_str(), mesh_filename.c_str(), scale_x, scale_y, scale_z);
            }
            else if (link->collision->geometry->type == urdf::Geometry::SPHERE) {
                urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(link->collision->geometry.get());
                if (!sphere) {
                    RCLCPP_WARN(this->get_logger(), "Failed to cast sphere geometry for link %s", link->name.c_str());
                    continue;
                }
                geom = std::make_shared<fcl::Sphered>(sphere->radius);
                RCLCPP_DEBUG(this->get_logger(), "Collision geometry for link %s: SPHERE [radius=%.3f]", link->name.c_str(), sphere->radius);
            }
            else {
                // Skip unknown geometry types for collision
                RCLCPP_WARN(this->get_logger(), "Skipping collision geometry for link %s: unknown geometry type", link->name.c_str());
                continue;
            }

            // Apply <origin> from collision tag
            Eigen::Isometry3d local_tf = Eigen::Isometry3d::Identity();
            const urdf::Pose& pose = link->collision->origin;
            const urdf::Vector3& p = pose.position;
            const urdf::Rotation& r = pose.rotation;
            double rx, ry, rz, rw;
            r.getQuaternion(rx, ry, rz, rw);
            Eigen::Quaterniond q(rw, rx, ry, rz);
            local_tf.linear() = q.toRotationMatrix();
            local_tf.translation() = Eigen::Vector3d(p.x, p.y, p.z);
            auto obj = std::make_shared<fcl::CollisionObjectd>(geom);
            link_collisions[link->name] = {geom, obj, link->name, local_tf};
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_names_ = msg->name;
        latest_joint_positions_ = msg->position;
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
        // Transform goal_pose from its source frame to base_link_ (torso_link)
        geometry_msgs::msg::PoseStamped pose_in_base;
        if (pose->header.frame_id.empty() || pose->header.frame_id == base_link_) {
            pose_in_base = *pose;
            RCLCPP_INFO(this->get_logger(), "goal_pose already in base frame '%s'", base_link_.c_str());
        } else {
            try {
                // Use latest available TF (stamp=0) to avoid TF_OLD_DATA
                auto latest_pose = *pose;
                latest_pose.header.stamp.sec = 0;
                latest_pose.header.stamp.nanosec = 0;
                pose_in_base = tf_buffer_.transform(latest_pose, base_link_, tf2::durationFromSec(0.5));
                RCLCPP_INFO(this->get_logger(), "Transformed goal_pose from '%s' to '%s': [%.3f, %.3f, %.3f]",
                    pose->header.frame_id.c_str(), base_link_.c_str(),
                    pose_in_base.pose.position.x, pose_in_base.pose.position.y, pose_in_base.pose.position.z);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(), "TF transform failed ('%s' -> '%s'): %s. Aborting goal.",
                    pose->header.frame_id.c_str(), base_link_.c_str(), ex.what());
                return;
            }
        }

        // Dynamically generate planning_joints from KDL chain
        const KDL::Chain& chain = kdl_chain_right;
        std::vector<std::string> planning_joints;
        std::set<std::string> planning_links; // <-- collect relevant links
        for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
            const auto& seg = chain.getSegment(i);
            planning_links.insert(seg.getName());
            const auto& joint = seg.getJoint();
            if (joint.getType() != KDL::Joint::None) {
                planning_joints.push_back(joint.getName());
            }
        }

        if (planning_joints.size() != 7u) {
            RCLCPP_ERROR(this->get_logger(),
                "Right-arm planning chain expected 7 joints, got %zu. Aborting goal.",
                planning_joints.size());
            return;
        }

        // Snapshot world-frame transforms of all non-planning links from the TF tree.
        // Right-arm chain segments will have their transforms updated per OMPL state
        // inside isInCollision(); everything else (torso, legs, left arm, head, hands)
        // is static during planning, so we look it up once and reuse.
        size_t tf_lookup_failures = 0;
        for (auto& [link_name, lc] : link_collisions) {
            if (planning_links.find(link_name) != planning_links.end()) {
                continue; // arm-chain links are handled per-state in isInCollision
            }
            try {
                auto tf_msg = tf_buffer_.lookupTransform(
                    base_link_, link_name, tf2::TimePointZero,
                    tf2::durationFromSec(0.2));
                const auto& t = tf_msg.transform.translation;
                const auto& q = tf_msg.transform.rotation;
                Eigen::Isometry3d world_tf = Eigen::Isometry3d::Identity();
                world_tf.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
                world_tf.translation() = Eigen::Vector3d(t.x, t.y, t.z);
                Eigen::Isometry3d final_tf = world_tf * lc.local_transform;
                lc.object->setTransform(fcl::Transform3d(final_tf.matrix()));
            } catch (const tf2::TransformException& ex) {
                ++tf_lookup_failures;
                RCLCPP_WARN(this->get_logger(),
                    "TF lookup failed for non-planning link '%s' (base='%s'): %s. "
                    "Using previous transform; collision check may be inaccurate.",
                    link_name.c_str(), base_link_.c_str(), ex.what());
            }
        }
        if (tf_lookup_failures > 0) {
            RCLCPP_WARN(this->get_logger(),
                "Body-link TF snapshot completed with %zu failures.",
                tf_lookup_failures);
        }

        // Note: KDL chain joint order does NOT need to match /joint_states
        // ordering -- the planning_positions lookup below is by-name. The
        // pre-existing joint-order ERROR check that lived here was removed
        // in Plan 01-05 because it was a false-positive on every goal.
        std::vector<double> planning_positions;
        int joint_name_fallback_count = 0;
        for (const auto& jname : planning_joints) {
            auto it = std::find(joint_names_.begin(), joint_names_.end(), jname);
            if (it != joint_names_.end()) {
                size_t idx = std::distance(joint_names_.begin(), it);
                double val = latest_joint_positions_[idx];
                planning_positions.push_back(val);
                RCLCPP_INFO(this->get_logger(), "  start joint %s = %.4f (matched from /joint_states[%zu])",
                    jname.c_str(), val, idx);
            } else {
                planning_positions.push_back(0.0); // fallback if not found
                ++joint_name_fallback_count;
                RCLCPP_ERROR(this->get_logger(), "  start joint %s = 0.0 (FALLBACK — name NOT found in /joint_states!)",
                    jname.c_str());
            }
        }
        if (joint_name_fallback_count > 0) {
            RCLCPP_ERROR(this->get_logger(),
                "CRITICAL: %d/%zu planning joints fell back to 0.0! Joint name mismatch between KDL chain and /joint_states.",
                joint_name_fallback_count, planning_joints.size());
            // Print /joint_states names for debugging
            std::ostringstream js_names;
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                if (i > 0) js_names << ", ";
                js_names << joint_names_[i];
            }
            RCLCPP_INFO(this->get_logger(), "/joint_states joint names (%zu): [%s]",
                joint_names_.size(), js_names.str().c_str());
        }
        const KDL::Vector original_target(
            pose_in_base.pose.position.x,
            pose_in_base.pose.position.y,
            pose_in_base.pose.position.z);
        const auto original_orientation = pose_in_base.pose.orientation;
        const KDL::Vector shoulder_to_target = original_target - right_shoulder_pos_in_base_;
        const double shoulder_to_target_norm = shoulder_to_target.Norm();
        KDL::Vector retract_unit(0.0, 0.0, 0.0);
        int max_retract_step = 0;
        if (fallback_enabled_ && fallback_max_retraction_ > 0.0 &&
            fallback_step_ > 0.0 && shoulder_to_target_norm > 1e-6) {
            retract_unit = shoulder_to_target / shoulder_to_target_norm;
            max_retract_step = static_cast<int>(
                std::floor(fallback_max_retraction_ / fallback_step_ + 1e-9));
            max_retract_step = std::max(0, max_retract_step);
        } else if (fallback_enabled_) {
            RCLCPP_WARN(this->get_logger(),
                "Fallback disabled for this goal: max_retraction=%.4f, step=%.4f, shoulder_target_norm=%.6f",
                fallback_max_retraction_, fallback_step_, shoulder_to_target_norm);
        }

        auto ik_result_str = [](int result) {
            switch (result) {
                case -1: return "Solver init error: invalid chain or limits";
                case -3: return "No solution found";
                default: return result > 0 ? "Success" : "Unknown error";
            }
        };

        std::vector<GoalCandidate> candidates;
        if (adaptive_orientation_enabled_) {
            for (int step = 0; step <= max_retract_step; ++step) {
                const double retraction = fallback_step_ * static_cast<double>(step);
                const KDL::Vector candidate_position(
                    original_target.x() - retraction * retract_unit.x(),
                    original_target.y() - retraction * retract_unit.y(),
                    original_target.z() - retraction * retract_unit.z());
                double qx = original_orientation.x;
                double qy = original_orientation.y;
                double qz = original_orientation.z;
                double qw = original_orientation.w;
                KDL::Vector dir;
                const auto status = computeAdaptiveOrientation(
                    candidate_position, qx, qy, qz, qw, dir);
                if (status == AdaptiveOrientationStatus::TARGET_TOO_CLOSE_TO_SHOULDER) {
                    if (step == 0) {
                        RCLCPP_ERROR(this->get_logger(),
                            "Target [%.3f, %.3f, %.3f] within 0.05 m of right shoulder "
                            "[%.3f, %.3f, %.3f]; adaptive orientation cannot produce a "
                            "stable direction. Aborting goal.",
                            candidate_position.x(), candidate_position.y(), candidate_position.z(),
                            right_shoulder_pos_in_base_.x(),
                            right_shoulder_pos_in_base_.y(),
                            right_shoulder_pos_in_base_.z());
                        return;
                    }
                    RCLCPP_WARN(this->get_logger(),
                        "Fallback candidate step=%d retraction=%.4f m is too close to shoulder; stopping fallback search",
                        step, retraction);
                    break;
                }
                RCLCPP_INFO(this->get_logger(),
                    "Adaptive orientation step=%d: target=[%.3f, %.3f, %.3f] "
                    "shoulder=[%.3f, %.3f, %.3f] dir=[%.3f, %.3f, %.3f] "
                    "q=[%.4f, %.4f, %.4f, %.4f]",
                    step,
                    candidate_position.x(), candidate_position.y(), candidate_position.z(),
                    right_shoulder_pos_in_base_.x(),
                    right_shoulder_pos_in_base_.y(),
                    right_shoulder_pos_in_base_.z(),
                    dir.x(), dir.y(), dir.z(),
                    qx, qy, qz, qw);
                GoalCandidate candidate;
                candidate.label = step == 0 ? "adaptive-base" : "adaptive-retract";
                candidate.position = candidate_position;
                candidate.qx = qx;
                candidate.qy = qy;
                candidate.qz = qz;
                candidate.qw = qw;
                candidate.retraction = retraction;
                candidate.retraction_step = step;
                candidate.fallback_candidate = step > 0;
                candidates.push_back(candidate);
            }
        } else {
            GoalCandidate base_candidate;
            base_candidate.label = "base";
            base_candidate.position = original_target;
            base_candidate.qx = original_orientation.x;
            base_candidate.qy = original_orientation.y;
            base_candidate.qz = original_orientation.z;
            base_candidate.qw = original_orientation.w;
            candidates.push_back(base_candidate);
            if (fallback_enabled_ && max_retract_step > 0) {
                for (int step = 1; step <= max_retract_step; ++step) {
                    const double retraction = fallback_step_ * static_cast<double>(step);
                    GoalCandidate candidate;
                    candidate.label = "base-retract";
                    candidate.position = KDL::Vector(
                        original_target.x() - retraction * retract_unit.x(),
                        original_target.y() - retraction * retract_unit.y(),
                        original_target.z() - retraction * retract_unit.z());
                    candidate.qx = original_orientation.x;
                    candidate.qy = original_orientation.y;
                    candidate.qz = original_orientation.z;
                    candidate.qw = original_orientation.w;
                    candidate.retraction = retraction;
                    candidate.retraction_step = step;
                    candidate.fallback_candidate = true;
                    candidates.push_back(candidate);
                }
            }
        }

        if (candidates.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No goal candidates generated. Aborting.");
            return;
        }
        const int max_candidate_idx = static_cast<int>(candidates.size()) - 1;
        const auto fallback_start_time = std::chrono::steady_clock::now();

        for (int candidate_idx = 0; candidate_idx <= max_candidate_idx; ++candidate_idx) {
            const auto& candidate = candidates[candidate_idx];
            if (!adaptive_orientation_enabled_ && candidate.fallback_candidate && fallback_total_timeout_s_ > 0.0) {
                const double elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - fallback_start_time).count();
                if (elapsed > fallback_total_timeout_s_) {
                    RCLCPP_ERROR(this->get_logger(),
                        "All fallback candidates exhausted by timeout after %.2f s", elapsed);
                    break;
                }
            }
            const double retraction = candidate.retraction;
            const KDL::Vector candidate_position = candidate.position;
            const double qx = candidate.qx;
            const double qy = candidate.qy;
            const double qz = candidate.qz;
            const double qw = candidate.qw;
            RCLCPP_INFO(this->get_logger(),
                "Attempting target idx=%d/%d source=%s retract_step=%d retract=%.4f m pos=[%.4f, %.4f, %.4f]",
                candidate_idx, max_candidate_idx, candidate.label.c_str(),
                candidate.retraction_step, retraction,
                candidate_position.x(), candidate_position.y(), candidate_position.z());
            KDL::Frame target_frame(KDL::Rotation::Quaternion(
                                        qx,
                                        qy,
                                        qz,
                                        qw),
                                    candidate_position);

            KDL::JntArray seed(planning_joints.size());
            for (size_t i = 0; i < planning_joints.size(); ++i) seed(i) = planning_positions[i];
            auto solver = ik_right;
            KDL::JntArray goal(planning_joints.size());
            auto try_ik_candidate = [&](const KDL::JntArray& candidate_seed, const std::string& label, bool log_failure) {
                KDL::JntArray candidate_goal(planning_joints.size());
                int result = solver->CartToJnt(candidate_seed, target_frame, candidate_goal);
                if (result <= 0) {
                    if (log_failure) {
                        RCLCPP_INFO(this->get_logger(), "TRAC-IK %s/%s result: %s (code: %d)",
                            candidate.label.c_str(), label.c_str(), ik_result_str(result), result);
                    } else {
                        RCLCPP_DEBUG(this->get_logger(), "TRAC-IK %s/%s result: %s (code: %d)",
                            candidate.label.c_str(), label.c_str(), ik_result_str(result), result);
                    }
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "TRAC-IK %s/%s result: %s (code: %d)",
                    candidate.label.c_str(), label.c_str(), ik_result_str(result), result);
                if (isInCollision(candidate_goal, this->collision_skip_pairs_, planning_links)) {
                    RCLCPP_WARN(this->get_logger(), "TRAC-IK %s/%s solution rejected: collision",
                        candidate.label.c_str(), label.c_str());
                    return false;
                }
                goal = candidate_goal;
                RCLCPP_INFO(this->get_logger(), "TRAC-IK %s/%s solution accepted",
                    candidate.label.c_str(), label.c_str());
                return true;
            };

            bool goal_found = try_ik_candidate(seed, "current-seed", true);
            if (!goal_found) {
                KDL::JntArray neutral_seed(planning_joints.size());
                for (size_t i = 0; i < planning_joints.size(); ++i) neutral_seed(i) = 0.0;
                goal_found = try_ik_candidate(neutral_seed, "neutral-seed", true);
            }
            if (!goal_found) {
                std::random_device rd;
                std::mt19937 gen(rd());
                const int max_random_tries = std::max(0, ik_random_seed_tries_);
                for (int random_tries = 0; random_tries < max_random_tries && !goal_found; ++random_tries) {
                    KDL::JntArray random_seed(planning_joints.size());
                    for (size_t i = 0; i < planning_joints.size(); ++i) {
                        auto lim_it = joint_limits_.find(planning_joints[i]);
                        double low = lim_it != joint_limits_.end() ? lim_it->second.first : -3.14;
                        double high = lim_it != joint_limits_.end() ? lim_it->second.second : 3.14;
                        std::uniform_real_distribution<> dis(low, high);
                        random_seed(i) = dis(gen);
                    }
                    goal_found = try_ik_candidate(random_seed, "random-seed", false);
                }
                if (!goal_found && max_random_tries > 0) {
                    RCLCPP_WARN(this->get_logger(),
                        "TRAC-IK random-seed exhausted for target idx=%d source=%s after %d tries",
                        candidate_idx, candidate.label.c_str(), max_random_tries);
                }
            }
            if (!goal_found) {
                if (candidate_idx < max_candidate_idx) {
                    RCLCPP_WARN(this->get_logger(),
                        "IK failed for target idx=%d source=%s retraction=%.4f m; trying next fallback candidate",
                        candidate_idx, candidate.label.c_str(), retraction);
                } else {
                    RCLCPP_ERROR(this->get_logger(),
                        "IK failed to find a collision-free goal state for final target candidate. Aborting.");
                }
                continue;
            }

        // ------------------------------------------------------------------
        // FK diagnostic: verify IK goal reaches target and start is correct
        // ------------------------------------------------------------------
        {
            KDL::Frame start_fk, goal_fk;
            KDL::JntArray start_jnt(planning_joints.size());
            for (size_t i = 0; i < planning_joints.size(); ++i) start_jnt(i) = planning_positions[i];
            int start_ret = fk_right_solver->JntToCart(start_jnt, start_fk);
            int goal_ret = fk_right_solver->JntToCart(goal, goal_fk);

            RCLCPP_INFO(this->get_logger(),
                "=== FK DIAGNOSTIC ===");
            RCLCPP_INFO(this->get_logger(),
                "  Target frame xyz:  [%.4f, %.4f, %.4f]",
                target_frame.p.x(), target_frame.p.y(), target_frame.p.z());
            RCLCPP_INFO(this->get_logger(),
                "  Start FK xyz:      [%.4f, %.4f, %.4f] (ret=%d)",
                start_fk.p.x(), start_fk.p.y(), start_fk.p.z(), start_ret);
            RCLCPP_INFO(this->get_logger(),
                "  Goal FK xyz:       [%.4f, %.4f, %.4f] (ret=%d)",
                goal_fk.p.x(), goal_fk.p.y(), goal_fk.p.z(), goal_ret);
            RCLCPP_INFO(this->get_logger(),
                "  Goal FK error:     [%.4f, %.4f, %.4f]  norm=%.4f m",
                goal_fk.p.x() - target_frame.p.x(),
                goal_fk.p.y() - target_frame.p.y(),
                goal_fk.p.z() - target_frame.p.z(),
                std::sqrt(std::pow(goal_fk.p.x() - target_frame.p.x(), 2) +
                          std::pow(goal_fk.p.y() - target_frame.p.y(), 2) +
                          std::pow(goal_fk.p.z() - target_frame.p.z(), 2)));
            RCLCPP_INFO(this->get_logger(),
                "  Start→Goal joint delta norm: %.4f rad",
                [&](){
                    double sum = 0.0;
                    for (size_t i = 0; i < planning_joints.size(); ++i)
                        sum += std::pow(goal(i) - planning_positions[i], 2);
                    return std::sqrt(sum);
                }());
            // Print start and goal joint values compactly
            {
                std::ostringstream s_joints, g_joints;
                for (size_t i = 0; i < planning_joints.size(); ++i) {
                    if (i > 0) { s_joints << ", "; g_joints << ", "; }
                    s_joints << std::fixed << std::setprecision(4) << planning_positions[i];
                    g_joints << std::fixed << std::setprecision(4) << goal(i);
                }
                RCLCPP_INFO(this->get_logger(), "  Start joints (rad): [%s]", s_joints.str().c_str());
                RCLCPP_INFO(this->get_logger(), "  Goal  joints (rad): [%s]", g_joints.str().c_str());
            }
            RCLCPP_INFO(this->get_logger(),
                "=== END FK DIAGNOSTIC ===");
        }

        // OMPL bounds: use URDF joint limits if available, else fallback to [-3.14, 3.14]
        auto space = std::make_shared<ob::RealVectorStateSpace>(planning_joints.size());
        ob::RealVectorBounds bounds(planning_joints.size());
        for (size_t i = 0; i < planning_joints.size(); ++i) {
            auto lim_it = joint_limits_.find(planning_joints[i]);
            if (lim_it != joint_limits_.end()) {
                bounds.setLow(i, lim_it->second.first);
                bounds.setHigh(i, lim_it->second.second);
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "OMPL bounds: joint '%s' has no URDF limit; falling back to [-3.14, 3.14]. "
                    "This indicates a URDF regression or missing right-arm joint.",
                    planning_joints[i].c_str());
                bounds.setLow(i, -3.14);
                bounds.setHigh(i, 3.14);
            }
        }
        {
            std::ostringstream bounds_oss;
            bounds_oss << "OMPL bounds set for " << planning_joints.size()
                       << " right-arm joints:";
            for (size_t i = 0; i < planning_joints.size(); ++i) {
                bounds_oss << " [" << planning_joints[i]
                           << "=" << bounds.low[i]
                           << "," << bounds.high[i] << "]";
            }
            RCLCPP_INFO(this->get_logger(), "%s", bounds_oss.str().c_str());
        }
        space->setBounds(bounds);
        auto ss = std::make_shared<og::SimpleSetup>(space);
        // Pass planning_links to the state validity checker
        ss->setStateValidityChecker([this, planning_joints, planning_links](const ob::State* state) {
            const double* values = state->as<ob::RealVectorStateSpace::StateType>()->values;
            KDL::JntArray joints(planning_joints.size());
            std::ostringstream state_oss;
            state_oss << "[ ";
            for (size_t i = 0; i < planning_joints.size(); ++i) {
                joints(i) = values[i];
                state_oss << planning_joints[i] << "=" << values[i];
                if (i + 1 < planning_joints.size()) state_oss << ", ";
            }
            state_oss << " ]";
            bool is_in_collision = isInCollision(joints, this->collision_skip_pairs_, planning_links);
            RCLCPP_DEBUG(this->get_logger(), "Checking collision for state %s, result: %s", state_oss.str().c_str(), is_in_collision ? "Collision" : "Free");
            return !is_in_collision;
        });
        ob::ScopedState<> start(space), goal_state(space);
        for (size_t i = 0; i < planning_joints.size(); ++i) start[i] = planning_positions[i];
        for (size_t i = 0; i < planning_joints.size(); ++i) goal_state[i] = goal(i);
        ss->setStartAndGoalStates(start, goal_state);

        {
            const bool start_valid =
                ss->getStateValidityChecker()->isValid(start.get());
            const bool goal_valid =
                ss->getStateValidityChecker()->isValid(goal_state.get());
            if (!start_valid || !goal_valid) {
                RCLCPP_WARN(this->get_logger(),
                    "OMPL state validity: start=%s goal=%s",
                    start_valid ? "VALID" : "INVALID",
                    goal_valid ? "VALID" : "INVALID");
            }
        }

        ss->getSpaceInformation()->setStateValidityCheckingResolution(0.01);
        
        if (planner_type_ == "RRTConnect") {
            ss->setPlanner(std::make_shared<og::RRTConnect>(ss->getSpaceInformation()));
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown planner_type '%s', defaulting to RRTConnect", planner_type_.c_str());
            ss->setPlanner(std::make_shared<og::RRTConnect>(ss->getSpaceInformation()));
        }
        if (ss->solve(planning_timeout_)) {
            auto path = ss->getSolutionPath();
            const std::size_t n_before = path.getStateCount();
            if (simplify_method_ == "simple") {
                ss->simplifySolution(simplify_timeout_);
                path = ss->getSolutionPath();
            } else if (simplify_method_ == "manual") {
                og::PathSimplifier ps(ss->getSpaceInformation());
                auto simplify_start = std::chrono::steady_clock::now();
                ps.partialShortcutPath(path, static_cast<unsigned int>(simplify_max_steps_), static_cast<unsigned int>(simplify_max_empty_steps_));
                ps.reduceVertices(path);
                double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - simplify_start).count();
                if (elapsed > simplify_timeout_) {
                    RCLCPP_WARN(this->get_logger(), "Manual path simplification exceeded simplify_timeout %.2f seconds (elapsed %.2f seconds)",
                                simplify_timeout_, elapsed);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown simplify_method '%s'; skipping path simplification", simplify_method_.c_str());
            }
            const std::size_t n_after = path.getStateCount();
            RCLCPP_INFO(this->get_logger(), "Simplified: %zu → %zu waypoints (-%zu%%)",
                        n_before, n_after, n_before > 0 ? (n_before - n_after) * 100 / n_before : 0);
            const std::size_t n_before_interp = path.getStateCount();
            path.interpolate();
            const std::size_t n_after_interp = path.getStateCount();
            RCLCPP_INFO(this->get_logger(), "After interpolate(): %zu → %zu states (+%zu)",
                n_before_interp, n_after_interp, n_after_interp - n_before_interp);
            const auto& states = path.getStates();
            trajectory_msgs::msg::JointTrajectory traj_msg;
            traj_msg.joint_names = planning_joints;
            double cumulative_time = 0.0;
            for (size_t idx = 0; idx < states.size(); ++idx) {
                const auto& state = states[idx];
                trajectory_msgs::msg::JointTrajectoryPoint point;
                for (size_t i = 0; i < planning_joints.size(); ++i) {
                    point.positions.push_back(state->as<ob::RealVectorStateSpace::StateType>()->values[i]);
                }
                double dt = min_time_step_;
                if (idx > 0) {
                    const auto& prev = states[idx - 1];
                    double max_dt = 0.0;
                    for (size_t i = 0; i < planning_joints.size(); ++i) {
                        double dq = std::abs(state->as<ob::RealVectorStateSpace::StateType>()->values[i] -
                                             prev->as<ob::RealVectorStateSpace::StateType>()->values[i]);
                        auto vel_it = velocity_limits_.find(planning_joints[i]);
                        double vel_limit = (vel_it != velocity_limits_.end()) ? vel_it->second : 10.0;
                        double seg_dt = dq / (vel_limit * velocity_scale_);
                        if (seg_dt > max_dt) max_dt = seg_dt;
                    }
                    dt = std::max(max_dt, min_time_step_);
                }
                cumulative_time += dt;
                point.time_from_start = rclcpp::Duration::from_seconds(cumulative_time);
                traj_msg.points.push_back(point);
            }
            RCLCPP_INFO(this->get_logger(), "Trajectory: %zu waypoints, %.3f seconds total duration (velocity_scale=%.3f, min_time_step=%.3f)",
                        traj_msg.points.size(), cumulative_time, velocity_scale_, min_time_step_);
            // --- Velocity limit & per-segment diagnostic ---
            {
                std::ostringstream vel_limits_oss;
                vel_limits_oss << "Velocity limits for " << planning_joints.size() << " right-arm joints:";
                int vel_found = 0, vel_missing = 0;
                for (size_t i = 0; i < planning_joints.size(); ++i) {
                    auto vel_it = velocity_limits_.find(planning_joints[i]);
                    bool found = (vel_it != velocity_limits_.end());
                    double v = found ? vel_it->second : 10.0;
                    vel_limits_oss << " [" << planning_joints[i] << "=" << std::fixed << std::setprecision(2) << v;
                    vel_limits_oss << (found ? "]" : "(fallback)]");
                    if (found) ++vel_found; else ++vel_missing;
                }
                RCLCPP_INFO(this->get_logger(), "%s", vel_limits_oss.str().c_str());
                if (vel_missing > 0) {
                    RCLCPP_WARN(this->get_logger(), "%d/%zu joints missing velocity limits — using fallback 10.0 rad/s",
                        vel_missing, planning_joints.size());
                }
                RCLCPP_INFO(this->get_logger(), "Per-segment dt (max across joints):");
                for (size_t idx = 1; idx < traj_msg.points.size(); ++idx) {
                    double t_prev = rclcpp::Duration(traj_msg.points[idx - 1].time_from_start).seconds();
                    double t_curr = rclcpp::Duration(traj_msg.points[idx].time_from_start).seconds();
                    double dt = t_curr - t_prev;
                    double max_dq = 0.0;
                    std::string max_joint;
                    for (size_t i = 0; i < planning_joints.size(); ++i) {
                        double dq = std::abs(traj_msg.points[idx].positions[i] - traj_msg.points[idx - 1].positions[i]);
                        if (dq > max_dq) { max_dq = dq; max_joint = planning_joints[i]; }
                    }
                    RCLCPP_INFO(this->get_logger(), "  seg[%zu]: dt=%.4fs  max_dq=%.4frad (%s)",
                        idx, dt, max_dq, max_joint.c_str());
                }
            }
            // Pre-publish trajectory validation with auto-fix
            bool position_fixed = false;
            bool velocity_fixed = false;
            for (auto& pt : traj_msg.points) {
                for (size_t i = 0; i < planning_joints.size(); ++i) {
                    auto lim_it = joint_limits_.find(planning_joints[i]);
                    if (lim_it == joint_limits_.end()) continue;
                    double lower = lim_it->second.first;
                    double upper = lim_it->second.second;
                    if (pt.positions[i] < lower || pt.positions[i] > upper) {
                        RCLCPP_WARN(this->get_logger(), "Auto-fix: clamped joint %s position %.4f to [%.4f, %.4f]",
                                    planning_joints[i].c_str(), pt.positions[i], lower, upper);
                        pt.positions[i] = std::max(lower, std::min(upper, pt.positions[i]));
                        position_fixed = true;
                    }
                }
            }
            for (size_t idx = 1; idx < traj_msg.points.size(); ++idx) {
                double t_prev = rclcpp::Duration(traj_msg.points[idx - 1].time_from_start).seconds();
                double t_curr = rclcpp::Duration(traj_msg.points[idx].time_from_start).seconds();
                double dt = t_curr - t_prev;
                if (dt <= 0.0) dt = min_time_step_;
                double max_required_dt = 0.0;
                for (size_t i = 0; i < planning_joints.size(); ++i) {
                    double dq = std::abs(traj_msg.points[idx].positions[i] - traj_msg.points[idx - 1].positions[i]);
                    auto vel_it = velocity_limits_.find(planning_joints[i]);
                    double vel_limit = (vel_it != velocity_limits_.end()) ? vel_it->second : 10.0;
                    double required_dt = dq / (vel_limit * velocity_scale_);
                    if (required_dt > max_required_dt) max_required_dt = required_dt;
                }
                max_required_dt = std::max(max_required_dt, min_time_step_);
                if (max_required_dt > dt + 1e-9) {
                    RCLCPP_WARN(this->get_logger(), "Auto-fix: stretched dt at waypoint %zu for velocity limit", idx);
                    double delta = max_required_dt - dt;
                    for (size_t j = idx; j < traj_msg.points.size(); ++j) {
                        double t = rclcpp::Duration(traj_msg.points[j].time_from_start).seconds();
                        traj_msg.points[j].time_from_start = rclcpp::Duration::from_seconds(t + delta);
                    }
                    velocity_fixed = true;
                }
            }
            bool re_valid = true;
            for (size_t idx = 1; idx < traj_msg.points.size(); ++idx) {
                double t_prev = rclcpp::Duration(traj_msg.points[idx - 1].time_from_start).seconds();
                double t_curr = rclcpp::Duration(traj_msg.points[idx].time_from_start).seconds();
                double dt = t_curr - t_prev;
                if (dt <= 0.0) dt = min_time_step_;
                for (size_t i = 0; i < planning_joints.size(); ++i) {
                    double dq = std::abs(traj_msg.points[idx].positions[i] - traj_msg.points[idx - 1].positions[i]);
                    auto vel_it = velocity_limits_.find(planning_joints[i]);
                    double vel_limit = (vel_it != velocity_limits_.end()) ? vel_it->second : 10.0;
                    if (dq / dt > vel_limit * velocity_scale_ + 1e-6) {
                        re_valid = false;
                        break;
                    }
                }
                if (!re_valid) break;
            }
            if (!re_valid) {
                RCLCPP_ERROR(this->get_logger(), "Trajectory validation failed after auto-fix, rejecting");
                continue;
            }
            bool any_fixed = position_fixed || velocity_fixed;
            RCLCPP_INFO(this->get_logger(), "Trajectory validation passed%s", any_fixed ? " (with auto-fix)" : "");
            bool point_size_valid = true;
            for (const auto& pt : traj_msg.points) {
                if (pt.positions.size() != traj_msg.joint_names.size()) {
                    RCLCPP_ERROR(this->get_logger(), "Trajectory point size mismatch: %zu vs %zu", pt.positions.size(), traj_msg.joint_names.size());
                    point_size_valid = false;
                    break;
                }
            }
            if (!point_size_valid) {
                continue;
            }
            // ------------------------------------------------------------------
            // FK verification: check first and last trajectory waypoint TCP pose
            // ------------------------------------------------------------------
            if (!traj_msg.points.empty()) {
                const auto& first_pt = traj_msg.points.front();
                const auto& last_pt = traj_msg.points.back();
                KDL::JntArray first_jnt(planning_joints.size()), last_jnt(planning_joints.size());
                for (size_t i = 0; i < planning_joints.size(); ++i) {
                    first_jnt(i) = first_pt.positions[i];
                    last_jnt(i) = last_pt.positions[i];
                }
                KDL::Frame first_fk, last_fk;
                int first_ret = fk_right_solver->JntToCart(first_jnt, first_fk);
                int last_ret = fk_right_solver->JntToCart(last_jnt, last_fk);
                RCLCPP_INFO(this->get_logger(),
                    "=== TRAJECTORY FK DIAGNOSTIC ===");
                RCLCPP_INFO(this->get_logger(),
                    "  First waypoint FK xyz: [%.4f, %.4f, %.4f] (ret=%d)",
                    first_fk.p.x(), first_fk.p.y(), first_fk.p.z(), first_ret);
                RCLCPP_INFO(this->get_logger(),
                    "  Last  waypoint FK xyz: [%.4f, %.4f, %.4f] (ret=%d)",
                    last_fk.p.x(), last_fk.p.y(), last_fk.p.z(), last_ret);
                RCLCPP_INFO(this->get_logger(),
                    "  Target frame xyz:     [%.4f, %.4f, %.4f]",
                    target_frame.p.x(), target_frame.p.y(), target_frame.p.z());
                RCLCPP_INFO(this->get_logger(),
                    "  Last waypoint FK error: [%.4f, %.4f, %.4f]  norm=%.4f m",
                    last_fk.p.x() - target_frame.p.x(),
                    last_fk.p.y() - target_frame.p.y(),
                    last_fk.p.z() - target_frame.p.z(),
                    std::sqrt(std::pow(last_fk.p.x() - target_frame.p.x(), 2) +
                              std::pow(last_fk.p.y() - target_frame.p.y(), 2) +
                              std::pow(last_fk.p.z() - target_frame.p.z(), 2)));
                RCLCPP_INFO(this->get_logger(),
                    "=== END TRAJECTORY FK DIAGNOSTIC ===");
            }
            traj_msg.header.stamp = this->now();
            traj_pub_->publish(traj_msg);
            RCLCPP_INFO(this->get_logger(),
                "Plan published: %zu waypoints over %zu right-arm joints",
                traj_msg.points.size(), traj_msg.joint_names.size());
            if (candidate_idx > 0) {
                RCLCPP_WARN(this->get_logger(),
                    "[fallback] Used candidate %d/%d, retraction=%.4f m, pos=[%.4f, %.4f, %.4f] "
                    "(original=[%.4f, %.4f, %.4f], source=%s)",
                    candidate_idx, max_candidate_idx, retraction,
                    candidate_position.x(), candidate_position.y(), candidate_position.z(),
                    original_target.x(), original_target.y(), original_target.z(),
                    candidate.label.c_str());
            }
            return;
        } else {
            if (candidate_idx < max_candidate_idx) {
                RCLCPP_WARN(this->get_logger(),
                    "OMPL failed for target idx=%d source=%s retraction=%.4f m; trying next fallback candidate",
                    candidate_idx, candidate.label.c_str(), retraction);
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "OMPL failed to find a path for final target candidate");
            }
        }
        }
        if (max_candidate_idx > 0) {
            RCLCPP_ERROR(this->get_logger(),
                "All %d target candidates failed for original target [%.4f, %.4f, %.4f]",
                max_candidate_idx + 1,
                original_target.x(), original_target.y(), original_target.z());
        }

        // ------------------------------------------------------------------
        // Last-resort IK-only fallback: skip collision check and OMPL,
        // just find any IK solution and publish a direct 2-point trajectory.
        // ------------------------------------------------------------------
        if (!fallback_ik_only_) return;

        RCLCPP_WARN(this->get_logger(),
            "[ik-only-fallback] All candidates failed; attempting IK-only on original target [%.4f, %.4f, %.4f]",
            original_target.x(), original_target.y(), original_target.z());

        KDL::Frame ik_only_target(KDL::Rotation::Quaternion(
            original_orientation.x, original_orientation.y,
            original_orientation.z, original_orientation.w),
            original_target);

        auto try_ik_no_collision = [&](const KDL::JntArray& seed, const std::string& label) -> bool {
            KDL::JntArray ik_goal(planning_joints.size());
            int result = ik_right->CartToJnt(seed, ik_only_target, ik_goal);
            if (result <= 0) {
                RCLCPP_DEBUG(this->get_logger(),
                    "[ik-only-fallback] TRAC-IK %s result: %s (code: %d)",
                    label.c_str(), ik_result_str(result), result);
                return false;
            }
            // Skip collision check — this is the last resort
            RCLCPP_WARN(this->get_logger(),
                "[ik-only-fallback] IK solution found via %s (collision check SKIPPED)", label.c_str());

            // Publish direct 2-point trajectory: start → goal
            trajectory_msgs::msg::JointTrajectory traj_msg;
            traj_msg.joint_names = planning_joints;
            trajectory_msgs::msg::JointTrajectoryPoint start_pt, goal_pt;
            for (size_t i = 0; i < planning_joints.size(); ++i) {
                start_pt.positions.push_back(planning_positions[i]);
                auto lim_it = joint_limits_.find(planning_joints[i]);
                double pos = ik_goal(i);
                if (lim_it != joint_limits_.end())
                    pos = std::max(lim_it->second.first, std::min(lim_it->second.second, pos));
                goal_pt.positions.push_back(pos);
            }
            start_pt.time_from_start = rclcpp::Duration::from_seconds(0.0);
            // Compute duration from max joint displacement / velocity
            double max_duration = 0.0;
            for (size_t i = 0; i < planning_joints.size(); ++i) {
                double dq = std::abs(goal_pt.positions[i] - planning_positions[i]);
                auto vel_it = velocity_limits_.find(planning_joints[i]);
                double vel_limit = (vel_it != velocity_limits_.end()) ? vel_it->second : 10.0;
                double seg_dt = dq / (vel_limit * velocity_scale_);
                if (seg_dt > max_duration) max_duration = seg_dt;
            }
            max_duration = std::max(max_duration, min_time_step_);
            goal_pt.time_from_start = rclcpp::Duration::from_seconds(max_duration);
            traj_msg.points.push_back(start_pt);
            traj_msg.points.push_back(goal_pt);
            traj_msg.header.stamp = this->now();
            traj_pub_->publish(traj_msg);
            RCLCPP_WARN(this->get_logger(),
                "[ik-only-fallback] Published direct trajectory: 2 waypoints, %.3f s duration",
                max_duration);
            return true;
        };

        // Try current seed, neutral seed, then random seeds
        KDL::JntArray seed(planning_joints.size());
        for (size_t i = 0; i < planning_joints.size(); ++i) seed(i) = planning_positions[i];
        if (try_ik_no_collision(seed, "current-seed")) return;

        KDL::JntArray neutral_seed(planning_joints.size());
        for (size_t i = 0; i < planning_joints.size(); ++i) neutral_seed(i) = 0.0;
        if (try_ik_no_collision(neutral_seed, "neutral-seed")) return;

        {
            std::random_device rd;
            std::mt19937 gen(rd());
            const int max_random_tries = std::max(10, ik_random_seed_tries_ * 5);
            for (int t = 0; t < max_random_tries; ++t) {
                KDL::JntArray random_seed(planning_joints.size());
                for (size_t i = 0; i < planning_joints.size(); ++i) {
                    auto lim_it = joint_limits_.find(planning_joints[i]);
                    double low = lim_it != joint_limits_.end() ? lim_it->second.first : -3.14;
                    double high = lim_it != joint_limits_.end() ? lim_it->second.second : 3.14;
                    std::uniform_real_distribution<> dis(low, high);
                    random_seed(i) = dis(gen);
                }
                if (try_ik_no_collision(random_seed, "random-seed")) return;
            }
        }

        RCLCPP_ERROR(this->get_logger(),
            "[ik-only-fallback] IK also failed for original target; giving up.");
    }

    // Restrict collision checking to only pairs where at least one link is in planning_links
    bool isInCollision(const KDL::JntArray& joints, const std::vector<std::string>& skip_pairs = {}, const std::set<std::string>& planning_links = {})
    {
        if (!collision_detection_enabled_) {
            return false;
        }
        auto& fk_solver = fk_right_solver;
        auto& kdl_chain = kdl_chain_right;
        std::map<std::string, KDL::Frame> segment_frames;
        KDL::Frame out;
        for (size_t i = 0; i < kdl_chain.getNrOfSegments(); ++i) {
            if (fk_solver->JntToCart(joints, out, i + 1) >= 0) {
                const auto& seg_name = kdl_chain.getSegment(i).getName();
                segment_frames[seg_name] = out;
            }
        }
        for (auto& [link_name, lc] : link_collisions) {
            auto it = segment_frames.find(lc.segment_name);
            if (it != segment_frames.end()) {
                const auto& frame = it->second;
                Eigen::Matrix3d rot;
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        rot(r, c) = frame.M(r, c);
                Eigen::Vector3d trans(frame.p.x(), frame.p.y(), frame.p.z());
                Eigen::Isometry3d world_tf = Eigen::Isometry3d::Identity();
                world_tf.linear() = rot;
                world_tf.translation() = trans;
                // Combine FK and local collision origin
                Eigen::Isometry3d final_tf = world_tf * lc.local_transform;
                fcl::Transform3d tf(final_tf.matrix());
                lc.object->setTransform(tf);
            }
        }
        for (auto it1 = link_collisions.begin(); it1 != link_collisions.end(); ++it1) {
            for (auto it2 = std::next(it1); it2 != link_collisions.end(); ++it2) {
                // Skip a pair only when NEITHER link is in the right-arm planning chain.
                // We want to check: arm-vs-arm, arm-vs-body. We don't care about
                // body-vs-body (those are static, irrelevant to arm motion safety).
                if (!planning_links.empty() &&
                    planning_links.find(it1->first) == planning_links.end() &&
                    planning_links.find(it2->first) == planning_links.end()) {
                    continue;
                }
                // Skip collision pairs if specified (format: "link1:link2")
                std::string pair1 = it1->first + ":" + it2->first;
                std::string pair2 = it2->first + ":" + it1->first;
                if (std::find(skip_pairs.begin(), skip_pairs.end(), pair1) != skip_pairs.end() ||
                    std::find(skip_pairs.begin(), skip_pairs.end(), pair2) != skip_pairs.end()) {
                    continue;
                }
                fcl::CollisionRequestd req;
                fcl::CollisionResultd res;
                fcl::collide(it1->second.object.get(), it2->second.object.get(), req, res);
                if (res.isCollision()) {
                    RCLCPP_WARN(this->get_logger(), "Collision detected between %s and %s", it1->first.c_str(), it2->first.c_str());
                    return true;
                }
            }
        }
        return false;
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IKFCLPlannerNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

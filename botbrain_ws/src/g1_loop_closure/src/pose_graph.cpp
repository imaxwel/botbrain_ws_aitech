#include "g1_loop_closure/pose_graph.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

namespace g1_loop_closure
{
namespace
{

Eigen::Vector3d ConstraintError(
  const Pose2 &from, const Pose2 &to, const Pose2 &measurement)
{
  const Pose2 residual = Between(measurement, Between(from, to));
  return {residual.x, residual.y, residual.yaw};
}

Pose2 Perturb(Pose2 pose, const int index, const double epsilon)
{
  if (index == 0) {
    pose.x += epsilon;
  } else if (index == 1) {
    pose.y += epsilon;
  } else {
    pose.yaw = WrapAngle(pose.yaw + epsilon);
  }
  return pose;
}

Eigen::Matrix3d NumericalJacobian(
  const Pose2 &from, const Pose2 &to, const Pose2 &measurement, const bool from_node)
{
  constexpr double translation_epsilon = 1e-3;
  constexpr double yaw_epsilon = 1e-4;
  const Eigen::Vector3d base = ConstraintError(from, to, measurement);
  Eigen::Matrix3d jacobian;
  for (int axis = 0; axis < 3; ++axis) {
    const double epsilon = axis == 2 ? yaw_epsilon : translation_epsilon;
    const Eigen::Vector3d shifted = from_node ?
      ConstraintError(Perturb(from, axis, epsilon), to, measurement) :
      ConstraintError(from, Perturb(to, axis, epsilon), measurement);
    Eigen::Vector3d difference = shifted - base;
    difference.z() = WrapAngle(difference.z());
    jacobian.col(axis) = difference / epsilon;
  }
  return jacobian;
}

void AddBlock(
  std::vector<Eigen::Triplet<double>> &triplets, const size_t row_node,
  const size_t col_node, const Eigen::Matrix3d &block)
{
  if (row_node == 0 || col_node == 0) {
    return;
  }
  const int row = static_cast<int>(3 * (row_node - 1));
  const int col = static_cast<int>(3 * (col_node - 1));
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      triplets.emplace_back(row + r, col + c, block(r, c));
    }
  }
}

void AddVector(Eigen::VectorXd &vector, const size_t node, const Eigen::Vector3d &value)
{
  if (node == 0) {
    return;
  }
  vector.segment<3>(static_cast<Eigen::Index>(3 * (node - 1))) += value;
}

}  // namespace

bool OptimizePoseGraph(
  std::vector<Pose2> &poses,
  const std::vector<PoseGraphEdge> &edges,
  const int iterations,
  std::string *failure_reason)
{
  if (poses.size() < 3 || iterations < 1) {
    return true;
  }
  const int dimension = static_cast<int>(3 * (poses.size() - 1));
  for (int iteration = 0; iteration < iterations; ++iteration) {
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(edges.size() * 36 + static_cast<size_t>(dimension));
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dimension);
    for (const auto &edge : edges) {
      if (edge.from >= poses.size() || edge.to >= poses.size()) {
        if (failure_reason != nullptr) {
          *failure_reason = "graph contains an out-of-range keyframe edge";
        }
        return false;
      }
      const Pose2 &from = poses.at(edge.from);
      const Pose2 &to = poses.at(edge.to);
      const Eigen::Vector3d error = ConstraintError(from, to, edge.measurement);
      const Eigen::Matrix3d from_jacobian = NumericalJacobian(from, to, edge.measurement, true);
      const Eigen::Matrix3d to_jacobian = NumericalJacobian(from, to, edge.measurement, false);
      Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
      information(0, 0) = edge.xy_weight;
      information(1, 1) = edge.xy_weight;
      information(2, 2) = edge.yaw_weight;
      AddBlock(triplets, edge.from, edge.from, from_jacobian.transpose() * information * from_jacobian);
      AddBlock(triplets, edge.from, edge.to, from_jacobian.transpose() * information * to_jacobian);
      AddBlock(triplets, edge.to, edge.from, to_jacobian.transpose() * information * from_jacobian);
      AddBlock(triplets, edge.to, edge.to, to_jacobian.transpose() * information * to_jacobian);
      AddVector(gradient, edge.from, from_jacobian.transpose() * information * error);
      AddVector(gradient, edge.to, to_jacobian.transpose() * information * error);
    }
    for (int index = 0; index < dimension; ++index) {
      triplets.emplace_back(index, index, 1e-5);
    }
    Eigen::SparseMatrix<double> hessian(dimension, dimension);
    hessian.setFromTriplets(
      triplets.begin(), triplets.end(),
      [](const double first, const double second) {return first + second;});
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(hessian);
    if (solver.info() != Eigen::Success) {
      if (failure_reason != nullptr) {
        *failure_reason = "pose-graph factorization failed";
      }
      return false;
    }
    const Eigen::VectorXd update = solver.solve(-gradient);
    if (solver.info() != Eigen::Success || !update.allFinite()) {
      if (failure_reason != nullptr) {
        *failure_reason = "pose-graph solve failed";
      }
      return false;
    }
    double largest_update = 0.0;
    for (size_t node = 1; node < poses.size(); ++node) {
      const Eigen::Index offset = static_cast<Eigen::Index>(3 * (node - 1));
      poses[node].x += update(offset);
      poses[node].y += update(offset + 1);
      poses[node].yaw = WrapAngle(poses[node].yaw + update(offset + 2));
      largest_update = std::max(largest_update, update.segment<3>(offset).norm());
    }
    if (largest_update < 1e-4) {
      break;
    }
  }
  return true;
}

}  // namespace g1_loop_closure

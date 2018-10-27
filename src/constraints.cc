/**
 * constraints.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 27, 2018
 * Authors: Toki Migimatsu
 */

#include "constraints.h"

#include <cassert>  // assert

namespace TrajOpt {

void JointPositionConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                                      Eigen::Ref<Eigen::VectorXd> constraints) {
  constraints = 0.5 * (Q.col(timestep) - q_des).array().square();
  // constraints.eval();
}

void JointPositionConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Jacobian = Q.col(timestep).eval() - q_des;
  // Jacobian.eval();
}

void JointPositionConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                              Eigen::Ref<Eigen::ArrayXi> idx_j) {
  for (size_t i = 0; i < len_jacobian; i++) {
    idx_i(i) += i;
    idx_j(i) = len_jacobian * timestep + i;
  }
  // idx_i.eval();
  // idx_j.eval();
}

void CartesianPoseConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::VectorXd> constraints) {
  ab_.set_q(Q.col(timestep));
  x_quat_err_.head<3>() = SpatialDyn::Position(ab_) - x_des;
  x_quat_err_.tail<3>() = SpatialDyn::Opspace::OrientationError(SpatialDyn::Orientation(ab_), quat_des);

  constraints(0) = 0.5 * x_quat_err_.squaredNorm();
  // constraints.eval();
}

void CartesianPoseConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::VectorXd> Jacobian) {
  // if ((ab_.q().array() != Q.col(timestep).array()).any()) {
    ab_.set_q(Q.col(timestep));
    x_quat_err_.head<3>() = SpatialDyn::Position(ab_) - x_des;
    x_quat_err_.tail<3>() = SpatialDyn::Opspace::OrientationError(SpatialDyn::Orientation(ab_), quat_des);
  // }

  Jacobian = (SpatialDyn::Jacobian(ab_).transpose() * x_quat_err_).eval();
  // Jacobian.eval();
}

void CartesianPoseConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                              Eigen::Ref<Eigen::ArrayXi> idx_j) {
  for (size_t i = 0; i < len_jacobian; i++) {
    idx_j(i) = ab_.dof() * timestep + i;
  }
  // idx_i.eval();
  // idx_j.eval();
}

}  // namespace TrajOpt

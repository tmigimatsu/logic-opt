/**
 * joint_position_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "TrajOpt/constraints/joint_position_constraint.h"

namespace TrajOpt {

void JointPositionConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::VectorXd> constraints) {
  constraints = 0.5 * (Q.col(t_start_) - q_des_).array().square();
  Constraint::Evaluate(Q, constraints);
}

void JointPositionConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Jacobian = Q.col(t_start_) - q_des_;
}

void JointPositionConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                              Eigen::Ref<Eigen::ArrayXi> idx_j) {
  const size_t& dof = len_jacobian_;
  for (size_t i = 0; i < len_jacobian_; i++) {
    idx_i(i) += i;
    idx_j(i) = dof * t_start_ + i;
  }
}

void JointPositionConstraint::Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                      Eigen::Ref<const Eigen::VectorXd> lambda,
                                      Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {
  const size_t& dof = len_jacobian_;
  for (size_t i = 0; i < dof; i++) {
    Hessian.coeffRef(t_start_ * dof + i, t_start_ * dof + i) += lambda(i);
  }
}

void JointPositionConstraint::HessianStructure(Eigen::SparseMatrix<bool>& Hessian) {
  const size_t& dof = len_jacobian_;
  for (size_t i = 0; i < dof; i++) {
    if (Hessian.coeff(t_start_ * dof + i, t_start_ * dof + i)) continue;
    Hessian.insert(t_start_ * dof + i, t_start_ * dof + i) = true;
  }
}

}  // namespace TrajOpt

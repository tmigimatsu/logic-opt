/**
 * cartesian_pose_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/cartesian_pose_constraint.h"

namespace LogicOpt {

CartesianPoseConstraint::CartesianPoseConstraint(World& world, size_t t_goal,
                                                 const std::string& control_frame,
                                                 const std::string& target_frame,
                                                 const Eigen::Vector6d& dx_des)
    : FrameConstraint(6, 6, t_goal, 1, control_frame, target_frame,
                      "constraint_cart_pos_t" + std::to_string(t_goal)),
      dx_des_(dx_des) {
  world.AttachFrame(control_frame_, target_frame_, t_goal);
}

void CartesianPoseConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                       Eigen::Ref<Eigen::VectorXd> constraints) {
  ComputeError(X);
  constraints = 0.5 * dx_err_.array().square();
  Constraint::Evaluate(X, constraints);
}

void CartesianPoseConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                       Eigen::Ref<Eigen::VectorXd> Jacobian) {
  ComputeError(X);
  Jacobian = dx_err_;
}

void CartesianPoseConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) {
  dx_err_ = X.col(t_start_) - dx_des_;
}

}  // namespace LogicOpt

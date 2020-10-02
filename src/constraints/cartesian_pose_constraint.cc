/**
 * cartesian_pose_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/cartesian_pose_constraint.h"

namespace logic_opt {

CartesianPoseConstraint::CartesianPoseConstraint(
    World& world, size_t t_goal, const std::string& control_frame,
    const std::string& target_frame, const Eigen::Vector3d& x_des,
    const Eigen::Quaterniond& quat_des)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_goal, kNumTimesteps,
                      control_frame, target_frame,
                      "constraint_t" + std::to_string(t_goal) + "_cart_pos") {
  x_des_ << x_des, quat_des.coeffs();
  world.ReserveTimesteps(t_goal + kNumTimesteps);
  world.AttachFrame(control_frame_, target_frame_, t_goal);
  world.set_controller("cart_pos", t_goal);
}

CartesianPoseConstraint::CartesianPoseConstraint(
    World& world, size_t t_goal, const std::string& control_frame,
    const std::string& target_frame, const Eigen::Isometry3d& T_des)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_goal, kNumTimesteps,
                      control_frame, target_frame,
                      "constraint_t" + std::to_string(t_goal) + "_cart_pos") {
  x_des_ << T_des.translation(), Eigen::Quaterniond(T_des.linear()).coeffs();
  world.ReserveTimesteps(t_goal + kNumTimesteps);
  world.AttachFrame(control_frame_, target_frame_, t_goal);
  world.set_controller("cart_pos", t_goal);
}

CartesianPoseConstraint::CartesianPoseConstraint(
    World& world, size_t t_goal, const std::string& control_frame,
    const std::string& target_frame, const spatial_opt::Isometry& T_des)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_goal, kNumTimesteps,
                      control_frame, target_frame,
                      "constraint_t" + std::to_string(t_goal) + "_cart_pos") {
  x_des_ << T_des.translation(), T_des.rotation().coeffs();
  world.ReserveTimesteps(t_goal + kNumTimesteps);
  world.AttachFrame(control_frame_, target_frame_, t_goal);
  world.set_controller("cart_pos", t_goal);
}

CartesianPoseConstraint::CartesianPoseConstraint(
    World& world, size_t t_goal, const std::string& control_frame,
    const std::string& target_frame, const Eigen::Vector3d& x_des)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_goal, kNumTimesteps,
                      control_frame, target_frame,
                      "constraint_t" + std::to_string(t_goal) + "_cart_pos") {
  x_des_ << x_des, Eigen::Quaterniond::Identity().coeffs();
  world.ReserveTimesteps(t_goal + kNumTimesteps);
  world.AttachFrame(control_frame_, target_frame_, t_goal);
  world.set_controller("cart_pos", t_goal);
}

void CartesianPoseConstraint::Evaluate(
    Eigen::Ref<const Eigen::MatrixXd> X,
    Eigen::Ref<Eigen::VectorXd> constraints) {
  x_err_ = X.col(t_start_) - x_des_;
  constraints = 0.5 * x_err_.array().square();
  Constraint::Evaluate(X, constraints);
}

void CartesianPoseConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                       Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Jacobian = x_err_;
  Constraint::Jacobian(X, Jacobian);
}

}  // namespace logic_opt

/**
 * pick_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/pick_constraint.h"

namespace {

const size_t kNumConstraints = 3;
const size_t kLenJacobian = kNumConstraints;
const size_t kNumTimesteps = 1;
const double kH = 1e-8;

}  // namespace

namespace LogicOpt {

PickConstraint::PickConstraint(World& world, size_t t_pick, const std::string& name_ee,
                               const std::string& name_object)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_pick, kNumTimesteps, name_ee, name_object,
                      "constraint_pick_t" + std::to_string(t_pick)),
      world_(world) {
  if (name_ee == world.kWorldFrame) {
    throw std::invalid_argument("PickConstraint::PickConstraint(): " + world.kWorldFrame +
                                " cannot be the ee frame.");
  } else if (name_ee == world.kWorldFrame) {
    throw std::invalid_argument("PickConstraint::PickConstraint(): " + world.kWorldFrame +
                                " cannot be the object frame.");
  }
  world.AttachFrame(name_ee, name_object, t_pick);
}

void PickConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> constraints) {
  x_err_ = ComputeError(X);
  constraints = 0.5 * x_err_.array().square();

  Constraint::Evaluate(X, constraints);
}

void PickConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> Jacobian) {
  // ComputeError(X);
  Jacobian = x_err_;
  // Eigen::Map<Eigen::MatrixXd> J(Jacobian.data(), kNumConstraints, kDof);
  // for (size_t i = 0; i < kDof; i++) {
  //   Eigen::MatrixXd X_h = X;
  //   X_h(i, t_start()) += kH;
  //   Eigen::Vector3d f_x_h = ComputeError(X_h);
  //   J.col(i) = (f_x_h - x_err_) / kH;
  // }
}

Eigen::Vector3d PickConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const {
  const Object& ee = world_.objects()->at(control_frame());
  const Object& object = world_.objects()->at(target_frame());
  const Eigen::Isometry3d T_ee_to_object = world_.T_control_to_target(X, t_start());
  const auto& x_ee = T_ee_to_object.translation();

  auto projection = object.collision->project_point(Eigen::Isometry3d::Identity(),
                                                    x_ee, false);
  return x_ee - projection.point;
}

}  // namespace LogicOpt

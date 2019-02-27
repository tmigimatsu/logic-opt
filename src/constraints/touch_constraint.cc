/**
 * touch_constraint.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 20, 2019
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/touch_constraint.h"

#include <ncollide/ncollide2d.h>

namespace {

#ifdef TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
const size_t kNumConstraints = 1;
const size_t kLenJacobian = 6;
#else  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
const size_t kNumConstraints = 3;
const size_t kLenJacobian = 3;
#endif  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
const size_t kNumTimesteps = 1;

const double kH = 1e-4;

}

namespace LogicOpt {

TouchConstraint::TouchConstraint(World& world, size_t t_touch,
                                 const std::string& name_control, const std::string& name_target)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_touch, kNumTimesteps, name_control, name_target,
                      "constraint_place_t" + std::to_string(t_touch)),
      world_(world) {
  if (name_control == world.kWorldFrame) {
    throw std::invalid_argument("TouchConstraint::TouchConstraint(): " + world.kWorldFrame +
                                " cannot be the control frame.");
  } else if (name_target == world.kWorldFrame) {
    throw std::invalid_argument("TouchConstraint::TouchConstraint(): " + world.kWorldFrame +
                                " cannot be the target frame.");
  }
  world.ReserveTimesteps(t_touch + kNumTimesteps);
  world.AttachFrame(name_control, name_target, t_touch);
}

void TouchConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                               Eigen::Ref<Eigen::VectorXd> constraints) {
  x_err_ = ComputeError(X);
#ifdef TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
  constraints(0) = x_err_;
#else  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
  constraints = 0.5 * x_err_.array().square();
#endif  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN

  Constraint::Evaluate(X, constraints);
}

void TouchConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                               Eigen::Ref<Eigen::VectorXd> Jacobian) {
#ifdef TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < kLenJacobian; i++) {
    // if (i == 3 || i == 4) continue;
    const double x_ij = X_h(i, t_start());
    X_h(i, t_start()) += kH;
    const double x_err_h = ComputeError(X_h);
    Jacobian(i) = (x_err_h - x_err_) / kH;
    X_h(i, t_start()) = x_ij;
    // if (i > 2) Jacobian(i) = 0.;
  }
  // std::cout << Jacobian.transpose() << std::endl;
#else  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
  Jacobian = x_err_;
#endif  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
}

#ifdef TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
void TouchConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                      Eigen::Ref<Eigen::ArrayXi> idx_j) {
  for (size_t j = 0; j < kLenJacobian; j++) {
    idx_j(j) = kDof * t_start() + j;
  }
  // std::cout << idx_j.transpose() << std::endl;
}
#endif  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN

#ifdef TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
double TouchConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const {
#else  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
Eigen::Vector3d TouchConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const {
#endif  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
  const Object& control = world_.objects()->at(control_frame());
  const Object& target = world_.objects()->at(target_frame());
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());
  // std::cout << "T_control_to_target: " << T_control_to_target.translation().transpose() << std::endl;

  auto contact = ncollide3d::query::contact(Eigen::Isometry3d::Identity(), *target.collision,
                                            T_control_to_target, *control.collision, 100.0);

#ifdef TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
  return -contact->depth;
#else  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
  return contact->world2 - contact->world1;
#endif  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
}

}  // namespace LogicOpt

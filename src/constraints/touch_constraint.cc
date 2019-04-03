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
const size_t kNumConstraints = 2;
const size_t kLenJacobian = 12;
#else  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
const size_t kNumConstraints = 3;
const size_t kLenJacobian = 3;
#endif  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
const size_t kNumTimesteps = 1;

const double kH = 5e-5;

}

namespace LogicOpt {

TouchConstraint::TouchConstraint(World& world, size_t t_touch,
                                 const std::string& name_control, const std::string& name_target)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_touch, kNumTimesteps, name_control, name_target,
                      "constraint_touch_t" + std::to_string(t_touch)),
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
  constraints(0) = x_err_ - 1e-3;
  constraints(1) = -x_err_ - 1e-3;
#else  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
  constraints = 0.5 * x_err_.array().square();
#endif  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN

  Constraint::Evaluate(X, constraints);
}

void TouchConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                               Eigen::Ref<Eigen::VectorXd> Jacobian) {
#ifdef TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < kDof; i++) {
    const double x_ij = X_h(i, t_start());
    X_h(i, t_start()) += kH;
    const double x_err_hp = ComputeError(X_h);
    X_h(i, t_start()) = x_ij - kH;
    const double x_err_hn = ComputeError(X_h);
    X_h(i, t_start()) = x_ij;

    const double dx_h = (x_err_hp - x_err_hn) / (2. * kH);
    Jacobian(i) = dx_h;
    Jacobian(kDof + i) = -dx_h;
  }
#else  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
  Jacobian = x_err_;
#endif  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
}

#ifdef TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
void TouchConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                      Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // for (size_t j = 0; j < kLenJacobian; j++) {
  for (size_t j = 0; j < kDof; j++) {
    idx_j(j) = kDof * t_start() + j;
    idx_j(kDof + j) = kDof * t_start() + j;
  }
  idx_i.segment(kDof, kDof) += 1;
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

  auto contact = ncollide3d::query::contact(Eigen::Isometry3d::Identity(), *target.collision,
                                            T_control_to_target, *control.collision, 100.0);

#ifdef TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
  return -contact->depth;
#else  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
  return contact->world2 - contact->world1;
#endif  // TOUCH_CONSTRAINT_NUMERICAL_JACOBIAN
}

}  // namespace LogicOpt

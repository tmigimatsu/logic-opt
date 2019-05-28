/**
 * touch_constraint.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 20, 2019
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/touch_constraint.h"

#define TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE

namespace {

const size_t kNumConstraints = 2;
const size_t kLenJacobian = 6 * kNumConstraints;
const size_t kNumTimesteps = 1;

#ifdef TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE
const double kH = 5e-5;
#else  // TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE
const double kH = 1e-4;
#endif  // TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE

const double kEpsilon = 1e-3;
const double kMaxDist = 100.;

}

namespace logic_opt {

TouchConstraint::TouchConstraint(World3& world, size_t t_touch,
                                 const std::string& name_control, const std::string& name_target)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_touch, kNumTimesteps, name_control, name_target,
                      "constraint_t" + std::to_string(t_touch) + "_touch"),
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
  x_err_ = ComputeError(X);             // signed_dist
  constraints(0) = x_err_ - kEpsilon;   // signed_dist < epsilon
  constraints(1) = -x_err_ - kEpsilon;  // signed_dist > -epsilon

  Constraint::Evaluate(X, constraints);
}

void TouchConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                               Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < kDof; i++) {
    double& x_it = X_h(i, t_start());
    const double x_it_0 = x_it;
    x_it = x_it_0 + kH;
    const double x_err_hp = ComputeError(X_h);
#ifdef TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0 - kH;
    const double x_err_hn = ComputeError(X_h);
#endif  // TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0;

#ifdef TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    const double dx_h = (x_err_hp - x_err_hn) / (2. * kH);
#else  // TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    const double dx_h = (x_err_hp - x_err_) / kH;
#endif  // TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE

    Jacobian(i) = dx_h;
    Jacobian(kDof + i) = -dx_h;
  }
}

void TouchConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                      Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0  0  0  0  0  0  1  1  1  1  1  1
  // j: px py pz wx wy wz px py pz wx wy wz
  idx_i.segment(kDof, kDof) += 1;
  const size_t var_t = kDof * t_start();
  idx_j.head<kDof>().setLinSpaced(var_t, var_t + kDof - 1);
  idx_j.tail<kDof>().setLinSpaced(var_t, var_t + kDof - 1);
}

double TouchConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const {
  const Object3& control = world_.objects()->at(control_frame());
  const Object3& target = world_.objects()->at(target_frame());
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());

  auto contact = ncollide3d::query::contact(Eigen::Isometry3d::Identity(), *target.collision,
                                            T_control_to_target, *control.collision, kMaxDist);

  // Signed distance: negative if objects overlap
  return -contact->depth;
}

}  // namespace logic_opt

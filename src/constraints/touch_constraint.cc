/**
 * touch_constraint.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 20, 2019
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/touch_constraint.h"

#include <cmath>  // std::abs

#define TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE

namespace {

// const size_t kNumConstraints = 2;
// const size_t kLenJacobian = 6 * kNumConstraints;
// const size_t kNumTimesteps = 1;

#ifdef TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE
const double kH = 1e-2;
#else  // TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE
const double kH = 1e-4;
#endif  // TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE

const double kEpsilon = 1e-6;
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
  contact_ = ComputeError(X);             // signed_dist
  if (!contact_) {
    std::cerr << name << "::Evaluate(): No contact!" << std::endl;
    std::cerr << X.col(t_start()).transpose() << std::endl;
    Constraint::Evaluate(X, constraints);
    return;
  }

  constraints(0) = -0.5 * std::abs(contact_->depth) * contact_->depth;   // signed_dist < epsilon
  // constraints(0) = contact_ ? -contact_->depth : 0.;// - kEpsilon;   // signed_dist < epsilon

  Constraint::Evaluate(X, constraints);
}

void TouchConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                               Eigen::Ref<Eigen::VectorXd> Jacobian) {
  if (!contact_) {
    Constraint::Jacobian(X, Jacobian);
    return;
  }
  // Jacobian.head<3>() = contact_->normal;
  Jacobian.head<3>() = std::abs(contact_->depth) * contact_->normal;
  const double x_err = -contact_->depth;

  Eigen::MatrixXd X_h = X;
  for (size_t i = 3; i < kDof; i++) {
    double& x_it = X_h(i, t_start());
    const double x_it_0 = x_it;
    x_it = x_it_0 + kH;
    const auto contact_hp = ComputeError(X_h);
    const double x_err_hp = contact_hp ? -contact_hp->depth : 0.;
#ifdef TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0 - kH;
    const auto contact_hn = ComputeError(X_h);
    const double x_err_hn = contact_hn ? -contact_hn->depth : 0.;
#endif  // TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0;

#ifdef TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    const double dx_h = 0.5 * (std::abs(x_err_hp) * x_err_hp - std::abs(x_err_hn) * x_err_hn) / (2. * kH);
    // const double dx_h = (x_err_hp - x_err_hn) / (2. * kH);
#else  // TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    const double dx_h = 0.5 * (x_err_hp * x_err_hp - x_err_ * x_err_) / kH;
#endif  // TOUCH_CONSTRAINT_SYMMETRIC_DIFFERENCE

    if (std::abs(dx_h) > 0.5) {
      // If the contact distance is small, ncollide will clip it to 0, which
      // will make either x_err_hp or x_err_hn 0, and dx_h will become huge. Get
      // around this by leaving the Jacobian element 0.

      std::cerr << "TouchConstraint::Jacobian(): Ill-conditioned J(" << i << ","
                << t_start() << "): " << dx_h << " " << contact_hn.has_value() << " " << x_err_hp << " " << x_err_hn << " " << x_err << std::endl;
      continue;
    }

    Jacobian(i) = dx_h;
    // if (i < 3 && (x_err_hp == 0 || x_err_hn == 0)) {
    //   Jacobian(kDof+0) = x_err_hp;
    //   Jacobian(kDof+1) = x_err_;
    //   Jacobian(kDof+2) = x_err_hn;
    // }
    // if (i == 2 && std::abs(Jacobian.head<3>().norm() - 1.) > 0.1) {
    //   Jacobian(kDof+3) = x_err_hp;
    //   Jacobian(kDof+4) = x_err_;
    //   Jacobian(kDof+5) = x_err_hn;
    // }
    // Jacobian(kDof + i) = -dx_h;
  }
  Constraint::Jacobian(X, Jacobian);
}

void TouchConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                      Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0  0  0  0  0  0  1  1  1  1  1  1
  // j: px py pz wx wy wz px py pz wx wy wz
  // idx_i.tail<kDof>() += 1;
  const size_t var_t = kDof * t_start();
  idx_j.head<kDof>().setLinSpaced(var_t, var_t + kDof - 1);
  // idx_j.tail<kDof>().setLinSpaced(var_t, var_t + kDof - 1);
}

std::optional<ncollide3d::query::Contact>
TouchConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const {
  const Object3& control = world_.objects()->at(control_frame());
  const Object3& target = world_.objects()->at(target_frame());
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());

  // Signed distance: positive if objects overlap
  return ncollide3d::query::contact(Eigen::Isometry3d::Identity(), *target.collision,
                                    T_control_to_target, *control.collision, kMaxDist);
}

}  // namespace logic_opt

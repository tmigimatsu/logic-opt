/**
 * pick_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/pick_constraint.h"

// #define PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE

namespace {

const size_t kNumConstraints = 1;
const size_t kLenJacobian = 3;
const size_t kNumTimesteps = 1;

#ifdef PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE
const double kH = 5e-5;
#else  // PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE
const double kH = 1e-4;
#endif  // PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE

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
  world.ReserveTimesteps(t_pick + kNumTimesteps);
  world.AttachFrame(name_ee, name_object, t_pick);
}

void PickConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> constraints) {
  x_err_ = ComputeError(X);
  constraints(0) = x_err_;

  Constraint::Evaluate(X, constraints);
}

void PickConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < 3; i++) {
    double& x_it = X_h(i, t_start());
    const double x_it_0 = x_it;
    x_it = x_it_0 + kH;
    const double x_err_hp = ComputeError(X_h);
#ifdef PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0 - kH;
    const double x_err_hn = ComputeError(X_h);
#endif  // PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0;

#ifdef PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE
    Jacobian(i) = (x_err_hp - x_err_hn) / (2. * kH);
#else  // PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE
    Jacobian(i) = (x_err_hp - x_err_) / kH;
#endif  // PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE
  }
}

void PickConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                     Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0  0  0
  // j: px py pz

  for (size_t j = 0; j < 3; j++) {
    idx_j(j) = kDof * t_start() + j;
  }
}

double PickConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const {
  const Object& ee = world_.objects()->at(control_frame());
  const Object& object = world_.objects()->at(target_frame());
  const Eigen::Isometry3d T_ee_to_object = world_.T_control_to_target(X, t_start());
  const auto& x_ee = T_ee_to_object.translation();

  const auto projection = object.collision->project_point(Eigen::Isometry3d::Identity(), x_ee, false);
  const double sign = projection.is_inside ? -1. : 1.;
  return sign * (x_ee - projection.point).norm();
}

}  // namespace LogicOpt

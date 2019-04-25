/**
 * pick_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/pick_constraint.h"

#define PICK_CONSTRAINT_NUMERIC_JACOBIAN
#define PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE

namespace {

const size_t kNumConstraints = 1;
const size_t kLenJacobian = 3;
const size_t kNumTimesteps = 1;

#ifdef PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE
const double kH = 5e-3;
#else  // PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE
const double kH = 1e-4;
#endif  // PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE

}  // namespace

namespace logic_opt {

PickConstraint::PickConstraint(World3& world, size_t t_pick, const std::string& name_ee,
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

  Eigen::VectorXd constraints(kNumConstraints);
  Evaluate(Eigen::MatrixXd::Zero(kDof, world.num_timesteps()), constraints);
}

void PickConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> constraints) {
#ifdef PICK_CONSTRAINT_NUMERIC_JACOBIAN
  // x_err_ = ComputeError(X);
// #else  // PICK_CONSTRAINT_NUMERIC_JACOBIAN
  const Object3& ee = world_.objects()->at(control_frame());
  const Object3& object = world_.objects()->at(target_frame());
  const Eigen::Isometry3d T_ee_to_object = world_.T_control_to_target(X, t_start());
  const auto& x_ee = T_ee_to_object.translation();

  const auto projection = object.collision->project_point(Eigen::Isometry3d::Identity(), x_ee, false);
  x_err_ =  (projection.point - x_ee).norm();
  x_ee_ = x_ee;
  dx_err_ = (projection.point - x_ee).normalized();
#endif  // PICK_CONSTRAINT_NUMERIC_JACOBIAN
  constraints(0) = x_err_;

  Constraint::Evaluate(X, constraints);
}

void PickConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> Jacobian) {
// #ifdef PICK_CONSTRAINT_NUMERIC_JACOBIAN
//   const Object& ee = world_.objects()->at(control_frame());
//   const Object3& object = world_.objects()->at(target_frame());
//   const Eigen::Isometry3d T_ee_to_object = world_.T_control_to_target(X, t_start());
//   const auto& x_ee = T_ee_to_object.translation();
//   Eigen::MatrixXd X_h = X;
//   for (size_t i = 0; i < 3; i++) {
//     double& x_it = X_h(i, t_start());
//     const double x_it_0 = x_it;
//     x_it = x_it_0 + kH;
//     const double x_err_hp = ComputeError(X_h);
// #ifdef PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE
//     x_it = x_it_0 - kH;
//     const double x_err_hn = ComputeError(X_h);
// #endif  // PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE
//     x_it = x_it_0;

// #ifdef PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE
//     Jacobian(i) = (x_err_hp - x_err_hn) / (2. * kH);
// #else  // PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE
//     Jacobian(i) = (x_err_hp - x_err_) / kH;
// #endif  // PICK_CONSTRAINT_SYMMETRIC_DIFFERENCE
//   }
// #else  // PICK_CONSTRAINT_NUMERIC_JACOBIAN
  ncollide3d::query::Ray ray(x_ee_, dx_err_);
  const Object3& object = world_.objects()->at(target_frame());
  const auto intersection = object.collision->toi_and_normal_with_ray(Eigen::Isometry3d::Identity(),
                                                                      ray, false);

  Jacobian = dx_err_.dot(intersection->normal) > 0. ? -intersection->normal : intersection->normal;
// #endif  // PICK_CONSTRAINT_NUMERIC_JACOBIAN
}

void PickConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                     Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0  0  0
  // j: px py pz
  const size_t var_t = kDof * t_start();
  idx_j.setLinSpaced(3, var_t, var_t + 2);
}

double PickConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const {
  const Object3& ee = world_.objects()->at(control_frame());
  const Object3& object = world_.objects()->at(target_frame());
  const Eigen::Isometry3d T_ee_to_object = world_.T_control_to_target(X, t_start());
  const auto& x_ee = T_ee_to_object.translation();

  const auto projection = object.collision->project_point(Eigen::Isometry3d::Identity(), x_ee, false);
  const double sign = projection.is_inside ? -1. : 1.;
  return 0.5 * sign * (x_ee - projection.point).squaredNorm();
}

}  // namespace logic_opt

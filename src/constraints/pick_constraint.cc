/**
 * pick_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/pick_constraint.h"

namespace {

const double kMaxToi = 100.;

}  // namespace

namespace logic_opt {

constexpr size_t PickConstraint::kDof;
constexpr size_t PickConstraint::kNumConstraints;
constexpr size_t PickConstraint::kLenJacobian;
constexpr size_t PickConstraint::kNumTimesteps;

PickConstraint::PickConstraint(World3& world, size_t t_pick, const std::string& name_ee,
                               const std::string& name_object)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_pick, kNumTimesteps, name_ee, name_object,
                      "constraint_t" + std::to_string(t_pick) + "_pick"),
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
  world.set_controller("pick", t_pick);

  Eigen::VectorXd constraints(kNumConstraints);
  Evaluate(Eigen::MatrixXd::Zero(kDof, world.num_timesteps()), constraints);
}

void PickConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> constraints) {
  const Object3& ee = world_.objects()->at(control_frame());
  const Object3& object = world_.objects()->at(target_frame());
  const Eigen::Isometry3d T_ee_to_object = world_.T_control_to_target(X, t_start());
  const auto& x_ee = T_ee_to_object.translation();

  // Project ee onto object
  const auto projection = object.collision->project_point(Eigen::Isometry3d::Identity(), x_ee, false);
  const double sign = projection.is_inside ? -1 : 1;

  // Linearize object surface at projection point
  dx_err_ = projection.point - x_ee;
  x_err_ = sign * dx_err_.norm();
  x_ee_ = x_ee;

  // constraints(0) = x_err_;
  constraints(0) = 0.5 * sign * x_err_ * x_err_;

  Constraint::Evaluate(X, constraints);
}

void PickConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> Jacobian) {
  if (dx_err_.norm() > std::numeric_limits<double>::epsilon()) {
    dx_err_.normalize();
  }
  ncollide3d::query::Ray ray(x_ee_, dx_err_);
  const Object3& object = world_.objects()->at(target_frame());
  const auto intersection = object.collision->toi_and_normal_with_ray(Eigen::Isometry3d::Identity(),
                                                                      ray, kMaxToi, false);
  if (!intersection) {
    throw std::runtime_error("PickConstraint::Jacobian(): Intersection not found!");
  }

  // Normal of object surface pointing away from object interior
  // Jacobian = x_err_ * dx_err_.dot(intersection->normal) > 0. ? -intersection->normal
  //                                                            : intersection->normal;
  Jacobian = dx_err_.dot(intersection->normal) > 0. ? -x_err_ * intersection->normal
                                                    : x_err_ * intersection->normal;

  Constraint::Jacobian(X, Jacobian);
}

void PickConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                     Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0  0  0
  // j: px py pz
  const size_t var_t = kDof * t_start();
  idx_j.setLinSpaced(3, var_t, var_t + 2);
}

}  // namespace logic_opt

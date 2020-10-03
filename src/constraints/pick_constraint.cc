/**
 * pick_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/pick_constraint.h"

#include <spatial_opt/constraints/quaternion_norm_constraint.h>

namespace {

const double kMaxToi = 1000.;

std::vector<std::unique_ptr<spatial_opt::Constraint>> InitializeConstraints(
    logic_opt::World& world, size_t t_pick, const std::string& name_ee,
    const std::string& name_object) {
  using namespace logic_opt;

  if (name_ee == world.kWorldFrame) {
    throw std::invalid_argument("PickConstraint::PickConstraint(): " +
                                world.kWorldFrame + " cannot be the ee frame.");
  } else if (name_ee == world.kWorldFrame) {
    throw std::invalid_argument(
        "PickConstraint::PickConstraint(): " + world.kWorldFrame +
        " cannot be the object frame.");
  }

  std::vector<std::unique_ptr<spatial_opt::Constraint>> constraints;

  // Constrain signed distance
  constraints.emplace_back(new PickConstraint::SignedDistanceConstraint(
      world, t_pick, name_ee, name_object));

  // Constrain quaternion norm to be unit norm
  constraints.emplace_back(new spatial_opt::QuaternionNormConstraint(t_pick));

  return constraints;
}

}  // namespace

namespace logic_opt {

PickConstraint::PickConstraint(World& world, size_t t_pick,
                               const std::string& name_ee,
                               const std::string& name_object)
    : spatial_opt::MultiConstraint(
          InitializeConstraints(world, t_pick, name_ee, name_object),
          "constraint_t" + std::to_string(t_pick) + "_pick") {
  world.set_controller("pick", t_pick);
}

const size_t PickConstraint::SignedDistanceConstraint::kNumConstraints;

PickConstraint::SignedDistanceConstraint::SignedDistanceConstraint(
    World& world, size_t t_pick, const std::string& name_ee,
    const std::string& name_object)
    : FrameConstraint(
          kNumConstraints, kLenJacobian, t_pick, kNumTimesteps, name_ee,
          name_object,
          "constraint_t" + std::to_string(t_pick) + "_signed_distance_pick"),
      world_(world) {
  world.ReserveTimesteps(t_pick + kNumTimesteps);
  world.AttachFrame(name_ee, name_object, t_pick);

  Eigen::VectorXd constraints(kNumConstraints);
  PickConstraint::SignedDistanceConstraint::Evaluate(
      Eigen::MatrixXd::Zero(kDof, world.num_timesteps()), constraints);
}

void PickConstraint::SignedDistanceConstraint::Evaluate(
    Eigen::Ref<const Eigen::MatrixXd> X,
    Eigen::Ref<Eigen::VectorXd> constraints) {
  const Object& ee = world_.objects()->at(control_frame());
  const Object& object = world_.objects()->at(target_frame());
  const spatial_opt::Isometry T_ee_to_object =
      world_.T_control_to_target(X, t_start());
  // const Eigen::Isometry3d T_ee_to_object =
  //     world_.T_control_to_target(X, t_start());
  const auto& x_ee = T_ee_to_object.translation();

  // Project ee onto object
  const auto projection = object.collision->project_point(
      Eigen::Isometry3d::Identity(), x_ee, false);
  const double sign = projection.is_inside ? -1 : 1;

  // Linearize object surface at projection point
  dx_err_ = projection.point - x_ee;
  x_err_ = sign * dx_err_.norm();
  x_ee_ = x_ee;

  // constraints(0) = x_err_;
  constraints(0) = 0.5 * sign * x_err_ * x_err_;

  Constraint::Evaluate(X, constraints);
}

void PickConstraint::SignedDistanceConstraint::Jacobian(
    Eigen::Ref<const Eigen::MatrixXd> X, Eigen::Ref<Eigen::VectorXd> Jacobian) {
  if (dx_err_.norm() > std::numeric_limits<double>::epsilon()) {
    dx_err_.normalize();
  }
  ncollide3d::query::Ray ray(x_ee_, dx_err_);
  const Object& object = world_.objects()->at(target_frame());
  const auto intersection = object.collision->toi_and_normal_with_ray(
      Eigen::Isometry3d::Identity(), ray, false);
  if (!intersection) {
    throw std::runtime_error(
        "PickConstraint::Jacobian(): Intersection not found!");
  }

  // Normal of object surface pointing away from object interior
  // Jacobian = x_err_ * dx_err_.dot(intersection->normal) > 0. ?
  // -intersection->normal
  //                                                            :
  //                                                            intersection->normal;
  Jacobian = dx_err_.dot(intersection->normal) > 0.
                 ? -x_err_ * intersection->normal
                 : x_err_ * intersection->normal;

  Constraint::Jacobian(X, Jacobian);
}

void PickConstraint::SignedDistanceConstraint::JacobianIndices(
    Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0  0  0
  // j: px py pz
  const size_t var_t = kDof * t_start();

  idx_j.setLinSpaced(3, var_t, var_t + 2);
}

}  // namespace logic_opt

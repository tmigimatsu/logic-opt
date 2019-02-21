/**
 * place_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/place_constraint.h"

#include <algorithm>  // std::max

#include <ncollide/ncollide2d.h>

#include "LogicOpt/constraints/touch_constraint.h"

namespace {

const size_t kNumTimesteps = 1;

std::vector<std::unique_ptr<LogicOpt::Constraint>>
InitializeConstraints(LogicOpt::World& world, size_t t_place,
                      const std::string& name_object, const std::string& name_target) {
  using namespace LogicOpt;

  std::vector<std::unique_ptr<Constraint>> constraints;
  constraints.emplace_back(new TouchConstraint(world, t_place, name_object, name_target));
  constraints.emplace_back(new PlaceConstraint::SupportAreaConstraint(world, t_place,
                                                                      name_object, name_target));
  return constraints;
}

}  // namespace

namespace LogicOpt {

PlaceConstraint::PlaceConstraint(World& world, size_t t_place,
                                 const std::string& name_object, const std::string& name_target)
    : MultiConstraint(InitializeConstraints(world, t_place, name_object, name_target),
                                            "constraint_place_t" + std::to_string(t_place)) {

  if (name_object == world.kWorldFrame) {
    throw std::invalid_argument("PlaceConstraint::PlaceConstraint(): " + world.kWorldFrame +
                                " cannot be the object frame.");
  } else if (name_target == world.kWorldFrame) {
    throw std::invalid_argument("PlaceConstraint::PlaceConstraint(): " + world.kWorldFrame +
                                " cannot be the target frame.");
  }
}

PlaceConstraint::SupportAreaConstraint::SupportAreaConstraint(World& world, size_t t_place,
                                                              const std::string& name_control,
                                                              const std::string& name_target)
    : FrameConstraint(3, 3, t_place, 1, name_control, name_target,
                      "constraint_place_support_area_t" + std::to_string(t_place)),
      world_(world) {
  const Object& control = world_.objects()->at(control_frame());
  const Object& target = world_.objects()->at(target_frame());
  z_surface_ = target.collision->aabb(Eigen::Isometry3d::Identity()).maxs()(2);
  Eigen::Vector3d half_extents = control.collision->aabb(Eigen::Isometry3d::Identity()).maxs();
  r_object_ = std::max(half_extents(0), half_extents(1));
}

void PlaceConstraint::SupportAreaConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                                      Eigen::Ref<Eigen::VectorXd> constraints) {
  ComputeError(X);
  constraints.head<2>() = 0.5 * xy_err_.array().square();
  constraints(2) = z_err_;

  Constraint::Evaluate(X, constraints);
}

void PlaceConstraint::SupportAreaConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                                      Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Jacobian.head<2>() = xy_err_;
  Jacobian(2) = -1.;
}

void PlaceConstraint::SupportAreaConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) {
  const Object& control = world_.objects()->at(control_frame());
  const Object& target = world_.objects()->at(target_frame());
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());
  Eigen::Vector2d com_control = (T_control_to_target * control.inertia().com).head<2>();

  std::shared_ptr<ncollide2d::shape::Shape> target_2d = target.collision->project_2d();
  auto projection = target_2d->project_point(Eigen::Isometry2d::Identity(), com_control, true);
  // double sign = projection.is_inside ? 1. : -1.;
  // Eigen::Vector2d normal = sign * (com_control - projection.point).normalized();

  xy_err_ = com_control - projection.point;//(projection.point + r_object_ * normal);
  z_err_ = z_surface_ - T_control_to_target.translation()(2);
}

Constraint::Type PlaceConstraint::SupportAreaConstraint::constraint_type(size_t idx_constraint) const {
  return idx_constraint == 2 ? Constraint::Type::kInequality : Constraint::Type::kEquality;
}

}  // namespace LogicOpt

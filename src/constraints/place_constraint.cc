/**
 * place_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/place_constraint.h"

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
                      name_object, name_target, "constraint_place_t" + std::to_string(t_place)) {
  world.AttachFrame(name_object, name_target, t_place);
}

PlaceConstraint::SupportAreaConstraint::SupportAreaConstraint(World& world, size_t t_place,
                                                              const std::string& name_control,
                                                              const std::string& name_target)
    : FrameConstraint(3, 3, t_place, 1, name_control, name_target,
                      "support_area_t" + std::to_string(t_place)),
      world_(world) {
  world.AttachFrame(name_control, name_target, t_place);
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

  xy_err_ = com_control - projection.point;
  z_err_ = -T_control_to_target.translation()(2);
}

Constraint::Type PlaceConstraint::SupportAreaConstraint::constraint_type(size_t idx_constraint) const {
  return idx_constraint == 2 ? Constraint::Type::INEQUALITY : Constraint::Type::EQUALITY;
}

}  // namespace LogicOpt

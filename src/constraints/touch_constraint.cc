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

const size_t kNumConstraints = 3;
const size_t kLenJacobian = kNumConstraints;
const size_t kNumTimesteps = 1;

}

namespace LogicOpt {

TouchConstraint::TouchConstraint(World& world, size_t t_place,
                                 const std::string& name_object, const std::string& name_target)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_place, kNumTimesteps, name_object, name_target,
                      "constraint_place_t" + std::to_string(t_place)),
      world_(world) {
  world.AttachFrame(name_object, name_target, t_place);
}

void TouchConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                               Eigen::Ref<Eigen::VectorXd> constraints) {
  x_err_ = ComputeError(X);
  constraints = 0.5 * x_err_.array().square();

  Constraint::Evaluate(X, constraints);
}

void TouchConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                               Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Jacobian = x_err_;
}

Eigen::Vector3d TouchConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const {
  const Object& control = world_.objects()->at(control_frame());
  const Object& target = world_.objects()->at(target_frame());
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());

  auto contact = ncollide3d::query::contact(Eigen::Isometry3d::Identity(), *target.collision,
                                            T_control_to_target, *control.collision, 100.0);

  return contact->world2 - contact->world1;
}

}  // namespace LogicOpt

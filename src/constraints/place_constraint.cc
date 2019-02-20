/**
 * place_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/place_constraint.h"

#include <ctrl_utils/string.h>
#include <ncollide/ncollide2d.h>

namespace {

const size_t kNumConstraints = 6;
const size_t kDof = 6;
const size_t kLenJacobian = kNumConstraints;
const size_t kNumTimesteps = 1;

}

namespace LogicOpt {

PlaceConstraint::PlaceConstraint(World& world, size_t t_place,
                                 const std::string& name_object, const std::string& name_target)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_place, kNumTimesteps, name_object, name_target,
                      "constraint_place_t" + std::to_string(t_place)),
      world_(world) {
  world.AttachFrame(name_object, name_target, t_place);
}

void PlaceConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                               Eigen::Ref<Eigen::VectorXd> constraints) {
  dx_err_ = ComputeError(X);
  constraints.head<5>() = 0.5 * dx_err_.head<5>().array().square();
  constraints(5) = dx_err_(5);
  // constraints.tail<4>().array() *= dx_err_.tail<4>().array().sign();
  Constraint::Evaluate(X, constraints);
}

void PlaceConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                               Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Jacobian = dx_err_;
  Jacobian(5) = -1.;
}

Eigen::Matrix<double,kNumConstraints,1> PlaceConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const {
  const Object& control = world_.objects()->at(control_frame());
  const Object& target = world_.objects()->at(target_frame());
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());
  Eigen::Vector2d com_control = (T_control_to_target * control.inertia().com).head<2>();

  std::shared_ptr<ncollide2d::shape::Shape> target_2d = target.collision->project_2d();
  auto projection = target_2d->project_point(Eigen::Isometry2d::Identity(), com_control, true);

  auto contact = ncollide3d::query::contact(Eigen::Isometry3d::Identity(), *target.collision,
                                            T_control_to_target, *control.collision, 100.0);

  Eigen::Matrix<double,kNumConstraints,1> dx_err;
  dx_err.head<3>() = contact->world2 - contact->world1;
  dx_err.segment<2>(3) = com_control - projection.point;
  dx_err(5) = -T_control_to_target.translation()(2);

  return dx_err;
}

void PlaceConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) {
  idx_i += Eigen::VectorXi::LinSpaced(num_constraints_, 0, num_constraints_ - 1).array();
  idx_j.fill(kDof * t_start_);
  idx_j(0) += 0;  // x
  idx_j(1) += 1;  // y
  idx_j(2) += 2;  // z
  idx_j(3) += 0;  // x
  idx_j(4) += 1;  // y
  idx_j(5) += 2;  // z
}

Constraint::Type PlaceConstraint::constraint_type(size_t idx_constraint) const {
  return idx_constraint < 5 ? Constraint::Type::EQUALITY : Constraint::Type::INEQUALITY;
}

}  // namespace LogicOpt

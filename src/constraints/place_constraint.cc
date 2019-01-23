/**
 * place_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/place_constraint.h"

namespace LogicOpt {

PlaceConstraint::PlaceConstraint(World& world, size_t t_place,
                                 const std::string& name_object, const std::string& name_target)
    : FrameConstraint(7, 7, t_place, 1, name_object, name_target,
                      "constraint_place_t" + std::to_string(t_place)) {

  if (world.objects()->at(name_object).graphics.empty() ||
      world.objects()->at(name_target).graphics.empty()) {
    throw std::runtime_error("PlaceConstraint(): object and target must have graphics.");
  }

  const spatial_dyn::Graphics::Geometry& geom_object = world.objects()->at(name_object).graphics.front().geometry;
  const spatial_dyn::Graphics::Geometry& geom_target = world.objects()->at(name_target).graphics.front().geometry;
  double z_offset = 0.;
  switch (geom_object.type) {
    case spatial_dyn::Graphics::Geometry::Type::BOX:
      z_offset += 0.5 * geom_object.scale(2);
      x_limits_ << 0.5 * geom_object.scale(0), -0.5 * geom_object.scale(0);
      y_limits_ << 0.5 * geom_object.scale(1), -0.5 * geom_object.scale(1);
      break;
    default:
      throw std::runtime_error("PlaceConstraint(): " + std::string(geom_object) + " not implemented yet.");
  }
  switch (geom_target.type) {
    case spatial_dyn::Graphics::Geometry::Type::BOX:
      z_offset += 0.5 * geom_target.scale(2);
      x_limits_(0) -= 0.5 * geom_target.scale(0);
      x_limits_(1) += 0.5 * geom_target.scale(0);
      y_limits_(0) -= 0.5 * geom_target.scale(1);
      y_limits_(1) += 0.5 * geom_target.scale(1);
      break;
    default:
      throw std::runtime_error("PlaceConstraint(): " + std::string(geom_target) + " not implemented yet.");
  }
  dx_des_(0) += z_offset;

  world.AttachFrame(name_object, name_target, t_place);
}

void PlaceConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                               Eigen::Ref<Eigen::VectorXd> constraints) {
  ComputeError(X);
  constraints = 0.5 * dx_err_.array().square();
  constraints.tail<4>().array() *= dx_err_.tail<4>().array().sign();
  Constraint::Evaluate(X, constraints);
}

void PlaceConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                               Eigen::Ref<Eigen::VectorXd> Jacobian) {
  ComputeError(X);
  Jacobian = dx_err_;
  Jacobian.tail<4>().array() *= dx_err_.tail<4>().array().sign();
  Jacobian(3) *= -1.;
  Jacobian(5) *= -1.;
}

void PlaceConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) {
  const auto& x_t = X.col(t_start_);
  dx_err_(0) = x_t(2);
  dx_err_.segment<2>(1) = x_t.segment<2>(3);
  dx_err_.head<3>() -= dx_des_;

  dx_err_(3) = x_limits_(0) - x_t(0);
  dx_err_(4) = x_t(0) - x_limits_(1);
  dx_err_(5) = y_limits_(0) - x_t(1);
  dx_err_(6) = x_t(1) - y_limits_(1);
}

void PlaceConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) {
  idx_i += Eigen::VectorXi::LinSpaced(num_constraints_, 0, num_constraints_ - 1).array();
  idx_j.fill(6 * t_start_);
  idx_j(0) += 2;  // z
  idx_j(1) += 3;  // wx
  idx_j(2) += 4;  // wy
  idx_j(3) += 0;  // x
  idx_j(4) += 0;  // x
  idx_j(5) += 1;  // y
  idx_j(6) += 1;  // y
}

Constraint::Type PlaceConstraint::constraint_type(size_t idx_constraint) const {
  return idx_constraint < 3 ? Constraint::Type::EQUALITY : Constraint::Type::INEQUALITY;
}

}  // namespace LogicOpt

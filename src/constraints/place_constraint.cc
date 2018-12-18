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
    : FrameConstraint(3, 3, t_place, 1, name_object, name_target,
                      "constraint_place_t" + std::to_string(t_place)) {

  const SpatialDyn::Graphics::Geometry& geom_object = world.objects()->at(name_object).graphics.geometry;
  const SpatialDyn::Graphics::Geometry& geom_target = world.objects()->at(name_target).graphics.geometry;
  double z_offset = 0.;
  switch (geom_object.type) {
    case SpatialDyn::Graphics::Geometry::Type::BOX:
      z_offset += 0.5 * geom_object.scale(2);
      break;
    default:
      throw std::runtime_error("PlaceConstraint(): " + std::string(geom_object) + " not implemented yet.");
  }
  switch (geom_target.type) {
    case SpatialDyn::Graphics::Geometry::Type::BOX:
      z_offset += 0.5 * geom_target.scale(2);
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
  Constraint::Evaluate(X, constraints);
}

void PlaceConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                               Eigen::Ref<Eigen::VectorXd> Jacobian) {
  ComputeError(X);
  Jacobian = dx_err_;
}

void PlaceConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) {
  dx_err_ << X.col(t_start_)(2), X.col(t_start_).segment<2>(3);
  dx_err_ -= dx_des_;
}

}  // namespace LogicOpt

/**
 * surface_contact_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/surface_contact_constraint.h"

namespace LogicOpt {

int SurfaceContactConstraint::Axis(Direction direction) {
  switch (direction) {
    case Direction::POS_X:
    case Direction::NEG_X:
      return 0;
    case Direction::POS_Y:
    case Direction::NEG_Y:
      return 1;
    case Direction::POS_Z:
    case Direction::NEG_Z:
      return 2;
  }
}

int SurfaceContactConstraint::SignAxis(Direction direction) {
  switch (direction) {
    case Direction::POS_X:
    case Direction::POS_Y:
    case Direction::POS_Z:
      return 1;
    case Direction::NEG_X:
    case Direction::NEG_Y:
    case Direction::NEG_Z:
      return -1;
  }
}

std::array<int, 2> SurfaceContactConstraint::OrthogonalAxes(Direction direction) {
  switch (direction) {
    case Direction::POS_X:
    case Direction::NEG_X:
      return {1, 2};
    case Direction::POS_Y:
    case Direction::NEG_Y:
      return {0, 2};
    case Direction::POS_Z:
    case Direction::NEG_Z:
      return {0, 1};
  }
}

SurfaceContactConstraint::SurfaceContactConstraint(World& world, size_t t_contact,
                                                   const std::string& name_object,
                                                   const std::string& name_surface,
                                                   Direction direction_surface)
    : Constraint(6, 6 * world.ab.dof(), t_contact, 1,
                 "constraint_surfacecontact_t" + std::to_string(t_contact)),
      PlaceConstraint(world, t_contact, name_object, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()),
      name_surface_(name_surface), axis_normal_(Axis(direction_surface)), sign_normal_(SignAxis(direction_surface)),
      axes_surface_(OrthogonalAxes(direction_surface)) {}

Constraint::Type SurfaceContactConstraint::constraint_type(size_t idx_constraint) const {
  if (idx_constraint > num_constraints_) {
    throw std::out_of_range("Constraint::constraint_type(): Constraint index out of range.");
  }
  return (idx_constraint < 4) ? Type::INEQUALITY : Type::EQUALITY;
}

void SurfaceContactConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                                        Eigen::Ref<Eigen::VectorXd> constraints) {
  ComputeError(Q);

  constraints.head<4>() = 0.5 * surface_err_.array().square() *
                          (surface_err_.array() < 0).select(-Eigen::Array4d::Ones(), Eigen::Array4d::Ones());
  constraints(4) = 0.5 * x_quat_err_(axis_normal_) * x_quat_err_(axis_normal_);
  constraints(5) = 0.5 * x_quat_err_.tail<3>().squaredNorm();

  Constraint::Evaluate(Q, constraints);
}

void SurfaceContactConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                        Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Eigen::Map<Eigen::MatrixXd> J(&Jacobian(0), num_constraints_, ab_.dof());

  ComputeError(Q);
  const Eigen::Matrix6Xd& J_x = spatial_dyn::Jacobian(ab_);

  J.row(0) = J_x.row(axes_surface_[0]) * surface_err_(0) * (surface_err_(0) < 0. ? -1. : 1.);
  J.row(1) = J_x.row(axes_surface_[0]) * surface_err_(1) * (surface_err_(1) > 0. ? -1. : 1.);
  J.row(2) = J_x.row(axes_surface_[1]) * surface_err_(2) * (surface_err_(2) < 0. ? -1. : 1.);
  J.row(3) = J_x.row(axes_surface_[1]) * surface_err_(3) * (surface_err_(3) > 0. ? -1. : 1.);
  J.row(4) = J_x.row(axis_normal_) * x_quat_err_(axis_normal_);
  J.row(5) = x_quat_err_.tail<3>().transpose() * J_x.bottomRows<3>();
}

void SurfaceContactConstraint::ComputePlacePose(Eigen::Ref<const Eigen::MatrixXd> Q) {
  world_.Simulate(Q);

  const spatial_dyn::RigidBody& rb_object = world_.objects.at(name_object_);
  const spatial_dyn::RigidBody& rb_place = world_.objects.at(name_surface_);
  const World::ObjectState& state_place = world_.object_state(name_surface_, t_start_);

  x_des_place_ = state_place.pos;
  quat_des_place_ = state_place.quat;

  if (rb_place.graphics.geometry.type == spatial_dyn::Graphics::Geometry::Type::BOX) {
    world_.Simulate(Q); // TODO: Move to Ipopt
    const World::ObjectState& object_state_prev = world_.object_state(name_object_, t_start_ - 1);
    Eigen::Isometry3d T_object_to_world = Eigen::Translation3d(object_state_prev.pos) * object_state_prev.quat;
    T_ee_to_object_ = T_object_to_world.inverse() * spatial_dyn::CartesianPose(world_.ab, Q.col(t_start_ - 1));
    const auto& p_ee = T_ee_to_object_.translation();
    surface_des_(0) = state_place.pos(axes_surface_[0]) +
                      0.5 * rb_place.graphics.geometry.scale(axes_surface_[0]) + p_ee(axes_surface_[0]);
    surface_des_(1) = state_place.pos(axes_surface_[0]) -
                      0.5 * rb_place.graphics.geometry.scale(axes_surface_[0]) + p_ee(axes_surface_[0]);
    surface_des_(2) = state_place.pos(axes_surface_[1]) +
                      0.5 * rb_place.graphics.geometry.scale(axes_surface_[1]) + p_ee(axes_surface_[1]),
    surface_des_(3) = state_place.pos(axes_surface_[1]) -
                      0.5 * rb_place.graphics.geometry.scale(axes_surface_[1]) + p_ee(axes_surface_[1]);
    x_des_place_(axis_normal_) += sign_normal_ * 0.5 * rb_place.graphics.geometry.scale(axis_normal_);
  } else {
    throw std::runtime_error("SurfaceContactConstraint::ComputePlacePose(): Geometry type not implemented!");
  }
  if (rb_object.graphics.geometry.type == spatial_dyn::Graphics::Geometry::Type::BOX) {
    x_des_place_(axis_normal_) += sign_normal_ * 0.5 * rb_object.graphics.geometry.scale(axis_normal_);
  }

  PlaceConstraint::ComputePlacePose(Q);
}

void SurfaceContactConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> Q) {
  ComputePlacePose(Q);
  CartesianPoseConstraint::ComputeError(Q);

  Eigen::Vector3d pos = spatial_dyn::Position(world_.ab, Q.col(t_start_));

  surface_err_ << pos(axes_surface_[0]) - surface_des_(0),
                  surface_des_(1) - pos(axes_surface_[0]),
                  pos(axes_surface_[1]) - surface_des_(2),
                  surface_des_(3) - pos(axes_surface_[1]);
}

}  // namespace LogicOpt

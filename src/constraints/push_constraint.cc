/**
 * push_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/push_constraint.h"

namespace LogicOpt {

PushConstraint::PushConstraint(World& world, size_t t_start, size_t num_timesteps,
                               const std::string& name_pusher, const std::string& name_pushee,
                               Direction direction_push)
    : Constraint(5 * num_timesteps + 1, 5 * world.ab.dof() * num_timesteps + world.ab.dof(), t_start,
                 num_timesteps, "constraint_push_t" + std::to_string(t_start)) {

  SurfaceContactConstraint::Direction direction_contact;
  switch (direction_push) {
    case Direction::POS_X:
      direction_contact = SurfaceContactConstraint::Direction::NEG_X;
      break;
    case Direction::NEG_X:
      direction_contact = SurfaceContactConstraint::Direction::POS_X;
      break;
    case Direction::POS_Y:
      direction_contact = SurfaceContactConstraint::Direction::NEG_Y;
      break;
    case Direction::NEG_Y:
      direction_contact = SurfaceContactConstraint::Direction::POS_Y;
      break;
  }

  constraints_.reserve(num_timesteps);
  constraints_.emplace_back(new PushSurfaceContactConstraint(world, t_start, name_pusher, name_pushee, direction_contact));
  for (size_t t = t_start + 1; t < t_start + num_timesteps; t++) {
    constraints_.emplace_back(new PushActionConstraint(world, t, name_pusher, name_pushee, direction_contact));
  }
}

void PushConstraint::RegisterSimulationStates(World& world) {
  MultiConstraint::RegisterSimulationStates(world);
  const std::string& name_object = dynamic_cast<PushSurfaceContactConstraint*>(constraints_.front().get())->name_object_;
  const std::string& name_surface = dynamic_cast<PushSurfaceContactConstraint*>(constraints_.front().get())->name_surface_;

  // Set last timestep state to FIXED to correct interpolation
  World::ObjectState& state_pusher = world.object_state(name_object, t_start_ + num_timesteps_ - 1);
  state_pusher.type = World::ObjectState::Type::FIXED;

  World::ObjectState& state_pushee = world.object_state(name_surface, t_start_ + num_timesteps_ - 1);
  state_pushee.type = World::ObjectState::Type::FIXED;
}

PushConstraint::PushSurfaceContactConstraint::PushSurfaceContactConstraint(
    World& world, size_t t_contact, const std::string& name_object,
    const std::string& name_surface, Direction direction_surface)
    : Constraint(6, 6 * world.ab.dof(), t_contact, 1,
                 "constraint_pushsurfacecontact_t" + std::to_string(t_contact)),
      SurfaceContactConstraint(world, t_contact, name_object, name_surface, direction_surface) {}

void PushConstraint::PushSurfaceContactConstraint::ComputePlacePose(Eigen::Ref<const Eigen::MatrixXd> Q) {
  SurfaceContactConstraint::ComputePlacePose(Q);

  const SpatialDyn::RigidBody& rb_surface = world_.objects.at(name_surface_);
  if (rb_surface.graphics.geometry.type == SpatialDyn::Graphics::Geometry::Type::BOX) {
    surface_des_(2) += 0.5 * rb_surface.graphics.geometry.scale(axes_surface_[1]);
    surface_des_(3) += 0.5 * rb_surface.graphics.geometry.scale(axes_surface_[1]);
  }
}

void PushConstraint::PushSurfaceContactConstraint::Simulate(World& world,
                                                            Eigen::Ref<const Eigen::MatrixXd> Q) {
  PlaceConstraint::Simulate(world, Q);

  const World::ObjectState& state_pusher = world.object_state(name_object_, t_start_);
  const World::ObjectState& state_pushee_prev = world.object_state(name_surface_, t_start_ - 1);
  World::ObjectState& state_pushee = world.object_state(name_surface_, t_start_);

  double x_pusher = state_pusher.pos(axis_normal_);
  const SpatialDyn::RigidBody& rb_pusher = world_.objects.at(name_object_);
  if (rb_pusher.graphics.geometry.type == SpatialDyn::Graphics::Geometry::Type::BOX) {
    x_pusher -= sign_normal_ * 0.5 * rb_pusher.graphics.geometry.scale(axes_surface_[1]);
  }
  const SpatialDyn::RigidBody& rb_pushee = world_.objects.at(name_surface_);
  if (rb_pushee.graphics.geometry.type == SpatialDyn::Graphics::Geometry::Type::BOX) {
    x_pusher -= sign_normal_ * 0.5 * rb_pushee.graphics.geometry.scale(axes_surface_[1]);
  }

  if (sign_normal_ > 0.) {
    state_pushee.pos(axis_normal_) = std::min(state_pushee_prev.pos(axis_normal_), x_pusher);
  } else {
    state_pushee.pos(axis_normal_) = std::max(state_pushee_prev.pos(axis_normal_), x_pusher);
  }

  for (size_t t = t_start_ + 1; t < world.T; t++) {
    World::ObjectState& object_state_t = world.object_state(name_surface_, t);
    if (object_state_t.owner != this) break;

    object_state_t.pos = state_pushee.pos;
    object_state_t.quat = state_pushee.quat;
  }
}

void PushConstraint::PushSurfaceContactConstraint::InterpolateSimulation(
    const World& world, Eigen::Ref<const Eigen::VectorXd> q,
    std::map<std::string, World::ObjectState>& object_states) const {

  // Interpolate pusher position
  PlaceConstraint::InterpolateSimulation(world, q, object_states);

  // Find distance between pusher and pushee at base timestep
  const World::ObjectState& state_pusher = world.object_state(name_object_, t_start_);
  const World::ObjectState& state_pushee = world.object_state(name_surface_, t_start_);
  double dz = state_pushee.pos(axis_normal_) - state_pusher.pos(axis_normal_);

  // Interpolate pushee position
  const World::ObjectState& state_pusher_t = object_states[name_object_];
  World::ObjectState& state_pushee_t = object_states[name_surface_];
  double z_new = state_pusher_t.pos(axis_normal_) + dz;

  // Compare to last pushee position
  const World::ObjectState& state_pushee_prev = world.object_state(name_surface_, t_start_ - 1);
  if (sign_normal_ > 0.) {
    state_pushee_t.pos(axis_normal_) = std::min(z_new, state_pushee_prev.pos(axis_normal_));
  } else {
    state_pushee_t.pos(axis_normal_) = std::max(z_new, state_pushee_prev.pos(axis_normal_));
  }
}

void PushConstraint::PushSurfaceContactConstraint::RegisterSimulationStates(World& world) {
  PlaceConstraint::RegisterSimulationStates(world);
  for (size_t t = t_start_; t < world.T; t++) {
    World::ObjectState& state_pusher = world.object_state(name_object_, t);
    state_pusher.type = (t == t_start_) ? World::ObjectState::Type::MANIPULATED :
                                         World::ObjectState::Type::FIXED;

    World::ObjectState& state_pushee = world.object_state(name_surface_, t);
    state_pushee.owner = this;
    state_pushee.type = state_pusher.type;
  }
}

PushConstraint::PushActionConstraint::PushActionConstraint(
    World& world, size_t t_contact, const std::string& name_object,
    const std::string& name_surface, Direction direction_surface)
    : Constraint(5, 5 * world.ab.dof(), t_contact, 1,
                 "constraint_pushaction_t" + std::to_string(t_contact)),
      PushSurfaceContactConstraint(world, t_contact, name_object, name_surface, direction_surface) {}

Constraint::Type PushConstraint::PushActionConstraint::constraint_type(size_t idx_constraint) const {
  if (idx_constraint >= num_constraints_) {
    throw std::out_of_range("PushActionConstraint::constraint_type(): Constraint index out of range.");
  }
  return (idx_constraint % 5 < 4) ? Type::INEQUALITY : Type::EQUALITY;
}

void PushConstraint::PushActionConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                                                    Eigen::Ref<Eigen::VectorXd> constraints) {
  ComputeError(Q);

  constraints.head<4>() = 0.5 * surface_err_.array().square() *
                          (surface_err_.array() < 0).select(-Eigen::Array4d::Ones(), Eigen::Array4d::Ones());
  constraints(4) = 0.5 * x_quat_err_.tail<3>().squaredNorm();

  Constraint::Evaluate(Q, constraints);
}

void PushConstraint::PushActionConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                                    Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Eigen::Map<Eigen::MatrixXd> J(&Jacobian(0), num_constraints_, ab_.dof());

  ComputeError(Q);
  const Eigen::Matrix6Xd& J_x = SpatialDyn::Jacobian(ab_);

  J.row(0) = J_x.row(axes_surface_[0]) * surface_err_(0) * (surface_err_(0) < 0. ? -1. : 1.);
  J.row(1) = J_x.row(axes_surface_[0]) * surface_err_(1) * (surface_err_(1) > 0. ? -1. : 1.);
  J.row(2) = J_x.row(axes_surface_[1]) * surface_err_(2) * (surface_err_(2) < 0. ? -1. : 1.);
  J.row(3) = J_x.row(axes_surface_[1]) * surface_err_(3) * (surface_err_(3) > 0. ? -1. : 1.);
  J.row(4) = x_quat_err_.tail<3>().transpose() * J_x.bottomRows<3>();
}

}  // namespace LogicOpt

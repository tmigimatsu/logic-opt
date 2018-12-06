/**
 * slide_on_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/slide_on_constraint.h"
#include "LogicOpt/constraints/place_on_constraint.h"

namespace LogicOpt {

SlideOnConstraint::SlideOnConstraint(World& world, size_t t_start, size_t num_timesteps,
                                     const std::string& name_object, const std::string& name_surface)
    : Constraint(6 * num_timesteps, 6 * world.ab.dof() * num_timesteps, t_start, num_timesteps,
                 "constraint_slideon_t" + std::to_string(t_start)) {

  constraints_.reserve(num_timesteps);
  for (size_t t = t_start; t < t_start + num_timesteps; t++) {
    constraints_.emplace_back(new PlaceOnConstraint(world, t, name_object, name_surface));
  }
}

void SlideOnConstraint::RegisterSimulationStates(World& world) {
  MultiConstraint::RegisterSimulationStates(world);
  const std::string& name_object = dynamic_cast<PlaceOnConstraint*>(constraints_.front().get())->name_object_;
  for (size_t t = t_start_; t < t_start_ + num_timesteps_ - 1; t++) {
    World::ObjectState& object_state = world.object_state(name_object, t);
    object_state.type = World::ObjectState::Type::MANIPULATED;
  }
}

Constraint::Type SlideOnConstraint::constraint_type(size_t idx_constraint) const {
  if (idx_constraint >= num_constraints_) {
    throw std::out_of_range("Constraint::constraint_type(): Constraint index out of range.");
  }
  return (idx_constraint % 6 < 4) ? Type::INEQUALITY : Type::EQUALITY;
}

}  // namespace LogicOpt

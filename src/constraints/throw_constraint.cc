/**
 * throw_constraint.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 4, 2019
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/throw_constraint.h"

#include "logic_opt/constraints/place_constraint.h"

namespace {

std::vector<std::unique_ptr<spatial_opt::Constraint>> InitializeConstraints(
    logic_opt::World& world, size_t t_throw, const std::string& name_object,
    const std::string& name_target) {
  using namespace logic_opt;

  if (name_object == world.kWorldFrame) {
    throw std::invalid_argument(
        "ThrowConstraint::ThrowConstraint(): " + world.kWorldFrame +
        " cannot be the object frame.");
  } else if (name_target == world.kWorldFrame) {
    throw std::invalid_argument(
        "ThrowConstraint::ThrowConstraint(): " + world.kWorldFrame +
        " cannot be the target frame.");
  }
  // Handled by PlaceConstraint
  // world.ReserveTimesteps(t_throw + ThrowConstraint::kNumTimesteps);
  // world.AttachFrame(name_object, name_target, t_throw);

  std::vector<std::unique_ptr<spatial_opt::Constraint>> constraints;

  constraints.emplace_back(
      new PlaceConstraint(world, t_throw, name_object, name_target));

  world.set_controller("throw", t_throw);

  return constraints;
}

}  // namespace

namespace logic_opt {

ThrowConstraint::ThrowConstraint(World& world, size_t t_place,
                                 const std::string& name_object,
                                 const std::string& name_target)
    : MultiConstraint(
          InitializeConstraints(world, t_place, name_object, name_target),
          "constraint_t" + std::to_string(t_place) + "_place") {}

}  // namespace logic_opt

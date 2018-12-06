/**
 * world.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 12, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/world.h"
#include "LogicOpt/constraints.h"

#include <algorithm>  // std::find
#include <iterator>   // std::begin, std::end
#include <limits>     // std::numeric_limits

namespace LogicOpt {

World::World(const SpatialDyn::ArticulatedBody& ab,
             const std::map<std::string, SpatialDyn::RigidBody>& objects, size_t num_timesteps)
    : ab(ab), objects(objects), T(num_timesteps),
      Q_(ab.dof(), num_timesteps) {

  Q_.fill(std::numeric_limits<double>::infinity());

  // Construct object states vector (T x num_objects)
  for (const auto& key_val : objects) {
    const std::string& name_object = key_val.first;
    object_states_[name_object] = std::vector<ObjectState>(T);
  }

  // Initialize object states at first timestep
  for (const auto& key_val : objects) {
    const std::string& name_object = key_val.first;
    const SpatialDyn::RigidBody& object = key_val.second;

    for (size_t t = 0; t < T; t++) {
      ObjectState& object_state = object_states_[name_object][t];
      object_state.pos = object.T_to_parent().translation();
      object_state.quat = Eigen::Quaterniond(object.T_to_parent().linear());
    }
  }
}

std::map<std::string, World::ObjectState> World::InterpolateSimulation(Eigen::Ref<const Eigen::VectorXd> q,
                                                                       size_t t) const {
  // Construct object states map for current timestep
  std::map<std::string, World::ObjectState> object_states_t;
  for (const auto& key_val : object_states_) {
    const std::string& object_name = key_val.first;
    const ObjectState& object_state = key_val.second[t];
    object_states_t[object_name] = object_state;
  }

  for (auto& key_val : object_states_t) {
    ObjectState& object_state = key_val.second;
    if (object_state.owner == nullptr) continue;

    object_state.owner->InterpolateSimulation(*this, q, object_states_t);
  }
  // object_states_t
  // // Call constraint interpolation
  // for (Constraint* c : schedule_[static_cast<size_t>(t)]) {
  //   c->InterpolateSimulation(*this, q, t, object_states_t);
  // }

  return object_states_t;
}

void World::InitializeConstraintSchedule(const Constraints& constraints) {
  // Construct schedule of constraints
  schedule_.resize(T);
  for (const std::unique_ptr<Constraint>& c : constraints) {
    // for (size_t t = c->t_start; t < c->t_start + c->num_timesteps; t++) {
    //   schedule_[t].push_back(c.get());
    // }
    schedule_[c->t_start()].push_back(c.get());
  }

  // Set object states at each timestep
  for (size_t t = 0; t < T; t++) {
    for (Constraint* c : schedule_[t]) {
      c->RegisterSimulationStates(*this);
    }
  }
}

void World::Simulate(Eigen::Ref<const Eigen::MatrixXd> Q) {
  // Return if Q has already been simulated
  if (Q_ == Q) return;

  for (size_t t = 0; t < T; t++) {
    for (Constraint* c : schedule_[t]) {
      c->Simulate(*this, Q);
    }
  }

  // Store Q for caching
  Q_ = Q;
}

}  // namespace LogicOpt

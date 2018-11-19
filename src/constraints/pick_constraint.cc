/**
 * pick_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "TrajOpt/constraints/pick_constraint.h"

namespace TrajOpt {

PickConstraint::PickConstraint(const World& world, size_t t_pick, const std::string& name_object,
                               const Eigen::Vector3d& ee_offset, Layout layout)
    : Constraint(NumConstraints(layout), NumConstraints(layout) * world.ab.dof(), t_pick, 1,
                 "constraint_pick_t" + std::to_string(t_pick)),
      CartesianPoseConstraint(world, t_pick, Eigen::Vector3d::Zero(),
                              Eigen::Quaterniond::Identity(), ee_offset, layout),
      world_(world), name_object_(name_object) {}

void PickConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                              Eigen::Ref<Eigen::VectorXd> constraints) {
  x_des_ = world_.object_state(name_object_, t_start_).pos;
  CartesianPoseConstraint::Evaluate(Q, constraints);
}

void PickConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                              Eigen::Ref<Eigen::VectorXd> Jacobian) {
  x_des_ = world_.object_state(name_object_, t_start_).pos;
  CartesianPoseConstraint::Jacobian(Q, Jacobian);
}

void PickConstraint::Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                             Eigen::Ref<const Eigen::VectorXd> lambda,
                             Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {
  x_des_ = world_.object_state(name_object_, t_start_).pos;
  CartesianPoseConstraint::Hessian(Q, lambda, Hessian);
}

void PickConstraint::Simulate(World& world, Eigen::Ref<const Eigen::MatrixXd> Q) {
  Eigen::Isometry3d T_pick_ee_to_world = SpatialDyn::CartesianPose(world.ab, Q.col(t_start_));
  const World::ObjectState& object_state = world.object_state(name_object_, t_start_);

  Eigen::Isometry3d T_pick_object_to_world = Eigen::Translation3d(object_state.pos) * object_state.quat;
  Eigen::Isometry3d T_object_to_ee = T_pick_ee_to_world.inverse() * T_pick_object_to_world;

  for (size_t t = t_start_ + 1; t < world.T; t++) {
    World::ObjectState& object_state = world.object_state(name_object_, t);
    if (object_state.owner != this) break;

    Eigen::Isometry3d T_object_to_world = SpatialDyn::CartesianPose(world.ab, Q.col(t)) * T_object_to_ee;
    object_state.pos = T_object_to_world.translation();
    object_state.quat = Eigen::Quaterniond(T_object_to_world.linear());
  }
}

void PickConstraint::RegisterSimulationStates(World& world) {
  for (size_t t = t_start_; t < world.T; t++) {
    World::ObjectState& object_state = world.object_state(name_object_, t);
    object_state.owner = this;
    object_state.type = World::ObjectState::Type::MANIPULATED;
  }
}

void PickConstraint::InterpolateSimulation(const World& world,
                                           Eigen::Ref<const Eigen::VectorXd> q,
                                           std::map<std::string, World::ObjectState>& object_states) const {

  Eigen::Isometry3d T_pick_ee_to_world = SpatialDyn::CartesianPose(world.ab, world.Q().col(t_start_));
  const World::ObjectState& object_state = world.object_state(name_object_, t_start_);

  Eigen::Isometry3d T_pick_object_to_world = Eigen::Translation3d(object_state.pos) * object_state.quat;
  Eigen::Isometry3d T_object_to_ee = T_pick_ee_to_world.inverse() * T_pick_object_to_world;

  World::ObjectState& object_state_t = object_states[name_object_];
  Eigen::Isometry3d T_object_to_world = SpatialDyn::CartesianPose(world.ab, q) * T_object_to_ee;
  object_state_t.pos = T_object_to_world.translation();
  object_state_t.quat = Eigen::Quaterniond(T_object_to_world.linear());
}

}  // namespace TrajOpt

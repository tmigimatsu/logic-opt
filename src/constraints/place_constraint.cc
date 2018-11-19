/**
 * place_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "TrajOpt/constraints/place_constraint.h"

namespace TrajOpt {

PlaceConstraint::PlaceConstraint(World& world, size_t t_place, const std::string& name_object,
                                 const Eigen::Vector3d& x_des, const Eigen::Quaterniond& quat_des,
                                 const Eigen::Vector3d& ee_offset, Layout layout)
      : Constraint(NumConstraints(layout), NumConstraints(layout) * world.ab.dof(), t_place, 1,
                   "constraint_place_t" + std::to_string(t_place)),
        CartesianPoseConstraint(world, t_place, x_des, quat_des, ee_offset, layout),
        x_des_place_(x_des), quat_des_place_(quat_des), name_object_(name_object), world_(world) {}

void PlaceConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                               Eigen::Ref<Eigen::VectorXd> constraints) {
  ComputePlacePose(Q);
  CartesianPoseConstraint::Evaluate(Q, constraints);
}

void PlaceConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                               Eigen::Ref<Eigen::VectorXd> Jacobian) {
  ComputePlacePose(Q);
  CartesianPoseConstraint::Jacobian(Q, Jacobian);
}

void PlaceConstraint::Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                              Eigen::Ref<const Eigen::VectorXd> lambda,
                              Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {
  ComputePlacePose(Q);
  CartesianPoseConstraint::Hessian(Q, lambda, Hessian);
}

void PlaceConstraint::Simulate(World& world, Eigen::Ref<const Eigen::MatrixXd> Q) {
  Eigen::Isometry3d T_pick_ee_to_world = SpatialDyn::CartesianPose(world.ab, Q.col(t_start_ - 1));
  const World::ObjectState& object_state_prev = world.object_state(name_object_, t_start_ - 1);

  Eigen::Isometry3d T_pick_object_to_world = Eigen::Translation3d(object_state_prev.pos) * object_state_prev.quat;
  Eigen::Isometry3d T_object_to_ee = T_pick_ee_to_world.inverse() * T_pick_object_to_world;

  Eigen::Isometry3d T_object_to_world = SpatialDyn::CartesianPose(world.ab, Q.col(t_start_)) * T_object_to_ee;
  World::ObjectState& object_state = world.object_state(name_object_, t_start_);
  object_state.pos = T_object_to_world.translation();
  object_state.quat = Eigen::Quaterniond(T_object_to_world.linear());

  for (size_t t = t_start_ + 1; t < world.T; t++) {
    World::ObjectState& object_state_t = world.object_state(name_object_, t);
    if (object_state_t.owner != this) break;  // TODO: Probably doesn't work with pick after

    object_state_t.pos = object_state.pos;
    object_state_t.quat = object_state.quat;
  }
}

void PlaceConstraint::RegisterSimulationStates(World& world) {
  for (size_t t = t_start_; t < world.T; t++) {
    World::ObjectState& object_state = world.object_state(name_object_, t);
    object_state.pos = x_des_place_;
    object_state.quat = quat_des_place_;
    object_state.owner = this;
    object_state.type = World::ObjectState::Type::FIXED;
  }
}

void PlaceConstraint::InterpolateSimulation(const World& world,
                                            Eigen::Ref<const Eigen::VectorXd> q,
                                            std::map<std::string, World::ObjectState>& object_states) const {

  Eigen::Isometry3d T_pick_ee_to_world = SpatialDyn::CartesianPose(world.ab, world.Q().col(t_start_ - 1));
  const World::ObjectState& object_state = world.object_state(name_object_, t_start_ - 1);

  Eigen::Isometry3d T_pick_object_to_world = Eigen::Translation3d(object_state.pos) * object_state.quat;
  Eigen::Isometry3d T_object_to_ee = T_pick_ee_to_world.inverse() * T_pick_object_to_world;

  World::ObjectState& object_state_t = object_states[name_object_];
  Eigen::Ref<const Eigen::VectorXd> q_t =
      (object_state_t.type == World::ObjectState::Type::MANIPULATED) ? q : world.Q().col(t_start_);
  Eigen::Isometry3d T_object_to_world = SpatialDyn::CartesianPose(world.ab, q_t) * T_object_to_ee;
  object_state_t.pos = T_object_to_world.translation();
  object_state_t.quat = Eigen::Quaterniond(T_object_to_world.linear());
}

void PlaceConstraint::ComputePlacePose(Eigen::Ref<const Eigen::MatrixXd> Q) {
  world_.Simulate(Q); // TODO: Move to Ipopt
  const World::ObjectState& object_state_prev = world_.object_state(name_object_, t_start_ - 1);
  Eigen::Isometry3d T_object_to_world = Eigen::Translation3d(object_state_prev.pos) * object_state_prev.quat;
  T_ee_to_object_ = T_object_to_world.inverse() * SpatialDyn::CartesianPose(world_.ab, Q.col(t_start_ - 1));

  x_des_ = x_des_place_ + T_ee_to_object_.translation();
  quat_des_ = T_ee_to_object_.linear() * quat_des_place_;

  ab_.set_q(Q.col(t_start_));
}

}  // namespace TrajOpt

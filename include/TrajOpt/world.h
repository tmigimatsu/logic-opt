/**
 * world.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 12, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_WORLD_H_
#define TRAJ_OPT_WORLD_H_

#include <SpatialDyn/SpatialDyn.h>

#include <map>     // std::map
#include <vector>  // std::vector

namespace TrajOpt {

class Constraint;
typedef std::vector<std::unique_ptr<Constraint>> Constraints;

class World {

 public:

  struct ObjectState {
    enum class Type {
      FIXED, FREE, MANIPULATED
    };

    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    Type type = Type::FIXED;

    Constraint* owner = nullptr;
  };

  World(const SpatialDyn::ArticulatedBody& ab,
        const std::map<std::string, SpatialDyn::RigidBody>& objects, size_t num_timesteps);

  void Simulate(Eigen::Ref<const Eigen::MatrixXd> Q);

  std::map<std::string, World::ObjectState> InterpolateSimulation(Eigen::Ref<const Eigen::VectorXd> q,
                                                                  size_t t) const;

  void InitializeConstraintSchedule(const Constraints& constraints);

  ObjectState& object_state(const std::string& name_object, size_t t) {
    return object_states_.at(name_object)[t];
  }
  const ObjectState& object_state(const std::string& name_object, size_t t) const {
    return object_states_.at(name_object)[t];
  }

  SpatialDyn::RigidBody object(const std::string& name_object, size_t t) {
    SpatialDyn::RigidBody rb = objects.at(name_object);
    const ObjectState& state = object_states_.at(name_object)[t];
    rb.set_T_to_parent(state.quat, state.pos);
    return rb;
  }

  const Eigen::MatrixXd& Q() const { return Q_; }

  mutable SpatialDyn::ArticulatedBody ab;
  const std::map<std::string, SpatialDyn::RigidBody>& objects;
  const size_t T;

 private:

  std::map<std::string, std::vector<ObjectState>> object_states_;
  std::vector<std::vector<Constraint*>> schedule_;
  Eigen::MatrixXd Q_;

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_WORLD_H_

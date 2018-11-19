/**
 * push_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_PUSH_CONSTRAINT_H_
#define TRAJ_OPT_PUSH_CONSTRAINT_H_

#include "TrajOpt/constraints/multi_constraint.h"
#include "TrajOpt/constraints/surface_contact_constraint.h"

namespace TrajOpt {

class PushConstraint : virtual public Constraint, protected MultiConstraint {

 public:

  enum class Direction { POS_X, POS_Y, NEG_X, NEG_Y };

  PushConstraint(World& world, size_t t_start, size_t num_timesteps,
                 const std::string& name_pusher, const std::string& name_pushee,
                 Direction direction_push);

  virtual ~PushConstraint() {}

  virtual void RegisterSimulationStates(World& world) override;

 protected:

  class PushSurfaceContactConstraint : virtual public Constraint, public SurfaceContactConstraint {

   public:

    PushSurfaceContactConstraint(World& world, size_t t_contact, const std::string& name_object,
                                 const std::string& name_surface, Direction direction_surface);

    virtual ~PushSurfaceContactConstraint() {}

    virtual void Simulate(World& world, Eigen::Ref<const Eigen::MatrixXd> Q) override;

    virtual void RegisterSimulationStates(World& world) override;

    virtual void InterpolateSimulation(const World& world, Eigen::Ref<const Eigen::VectorXd> q,
                                       std::map<std::string, World::ObjectState>& object_states) const override;

   protected:

    virtual void ComputePlacePose(Eigen::Ref<const Eigen::MatrixXd> Q) override;

    friend class PushConstraint;

  };

  class PushActionConstraint : virtual public Constraint, public PushSurfaceContactConstraint {

   public:

    PushActionConstraint(World& world, size_t t_contact, const std::string& name_object,
                         const std::string& name_surface, Direction direction_surface);

    virtual ~PushActionConstraint() {}

    virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                          Eigen::Ref<Eigen::VectorXd> constraints) override;

    virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                          Eigen::Ref<Eigen::VectorXd> Jacobian) override;

    virtual Type constraint_type(size_t idx_constraint) const override;

  };

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_PUSH_CONSTRAINT_H_

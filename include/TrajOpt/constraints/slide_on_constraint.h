/**
 * slide_on_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_SLIDE_ON_CONSTRAINT_H_
#define TRAJ_OPT_SLIDE_ON_CONSTRAINT_H_

#include "TrajOpt/constraints/multi_constraint.h"

namespace TrajOpt {

class SlideOnConstraint : virtual public Constraint, protected MultiConstraint {

 public:

  SlideOnConstraint(World& world, size_t t_start, size_t num_timesteps,
                    const std::string& name_object, const std::string& name_surface);

  virtual ~SlideOnConstraint() {}

  // Simulation methods
  virtual void RegisterSimulationStates(World& world) override;

  // Optimization properties
  virtual Type constraint_type(size_t idx_constraint) const override;

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_SLIDE_ON_CONSTRAINT_H_

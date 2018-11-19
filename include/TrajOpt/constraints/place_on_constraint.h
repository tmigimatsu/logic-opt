/**
 * place_on_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_PLACE_ON_CONSTRAINT_H_
#define TRAJ_OPT_PLACE_ON_CONSTRAINT_H_

#include "TrajOpt/constraints/surface_contact_constraint.h"

namespace TrajOpt {

class PlaceOnConstraint : virtual public Constraint, protected SurfaceContactConstraint {

 public:

  PlaceOnConstraint(World& world, size_t t_place, const std::string& name_object,
                    const std::string& name_surface)
      : Constraint(6, 6 * world.ab.dof(), t_place, 1,
                   "constraint_placeon_t" + std::to_string(t_place)),
        SurfaceContactConstraint(world, t_place, name_object, name_surface, Direction::POS_Z) {}

  virtual ~PlaceOnConstraint() {}

 protected:

  friend class SlideOnConstraint;

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_PLACE_ON_CONSTRAINT_H_

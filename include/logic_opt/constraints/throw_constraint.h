/**
 * throw_constraint.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 4, 2019
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_THROW_CONSTRAINT_H_
#define LOGIC_OPT_THROW_CONSTRAINT_H_

#include "logic_opt/constraints/constraint.h"
#include "logic_opt/constraints/multi_constraint.h"
#include "logic_opt/constraints/place_constraint.h"

namespace logic_opt {

class ThrowConstraint : public MultiConstraint {

 public:

  static constexpr size_t kNumTimesteps = 1;

  ThrowConstraint(World3& world, size_t t_throw, const std::string& name_object,
                  const std::string& name_target);

  virtual ~ThrowConstraint() = default;

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_THROW_CONSTRAINT_H_

/**
 * parameter_generator.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_PLANNING_PARAMETER_GENERATOR_H_
#define TRAJ_OPT_PLANNING_PARAMETER_GENERATOR_H_

#include <memory>    // std::shared_ptr
#include <vector>    // std::vector

#include "ptree.h"

#include "TrajOpt/planning/combination_generator.h"
#include "TrajOpt/planning/objects.h"

namespace TrajOpt {

class ParameterGenerator : public CombinationGenerator<const std::vector<const VAL::parameter_symbol*>> {

 public:

  ParameterGenerator() {}

  ParameterGenerator(const std::shared_ptr<const ObjectTypeMap>& objects,
                     const VAL::parameter_symbol_list* params);

  ParameterGenerator(const std::shared_ptr<const ObjectTypeMap>& objects,
                     const VAL::var_symbol_list* params);

 private:

  std::shared_ptr<const ObjectTypeMap> objects_;

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_PLANNING_PARAMETER_GENERATOR_H_

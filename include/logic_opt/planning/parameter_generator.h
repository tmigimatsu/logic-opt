/**
 * parameter_generator.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_PLANNING_PARAMETER_GENERATOR_H_
#define LOGIC_OPT_PLANNING_PARAMETER_GENERATOR_H_

#include <memory>    // std::shared_ptr
#include <vector>    // std::vector

#include "ptree.h"

#include "logic_opt/planning/combination_generator.h"
#include "logic_opt/planning/objects.h"

namespace logic_opt {

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

}  // namespace logic_opt

#endif  // LOGIC_OPT_PLANNING_PARAMETER_GENERATOR_H_

/**
 * parameter_generator.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/planning/parameter_generator.h"

namespace logic_opt {

template<typename T>
std::vector<const std::vector<const VAL::parameter_symbol*>*>
ParamTypes(const std::shared_ptr<const ObjectTypeMap> objects,
           const VAL::typed_symbol_list<T>* params) {
  std::vector<const std::vector<const VAL::parameter_symbol*>*> types;
  types.reserve(params->size());
  for (const VAL::parameter_symbol* param : *params) {
    types.push_back(&objects->at(param->type));
  }
  return types;
}

ParameterGenerator::ParameterGenerator(
    const std::shared_ptr<const ObjectTypeMap>& objects,
    const VAL::parameter_symbol_list* params)
    : CombinationGenerator(ParamTypes(objects, params)), objects_(objects) {}

ParameterGenerator::ParameterGenerator(
    const std::shared_ptr<const ObjectTypeMap>& objects,
    const VAL::var_symbol_list* params)
    : CombinationGenerator(ParamTypes(objects, params)), objects_(objects) {}

}  // namespace logic_opt

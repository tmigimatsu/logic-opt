/**
 * objects.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#include "TrajOpt/planning/objects.h"

namespace TrajOpt {

std::shared_ptr<const ObjectTypeMap> CreateObjectsMap(const VAL::const_symbol_list* constants,
                                                      const VAL::const_symbol_list* objects) {
  auto map_objects = std::make_shared<ObjectTypeMap>();
  for (const VAL::const_symbol* object : *constants) {
    const VAL::parameter_symbol* param = dynamic_cast<const VAL::parameter_symbol*>(object);
    (*map_objects)[param->type].push_back(param);
  }
  for (const VAL::const_symbol* object : *objects) {
    const VAL::parameter_symbol* param = dynamic_cast<const VAL::parameter_symbol*>(object);
    (*map_objects)[param->type].push_back(param);
  }
  return map_objects;
}

}  // namespace TrajOpt

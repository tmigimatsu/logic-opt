/**
 * objects.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/planning/objects.h"

namespace logic_opt {

/**
 * Create a map from object types to member objects.
 */
std::shared_ptr<const ObjectTypeMap> CreateObjectsMap(const VAL::const_symbol_list* constants,
                                                      const VAL::const_symbol_list* objects) {
  auto map_objects = std::make_shared<ObjectTypeMap>();

  // Iterate over constants
  for (const VAL::const_symbol* object : *constants) {
    const VAL::parameter_symbol* param = dynamic_cast<const VAL::parameter_symbol*>(object);

    // Iterate over this object's type superclasses
    for (const VAL::pddl_type* type = param->type; type != nullptr; type = type->type) {
      // Add object to the type's member vector'
      (*map_objects)[type].push_back(param);
    }
  }

  // Iterate over objects
  for (const VAL::const_symbol* object : *objects) {
    const VAL::parameter_symbol* param = dynamic_cast<const VAL::parameter_symbol*>(object);

    // Iterate over this object's type superclasses
    for (const VAL::pddl_type* type = param->type; type != nullptr; type = type->type) {
      // Add object to the type's member vector'
      (*map_objects)[type].push_back(param);
    }
  }
  return map_objects;
}

}  // namespace logic_opt

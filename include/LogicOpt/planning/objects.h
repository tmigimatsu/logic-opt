/**
 * objects.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_PLANNING_OBJECTS_H_
#define LOGIC_OPT_PLANNING_OBJECTS_H_

#include <map>     // std::map
#include <memory>  // std::shared_ptr
#include <vector>  // std::vector

#include "ptree.h"

namespace LogicOpt {

using ObjectTypeMap = std::map<const VAL::pddl_type*, std::vector<const VAL::parameter_symbol*>>;

std::shared_ptr<const ObjectTypeMap> CreateObjectsMap(const VAL::const_symbol_list* constants,
                                                      const VAL::const_symbol_list* objects);

}  // namespace LogicOpt

#endif  // LOGIC_OPT_PLANNING_OBJECTS_H_

/**
 * objects.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/planning/objects.h"

#include <algorithm>  // std::min, std::replace
#include <cassert>    // assert
#include <sstream>    // std::stringstream

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

namespace {

const VAL::parameter_symbol* GetValSymbol(const std::vector<const VAL::parameter_symbol*>& objects,
                                      const std::string& name_object) {
  for (const VAL::parameter_symbol* obj : objects) {
    assert(obj != nullptr);
    if (obj->getName() == name_object) return obj;
  }
  // std::cerr << "GetValSymbol(): could not find " << name_object << std::endl;
  return nullptr;
}

std::vector<std::string> TokenizeArguments(const std::string& proposition) {
  const size_t idx_start = proposition.find_first_of('(') + 1;
  const size_t idx_end = std::min(proposition.size(), proposition.find_last_of(')')) - idx_start;
  std::string str_args = proposition.substr(idx_start, idx_end);
  std::replace(str_args.begin(), str_args.end(), ',', ' ');
  std::stringstream ss(str_args);

  std::string arg;
  std::vector<std::string> args;
  while (ss >> arg) {
    args.emplace_back(std::move(arg));
  }
  return args;
}

}  // namespace

Symbol::Symbol(const std::vector<const VAL::parameter_symbol*>& objects,
               const std::string& name_object)
    : symbol_(GetValSymbol(objects, name_object)) {}

std::vector<Symbol> ParseArguments(const std::vector<const VAL::parameter_symbol*>& objects,
                                   const std::string& atom) {
  // std::cout << "ParseArguments: " << atom << " -> ";
  const std::vector<std::string> name_args = TokenizeArguments(atom);
  std::vector<Symbol> args;
  args.reserve(name_args.size());
  for (const std::string& name_arg : name_args) {
    // std::cout << name_arg << " ";
    args.emplace_back(objects, name_arg);
  }
  // std::cout << std::endl;
  return args;
}

}  // namespace logic_opt

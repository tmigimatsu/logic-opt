/**
 * proposition.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/planning/proposition.h"

#include <sstream>  // std::stringstream

namespace {

/**
 * Test whether subtype is a subtype of supertype. Types are subtypes of themselves.
 */
bool IsSubtype(const VAL::pddl_type* subtype, const VAL::pddl_type* supertype) {
  for (const VAL::pddl_type* type = subtype; type != nullptr; type = type->type) {
    if (type == supertype) return true;
  }
  return false;
}

}  // namespace

namespace logic_opt {

Proposition::Proposition(const VAL::proposition* predicate,
                         std::vector<const VAL::parameter_symbol*>&& a_variables)
    : predicate_(predicate->head->getName()), variables_(std::move(a_variables)) {

  // Check number of arguments
  if (predicate->args->size() != variables_.size()) {
    std::stringstream ss;
    ss << "Proposition(): " << predicate->head->getName() << "(";
    std::string delimiter;
    for (const VAL::parameter_symbol* var : variables_) {
      ss << delimiter << var->getName() << ": " << var->type->getName();
      if (delimiter.empty()) delimiter = ", ";
    }
    ss << ") constructed with incorrect number of arguments. Requires " << predicate->args->size() << ".";
    throw std::invalid_argument(ss.str());
  }

  // Check argument types
  size_t i = 0;
  for (const VAL::parameter_symbol* arg : *predicate->args) {
    if (!IsSubtype(variables_[i]->type, arg->type)) {
      std::stringstream ss;
      ss << "Proposition(): " << predicate->head->getName() << "(";
      std::string delimiter;
      for (const VAL::parameter_symbol* var : variables_) {
      ss << delimiter << var->getName() << ": " << var->type->getName();
        if (delimiter.empty()) delimiter = ", ";
      }
      ss << ") constructed with incorrect argument types. Requires (";
      delimiter.clear();
      for (const VAL::parameter_symbol* arg : *predicate->args) {
        ss << delimiter << arg->type->getName();
        if (delimiter.empty()) delimiter = ", ";
      }
      ss << ").";
      throw std::invalid_argument(ss.str());
    }
    ++i;
  }
}

Proposition::Proposition(const std::string& name_predicate,
                         std::vector<const VAL::parameter_symbol*>&& a_variables)
    : predicate_(name_predicate), variables_(std::move(a_variables)) {}

Proposition::Proposition(const std::string& name_predicate,
                         const std::vector<const VAL::parameter_symbol*>& a_variables)
    : predicate_(name_predicate), variables_(a_variables) {}

bool Proposition::operator<(const Proposition& rhs) const {
  if (predicate_ != rhs.predicate_) return predicate_ < rhs.predicate_;
  for (size_t i = 0; i < variables_.size(); i++) {
    if (variables_[i] != rhs.variables_[i]) return variables_[i] < rhs.variables_[i];
  }
  return false;
}

bool Proposition::operator==(const Proposition& rhs) const {
  if (predicate_ != rhs.predicate_) return false;
  for (size_t i = 0; i < variables_.size(); i++) {
    if (variables_[i] != rhs.variables_[i]) return false;
  }
  return true;
}

bool Proposition::operator!=(const Proposition& rhs) const {
  return !(*this == rhs);
}

std::ostream& operator<<(std::ostream& os, const logic_opt::Proposition& P) {
  os << P.predicate() << "(";
  std::string separator;
  for (const VAL::parameter_symbol* param : P.variables()) {
    os << separator << param->getName();
    if (separator.empty()) separator = ", ";
  }
  os << ")";
  return os;
}

}  // namespace logic_opt

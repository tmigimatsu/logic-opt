/**
 * proposition.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/planning/proposition.h"

namespace LogicOpt {

Proposition::Proposition(const VAL::proposition* predicate,
                         std::vector<const VAL::parameter_symbol*>&& a_variables)
    : predicate_(predicate->head->getName()), variables_(std::move(a_variables)) {

  if (predicate->args->size() != variables_.size()) {
    throw std::invalid_argument("Proposition(): " + predicate->head->getName() + "() constructed with incorrect number of arguments.");
  }

  size_t i = 0;
  for (const VAL::parameter_symbol* arg : *predicate->args) {
    if (arg->type != variables_[i]->type) {
      throw std::invalid_argument("Proposition(): " + predicate->head->getName() + "() constructed with incorrect argument types.");
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

std::ostream& operator<<(std::ostream& os, const LogicOpt::Proposition& P) {
  os << P.predicate() << "(";
  std::string separator;
  for (const VAL::parameter_symbol* param : P.variables()) {
    os << separator << param->getName();
    if (separator.empty()) separator = ", ";
  }
  os << ")";
  return os;
}

}  // namespace LogicOpt

/**
 * proposition.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#include "TrajOpt/planning/proposition.h"

namespace TrajOpt {

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

std::ostream& operator<<(std::ostream& os, const TrajOpt::Proposition& P) {
  os << P.predicate() << "(";
  std::string separator;
  for (const VAL::parameter_symbol* param : P.variables()) {
    os << separator << param->getName();
    if (separator.empty()) separator = ", ";
  }
  os << ")";
  return os;
}

std::set<Proposition> CreateInitialPropositions(const VAL::effect_lists* initial_state,
                                                const VAL::const_symbol_list* constants,
                                                const VAL::const_symbol_list* objects) {
  std::set<Proposition> propositions;
  for (const VAL::simple_effect* effect : initial_state->add_effects) {
    std::vector<const VAL::parameter_symbol*> params(effect->prop->args->begin(), effect->prop->args->end());
    propositions.emplace(effect->prop, std::move(params));
  }
  for (const VAL::const_symbol* object : *constants) {
    std::vector<const VAL::parameter_symbol*> params(2, object);
    propositions.emplace("=", std::move(params));
  }
  for (const VAL::const_symbol* object : *objects) {
    std::vector<const VAL::parameter_symbol*> params(2, object);
    propositions.emplace("=", std::move(params));
  }
  return propositions;
}

}  // namespace TrajOpt

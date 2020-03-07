/**
 * validator.cc
 *
 * Copyright 2020. All Rights Reserved.
 *
 * Created: March 05, 2020
 * Authors: Toki Migimatsu
 */

#include "logic_opt/planning/validator.h"

#include <algorithm>  // std::replace
#include <sstream>    // std::stringstream

#include "ptree.h"

#include "logic_opt/planning/actions.h"
#include "logic_opt/planning/formula.h"
#include "logic_opt/planning/objects.h"
#include "logic_opt/planning/pddl.h"
#include "logic_opt/planning/proposition.h"

namespace logic_opt {

namespace {

VAL::parameter_symbol_list CreateGoalParams(const std::shared_ptr<const ObjectTypeMap>& objects) {
  VAL::parameter_symbol_list goal_params;
  for (const auto& key_val : *objects) {
    for (const VAL::parameter_symbol* object : key_val.second) {
      goal_params.push_back(const_cast<VAL::parameter_symbol*>(object));
    }
  }
  return goal_params;
}

// TODO: From planner, move to proposition
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

// TODO: Remove
std::set<std::string> ConvertState(const std::set<Proposition>& propositions) {
  std::set<std::string> state;
  for (const Proposition& prop : propositions) {
    std::stringstream ss;
    ss << prop.predicate() << "(";
    std::string delimiter;
    for (const VAL::parameter_symbol* var : prop.variables()) {
      ss << delimiter << var->getName();
      if (delimiter.empty()) delimiter = ", ";
    }
    ss << ")";
    state.insert(ss.str());
  }
  return state;
}

}  // namespace

Validator::Validator(const std::string& domain_pddl, const std::string& problem_pddl)
    : analysis_(ParsePddl(domain_pddl, problem_pddl)),
      domain_(analysis_->the_domain),
      problem_(analysis_->the_problem),
      objects_(CreateObjectsMap(domain_->constants, problem_->objects)),
      list_objects_(CreateGoalParams(objects_)),
      vec_objects_(list_objects_.begin(), list_objects_.end()),
      initial_state_(ConvertState(CreateInitialPropositions(problem_->initial_state, domain_->constants, problem_->objects))) {}

std::string ParsePredicate(const std::string& proposition) {
  return proposition.substr(0, proposition.find_first_of('('));
}

std::vector<const VAL::parameter_symbol*> Validator::GetValArguments(const std::string& atom) const {
  const std::vector<Symbol> args = ParseArguments(vec_objects_, atom);
  std::vector<const VAL::parameter_symbol*> val_args;
  val_args.reserve(args.size());
  for (const Symbol& arg : args) {
    val_args.push_back(arg.symbol());
  }
  return val_args;
}

Proposition Validator::GetProposition(const std::string& proposition) const {
  const std::string name_predicate = ParsePredicate(proposition);
  std::vector<const VAL::parameter_symbol*> val_args = GetValArguments(proposition);
  // TODO: Implement proposition using Object, not VAL::parameter_symbol
  return Proposition(name_predicate, std::move(val_args));
}

std::set<Proposition> Validator::GetPropositions(const std::set<std::string>& state) const {
  std::set<Proposition> propositions;
  for (const std::string& proposition : state) {
    propositions.insert(GetProposition(proposition));
  }
  return propositions;
}

std::set<std::string> Validator::NextState(const std::set<std::string>& state,
                                           const std::string& action_call) const {
  // if (!IsValidAction(state, action_call)) throw std::runtime_error("TODO");
  const std::set<Proposition> propositions = GetPropositions(state);
  const std::string name_action = ParsePredicate(action_call);
  const Action action(domain_, name_action);
  const std::vector<const VAL::parameter_symbol*> action_args = GetValArguments(action_call);
  const std::set<Proposition> post = ApplyEffects(formulas_, objects_,
                                                  action_args, action.symbol()->parameters,
                                                  action.symbol()->effects, propositions);
  const std::set<std::string> next_state = ConvertState(post);
  return next_state;
}

bool Validator::IsValidAction(const std::set<std::string>& state,
                              const std::string& action_call) const {
  const std::set<Proposition> propositions = GetPropositions(state);
  const std::string name_action = ParsePredicate(action_call);
  const Action action(domain_, name_action);
  // TODO: Implement formula using Action
  const Formula& P = GetFormula(formulas_, objects_,
                                action.symbol()->precondition, action.symbol()->parameters);
  const std::vector<const VAL::parameter_symbol*> action_args = GetValArguments(action_call);
  if (action_args.size() != action.symbol()->parameters->size()) {
    std::cout << "Incorrect number of arguments: " << action_call << std::endl;
    return false;
  }
  return P(propositions, action_args);
}

bool Validator::IsValidTuple(const std::set<std::string>& state,
                             const std::string& action,
                             const std::set<std::string>& next_state) const {
  return IsValidAction(state, action) && next_state == NextState(state, action);
}

bool Validator::IsGoalSatisfied(const std::set<std::string>& state) const {
  const std::set<Proposition> propositions = GetPropositions(state);
  const Formula& G = GetFormula(formulas_, objects_, problem_->the_goal, &list_objects_);
  return G(propositions, vec_objects_);
}

bool Validator::IsValidPlan(const std::vector<std::string>& actions) const {
  std::set<std::string> state = initial_state_;

  for (const std::string& action : actions) {
    if (!IsValidAction(state, action)) return false;
    state = NextState(state, action);
  }

  return IsGoalSatisfied(state);
}

}  // namespace logic_opt

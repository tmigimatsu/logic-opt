/**
 * validator.h
 *
 * Copyright 2020. All Rights Reserved.
 *
 * Created: March 05, 2020
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_PLANNING_VALIDATOR_H_
#define LOGIC_OPT_PLANNING_VALIDATOR_H_

#include <set>     // std::set
#include <string>  // std::string
#include <vector>  // std::vector

#include "ptree.h"

#include "logic_opt/planning/formula.h"
#include "logic_opt/planning/proposition.h"

namespace logic_opt {

class Validator {

 public:

  Validator(const std::string& domain_pddl, const std::string& problem_pddl);

  std::set<std::string> NextState(const std::set<std::string>& state,
                                  const std::string& action_call) const;

  bool IsValidAction(const std::set<std::string>& state,
                     const std::string& action_call) const;

  bool IsValidTuple(const std::set<std::string>& state,
                    const std::string& action_call,
                    const std::set<std::string>& next_state) const;

  bool IsGoalSatisfied(const std::set<std::string>& state) const;

  bool IsValidPlan(const std::vector<std::string>& action_skeleton) const;

  const std::set<std::string>& initial_state() const { return initial_state_; }

 private:

  std::vector<const VAL::parameter_symbol*> GetValArguments(const std::string& atom) const;
  Proposition GetProposition(const std::string& proposition) const;
  std::set<Proposition> GetPropositions(const std::set<std::string>& state) const;

  const std::unique_ptr<VAL::analysis> analysis_;
  const VAL::domain* domain_;
  const VAL::problem* problem_;

  // TODO: Consolidate
  const std::shared_ptr<const ObjectTypeMap> objects_;
  const VAL::parameter_symbol_list list_objects_;
  const std::vector<const VAL::parameter_symbol*> vec_objects_;

  const std::set<std::string> initial_state_;

  mutable FormulaMap formulas_;

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_PLANNING_VALIDATOR_H_

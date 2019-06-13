/**
 * formula.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_PLANNING_FORMULA_H_
#define LOGIC_OPT_PLANNING_FORMULA_H_

#include <functional>  // std::function
#include <set>         // std::set
#include <vector>      // std::vector

#include "ptree.h"

#include "logic_opt/planning/objects.h"
#include "logic_opt/planning/parameter_generator.h"
#include "logic_opt/planning/proposition.h"

namespace logic_opt {

using Formula = std::function<bool(const std::set<Proposition>& propositions,
                                   const std::vector<const VAL::parameter_symbol*>& variables)>;

using FormulaMap = std::map<const VAL::goal*, Formula>;

template <typename T>
Formula& GetFormula(FormulaMap& formulas,
                    const std::shared_ptr<const ObjectTypeMap>& objects,
                    const VAL::goal* goal, const VAL::typed_symbol_list<T>* action_params);

template <typename T>
Formula CreateProposition(const VAL::typed_symbol_list<T>* action_params,
                          const VAL::simple_goal* simple_goal);

template <typename T>
Formula CreateConjunction(FormulaMap& formulas,
                          const std::shared_ptr<const ObjectTypeMap>& objects,
                          const VAL::typed_symbol_list<T>* action_params,
                          const VAL::conj_goal* conj_goal);

template <typename T>
Formula CreateNegation(FormulaMap& formulas,
                       const std::shared_ptr<const ObjectTypeMap>& objects,
                       const VAL::typed_symbol_list<T>* action_params,
                       const VAL::neg_goal* neg_goal);

template <typename T>
Formula CreateForall(FormulaMap& formulas,
                     const std::shared_ptr<const ObjectTypeMap>& objects,
                     const VAL::typed_symbol_list<T>* action_params,
                     const VAL::qfied_goal* qfied_goal);

template <typename T>
Formula CreateExists(FormulaMap& formulas,
                     const std::shared_ptr<const ObjectTypeMap>& objects,
                     const VAL::typed_symbol_list<T>* action_params,
                     const VAL::qfied_goal* qfied_goal);

/**
 * Get goal formula from the cache, or create it if it doesn't exist.
 */
template <typename T>
Formula& GetFormula(FormulaMap& formulas,
                    const std::shared_ptr<const ObjectTypeMap>& objects,
                    const VAL::goal* goal, const VAL::typed_symbol_list<T>* action_params) {

  auto it = formulas.find(goal);
  if (it != formulas.end()) {
    return it->second;
  }

  // Proposition
  const VAL::simple_goal* simple_goal = dynamic_cast<const VAL::simple_goal*>(goal);
  if (simple_goal != nullptr) {
    it = formulas.emplace(std::move(goal), CreateProposition(action_params, simple_goal)).first;
    return it->second;
  }

  // Conjunction
  const VAL::conj_goal* conj_goal = dynamic_cast<const VAL::conj_goal*>(goal);
  if (conj_goal != nullptr) {
    it = formulas.emplace(std::move(goal), CreateConjunction(formulas, objects, action_params, conj_goal)).first;
    return it->second;
  }

  // Negation
  const VAL::neg_goal* neg_goal = dynamic_cast<const VAL::neg_goal*>(goal);
  if (neg_goal != nullptr) {
    it = formulas.emplace(std::move(goal), CreateNegation(formulas, objects, action_params, neg_goal)).first;
    return it->second;
  }

  // Forall
  const VAL::qfied_goal* qfied_goal = dynamic_cast<const VAL::qfied_goal*>(goal);
  if (qfied_goal != nullptr) {
    Formula P;
    switch (qfied_goal->getQuantifier()) {
      case VAL::quantifier::E_FORALL:
        P = CreateForall(formulas, objects, action_params, qfied_goal);
        break;
      case VAL::quantifier::E_EXISTS:
        P = CreateExists(formulas, objects, action_params, qfied_goal);
        break;
    }
    it = formulas.emplace(std::move(goal), std::move(P)).first;
    return it->second;
  }

  throw std::runtime_error("GetFormula(): Goal type not implemented.");
}

template <typename T>
static std::vector<size_t> IdxPredicateToActionParams(const VAL::typed_symbol_list<T>* action_params,
                                                      const VAL::parameter_symbol_list* pred_params) {
  // Map predicate params to action params
  std::map<const VAL::parameter_symbol*, size_t> map_args;  // Param to action arg idx
  size_t i = 0;
  for (const VAL::parameter_symbol* param : *action_params) {
    map_args[param] = i;
    ++i;
  }

  std::vector<size_t> idx_pred_to_action;  // Pred arg idx to action arg idx
  idx_pred_to_action.reserve(pred_params->size());
  for (const VAL::parameter_symbol* param : *pred_params) {
    idx_pred_to_action.push_back(map_args.at(param));
  }
  return idx_pred_to_action;
}

static std::vector<const VAL::parameter_symbol*> ActionToPredicateArgs(
    const std::vector<const VAL::parameter_symbol*>& action_args,
    const std::vector<size_t>& idx_pred_to_action) {

  // Prepare predicate arguments
  std::vector<const VAL::parameter_symbol*> pred_args;
  pred_args.reserve(idx_pred_to_action.size());
  for (size_t idx_arg : idx_pred_to_action) {
    pred_args.push_back(action_args[idx_arg]);
  }
  return pred_args;
}

template <typename T>
Formula CreateProposition(const VAL::typed_symbol_list<T>* action_params, const VAL::simple_goal* simple_goal) {
  const VAL::proposition* pred = simple_goal->getProp();
  return [pred, idx_pred_to_action = IdxPredicateToActionParams(action_params, pred->args)](
      const std::set<Proposition>& propositions,
      const std::vector<const VAL::parameter_symbol*>& action_args) -> bool {
    // Search for predicate in knowledge base
    Proposition P(pred, ActionToPredicateArgs(action_args, idx_pred_to_action));
    return propositions.find(P) != propositions.end();
  };
}

template <typename T>
Formula CreateConjunction(FormulaMap& formulas,
                          const std::shared_ptr<const ObjectTypeMap>& objects,
                          const VAL::typed_symbol_list<T>* action_params,
                          const VAL::conj_goal* conj_goal) {
  const VAL::goal_list* goals = conj_goal->getGoals();
  return [&formulas, objects, action_params, goals](
      const std::set<Proposition>& propositions,
      const std::vector<const VAL::parameter_symbol*>& action_args) -> bool {
    for (const VAL::goal* g : *goals) {
      const Formula& P = GetFormula(formulas, objects, g, action_params);
      if (!P(propositions, action_args)) return false;
    }
    return true;
  };
}

template <typename T>
Formula CreateNegation(FormulaMap& formulas,
                       const std::shared_ptr<const ObjectTypeMap>& objects,
                       const VAL::typed_symbol_list<T>* action_params,
                       const VAL::neg_goal* neg_goal) {
  const VAL::goal* g = neg_goal->getGoal();
  return [&formulas, objects, action_params, g](
      const std::set<Proposition>& propositions,
      const std::vector<const VAL::parameter_symbol*>& action_args) -> bool {
    // Negate positive formula
    const Formula& P = GetFormula(formulas, objects, g, action_params);
    return !P(propositions, action_args);
  };
}

template <typename T>
Formula CreateForall(FormulaMap& formulas,
                     const std::shared_ptr<const ObjectTypeMap>& objects,
                     const VAL::typed_symbol_list<T>* action_params,
                     const VAL::qfied_goal* qfied_goal) {
  const VAL::goal* g = qfied_goal->getGoal();
  const VAL::var_symbol_list* qfied_vars = qfied_goal->getVars();
  VAL::typed_symbol_list<T> qfied_params(*action_params);
  for (T* var : *qfied_vars) {
    qfied_params.push_back(var);
  }

  return [&formulas, objects, qfied_params = std::move(qfied_params), qfied_vars, g](
      const std::set<Proposition>& propositions,
      const std::vector<const VAL::parameter_symbol*>& action_args) -> bool {
    const Formula& P = GetFormula(formulas, objects, g, &qfied_params);
    ParameterGenerator gen(objects, qfied_vars);
    for (const std::vector<const VAL::parameter_symbol*>& variables : gen) {
      std::vector<const VAL::parameter_symbol*> qfied_action_args(action_args);
      qfied_action_args.insert(qfied_action_args.end(), variables.begin(), variables.end());
      if (!P(propositions, qfied_action_args)) return false;
    }
    return true;
  };
}

template <typename T>
Formula CreateExists(FormulaMap& formulas,
                     const std::shared_ptr<const ObjectTypeMap>& objects,
                     const VAL::typed_symbol_list<T>* action_params,
                     const VAL::qfied_goal* qfied_goal) {
  const VAL::goal* g = qfied_goal->getGoal();
  const VAL::var_symbol_list* qfied_vars = qfied_goal->getVars();
  VAL::typed_symbol_list<T> qfied_params(*action_params);
  for (T* var : *qfied_vars) {
    qfied_params.push_back(var);
  }

  return [&formulas, objects, qfied_params = std::move(qfied_params), qfied_vars, g](
      const std::set<Proposition>& propositions,
      const std::vector<const VAL::parameter_symbol*>& action_args) -> bool {
    const Formula& P = GetFormula(formulas, objects, g, &qfied_params);
    ParameterGenerator gen(objects, qfied_vars);
    for (const std::vector<const VAL::parameter_symbol*>& variables : gen) {
      std::vector<const VAL::parameter_symbol*> qfied_action_args(action_args);
      qfied_action_args.insert(qfied_action_args.end(), variables.begin(), variables.end());
      if (P(propositions, qfied_action_args)) return true;
    }
    return true;
  };
}

}  // namespace logic_opt

#endif  // LOGIC_OPT_PLANNING_FORMULA_H_

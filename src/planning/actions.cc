/**
 * actions.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/planning/actions.h"

#include <cassert>  // assert
#include <map>      // std::map

#include "logic_opt/planning/objects.h"
#include "logic_opt/planning/parameter_generator.h"
#include "logic_opt/planning/proposition.h"

namespace logic_opt {

std::vector<const VAL::parameter_symbol*> FilterEffectArgs(const std::vector<const VAL::parameter_symbol*>& action_args,
                                                           const VAL::var_symbol_list* action_params,
                                                           const VAL::parameter_symbol_list* effect_params) {
  std::map<const VAL::parameter_symbol*, size_t> map_args;
  size_t i = 0;
  for (VAL::parameter_symbol* param : *action_params) {
    map_args[param] = i;
    ++i;
  }

  std::vector<const VAL::parameter_symbol*> effect_args;
  effect_args.reserve(effect_params->size());
  for (const VAL::parameter_symbol* param : *effect_params) {
    effect_args.push_back(action_args[map_args.at(param)]);
  }
  return effect_args;
}

static void ApplyEffectsInternal(FormulaMap& formulas,
                                 const std::shared_ptr<const ObjectTypeMap>& objects,
                                 const std::vector<const VAL::parameter_symbol*>& action_args,
                                 const VAL::var_symbol_list* action_params,
                                 const VAL::effect_lists* effects,
                                 std::set<Proposition>* propositions) {
  for (const VAL::forall_effect* forall_effect : effects->forall_effects) {
    // Create list of forall arg types
    const VAL::var_symbol_list* forall_params = forall_effect->getVarsList();
    ParameterGenerator gen(objects, forall_params);
    VAL::var_symbol_list forall_action_params(*action_params);
    forall_action_params.insert(forall_action_params.end(),
                                forall_params->begin(), forall_params->end());
    for (const std::vector<const VAL::parameter_symbol*>& variables : gen) {
      std::vector<const VAL::parameter_symbol*> forall_action_args(action_args);
      forall_action_args.insert(forall_action_args.end(), variables.begin(), variables.end());

      const VAL::effect_lists* forall_effect_lists = forall_effect->getEffects();
      ApplyEffectsInternal(formulas, objects, forall_action_args, &forall_action_params,
                           forall_effect_lists, propositions);
    }
  }
  for (const VAL::simple_effect* effect : effects->add_effects) {
    propositions->emplace(effect->prop, FilterEffectArgs(action_args, action_params, effect->prop->args));
  }
  for (const VAL::simple_effect* effect : effects->del_effects) {
    propositions->erase(Proposition(effect->prop, FilterEffectArgs(action_args, action_params, effect->prop->args)));
  }
  for (const VAL::cond_effect* effect : effects->cond_effects) {
    const VAL::goal* condition = effect->getCondition();
    const Formula& P = GetFormula(formulas, objects, condition, action_params);
    if (P(*propositions, action_args)) {
      ApplyEffectsInternal(formulas, objects, action_args, action_params, effect->getEffects(), propositions);
    }
  }
}

std::set<Proposition> ApplyEffects(FormulaMap& formulas,
                                   const std::shared_ptr<const ObjectTypeMap>& objects,
                                   const std::vector<const VAL::parameter_symbol*>& action_args,
                                   const VAL::var_symbol_list* action_params,
                                   const VAL::effect_lists* effects,
                                   const std::set<Proposition>& propositions) {
  std::set<Proposition> new_propositions(propositions);
  ApplyEffectsInternal(formulas, objects, action_args, action_params, effects, &new_propositions);
  return new_propositions;
}

namespace {

const VAL::operator_* GetValAction(const VAL::domain* domain,
                                   const std::string& name_action) {
  assert(domain != nullptr && domain->ops != nullptr);
  for (const VAL::operator_* op : *domain->ops) {
    assert(op != nullptr && op->name != nullptr);
    if (op->name->getName() == name_action) return op;
  }
  // std::cerr << "GetValAction(): could not find " << name_action << std::endl;
  return nullptr;
}

}  // namespace

Action::Action(const VAL::domain* domain, const std::string& name_action)
    : symbol_(GetValAction(domain, name_action)) {}

}  // namespace logic_opt

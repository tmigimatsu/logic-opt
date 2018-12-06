/**
 * actions.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/planning/actions.h"

#include <map>  // std::map

#include "LogicOpt/planning/objects.h"
#include "LogicOpt/planning/parameter_generator.h"
#include "LogicOpt/planning/proposition.h"

namespace LogicOpt {

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

static void ApplyEffectsInternal(const std::shared_ptr<const ObjectTypeMap>& objects,
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
      for (const VAL::simple_effect* effect : forall_effect_lists->add_effects) {
        propositions->emplace(effect->prop,
                              FilterEffectArgs(forall_action_args, &forall_action_params,
                                               effect->prop->args));
      }
      for (const VAL::simple_effect* effect : forall_effect_lists->del_effects) {
        propositions->erase(Proposition(effect->prop,
                                        FilterEffectArgs(forall_action_args,
                                                         &forall_action_params,
                                                         effect->prop->args)));
      }
    }
  }
  for (const VAL::simple_effect* effect : effects->add_effects) {
    propositions->emplace(effect->prop, FilterEffectArgs(action_args, action_params, effect->prop->args));
  }
  for (const VAL::simple_effect* effect : effects->del_effects) {
    propositions->erase(Proposition(effect->prop, FilterEffectArgs(action_args, action_params, effect->prop->args)));
  }
}

std::set<Proposition> ApplyEffects(const std::shared_ptr<const ObjectTypeMap>& objects,
                                   const std::vector<const VAL::parameter_symbol*>& action_args,
                                   const VAL::var_symbol_list* action_params,
                                   const VAL::effect_lists* effects,
                                   const std::set<Proposition>& propositions) {
  std::set<Proposition> new_propositions(propositions);
  ApplyEffectsInternal(objects, action_args, action_params, effects, &new_propositions);
  return new_propositions;
}

}  // namespace LogicOpt

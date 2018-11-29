/**
 * actions.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_PLANNING_ACTIONS_H_
#define TRAJ_OPT_PLANNING_ACTIONS_H_

#include <memory>  // std::shared_ptr
#include <vector>  // std::vector
#include <set>     // std::set

#include "ptree.h"

#include "TrajOpt/planning/objects.h"
#include "TrajOpt/planning/proposition.h"

namespace TrajOpt {

std::vector<const VAL::parameter_symbol*> FilterEffectArgs(const std::vector<const VAL::parameter_symbol*>& action_args,
                                                           const VAL::var_symbol_list* action_params,
                                                           const VAL::parameter_symbol_list* effect_params);

std::set<Proposition> ApplyEffects(const std::shared_ptr<const ObjectTypeMap>& objects,
                                   const std::vector<const VAL::parameter_symbol*>& action_args,
                                   const VAL::var_symbol_list* action_params,
                                   const VAL::effect_lists* effects,
                                   const std::set<Proposition>& propositions);

}  // namespace TrajOpt

#endif  // TRAJ_OPT_PLANNING_ACTIONS_H_

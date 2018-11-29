/**
 * proposition.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_PLANNING_PROPOSITION_H_
#define TRAJ_OPT_PLANNING_PROPOSITION_H_

#include <ostream>  // std::ostream
#include <vector>   // std::vector
#include <set>      // std::set
#include <string>   // std::string

#include "ptree.h"

namespace TrajOpt {

class Proposition {

 public:

  Proposition() {}

  Proposition(const VAL::proposition* predicate,
              std::vector<const VAL::parameter_symbol*>&& a_variables);

  Proposition(const std::string& name_predicate,
              std::vector<const VAL::parameter_symbol*>&& a_variables);

  Proposition(const std::string& name_predicate,
              const std::vector<const VAL::parameter_symbol*>& a_variables);

  // Properties
  const std::string& predicate() const { return predicate_; }
  const std::vector<const VAL::parameter_symbol*>& variables() const { return variables_; }

  // Operators
  bool operator<(const Proposition& rhs) const;
  bool operator==(const Proposition& rhs) const;

 private:

  std::string predicate_;
  std::vector<const VAL::parameter_symbol*> variables_;

};

std::ostream& operator<<(std::ostream& os, const TrajOpt::Proposition& P);

std::set<Proposition> CreateInitialPropositions(const VAL::effect_lists* initial_state,
                                                const VAL::const_symbol_list* constants,
                                                const VAL::const_symbol_list* objects);

}  // namespace TrajOpt

#endif  // TRAJ_OPT_PLANNING_PROPOSITION_H_

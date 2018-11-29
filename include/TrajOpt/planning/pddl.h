/**
 * pddl.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_PLANNING_PDDL_H_
#define TRAJ_OPT_PLANNING_PDDL_H_

#include <iostream>  // std::ostream
#include <memory>    // std::unique_ptr

#include "ptree.h"

namespace TrajOpt {

std::unique_ptr<VAL::analysis> ParsePddl(const char* filename_domain, const char* filename_problem);

void Validate(const std::unique_ptr<VAL::analysis>& analysis, bool verbose = false,
              std::ostream& os = std::cout);

}  // namespace TrajOpt

std::ostream& operator<<(std::ostream& os, const VAL::domain& domain);

std::ostream& operator<<(std::ostream& os, const VAL::problem& problem);

std::ostream& operator<<(std::ostream& os, const VAL::simple_effect& effect);

std::ostream& operator<<(std::ostream& os, const VAL::var_symbol_list& args);

std::ostream& operator<<(std::ostream& os, const VAL::parameter_symbol_list& args);

#endif  // TRAJ_OPT_PLANNING_PDDL_H_

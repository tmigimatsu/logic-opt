/**
 * pddl.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 21, 2018
 * Authors: Toki Migimatsu
 */

#include <iostream>  // std::cout
#include <set>       // std::set
#include <vector>    // std::vector

#include "TrajOpt/planning/breadth_first_search.h"
#include "TrajOpt/planning/depth_first_search.h"
#include "TrajOpt/planning/pddl.h"
#include "TrajOpt/planning/planner.h"

namespace {

struct Args {
  char* filename_domain = nullptr;
  char* filename_problem = nullptr;
};

Args ParseArgs(int argc, char *argv[]) {
  Args parsed_args;
  int i;
  std::string arg;
  for (i = 1; i < argc; i++) {
    arg = argv[i];
    if (parsed_args.filename_domain == nullptr) {
      parsed_args.filename_domain = argv[i];
    } else if (parsed_args.filename_problem == nullptr) {
      parsed_args.filename_problem = argv[i];
    } else {
      break;
    }
  }

  if (parsed_args.filename_domain == nullptr || parsed_args.filename_problem == nullptr) {
    throw std::invalid_argument("ParseArgs(): PDDL domain and problem files required.");
  }
  if (i != argc) throw std::invalid_argument("ParseArgs(): Invalid '" + arg + "' argument.");
  return parsed_args;
}

}  // namespace

int main(int argc, char* argv[]) {
  Args args = ParseArgs(argc, argv);

  std::unique_ptr<VAL::analysis> analysis = TrajOpt::ParsePddl(args.filename_domain, args.filename_problem);

  // Type check
  TrajOpt::Validate(analysis, false);

  // Output the errors from all input files
  analysis->error_list.report();
  std::cout << std::endl;

  std::cout << *analysis->the_domain << std::endl;
  std::cout << *analysis->the_problem << std::endl;

  TrajOpt::Planner planner(analysis->the_domain, analysis->the_problem);

  TrajOpt::BreadthFirstSearch<TrajOpt::Planner::Node> bfs(planner.root(), 5);
  for (const std::vector<TrajOpt::Planner::Node>& plan : bfs) {
    for (const TrajOpt::Planner::Node& node : plan) {
      std::cout << node << std::endl;
    }
    std::cout << std::endl;
  }

  TrajOpt::DepthFirstSearch<TrajOpt::Planner::Node> dfs(planner.root(), 5);
  for (const std::vector<TrajOpt::Planner::Node>& plan : dfs) {
    for (const TrajOpt::Planner::Node& node : plan) {
      std::cout << node << std::endl;
    }
    std::cout << std::endl;
  }
}

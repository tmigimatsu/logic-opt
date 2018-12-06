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

#include "LogicOpt/planning/a_star.h"
#include "LogicOpt/planning/breadth_first_search.h"
#include "LogicOpt/planning/depth_first_search.h"
#include "LogicOpt/planning/pddl.h"
#include "LogicOpt/planning/planner.h"

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

  std::unique_ptr<VAL::analysis> analysis = LogicOpt::ParsePddl(args.filename_domain, args.filename_problem);

  // Type check
  LogicOpt::Validate(analysis, false);

  // Output the errors from all input files
  analysis->error_list.report();
  std::cout << std::endl;

  std::cout << *analysis->the_domain << std::endl;
  std::cout << *analysis->the_problem << std::endl;

  LogicOpt::Planner planner(analysis->the_domain, analysis->the_problem);

  LogicOpt::BreadthFirstSearch<LogicOpt::Planner::Node> bfs(planner.root(), 5);
  for (const std::vector<LogicOpt::Planner::Node>& plan : bfs) {
    for (const LogicOpt::Planner::Node& node : plan) {
      std::cout << node << std::endl;
    }
    std::cout << std::endl;
  }

  // LogicOpt::DepthFirstSearch<LogicOpt::Planner::Node> dfs(planner.root(), 5);
  // for (const std::vector<LogicOpt::Planner::Node>& plan : dfs) {
  //   for (const LogicOpt::Planner::Node& node : plan) {
  //     std::cout << node << std::endl;
  //   }
  //   std::cout << std::endl;
  // }

  auto Heuristic = [](const LogicOpt::SearchNode<LogicOpt::Planner::Node>& left,
                      const LogicOpt::SearchNode<LogicOpt::Planner::Node>& right) -> bool {
    return left.ancestors.size() > right.ancestors.size();
  };
  LogicOpt::AStar<LogicOpt::Planner::Node, decltype(Heuristic)> astar(Heuristic, planner.root(), 5);
  for (const std::vector<LogicOpt::Planner::Node>& plan : astar) {
    // for (const LogicOpt::Planner::Node& node : plan) {
    //   std::cout << node << std::endl;
    // }
    // std::cout << std::endl;
  }


  // std::vector<std::vector<int>> options = {{11,12},{21,22,23},{31,32}};
  // LogicOpt::CombinationGenerator<std::vector<int>> gen({{11,12},{21,22,23},{31,32}});
  // for (auto it = gen.begin(); it != gen.end(); ++it) {
  //   const auto& values = *it;
  //   for (int v : values) {
  //     std::cout << v << " ";
  //   }
  //   std::cout << std::endl;
  // }
  // std::cout << std::endl;
  // for (auto it = gen.crbegin(); it != gen.crend(); ++it) {
  //   for (int v : *it) {
  //     std::cout << v << " ";
  //   }
  //   std::cout << std::endl;
  // }
}

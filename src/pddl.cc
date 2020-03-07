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

#include "logic_opt/planning/a_star.h"
#include "logic_opt/planning/breadth_first_search.h"
#include "logic_opt/planning/depth_first_search.h"
#include "logic_opt/planning/pddl.h"
#include "logic_opt/planning/planner.h"
#include "logic_opt/planning/validator.h"

namespace {

struct Args {
  std::string filename_domain;
  std::string filename_problem;
};

Args ParseArgs(int argc, char *argv[]) {
  Args parsed_args;
  int i;
  std::string arg;
  for (i = 1; i < argc; i++) {
    arg = argv[i];
    if (parsed_args.filename_domain.empty()) {
      parsed_args.filename_domain = argv[i];
    } else if (parsed_args.filename_problem.empty()) {
      parsed_args.filename_problem = argv[i];
    } else {
      break;
    }
  }

  if (parsed_args.filename_domain.empty() || parsed_args.filename_problem.empty()) {
    throw std::invalid_argument("ParseArgs(): PDDL domain and problem files required.");
  }
  if (i != argc) throw std::invalid_argument("ParseArgs(): Invalid '" + arg + "' argument.");
  return parsed_args;
}

}  // namespace

int main(int argc, char* argv[]) {
  Args args = ParseArgs(argc, argv);

  std::unique_ptr<VAL::analysis> analysis = logic_opt::ParsePddl(args.filename_domain, args.filename_problem);

  // Type check
  logic_opt::Validate(analysis, false);

  // Output the errors from all input files
  analysis->error_list.report();
  std::cout << std::endl;

  std::cout << *analysis->the_domain << std::endl;
  std::cout << *analysis->the_problem << std::endl;

  logic_opt::Planner planner(analysis->the_domain, analysis->the_problem);

  logic_opt::BreadthFirstSearch<logic_opt::Planner::Node> bfs(planner.root(), 14);
  for (const std::vector<logic_opt::Planner::Node>& plan : bfs) {
    for (const logic_opt::Planner::Node& node : plan) {
      std::cout << node << std::endl;
    }
    std::cout << std::endl;
  }

  // logic_opt::DepthFirstSearch<logic_opt::Planner::Node> dfs(planner.root(), 5);
  // for (const std::vector<logic_opt::Planner::Node>& plan : dfs) {
  //   for (const logic_opt::Planner::Node& node : plan) {
  //     std::cout << node << std::endl;
  //   }
  //   std::cout << std::endl;
  // }

  // auto Heuristic = [](const logic_opt::SearchNode<logic_opt::Planner::Node>& left,
  //                     const logic_opt::SearchNode<logic_opt::Planner::Node>& right) -> bool {
  //   return left.ancestors.size() > right.ancestors.size();
  // };
  // logic_opt::AStar<logic_opt::Planner::Node, decltype(Heuristic)> astar(Heuristic, planner.root(), 5);
  // for (const std::vector<logic_opt::Planner::Node>& plan : astar) {
  //   // for (const logic_opt::Planner::Node& node : plan) {
  //   //   std::cout << node << std::endl;
  //   // }
  //   // std::cout << std::endl;
  // }


  // std::vector<std::vector<int>> options = {{11,12},{21,22,23},{31,32}};
  // logic_opt::CombinationGenerator<std::vector<int>> gen({{11,12},{21,22,23},{31,32}});
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

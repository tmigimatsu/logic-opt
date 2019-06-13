/**
 * pddl.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/planning/pddl.h"

#include <fstream>  // std::ifstream
#include <string>   // std::string

#include "FlexLexer.h"
#include "typecheck.h"

extern int yyparse();
extern int yydebug;

namespace VAL {

// Expected in pddl+.cpp
parse_category* top_thing = nullptr;
analysis* current_analysis = nullptr;
yyFlexLexer* yfl = nullptr;

// Expected in typechecker.cpp
bool Verbose = false;
std::ostream* report = &std::cout;

}  // namespace VAL

char* current_filename = nullptr;  // Expected in parse_error.h

namespace logic_opt {

std::unique_ptr<VAL::analysis> ParsePddl(const std::string& filename_domain,
                                         const std::string& filename_problem) {
  std::unique_ptr<VAL::analysis> analysis = std::make_unique<VAL::analysis>();
  yyFlexLexer yfl;

  VAL::current_analysis = analysis.get();
  VAL::yfl = &yfl;
  yydebug = 0;  // Set to 1 to output yacc trace

  // Parse domain
  current_filename = const_cast<char*>(filename_domain.c_str());
  std::ifstream pddl_domain(filename_domain);
  yfl.switch_streams(&pddl_domain, &std::cout);
  yyparse();
  if (analysis->the_domain == nullptr) {
    throw std::runtime_error("ParsePddl(): Unable to parse domain from file: " + std::string(filename_domain));
  }

  // Parse problem
  current_filename = const_cast<char*>(filename_problem.c_str());
  std::ifstream pddl_problem(filename_problem);
  yfl.switch_streams(&pddl_problem, &std::cout);
  yyparse();
  if (analysis->the_domain == nullptr) {
    throw std::runtime_error("ParsePddl(): Unable to parse problem from file: " + std::string(filename_problem));
  }

  return analysis;
}

void Validate(const std::unique_ptr<VAL::analysis>& analysis, bool verbose, std::ostream& os) {
  VAL::Verbose = verbose;
  VAL::report = &os;

  VAL::TypeChecker tc(analysis.get());
  tc.typecheckDomain();
  tc.typecheckProblem();
}

}  // namespace logic_opt

namespace {

void PrintGoal(std::ostream& os, const VAL::goal* goal, size_t depth) {
  std::string padding(depth, '\t');

  // Proposition
  const VAL::simple_goal* simple_goal = dynamic_cast<const VAL::simple_goal*>(goal);
  if (simple_goal != nullptr) {
    const VAL::proposition* prop = simple_goal->getProp();
    os << padding << prop->head->getName() << *prop->args << " [" << prop << "]" << std::endl;
    return;
  }

  // Conjunction
  const VAL::conj_goal* conj_goal = dynamic_cast<const VAL::conj_goal*>(goal);
  if (conj_goal != nullptr) {
    os << padding << "and:" << std::endl;
    for (const VAL::goal* g : *conj_goal->getGoals()) {
      PrintGoal(os, g, depth + 1);
    }
    return;
  }

  // Negation
  const VAL::neg_goal* neg_goal = dynamic_cast<const VAL::neg_goal*>(goal);
  if (neg_goal != nullptr) {
    os << padding << "neg:" << std::endl;
    PrintGoal(os, neg_goal->getGoal(), depth + 1);
    return;
  }

  // Quantification
  const auto qfied_goal = dynamic_cast<const VAL::qfied_goal*>(goal);
  if (qfied_goal != nullptr) {
    std::string quantifier;
    switch (qfied_goal->getQuantifier()) {
      case VAL::quantifier::E_FORALL:
        quantifier = "forall";
        break;
      case VAL::quantifier::E_EXISTS:
        quantifier = "exists";
        break;
    }
    os << padding << quantifier << *qfied_goal->getVars() << ":" << std::endl;
    PrintGoal(os, qfied_goal->getGoal(), depth + 1);
    return;
  }

  const auto con_goal = dynamic_cast<const VAL::con_goal*>(goal);
  const auto constraint_goal = dynamic_cast<const VAL::constraint_goal*>(goal);
  const auto preference = dynamic_cast<const VAL::preference*>(goal);
  const auto disj_goal = dynamic_cast<const VAL::disj_goal*>(goal);
  const auto imply_goal = dynamic_cast<const VAL::imply_goal*>(goal);
  const auto timed_goal = dynamic_cast<const VAL::timed_goal*>(goal);
  const auto comparison = dynamic_cast<const VAL::comparison*>(goal);
  os << "con_goal: " << con_goal << std::endl;
  os << "constraint_goal: " << constraint_goal << std::endl;
  os << "preference: " << preference << std::endl;
  os << "disj_goal: " << disj_goal << std::endl;
  os << "imply_goal: " << imply_goal << std::endl;
  os << "timed_goal: " << timed_goal << std::endl;
  os << "comparison: " << comparison << std::endl;

  throw std::runtime_error("PrintGoal(): Goal type not implemented.");
}

void PrintEffects(std::ostream& os, const VAL::effect_lists* effects, size_t depth) {
  std::string padding(depth, '\t');
  for (const VAL::simple_effect* effect : effects->add_effects) {
    os << padding << "(+) " << *effect << std::endl;
  }
  for (const VAL::simple_effect* effect : effects->del_effects) {
    os << padding << "(-) " << *effect << std::endl;
  }
  for (const VAL::forall_effect* effect : effects->forall_effects) {
    const VAL::var_symbol_table* vars = effect->getVars();
    os << padding << "forall" << *effect->getVarsList() << ":" << std::endl;
    PrintEffects(os, effect->getEffects(), depth + 1);
  }
  for (const VAL::cond_effect* effect : effects->cond_effects) {
    os << padding << "when:" << std::endl;
    PrintGoal(os, effect->getCondition(), depth + 1);
    os << padding << "then:" << std::endl;
    PrintEffects(os, effect->getEffects(), depth + 1);
  }
}

template<typename T>
void PrintArgs(std::ostream& os, const VAL::typed_symbol_list<T>& args) {
  std::string separator;
  os << "(";
  for (const VAL::parameter_symbol* param : args) {
    os << separator << param->getName() << " [" << param << "]: " << param->type->getName();
    if (separator.empty()) separator = ", ";
  }
  os << ")";
}

}  // namespace

namespace VAL {

std::ostream& operator<<(std::ostream& os, const VAL::domain& domain) {
  os << "DOMAIN" << std::endl;
  os << "======" << std::endl;
  os << "Name: " << domain.name << std::endl;

  os << "Requirements: " << VAL::pddl_req_flags_string(domain.req) << std::endl;

  os << "Types: " << std::endl;
  if (domain.types != nullptr) {
    for (const VAL::pddl_type* type : *domain.types) {
      os << "\t" << type->getName() << ": " << type->type->getName() << " [" << type << "]" << std::endl;
    }
  }

  os << "Constants: " << std::endl;
  if (domain.constants != nullptr) {
    for (const VAL::const_symbol* c : *domain.constants) {
      os << "\t" << c->getName() << " [" << c << "]" << ": " << c->type->getName() << std::endl;
    }
  }

  os << "Predicates:" << std::endl;
  if (domain.predicates != nullptr) {
    for (const VAL::pred_decl* pred : *domain.predicates) {
      os << "\t" << pred->getPred()->getName() << *pred->getArgs() << " [" << pred << "]" << std::endl;
    }
  }

  os << "Actions: " << std::endl;
  if (domain.ops != nullptr) {
    for (const VAL::operator_* op : *domain.ops) {
      os << "\t" << op->name->getName() << *op->parameters << std::endl;

      os << "\t\tPreconditions:" << std::endl;
      PrintGoal(os, op->precondition, 3);

      os << "\t\tEffects:" << std::endl;
      PrintEffects(os, op->effects, 3);
    }
  }

  return os;
}

std::ostream& operator<<(std::ostream& os, const VAL::problem& problem) {
  os << "PROBLEM" << std::endl;
  os << "=======" << std::endl;
  os << "Name: " << problem.name << std::endl;

  os << "Domain: " << problem.domain_name << std::endl;

  os << "Requirements: " << VAL::pddl_req_flags_string(problem.req) << std::endl;

  os << "Objects:" << std::endl;
  for (const VAL::const_symbol* object : *problem.objects) {
    os << "\t" << object->getName() << " [" << object << "]" << ": " << object->type->getName() << std::endl;
  }

  os << "Initial State:" << std::endl;
  PrintEffects(os, problem.initial_state, 1);

  os << "Goal:" << std::endl;
  PrintGoal(os, problem.the_goal, 1);

  return os;
}

std::ostream& operator<<(std::ostream& os, const VAL::simple_effect& effect) {
  os << effect.prop->head->getName() << *effect.prop->args << " [" << effect.prop->head << "]";
  return os;
}

std::ostream& operator<<(std::ostream& os, const VAL::var_symbol_list& args) {
  PrintArgs(os, args);
  return os;
}

std::ostream& operator<<(std::ostream& os, const VAL::parameter_symbol_list& args) {
  PrintArgs(os, args);
  return os;
}

}  // namespace VAL

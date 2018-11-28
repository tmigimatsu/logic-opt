/**
 * pddl.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 21, 2018
 * Authors: Toki Migimatsu
 */

#include <cstdio>
#include <iostream>  // std::cout
#include <fstream>   // std::ifstream
#include <set>       // std::set
#include <sstream>   // std::stringstream

#include "FlexLexer.h"
#include "ptree.h"
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

struct Args {
  char* filename_domain = nullptr;
  char* filename_problem = nullptr;
};

static Args ParseArgs(int argc, char *argv[]) {
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

static std::unique_ptr<VAL::analysis> ParsePddl(const char* filename_domain, const char* filename_problem) {
  std::unique_ptr<VAL::analysis> analysis = std::make_unique<VAL::analysis>();
  yyFlexLexer yfl;

  VAL::current_analysis = analysis.get();
  VAL::yfl = &yfl;
  current_filename = const_cast<char*>(filename_domain);
  yydebug = 0;  // Set to 1 to output yacc trace

  // Loop over given args
  std::cout << "Domain file: " << filename_domain << std::endl;
  std::ifstream pddl_domain(filename_domain);
  yfl.switch_streams(&pddl_domain, &std::cout);
  yyparse();

  std::cout << "Problem file: " << filename_problem << std::endl;
  std::ifstream pddl_problem(filename_problem);
  yfl.switch_streams(&pddl_problem, &std::cout);
  yyparse();

  std::cout << std::endl;

  return analysis;
}

template<typename T>
static void PrintArgs(const VAL::typed_symbol_list<T>* args) {
  std::stringstream ss;
  std::string separator;
  ss << "(";
  for (const VAL::parameter_symbol* param : *args) {
    ss << separator << param->getName() << " [" << param << "]: " << param->type->getName();
    if (separator.empty()) separator = ", ";
  }
  ss << ")";
  std::cout << ss.str();
}

static void PrintGoal(const VAL::goal* goal, size_t depth = 0) {
  std::string padding(depth, '\t');

  // Proposition
  const VAL::simple_goal* simple_goal = dynamic_cast<const VAL::simple_goal*>(goal);
  if (simple_goal != nullptr) {
    const VAL::proposition* prop = simple_goal->getProp();
    std::cout << padding << prop->head->getName();
    PrintArgs(prop->args);
    std::cout << " [" << prop << "]" << std::endl;
    return;
  }

  // Conjunction
  const VAL::conj_goal* conj_goal = dynamic_cast<const VAL::conj_goal*>(goal);
  if (conj_goal != nullptr) {
    std::cout << padding << "and:" << std::endl;
    for (const VAL::goal* g : *conj_goal->getGoals()) {
      PrintGoal(g, depth + 1);
    }
    return;
  }

  // Negation
  const VAL::neg_goal* neg_goal = dynamic_cast<const VAL::neg_goal*>(goal);
  if (neg_goal != nullptr) {
    std::cout << padding << "neg:" << std::endl;
    PrintGoal(neg_goal->getGoal(), depth + 1);
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
    std::cout << padding << quantifier;
    PrintArgs(qfied_goal->getVars());
    std::cout << ":" << std::endl;
    PrintGoal(qfied_goal->getGoal(), depth + 1);
    return;
  }

  const auto con_goal = dynamic_cast<const VAL::con_goal*>(goal);
  const auto constraint_goal = dynamic_cast<const VAL::constraint_goal*>(goal);
  const auto preference = dynamic_cast<const VAL::preference*>(goal);
  const auto disj_goal = dynamic_cast<const VAL::disj_goal*>(goal);
  const auto imply_goal = dynamic_cast<const VAL::imply_goal*>(goal);
  const auto timed_goal = dynamic_cast<const VAL::timed_goal*>(goal);
  const auto comparison = dynamic_cast<const VAL::comparison*>(goal);
  std::cout << "con_goal: " << con_goal << std::endl;
  std::cout << "constraint_goal: " << constraint_goal << std::endl;
  std::cout << "preference: " << preference << std::endl;
  std::cout << "disj_goal: " << disj_goal << std::endl;
  std::cout << "imply_goal: " << imply_goal << std::endl;
  std::cout << "timed_goal: " << timed_goal << std::endl;
  std::cout << "comparison: " << comparison << std::endl;

  throw std::runtime_error("GOAL NOT IMPLEMENTED");
}

static void PrintEffect(const VAL::simple_effect* effect) {
  std::cout << effect->prop->head->getName();
  PrintArgs(effect->prop->args);
  std::cout << " [" << effect->prop->head << "]" << std::endl;
}

static void PrintEffects(const VAL::effect_lists* effects, size_t depth = 0) {
  std::string padding(depth, '\t');
  for (const VAL::simple_effect* effect : effects->add_effects) {
    std::cout << padding << "(+) ";
    PrintEffect(effect);
  }
  for (const VAL::simple_effect* effect : effects->del_effects) {
    std::cout << padding << "(-) ";
    PrintEffect(effect);
  }
  for (const VAL::forall_effect* effect : effects->forall_effects) {
    const VAL::var_symbol_table* vars = effect->getVars();
    std::cout << padding << "forall";
    PrintArgs(effect->getVarsList());
    std::cout << ":" << std::endl;
    PrintEffects(effect->getEffects(), depth + 1);
  }
}

static void PrintDomain(const VAL::domain* domain) {
  std::cout << "DOMAIN" << std::endl;
  std::cout << "======" << std::endl;
  if (domain != nullptr) {
    std::cout << "Name: " << domain->name << std::endl;

    std::cout << "Requirements: " << VAL::pddl_req_flags_string(domain->req) << std::endl;

    std::cout << "Types: " << std::endl;
    if (domain->types != nullptr) {
      for (const VAL::pddl_type* type : *domain->types) {
        std::cout << "\t" << type->getName() << ": " << type->type->getName() << " [" << type << "]" << std::endl;
      }
    }

    std::cout << "Constants: " << std::endl;
    if (domain->constants != nullptr) {
      for (const VAL::const_symbol* c : *domain->constants) {
        std::cout << "\t" << c->getName() << " [" << c << "]" << ": " << c->type->getName() << std::endl;
      }
    }

    std::cout << "Predicates:" << std::endl;
    if (domain->predicates != nullptr) {
      for (const VAL::pred_decl* pred : *domain->predicates) {
        std::cout << "\t" << pred->getPred()->getName();
        PrintArgs(pred->getArgs());
        std::cout << " [" << pred << "]" << std::endl;
      }
    }

    std::cout << "Actions: " << std::endl;
    if (domain->ops != nullptr) {
      for (const VAL::operator_* op : *domain->ops) {
        std::cout << "\t" << op->name->getName();
        PrintArgs(op->parameters);
        std::cout << std::endl;

        std::cout << "\t\tPreconditions:" << std::endl;
        PrintGoal(op->precondition, 3);

        std::cout << "\t\tEffects:" << std::endl;
        PrintEffects(op->effects, 3);
      }
    }
  }
  std::cout << std::endl;
}

const void PrintProblem(const VAL::problem* problem) {
  std::cout << "PROBLEM" << std::endl;
  std::cout << "=======" << std::endl;
  if (problem != nullptr) {
    std::cout << "Name: " << problem->name << std::endl;

    std::cout << "Domain: " << problem->domain_name << std::endl;

    std::cout << "Requirements: " << VAL::pddl_req_flags_string(problem->req) << std::endl;

    std::cout << "Objects:" << std::endl;
    for (const VAL::const_symbol* object : *problem->objects) {
      std::cout << "\t" << object->getName() << " [" << object << "]" << ": " << object->type->getName() << std::endl;
    }

    std::cout << "Initial State:" << std::endl;
    PrintEffects(problem->initial_state, 1);

    std::cout << "Goal:" << std::endl;
    PrintGoal(problem->the_goal, 1);
  }
}

struct Proposition {

  Proposition(const VAL::proposition* predicate,
              std::vector<const VAL::parameter_symbol*>&& a_variables)
      : predicate(predicate->head->getName()), variables(std::move(a_variables)) {

    if (predicate->args->size() != variables.size()) {
      throw std::invalid_argument("Proposition(): " + predicate->head->getName() + "() constructed with incorrect number of arguments.");
    }

    size_t i = 0;
    for (const VAL::parameter_symbol* arg : *predicate->args) {
      if (arg->type != variables[i]->type) {
        throw std::invalid_argument("Proposition(): " + predicate->head->getName() + "() constructed with incorrect argument types.");
      }
      ++i;
    }
  }

  Proposition(const std::string& name_predicate,
              std::vector<const VAL::parameter_symbol*>&& a_variables)
      : predicate(name_predicate), variables(std::move(a_variables)) {}

  const std::string predicate;
  const std::vector<const VAL::parameter_symbol*> variables;

  bool operator<(const Proposition& rhs) const {
    if (predicate != rhs.predicate) return predicate < rhs.predicate;
    for (size_t i = 0; i < variables.size(); i++) {
      if (variables[i] != rhs.variables[i]) return variables[i] < rhs.variables[i];
    }
    return false;
  }

  bool operator==(const Proposition& rhs) const {
    if (predicate != rhs.predicate) return false;
    for (size_t i = 0; i < variables.size(); i++) {
      if (variables[i] != rhs.variables[i]) return false;
    }
    return true;
  }
};

std::ostream& operator<<(std::ostream& os, const Proposition& P) {
  os << P.predicate << "(";
  std::string separator;
  for (const VAL::parameter_symbol* param : P.variables) {
    os << separator << param->getName();
    if (separator.empty()) separator = ", ";
  }
  os << ")";
  return os;
}

struct Formula {

  using Function = std::function<bool(const std::set<Proposition>& propositions,
                                      const std::vector<const VAL::parameter_symbol*>& variables)>;

  const Function Evaluate;

  bool operator()(const std::set<Proposition>& propositions,
                  const std::vector<const VAL::parameter_symbol*>& variables) const {
    return Evaluate(propositions, variables);
  }
};

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

class ParameterGenerator {

 public:

  template<typename T>
  ParameterGenerator(const std::shared_ptr<const std::map<const VAL::pddl_type*,
                                                          std::vector<const VAL::parameter_symbol*>>>& objects,
                     const VAL::typed_symbol_list<T>* params)
      : objects_(objects) {
    types_.reserve(params->size());
    for (const VAL::parameter_symbol* param : *params) {
      types_.push_back(param->type);
    }
  }

  class iterator {

   public:

    using iterator_category = std::input_iterator_tag;
    using value_type = std::vector<const VAL::parameter_symbol*>;
    using difference_type = std::ptrdiff_t;
    using pointer = const value_type*;
    using reference = const value_type&;

    iterator(const ParameterGenerator& gen, std::vector<size_t>&& idx_objects)
        : gen_(gen), idx_objects_(std::move(idx_objects)), objects_(idx_objects_.size()) {
      for (size_t i = 0; i < idx_objects_.size(); i++) {
        const std::vector<const VAL::parameter_symbol*>& objects = gen_.objects_->at(gen_.types_[i]);
        if (idx_objects_[i] >= objects.size()) return;
        objects_[i] = objects[idx_objects_[i]];
      }
    }

    iterator& operator++() {
      // Check for end flag
      if (idx_objects_[0] >= gen_.objects_->at(gen_.types_[0]).size()) return *this;

      for (size_t i = idx_objects_.size() - 1; i >= 0; i--) {
        ++idx_objects_[i];
        if (idx_objects_[i] < gen_.objects_->at(gen_.types_[i]).size()) {
          objects_[i] = gen_.objects_->at(gen_.types_[i])[idx_objects_[i]];
          break;
        }
        // At the end, leave the first index high
        if (i == 0) return *this;

        idx_objects_[i] = 0;
        objects_[i] = gen_.objects_->at(gen_.types_[i])[idx_objects_[i]];
      }
      return *this;
    }
    bool operator==(const iterator& other) const {
      if (&gen_ != &other.gen_) return false;
      if (idx_objects_.size() != other.idx_objects_.size()) return false;
      for (size_t i = 0; i < idx_objects_.size(); i++) {
        if (idx_objects_[i] != other.idx_objects_[i]) return false;
      }
      return true;
    }
    bool operator!=(const iterator& other) const {
      return !(*this == other);
    }
    reference operator*() const {
      if (idx_objects_[0] >= gen_.objects_->at(gen_.types_[0]).size()) {
        const std::vector<const VAL::parameter_symbol*>* p_objects = nullptr;
        return *p_objects;
      }
      return objects_;
    }

   private:

    const ParameterGenerator& gen_;
    std::vector<size_t> idx_objects_;
    std::vector<const VAL::parameter_symbol*> objects_;

  };

  iterator begin() {
    return iterator(*this, std::vector<size_t>(types_.size(), 0));
  }
  iterator end() {
    std::vector<size_t> idx_objects(types_.size(), 0);
    idx_objects[0] = objects_->at(types_[0]).size();
    return iterator(*this, std::move(idx_objects));
  }

 private:

  std::shared_ptr<const std::map<const VAL::pddl_type*, std::vector<const VAL::parameter_symbol*>>> objects_;
  std::vector<const VAL::pddl_type*> types_;

};

template <typename T>
static Formula& GetFormula(const std::shared_ptr<std::map<const VAL::goal*, Formula>>& formulas,
                           const std::shared_ptr<const std::map<const VAL::pddl_type*,
                                                                std::vector<const VAL::parameter_symbol*>>>& objects,
                           const VAL::goal* goal, const VAL::typed_symbol_list<T>* action_params) {

  auto it = formulas->find(goal);
  if (it != formulas->end()) {
    return it->second;
  }

  // Proposition
  const VAL::simple_goal* simple_goal = dynamic_cast<const VAL::simple_goal*>(goal);
  if (simple_goal != nullptr) {
    const VAL::proposition* pred = simple_goal->getProp();

    Formula P{
      [pred, idx_pred_to_action = IdxPredicateToActionParams(action_params, pred->args)](
          const std::set<Proposition>& propositions,
          const std::vector<const VAL::parameter_symbol*>& action_args) -> bool {
        // Search for predicate in knowledge base
        Proposition P(pred, ActionToPredicateArgs(action_args, idx_pred_to_action));
        return propositions.find(P) != propositions.end();
      }
    };
    it = formulas->emplace(std::move(goal), std::move(P)).first;
    return it->second;
  }

  // Conjunction
  const VAL::conj_goal* conj_goal = dynamic_cast<const VAL::conj_goal*>(goal);
  if (conj_goal != nullptr) {
    const VAL::goal_list* goals = conj_goal->getGoals();

    Formula P{
      [formulas, objects, action_params, goals](
          const std::set<Proposition>& propositions,
          const std::vector<const VAL::parameter_symbol*>& action_args) -> bool {
        for (const VAL::goal* g : *goals) {
          const Formula& P = GetFormula(formulas, objects, g, action_params);
          if (!P(propositions, action_args)) return false;
        }
        return true;
      }
    };
    it = formulas->emplace(std::move(goal), std::move(P)).first;
    return it->second;
  }

  // Negation
  const VAL::neg_goal* neg_goal = dynamic_cast<const VAL::neg_goal*>(goal);
  if (neg_goal != nullptr) {
    const VAL::goal* g = neg_goal->getGoal();

    // Get positive formula
    const Formula& P = GetFormula(formulas, objects, g, action_params);

    // Create negative formula
    Formula P_neg{
      [formulas, objects, action_params, g](
          const std::set<Proposition>& propositions,
          const std::vector<const VAL::parameter_symbol*>& action_args) -> bool {
        // Negate positive formula
        const Formula& P = GetFormula(formulas, objects, g, action_params);
        return !P(propositions, action_args);
      }
    };
    it = formulas->emplace(std::move(goal), std::move(P_neg)).first;
    return it->second;
  }

  // Forall
  const VAL::qfied_goal* qfied_goal = dynamic_cast<const VAL::qfied_goal*>(goal);
  if (qfied_goal != nullptr) {
    const VAL::goal* g = qfied_goal->getGoal();
    const VAL::var_symbol_list* qfied_vars = qfied_goal->getVars();
    VAL::typed_symbol_list<T> qfied_params(*action_params);
    for (T* var : *qfied_vars) {
      qfied_params.push_back(var);
    }

    Formula::Function P;
    switch (qfied_goal->getQuantifier()) {
      case VAL::quantifier::E_FORALL:
        P = [formulas, objects, qfied_params = std::move(qfied_params), qfied_vars, g](
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
        break;
      case VAL::quantifier::E_EXISTS:
        P = [formulas, objects, qfied_params = std::move(qfied_params), g](
            const std::set<Proposition>& propositions,
            const std::vector<const VAL::parameter_symbol*>& action_args) -> bool {
          const Formula& P = GetFormula(formulas, objects, g, &qfied_params);
          ParameterGenerator gen(objects, &qfied_params);
          for (const std::vector<const VAL::parameter_symbol*>& variables : gen) {
            std::vector<const VAL::parameter_symbol*> qfied_action_args(action_args);
            qfied_action_args.insert(qfied_action_args.end(), variables.begin(), variables.end());
            if (P(propositions, qfied_action_args)) return true;
          }
          return false;
        };
        break;
    }
    it = formulas->emplace(std::move(goal), Formula{std::move(P)}).first;
    return it->second;
  }

  throw std::runtime_error("GetFormula(): Goal type not implemented.");
}

static std::vector<const VAL::parameter_symbol*> FilterEffectArgs(const std::vector<const VAL::parameter_symbol*>& action_args,
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

static void ApplyEffectsInternal(const std::shared_ptr<const std::map<const VAL::pddl_type*, std::vector<const VAL::parameter_symbol*>>>& objects,
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

static std::set<Proposition> ApplyEffects(const std::shared_ptr<const std::map<const VAL::pddl_type*, std::vector<const VAL::parameter_symbol*>>>& objects,
                                          const std::vector<const VAL::parameter_symbol*>& action_args,
                                          const VAL::var_symbol_list* action_params,
                                          const VAL::effect_lists* effects,
                                          const std::set<Proposition>& propositions) {
  std::set<Proposition> new_propositions(propositions);
  ApplyEffectsInternal(objects, action_args, action_params, effects, &new_propositions);
  return new_propositions;
}

static VAL::parameter_symbol_list CreateGoalParams(
    const std::shared_ptr<const std::map<const VAL::pddl_type*, std::vector<const VAL::parameter_symbol*>>>& objects) {
  VAL::parameter_symbol_list goal_params;
  for (const auto& key_val : *objects) {
    for (const VAL::parameter_symbol* object : key_val.second) {
      goal_params.push_back(const_cast<VAL::parameter_symbol*>(object));
    }
  }
  return goal_params;
}

static bool Search(const VAL::operator_list* operators, const VAL::goal* goal,
                   const std::shared_ptr<std::map<const VAL::goal*, Formula>>& formulas,
                   const std::shared_ptr<const std::map<const VAL::pddl_type*, std::vector<const VAL::parameter_symbol*>>>& objects,
                   const std::set<Proposition>& propositions, size_t depth = 0) {
  static const VAL::parameter_symbol_list goal_params = CreateGoalParams(objects);
  static const std::vector<const VAL::parameter_symbol*> goal_args(goal_params.begin(), goal_params.end());
  const Formula& G = GetFormula(formulas, objects, goal, &goal_params);
  if (G(propositions, goal_args)) return true;
  if (depth >= 5) return false;

  for (const VAL::operator_* op : *operators) {
    ParameterGenerator gen(objects, op->parameters);
    const Formula& P = GetFormula(formulas, objects, op->precondition, op->parameters);
    for (const std::vector<const VAL::parameter_symbol*>& action_args : gen) {
      if (!P(propositions, action_args)) continue;

      for (size_t i = 0; i < depth; i++) {
        std::cout << " ";
      }
      std::cout << op->name->getName() << "(";
      std::string separator;
      for (const VAL::parameter_symbol* arg : action_args) {
        std::cout << separator << arg->getName();
        if (separator.empty()) separator = ", ";
      }
      std::cout << ")\t";
      for (const auto& P : propositions) {
        if (P.predicate == "=") continue;
        std::cout << P << "; ";
      }
      std::cout << std::endl;

      if (Search(operators, goal, formulas, objects, ApplyEffects(objects, action_args, op->parameters, op->effects, propositions), depth + 1)) return true;
    }
  }

  return false;
}

std::shared_ptr<const std::map<const VAL::pddl_type*, std::vector<const VAL::parameter_symbol*>>>
CreateObjectsMap(const VAL::const_symbol_list* constants, const VAL::const_symbol_list* objects) {
  auto map_objects = std::make_shared<std::map<const VAL::pddl_type*, std::vector<const VAL::parameter_symbol*>>>();
  for (const VAL::const_symbol* object : *constants) {
    const VAL::parameter_symbol* param = dynamic_cast<const VAL::parameter_symbol*>(object);
    (*map_objects)[param->type].push_back(param);
  }
  for (const VAL::const_symbol* object : *objects) {
    const VAL::parameter_symbol* param = dynamic_cast<const VAL::parameter_symbol*>(object);
    (*map_objects)[param->type].push_back(param);
  }
  return map_objects;
}

std::set<Proposition> CreateInitialPropositions(const VAL::effect_lists* initial_state,
                                                const VAL::const_symbol_list* constants,
                                                const VAL::const_symbol_list* objects) {
  std::set<Proposition> propositions;
  for (const VAL::simple_effect* effect : initial_state->add_effects) {
    std::vector<const VAL::parameter_symbol*> params(effect->prop->args->begin(), effect->prop->args->end());
    propositions.emplace(effect->prop, std::move(params));
  }
  for (const VAL::const_symbol* object : *constants) {
    std::vector<const VAL::parameter_symbol*> params(2, object);
    propositions.emplace("=", std::move(params));
  }
  for (const VAL::const_symbol* object : *objects) {
    std::vector<const VAL::parameter_symbol*> params(2, object);
    propositions.emplace("=", std::move(params));
  }
  return propositions;
}

static bool Search(const VAL::domain* domain, const VAL::problem* problem) {
  auto formulas = std::make_shared<std::map<const VAL::goal*, Formula>>();
  auto objects = CreateObjectsMap(domain->constants, problem->objects);
  auto propositions = CreateInitialPropositions(problem->initial_state, domain->constants, problem->objects);

  std::cout << "Objects" << std::endl;
  for (const auto& key_val : *objects) {
    const VAL::pddl_type* type = key_val.first;
    std::cout << "  " << type->getName() << ": ";
    for (const VAL::parameter_symbol* object : key_val.second) {
      std::cout << object->getName() << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
  std::cout << "Initial propositions" << std::endl;
  for (const auto& P : propositions) {
    std::cout << "  " << P << std::endl;
  }
  std::cout << std::endl;

  bool result = Search(domain->ops, problem->the_goal, formulas, objects, propositions);
  std::cout << (result ? "Solution found!" : "No solution :(") << std::endl;
  return result;
}

int main(int argc, char* argv[]) {
  Args args = ParseArgs(argc, argv);

  std::unique_ptr<VAL::analysis> analysis = ParsePddl(args.filename_domain, args.filename_problem);

  PrintDomain(analysis->the_domain);
  PrintProblem(analysis->the_problem);

  VAL::TypeChecker tc(VAL::current_analysis);
  tc.typecheckDomain();
  tc.typecheckProblem();

  // if (VAL::top_thing) VAL::top_thing->display(0);
  // Output the errors from all input files
  analysis->error_list.report();
  std::cout << std::endl;

  Search(analysis->the_domain, analysis->the_problem);
}

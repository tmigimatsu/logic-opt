/**
 * planner.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/planning/planner.h"

#include "logic_opt/planning/actions.h"

namespace logic_opt {

namespace {

VAL::parameter_symbol_list CreateGoalParams(const std::shared_ptr<const ObjectTypeMap>& objects) {
  VAL::parameter_symbol_list goal_params;
  for (const auto& key_val : *objects) {
    for (const VAL::parameter_symbol* object : key_val.second) {
      goal_params.push_back(const_cast<VAL::parameter_symbol*>(object));
    }
  }
  return goal_params;
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

}  // namespace

Planner::Planner(const VAL::domain* domain, const VAL::problem* problem)
    : objects_(CreateObjectsMap(domain->constants, problem->objects)),
      operators_(*domain->ops),
      goal_(problem->the_goal),
      goal_params_(CreateGoalParams(objects_)),
      goal_args_(goal_params_.begin(), goal_params_.end()),
      root_(this, CreateInitialPropositions(problem->initial_state, domain->constants, problem->objects)) {}

Planner::Node::Node(const Planner* planner, std::set<Proposition>&& propositions, size_t depth)
    : planner_(planner), propositions_(std::move(propositions)), depth_(depth) {}

std::ostream& operator<<(std::ostream& os, const logic_opt::Planner::Node& node) {

  for (size_t i = 0; i < node.depth_; i++) {
    os << "-";
  }
  os << (node.depth_ > 0 ? " " : "") << node.action_ << " -> ";
  std::string separator;
  for (const auto& P : node.propositions_) {
    if (P.predicate() == "=") continue;
    os << separator << P;
    if (separator.empty()) separator = ", ";
  }

  return os;
}

Planner::Node::iterator::iterator(const Node* parent)
    : planner_(parent->planner_), parent_(parent), child_(planner_, parent->depth_ + 1),
      it_op_(planner_->operators_.begin()),
      param_gen_(ParameterGenerator(planner_->objects_, (*it_op_)->parameters)),
      it_param_(param_gen_.begin()) {}

Planner::Node::iterator& Planner::Node::iterator::operator++() {

  while (it_op_ != planner_->operators_.end()) {
    const VAL::operator_* op = *it_op_;
    if (it_param_ == param_gen_.end()) {
      // Move onto next action
      ++it_op_;
      if (it_op_ == planner_->operators_.end()) break;
      op = *it_op_;

      // Generate new parameters
      param_gen_ = ParameterGenerator(planner_->objects_, op->parameters);
      it_param_ = param_gen_.begin();
    } else {
      // Move onto next parameters
      ++it_param_;
      if (it_param_ == param_gen_.end()) continue;
    }

    // Check action preconditions
    const Formula& P = GetFormula(planner_->formulas_, planner_->objects_, op->precondition, op->parameters);
    const std::vector<const VAL::parameter_symbol*>& action_args = *it_param_;
    if (P(parent_->propositions_, action_args)) {
      // Set action and apply postconditions to child
      child_.action_ = Proposition(op->name->getName(), action_args);
      child_.propositions_ = ApplyEffects(planner_->formulas_, planner_->objects_,
                                          action_args, op->parameters,
                                          op->effects, parent_->propositions_);
      break;
    }
  }

  return *this;
}

Planner::Node::iterator& Planner::Node::iterator::operator--() {

  if (it_op_ == planner_->operators_.end()) {
    --it_op_;
    const VAL::operator_* op = *it_op_;
    param_gen_ = ParameterGenerator(planner_->objects_, op->parameters);
    it_param_ = --param_gen_.end();

    const Formula& P = GetFormula(planner_->formulas_, planner_->objects_, op->precondition, op->parameters);
    const std::vector<const VAL::parameter_symbol*>& action_args = *it_param_;
    if (P(parent_->propositions_, action_args)) {
      child_.action_ = Proposition(op->name->getName(), action_args);
      child_.propositions_ = ApplyEffects(planner_->formulas_, planner_->objects_,
                                          action_args, op->parameters,
                                          op->effects, parent_->propositions_);
      return *this;
    }
  }

  while (it_op_ != planner_->operators_.begin() || it_param_ != param_gen_.begin()) {
    const VAL::operator_* op = *it_op_;
    if (it_param_ == param_gen_.begin()) {
      // Move onto next action
      --it_op_;
      op = *it_op_;

      // Generate new parameters
      param_gen_ = ParameterGenerator(planner_->objects_, op->parameters);
      it_param_ = --param_gen_.end();
    } else {
      // Move onto next parameters
      --it_param_;
    }

    const Formula& P = GetFormula(planner_->formulas_, planner_->objects_, op->precondition, op->parameters);
    const std::vector<const VAL::parameter_symbol*>& action_args = *it_param_;
    if (P(parent_->propositions_, action_args)) {
      child_.action_ = Proposition(op->name->getName(), action_args);
      child_.propositions_ = ApplyEffects(planner_->formulas_, planner_->objects_,
                                          action_args, op->parameters,
                                          op->effects, parent_->propositions_);
      break;
    }
  }

  return *this;
}

bool Planner::Node::iterator::operator==(const iterator& other) const {
  return it_op_ == planner_->operators_.end() && other.it_op_ == other.planner_->operators_.end();
}

Planner::Node::iterator Planner::Node::begin() const {
  iterator it(this);
  if (it == end()) return it;

  const VAL::operator_* op = *it.it_op_;
  const Formula& P = GetFormula(planner_->formulas_, planner_->objects_, op->precondition, op->parameters);
  const std::vector<const VAL::parameter_symbol*>& action_args = *it.it_param_;
  if (P(propositions_, action_args)) {
    it.child_.action_ = Proposition(op->name->getName(), action_args);
    it.child_.propositions_ = ApplyEffects(planner_->formulas_, planner_->objects_,
                                           action_args, op->parameters,
                                           op->effects, propositions_);
    return it;
  }
  ++it;
  return it;
}

Planner::Node::iterator Planner::Node::end() const {
  iterator it(this);
  it.it_op_ = planner_->operators_.end();
  return it;
}

Planner::Node::operator bool() const {
  const Formula& G = GetFormula(planner_->formulas_, planner_->objects_, planner_->goal_, &planner_->goal_params_);
  return G(propositions_, planner_->goal_args_);
}

}  // namespace logic_opt

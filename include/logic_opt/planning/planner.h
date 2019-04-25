/**
 * planner.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_PLANNING_PLANNER_H_
#define LOGIC_OPT_PLANNING_PLANNER_H_

#include <memory>    // std::shared_ptr
#include <iostream>  // std::ostream
#include <set>       // std::ostream
#include <vector>    // std::vector

#include "ptree.h"

#include "logic_opt/planning/formula.h"
#include "logic_opt/planning/objects.h"
#include "logic_opt/planning/proposition.h"

namespace logic_opt {

class Planner {

 public:

  class Node {

   public:

    class iterator;
    class reverse_iterator;

    Node(const Planner* planner, size_t depth) : planner_(planner), depth_(depth) {}
    Node(const Planner* planner, std::set<Proposition>&& propositions, size_t depth = 0);

    const Proposition& action() const { return action_; }

    iterator begin() const;
    iterator end() const;

    explicit operator bool() const;

   private:

    const Planner* planner_;
    Proposition action_;
    std::set<Proposition> propositions_;
    size_t depth_;

    friend std::ostream& operator<<(std::ostream&, const Planner::Node&);

  };

  Planner(const VAL::domain* domain, const VAL::problem* problem);

  const Node& root() const { return root_; }

  const ObjectTypeMap& objects() const { return *objects_; }

 private:

  mutable FormulaMap formulas_;
  const std::shared_ptr<const ObjectTypeMap> objects_;
  const VAL::operator_list& operators_;
  const VAL::goal* goal_;
  const VAL::parameter_symbol_list goal_params_;
  const std::vector<const VAL::parameter_symbol*> goal_args_;

  Node root_;

};

std::ostream& operator<<(std::ostream& os, const logic_opt::Planner::Node& node);

class Planner::Node::iterator {

 public:

  using iterator_category = std::input_iterator_tag;
  using value_type = Node;
  using difference_type = ptrdiff_t;
  using pointer = const value_type*;
  using reference = const value_type&;

  iterator(const Node* parent);

  iterator& operator++();
  iterator& operator--();
  bool operator==(const iterator& other) const;
  bool operator!=(const iterator& other) const { return !(*this == other); }
  reference operator*() const { return child_; }

 private:

  const Planner* planner_;

  const Node* parent_;
  Node child_;

  VAL::operator_list::const_iterator it_op_;

  ParameterGenerator param_gen_;
  ParameterGenerator::const_iterator it_param_;

  friend class Node;

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_PLANNING_PLANNER_H_

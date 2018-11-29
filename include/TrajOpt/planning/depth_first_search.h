/**
 * depth_first_search.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_PLANNING_DEPTH_FIRST_SEARCH_H_
#define TRAJ_OPT_PLANNING_DEPTH_FIRST_SEARCH_H_

#include <cstddef>   // ptrdiff_t
#include <iterator>  // std::input_iterator_tag
#include <stack>     // std::stack
#include <vector>    // std::vector
#include <utility>   // std::pair

namespace TrajOpt {

template<typename NodeT>
class DepthFirstSearch {

 public:

  class iterator;

  DepthFirstSearch(const NodeT& root, size_t max_depth) : kMaxDepth(max_depth), root_(root) {}

  iterator begin() { iterator it(root_, kMaxDepth); return ++it; }
  iterator end() { return iterator(); }

 private:

  const size_t kMaxDepth;

  const NodeT& root_;

};

template<typename NodeT>
class DepthFirstSearch<NodeT>::iterator {

 public:

  using iterator_category = std::input_iterator_tag;
  using value_type = std::vector<NodeT>;
  using difference_type = ptrdiff_t;
  using pointer = const value_type*;
  using reference = const value_type&;

  iterator() {}
  iterator(const NodeT& root, size_t max_depth)
      : stack_({{root, std::vector<NodeT>()}}), kMaxDepth(max_depth) {}

  iterator& operator++();
  bool operator==(const iterator& other) const { return stack_.empty() && other.stack_.empty(); }
  bool operator!=(const iterator& other) const { return !(*this == other); }
  reference operator*() const { return ancestors_; }

 private:

  const size_t kMaxDepth = 0;
  std::stack<std::pair<NodeT, std::vector<NodeT>>> stack_;
  std::vector<NodeT> ancestors_;

};

template<typename NodeT>
typename DepthFirstSearch<NodeT>::iterator& DepthFirstSearch<NodeT>::iterator::operator++() {
  while (!stack_.empty()) {
    std::pair<NodeT, std::vector<NodeT>>& top = stack_.top();

    // Take ancestors list and append current node
    ancestors_.swap(top.second);
    ancestors_.push_back(std::move(top.first));
    stack_.pop();

    // Return if node evaluates to true
    const NodeT& node = ancestors_.back();
    if (node) break;

    // Skip children if max depth has been reached
    if (ancestors_.size() > kMaxDepth) continue;

    // Add node's children to stack
    for (const NodeT& child : node) {
      stack_.emplace(child, ancestors_);
    }
  }
  return *this;
}

}  // namespace TrajOpt

#endif  // TRAJ_OPT_PLANNING_DEPTH_FIRST_SEARCH_H_

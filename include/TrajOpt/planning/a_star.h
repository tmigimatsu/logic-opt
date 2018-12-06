/**
 * a_star.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 29, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_PLANNING_A_STAR_H_
#define TRAJ_OPT_PLANNING_A_STAR_H_

#include <cstddef>   // ptrdiff_t
#include <iterator>  // std::input_iterator_tag
#include <queue>     // std::priority_queue
#include <vector>    // std::vector

namespace TrajOpt {

template<typename NodeT>
struct SearchNode {

  SearchNode(const NodeT& node, const std::vector<NodeT>& ancestors)
      : node(node), ancestors(ancestors) {}

  SearchNode(NodeT&& node, std::vector<NodeT>&& ancestors)
      : node(std::move(node)), ancestors(std::move(ancestors)) {}

  NodeT node;
  std::vector<NodeT> ancestors;

};

template<typename NodeT, typename Compare>
class AStar {

 public:

  class iterator;

  AStar(const Compare& compare, const NodeT& root, size_t max_depth)
      : kMaxDepth(max_depth), compare_(compare), root_(root) {}

  iterator begin() { iterator it(compare_, root_, kMaxDepth); return ++it; }
  iterator end() { return iterator(compare_); }

 private:

  const size_t kMaxDepth;

  const Compare& compare_;
  const NodeT& root_;

};

template<typename NodeT, typename Compare>
class AStar<NodeT, Compare>::iterator {

 public:

  using iterator_category = std::input_iterator_tag;
  using value_type = std::vector<NodeT>;
  using difference_type = ptrdiff_t;
  using pointer = const value_type*;
  using reference = const value_type&;

  iterator(const Compare& compare) : queue_(compare) {}
  iterator(const Compare& compare, const NodeT& root, size_t max_depth)
      : queue_(compare), kMaxDepth(max_depth) { queue_.emplace(root, std::vector<NodeT>()); }

  iterator& operator++();
  bool operator==(const iterator& other) const { return queue_.empty() && other.queue_.empty(); }
  bool operator!=(const iterator& other) const { return !(*this == other); }
  reference operator*() const { return ancestors_; }

 private:

  const size_t kMaxDepth = 0;

  std::priority_queue<SearchNode<NodeT>, std::vector<SearchNode<NodeT>>, Compare> queue_;
  std::vector<NodeT> ancestors_;

};

template<typename NodeT, typename Compare>
typename AStar<NodeT, Compare>::iterator& AStar<NodeT, Compare>::iterator::operator++() {
  while (!queue_.empty()) {
    SearchNode<NodeT> top = queue_.top();

    // Take ancestors list and append current node
    ancestors_.swap(top.ancestors);
    ancestors_.push_back(std::move(top.node));
    queue_.pop();

    // Return if node evaluates to true
    const NodeT& node = ancestors_.back();
    std::cout << node << std::endl;
    if (node) break;

    // Skip children if max depth has been reached
    if (ancestors_.size() > kMaxDepth) continue;

    // Add node's children to queue
    for (const NodeT& child : node) {
      queue_.emplace(child, ancestors_);
    }
  }
  return *this;
}

}  // namespace TrajOpt

#endif  // TRAJ_OPT_PLANNING_A_STAR_H_

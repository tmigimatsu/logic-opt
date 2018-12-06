/**
 * breadth_first_search.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 29, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_PLANNING_BREADTH_FIRST_SEARCH_H_
#define LOGIC_OPT_PLANNING_BREADTH_FIRST_SEARCH_H_

#include <cstddef>   // ptrdiff_t
#include <iterator>  // std::input_iterator_tag
#include <queue>     // std::queue
#include <vector>    // std::vector
#include <utility>   // std::pair

namespace LogicOpt {

template<typename NodeT>
class BreadthFirstSearch {

 public:

  class iterator;

  BreadthFirstSearch(const NodeT& root, size_t max_depth) : kMaxDepth(max_depth), root_(root) {}

  iterator begin() { iterator it(root_, kMaxDepth); return ++it; }
  iterator end() { return iterator(); }

 private:

  const size_t kMaxDepth;

  const NodeT& root_;

};

template<typename NodeT>
class BreadthFirstSearch<NodeT>::iterator {

 public:

  using iterator_category = std::input_iterator_tag;
  using value_type = std::vector<NodeT>;
  using difference_type = ptrdiff_t;
  using pointer = const value_type*;
  using reference = const value_type&;

  iterator() {}
  iterator(const NodeT& root, size_t max_depth)
      : queue_({{root, std::vector<NodeT>()}}), kMaxDepth(max_depth) {}

  iterator& operator++();
  bool operator==(const iterator& other) const { return queue_.empty() && other.queue_.empty(); }
  bool operator!=(const iterator& other) const { return !(*this == other); }
  reference operator*() const { return ancestors_; }

 private:

  const size_t kMaxDepth = 0;

  std::queue<std::pair<NodeT, std::vector<NodeT>>> queue_;
  std::vector<NodeT> ancestors_;

};

template<typename NodeT>
typename BreadthFirstSearch<NodeT>::iterator& BreadthFirstSearch<NodeT>::iterator::operator++() {
  while (!queue_.empty()) {
    std::pair<NodeT, std::vector<NodeT>>& front = queue_.front();

    // Take ancestors list and append current node
    ancestors_.swap(front.second);
    ancestors_.push_back(std::move(front.first));
    queue_.pop();

    // Return if node evaluates to true
    const NodeT& node = ancestors_.back();
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

}  // namespace LogicOpt

#endif  // LOGIC_OPT_PLANNING_BREADTH_FIRST_SEARCH_H_

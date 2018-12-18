/**
 * tree.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 12, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_UTILS_TREE_H_
#define LOGIC_OPT_UTILS_TREE_H_

#include <map>          // std::map
#include <type_traits>  // std::conditional_t
#include <utility>      // std::pair

#if __cplusplus >= 201703L
#include <optional>     // std::optional
#else   // __cplusplus
#include "LogicOpt/utils/optional.h"
#endif  // __cplusplus

namespace LogicOpt {

template<typename Key, typename T>
class Tree {

 protected:

  template<bool Const>
  class ChainView;

 public:

  using const_chain_view = ChainView<true>;
  using chain_view = ChainView<false>;

  const std::map<Key, T>& values() const { return values_; }

  const T& at(const Key& id) const { return values_.at(id); }
  T& at(const Key& id) { return values_.at(id); }

  chain_view ancestors(const Key& id) { return chain_view(this, id); }

  const_chain_view ancestors(const Key& id) const { return const_chain_view(this, id); }

  void insert_child(const Key& id_parent, const Key& id, const T& value) {
    values_[id] = value;
    id_parents_[id] = id_parent;
  }
  void insert_child(const Key& id_parent, const Key& id, T&& value) {
    values_[id] = std::move(value);
    id_parents_[id] = id_parent;
  }
  void insert(const Key& id, const T& value) {
    values_[id] = value;
    id_parents_[id].reset();
  }
  void insert(const Key& id, T&& value) {
    values_[id] = std::move(value);
    id_parents_[id].reset();
  }

  const std::optional<Key>& parent(const Key& id) const { return id_parents_.at(id); }
  void set_parent(const Key& id, const Key& id_parent) {
    id_parents_[id] = id_parent;
  }
  void clear_parent(const Key& id) {
    id_parents_[id].reset();
  }

  bool is_ancestor(const Key& id_ancestor, const Key& id) const {
    for (const auto& key_val : ancestors(id)) {
      if (key_val.first == id_ancestor) return true;
    }
    return false;
  }
  bool is_descendant(const Key& id_descendant, const Key& id) const {
    return is_ancestor(id, id_descendant);
  }

 protected:

  std::map<Key, T> values_;

  std::map<Key, std::optional<Key>> id_parents_;

};

template<typename Key, typename T>
template<bool Const>
class Tree<Key, T>::ChainView {

 public:

  using TreeT = typename std::conditional_t<Const, const Tree<Key, T>, Tree<Key, T>>;

  class iterator;

  ChainView(TreeT* tree, const Key& id) : tree_(tree), id_(id) {}

  iterator begin() { return iterator(tree_, id_); }
  iterator end() { return iterator(tree_, std::optional<Key>()); }

 protected:

  TreeT* tree_;
  std::optional<Key> id_;

};

template<typename Key, typename T>
template<bool Const>
class Tree<Key, T>::ChainView<Const>::iterator {

 public:

  using TreeT = typename std::conditional_t<Const, const Tree<Key, T>, Tree<Key, T>>;
  using ValueT = typename std::conditional_t<Const, const std::pair<const Key, T>, std::pair<const Key, T>>;

  using iterator_category = std::input_iterator_tag;
  using value_type = ValueT;
  using difference_type = ptrdiff_t;
  using pointer = value_type*;
  using reference = value_type&;

  iterator(TreeT* tree, const std::optional<Key>& id) : tree_(tree), id_(id) {}

  iterator& operator++() { id_ = tree_->id_parents_.at(*id_); return *this; }
  iterator& operator+=(size_t n) { for (size_t i = 0; i < n; i++) ++(*this); return *this; }
  iterator operator+(size_t n) { iterator it(*this); it += n; return it; }
  bool operator==(const iterator& other) const { return tree_ == other.tree_ && id_ == other.id_; }
  bool operator!=(const iterator& other) const { return !(*this == other); }
  reference operator*() const { return *tree_->values_.find(*id_); }
  pointer operator->() const { return tree_->values_.find(*id_).operator->(); }

 protected:

  TreeT* tree_;
  std::optional<Key> id_;

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_UTILS_TREE_H_

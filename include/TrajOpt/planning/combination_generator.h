/**
 * combination_generator.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_PLANNING_COMBINATION_GENERATOR_H_
#define TRAJ_OPT_PLANNING_COMBINATION_GENERATOR_H_

#include <cstddef>      // ptrdiff_t
#include <iterator>     // std::forward_iterator_tag, std::iterator_traits
#include <type_traits>  // std::conditional_t, std::is_const
#include <vector>       // std::vector

namespace TrajOpt {

template<typename ContainerT>
class CombinationGenerator {

 private:

  template<bool Const>
  class Iterator;

  template<typename IteratorT>
  class ReverseIterator;

 public:

  using iterator = typename std::conditional_t<std::is_const<ContainerT>::value,
                                               Iterator<true>, Iterator<false>>;
  using const_iterator = Iterator<true>;

  using reverse_iterator = ReverseIterator<iterator>;
  using const_reverse_iterator = ReverseIterator<const_iterator>;

  CombinationGenerator() {}

  CombinationGenerator(const std::vector<ContainerT*>& options) : options_(options) {}

  iterator begin() { return iterator::begin(options_); };
  iterator end() { return iterator::end(options_); };
  const_iterator begin() const { return const_iterator::begin(options_); };
  const_iterator end() const { return const_iterator::end(options_); };
  const_iterator cbegin() const { return const_iterator::begin(options_); };
  const_iterator cend() const { return const_iterator::end(options_); };

  reverse_iterator rbegin() { return ReverseIterator<iterator>(end()); }
  reverse_iterator rend() { return ReverseIterator<iterator>(begin()); }

 private:

  std::vector<ContainerT*> options_;

};

template<typename ContainerT>
template<bool Const>
class CombinationGenerator<ContainerT>::Iterator {

 public:

  using IteratorT = typename std::conditional_t<Const, typename ContainerT::const_iterator,
                                                       typename ContainerT::iterator>;
  using ValueT = typename std::iterator_traits<IteratorT>::value_type;

  using iterator_category = std::bidirectional_iterator_tag;
  using value_type = std::vector<ValueT>;
  using difference_type = ptrdiff_t;
  using pointer = typename std::conditional_t<Const, const value_type*, value_type*>;
  using reference = typename std::conditional_t<Const, const value_type&, value_type&>;

  Iterator(const std::vector<ContainerT*>& options, std::vector<IteratorT>&& it_options);

  Iterator& operator++();
  Iterator& operator--();
  bool operator==(const Iterator& other) const;
  bool operator!=(const Iterator& other) const { return !(*this == other); }
  reference operator*();

 private:

  static Iterator begin(const std::vector<ContainerT*>& options);
  static Iterator end(const std::vector<ContainerT*>& options);

  std::vector<ContainerT*> options_;
  std::vector<IteratorT> it_options_;
  std::vector<ValueT> combination_;

  friend class CombinationGenerator;

};

template<typename ContainerT>
template<typename IteratorT>
class CombinationGenerator<ContainerT>::ReverseIterator {

 public:

  using iterator_category = typename std::iterator_traits<IteratorT>::iterator_category;
  using value_type = typename std::iterator_traits<IteratorT>::value_type;
  using difference_type = typename std::iterator_traits<IteratorT>::difference_type;
  using pointer = typename std::iterator_traits<IteratorT>::pointer;
  using reference = typename std::iterator_traits<IteratorT>::reference;

  ReverseIterator(IteratorT&& it);

  ReverseIterator& operator++();
  ReverseIterator& operator--();
  bool operator==(const ReverseIterator& other) const { return it_ == other.it_; }
  bool operator!=(const ReverseIterator& other) const { return !(*this == other); }
  reference operator*() { return *it_; }

 private:

  IteratorT it_;

};

template<typename ContainerT>
template<bool Const>
CombinationGenerator<ContainerT>::Iterator<Const>::Iterator(const std::vector<ContainerT*>& options,
                                                            std::vector<IteratorT>&& it_options)
    : options_(options), it_options_(std::move(it_options)), combination_(options_.size()) {
  if (it_options_.front() == options_.front()->end()) return;
  for (size_t i = 0; i < options_.size(); i++) {
    combination_[i] = *it_options_[i];
  }
}

template<typename ContainerT>
template<bool Const>
typename CombinationGenerator<ContainerT>::template Iterator<Const>&
CombinationGenerator<ContainerT>::Iterator<Const>::operator++() {
  // Check for end flag
  if (options_.empty() || it_options_.front() == options_.front()->end()) return *this;

  for (size_t i = options_.size() - 1; i >= 0; i--) {
    ContainerT& values = *options_[i];
    IteratorT& it_value = it_options_[i];
    ValueT& value = combination_[i];

    ++it_value;

    // Return if param is not the last one in the current digit place
    if (it_value != values.end()) {
      value = *it_value;
      break;
    }

    // For the last combination, leave the index high
    if (i == 0) break;

    // Reset index to 0 and increment next digit
    it_value = values.begin();
    value = *it_value;
  }
  return *this;
}

template<typename ContainerT>
template<bool Const>
typename CombinationGenerator<ContainerT>::template Iterator<Const>&
CombinationGenerator<ContainerT>::Iterator<Const>::operator--() {
  // Don't decrement past beginning
  if (*this == begin(options_)) return *this;

  // Check for end flag
  if (it_options_.front() == options_.front()->end()) {
    // Reset
    for (size_t i = 0; i < options_.size(); i++) {
      ContainerT& values = *options_[i];
      IteratorT& it_value = it_options_[i];
      ValueT& value = combination_[i];

      it_value = --values.end();
      value = *it_value;
    }
    return *this;
  }

  for (size_t i = options_.size() - 1; i >= 0; i--) {
    ContainerT& values = *options_[i];
    IteratorT& it_value = it_options_[i];
    ValueT& value = combination_[i];

    // Return if param is not yet the first one in the current digit place
    if (it_value != values.begin()) {
      --it_value;
      value = *it_value;
      break;
    }

    it_value = values.end();

    // For the last combination, leave the index high
    if (i == 0) break;

    // Set index to last assignable value and decrement next digit
    --it_value;
    value = *it_value;
  }
  return *this;
}

template<typename ContainerT>
template<bool Const>
bool CombinationGenerator<ContainerT>::Iterator<Const>::operator==(const Iterator& other) const {
  // Check if empty
  if (options_.empty() && other.options_.empty()) return true;

  // Check if options are equal
  if (options_.size() != other.options_.size()) return false;
  for (size_t i = 0; i < options_.size(); i++) {
    if (options_[i] != other.options_[i]) return false;
  }

  // Check end flag
  if (it_options_.front() == options_.front()->end() &&
      other.it_options_.front() == other.options_.front()->end()) return true;

  // Check iterator positions
  for (size_t i = 0; i < options_.size(); i++) {
    if (it_options_[i] != other.it_options_[i]) return false;
  }
  return true;
}

template<typename ContainerT>
template<bool Const>
typename CombinationGenerator<ContainerT>::template Iterator<Const>::reference
CombinationGenerator<ContainerT>::Iterator<Const>::operator*() {
  if (options_.empty()) return combination_;
  if (it_options_.front() == options_.front()->end()) {
    throw std::out_of_range("ParameterGenerator::iterator::operator*(): Cannot dereference.");
  }
  return combination_;
}

template<typename ContainerT>
template<bool Const>
typename CombinationGenerator<ContainerT>::template Iterator<Const>
CombinationGenerator<ContainerT>::Iterator<Const>::begin(const std::vector<ContainerT*>& options) {
  std::vector<IteratorT> it_options;
  it_options.reserve(options.size());
  for (ContainerT* option : options) {
    it_options.push_back(option->begin());
  }
  return Iterator(options, std::move(it_options));
}

template<typename ContainerT>
template<bool Const>
typename CombinationGenerator<ContainerT>::template Iterator<Const>
CombinationGenerator<ContainerT>::Iterator<Const>::end(const std::vector<ContainerT*>& options) {
  std::vector<IteratorT> it_options;
  it_options.reserve(options.size());
  for (ContainerT* option : options) {
    it_options.push_back(option->end());
  }
  return Iterator(options, std::move(it_options));
}

template<typename ContainerT>
template<typename IteratorT>
CombinationGenerator<ContainerT>::ReverseIterator<IteratorT>::ReverseIterator(IteratorT&& it)
    : it_(std::move(it)) {
  if (it_ == IteratorT::begin(it_.options_)) {
    it_ = IteratorT::end(it_.options_);
  } else {
    --it_;
  }
}

template<typename ContainerT>
template<typename IteratorT>
typename CombinationGenerator<ContainerT>::template ReverseIterator<IteratorT>&
CombinationGenerator<ContainerT>::ReverseIterator<IteratorT>::operator++() {
  if (it_ == IteratorT::begin(it_.options_)) {
    it_ = IteratorT::end(it_.options_);
  } else if (it_ != IteratorT::end(it_.options_)) {
    --it_;
  }
  return *this;
}

template<typename ContainerT>
template<typename IteratorT>
typename CombinationGenerator<ContainerT>::template ReverseIterator<IteratorT>&
CombinationGenerator<ContainerT>::ReverseIterator<IteratorT>::operator--() {
  if (it_ == IteratorT::end(it_.options_)) {
    it_ = IteratorT::begin(it_.options_);
  } else {
    ++it_;
  }
  return *this;
}

}  // namespace TrajOpt

#endif  // TRAJ_OPT_PLANNING_COMBINATION_GENERATOR_H_

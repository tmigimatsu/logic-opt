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

#include <cstddef>   // ptrdiff_t
#include <iterator>  // std::input_iterator_tag, std::iterator_traits
#include <vector>    // std::vector

namespace TrajOpt {

template<typename ContainerT>
class CombinationGenerator {

 public:

  class iterator;

  CombinationGenerator() {}

  CombinationGenerator(const std::vector<ContainerT*>& options) : options_(options) {}

  iterator begin() const;
  iterator end() const;

 private:

  std::vector<ContainerT*> options_;

};

template<typename ContainerT>
class CombinationGenerator<ContainerT>::iterator {

 public:

  using IteratorT = typename ContainerT::const_iterator;
  using ValueT = typename std::iterator_traits<IteratorT>::value_type;

  using iterator_category = std::input_iterator_tag;
  using value_type = std::vector<ValueT>;
  using difference_type = ptrdiff_t;
  using pointer = const value_type*;
  using reference = const value_type&;

  iterator(const std::vector<ContainerT*>& options, std::vector<IteratorT>&& it_options);

  iterator& operator++();
  bool operator==(const iterator& other) const;
  bool operator!=(const iterator& other) const { return !(*this == other); };
  reference operator*() const;

 private:

  std::vector<ContainerT*> options_;
  std::vector<IteratorT> it_options_;
  std::vector<ValueT> combination_;

};

template<typename ContainerT>
typename CombinationGenerator<ContainerT>::iterator CombinationGenerator<ContainerT>::begin() const {
  std::vector<typename iterator::IteratorT> it_options;
  it_options.reserve(options_.size());
  for (ContainerT* option : options_) {
    it_options.push_back(option->begin());
  }
  return iterator(options_, std::move(it_options));
}

template<typename ContainerT>
typename CombinationGenerator<ContainerT>::iterator CombinationGenerator<ContainerT>::end() const {
  std::vector<typename iterator::IteratorT> it_options;
  it_options.reserve(options_.size());
  for (ContainerT* option : options_) {
    it_options.push_back(option->end());
  }
  return iterator(options_, std::move(it_options));
}

template<typename ContainerT>
CombinationGenerator<ContainerT>::iterator::iterator(const std::vector<ContainerT*>& options,
                                                     std::vector<IteratorT>&& it_options)
    : options_(options), it_options_(std::move(it_options)), combination_(options_.size()) {
  for (size_t i = 0; i < options_.size(); i++) {
    combination_[i] = *it_options_[i];
  }
}

template<typename ContainerT>
typename CombinationGenerator<ContainerT>::iterator&
CombinationGenerator<ContainerT>::iterator::operator++() {
  // Check for end flag
  if (options_.empty() || it_options_.front() == options_.front()->end()) return *this;

  for (size_t i = options_.size() - 1; i >= 0; i--) {
    const ContainerT& values = *options_[i];
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
bool CombinationGenerator<ContainerT>::iterator::operator==(const iterator& other) const {
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
typename CombinationGenerator<ContainerT>::iterator::reference
CombinationGenerator<ContainerT>::iterator::operator*() const {
  if (options_.empty()) return combination_;
  if (it_options_.front() == options_.front()->end()) {
    throw std::out_of_range("ParameterGenerator::iterator::operator*(): Cannot dereference.");
  }
  return combination_;
}

}  // namespace TrajOpt

#endif  // TRAJ_OPT_PLANNING_COMBINATION_GENERATOR_H_

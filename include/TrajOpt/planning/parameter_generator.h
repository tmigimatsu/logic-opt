/**
 * parameter_generator.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_PLANNING_PARAMETER_GENERATOR_H_
#define TRAJ_OPT_PLANNING_PARAMETER_GENERATOR_H_

#include <cstddef>   // ptrdiff_t
#include <iterator>  // std::input_iterator_tag
#include <memory>    // std::shared_ptr
#include <ostream>   // std::ostream
#include <string>    // std::string
#include <vector>    // std::vector

#include "ptree.h"

#include "TrajOpt/planning/objects.h"

namespace TrajOpt {

class ParameterGenerator {

 public:

  class iterator;

  ParameterGenerator() {}

  ParameterGenerator(const std::shared_ptr<const ObjectTypeMap>& objects,
                     const VAL::parameter_symbol_list* params);

  ParameterGenerator(const std::shared_ptr<const ObjectTypeMap>& objects,
                     const VAL::var_symbol_list* params);

  iterator begin() const;
  iterator end() const;

 private:

  std::shared_ptr<const ObjectTypeMap> objects_;
  std::vector<const std::vector<const VAL::parameter_symbol*>*> param_options_;

};

class ParameterGenerator::iterator {

 public:

  using iterator_category = std::input_iterator_tag;
  using value_type = std::vector<const VAL::parameter_symbol*>;
  using difference_type = ptrdiff_t;
  using pointer = const value_type*;
  using reference = const value_type&;

  iterator(const ParameterGenerator* gen, std::vector<size_t>&& idx_params);

  iterator& operator++();
  bool operator==(const iterator& other) const;
  bool operator!=(const iterator& other) const;
  reference operator*() const;

 private:

  std::vector<size_t> idx_params_;
  std::vector<const VAL::parameter_symbol*> params_;
  std::vector<const std::vector<const VAL::parameter_symbol*>*> param_options_;

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_PLANNING_PARAMETER_GENERATOR_H_

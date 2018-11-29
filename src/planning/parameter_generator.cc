/**
 * parameter_generator.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 28, 2018
 * Authors: Toki Migimatsu
 */

#include "TrajOpt/planning/parameter_generator.h"

namespace TrajOpt {

template<typename T>
std::vector<const std::vector<const VAL::parameter_symbol*>*>
ParamTypes(const std::shared_ptr<const ObjectTypeMap> objects,
           const VAL::typed_symbol_list<T>* params) {
  std::vector<const std::vector<const VAL::parameter_symbol*>*> types;
  types.reserve(params->size());
  for (const VAL::parameter_symbol* param : *params) {
    types.push_back(&objects->at(param->type));
  }
  return types;
}

ParameterGenerator::ParameterGenerator(
    const std::shared_ptr<const ObjectTypeMap>& objects,
    const VAL::parameter_symbol_list* params)
    : objects_(objects), param_options_(ParamTypes(objects, params)) {}

ParameterGenerator::ParameterGenerator(
    const std::shared_ptr<const ObjectTypeMap>& objects,
    const VAL::var_symbol_list* params)
    : objects_(objects), param_options_(ParamTypes(objects, params)) {}

ParameterGenerator::iterator::iterator(const ParameterGenerator* gen, std::vector<size_t>&& idx_params)
    : idx_params_(std::move(idx_params)), params_(idx_params_.size()),
      param_options_(gen->param_options_) {
  for (size_t i = 0; i < idx_params_.size(); i++) {
    const std::vector<const VAL::parameter_symbol*>& options = *param_options_[i];
    if (idx_params_[i] >= options.size()) return;
    params_[i] = options[idx_params_[i]];
  }
}

ParameterGenerator::iterator& ParameterGenerator::iterator::operator++() {
  // Check for end flag
  if (idx_params_.empty() || idx_params_.front() >= param_options_.front()->size()) return *this;

  for (size_t i = idx_params_.size() - 1; i >= 0; i--) {
    size_t& idx_param = idx_params_[i];
    const VAL::parameter_symbol*& param = params_[i];
    const std::vector<const VAL::parameter_symbol*>& options = *param_options_[i];

    ++idx_param;

    // Return if param is not the last one in the current digit place
    if (idx_param < options.size()) {
      param = options[idx_param];
      break;
    }

    // For the last combination, leave the index high
    if (i == 0) break;

    // Reset index to 0 and increment next digit
    idx_param = 0;
    param = options[idx_param];
  }
  return *this;
}

bool ParameterGenerator::iterator::operator==(const iterator& other) const {
  if (idx_params_.size() != other.idx_params_.size()) return false;
  for (size_t i = 0; i < idx_params_.size(); i++) {
    if (idx_params_[i] != other.idx_params_[i]) return false;
    if (param_options_[i] != other.param_options_[i]) return false;
  }
  return true;
}

bool ParameterGenerator::iterator::operator!=(const iterator& other) const {
  return !(*this == other);
}

ParameterGenerator::iterator::reference ParameterGenerator::iterator::operator*() const {
  if (params_.empty()) return params_;
  if (idx_params_.front() >= param_options_.front()->size()) {
    throw std::out_of_range("ParameterGenerator::iterator::operator*(): Cannot dereference.");
  }
  return params_;
}

ParameterGenerator::iterator ParameterGenerator::begin() const {
  return iterator(this, std::vector<size_t>(param_options_.size(), 0));
}

ParameterGenerator::iterator ParameterGenerator::end() const {
  std::vector<size_t> idx_objects(param_options_.size(), 0);
  if (!idx_objects.empty()) {
    idx_objects.front() = param_options_.front()->size();
  }
  return iterator(this, std::move(idx_objects));
}

}  // namespace TrajOpt

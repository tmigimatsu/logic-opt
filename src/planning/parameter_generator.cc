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
std::vector<const VAL::pddl_type*> ParamTypes(const VAL::typed_symbol_list<T>* params) {
  std::vector<const VAL::pddl_type*> types;
  types.reserve(params->size());
  for (const VAL::parameter_symbol* param : *params) {
    types.push_back(param->type);
  }
  return types;
}

ParameterGenerator::ParameterGenerator(
    const std::shared_ptr<const ObjectTypeMap>& objects,
    const VAL::parameter_symbol_list* params)
    : objects_(objects), types_(ParamTypes(params)) {}

ParameterGenerator::ParameterGenerator(
    const std::shared_ptr<const ObjectTypeMap>& objects,
    const VAL::var_symbol_list* params)
    : objects_(objects), types_(ParamTypes(params)) {}

ParameterGenerator::iterator::iterator(const ParameterGenerator* gen, std::vector<size_t>&& idx_objects)
    : gen_(gen), idx_objects_(std::move(idx_objects)), objects_(idx_objects_.size()) {
  for (size_t i = 0; i < idx_objects_.size(); i++) {
    const std::vector<const VAL::parameter_symbol*>& objects = gen_->objects_->at(gen_->types_[i]);
    if (idx_objects_[i] >= objects.size()) return;
    objects_[i] = objects[idx_objects_[i]];
  }
}

ParameterGenerator::iterator& ParameterGenerator::iterator::operator++() {
  // Check for end flag
  if (idx_objects_[0] >= gen_->objects_->at(gen_->types_[0]).size()) {
    return *this;
  }

  for (size_t i = idx_objects_.size() - 1; i >= 0; i--) {
    ++idx_objects_[i];
    if (idx_objects_[i] < gen_->objects_->at(gen_->types_[i]).size()) {
      objects_[i] = gen_->objects_->at(gen_->types_[i])[idx_objects_[i]];
      break;
    }
    // At the end, leave the first index high
    if (i == 0) break;

    idx_objects_[i] = 0;
    objects_[i] = gen_->objects_->at(gen_->types_[i])[idx_objects_[i]];
  }
  return *this;
}

bool ParameterGenerator::iterator::operator==(const iterator& other) const {
  if (gen_ != other.gen_) return false;
  if (idx_objects_.size() != other.idx_objects_.size()) return false;
  for (size_t i = 0; i < idx_objects_.size(); i++) {
    if (idx_objects_[i] != other.idx_objects_[i]) return false;
  }
  return true;
}

bool ParameterGenerator::iterator::operator!=(const iterator& other) const {
  return !(*this == other);
}

ParameterGenerator::iterator::reference ParameterGenerator::iterator::operator*() const {
  if (idx_objects_[0] >= gen_->objects_->at(gen_->types_[0]).size()) {
    throw std::out_of_range("ParameterGenerator::iterator::operator*(): Cannot dereference.");
    const std::vector<const VAL::parameter_symbol*>* p_objects = nullptr;
    return *p_objects;
  }
  return objects_;
}

ParameterGenerator::iterator ParameterGenerator::begin() const {
  return iterator(this, std::vector<size_t>(types_.size(), 0));
}

ParameterGenerator::iterator ParameterGenerator::end() const {
  if (!objects_) return begin();
  std::vector<size_t> idx_objects(types_.size(), 0);
  idx_objects[0] = objects_->at(types_[0]).size();
  return iterator(this, std::move(idx_objects));
}

}  // namespace TrajOpt

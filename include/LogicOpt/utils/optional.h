/**
 * optional.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: December 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_UTILS_OPTIONAL_H_
#define LOGIC_OPT_UTILS_OPTIONAL_H_

namespace std {

template<typename T>
class optional {

 public:

  optional() {}

  optional(const optional<T>& other) : value_(other.value_), has_value_(other.has_value_) {}
  optional(optional<T>&& other) : value_(std::move(other.value_)), has_value_(other.has_value_) {}

  optional(const T& value) : value_(value), has_value_(true) {}
  optional(T&& value) : value_(std::forward<T>(value)), has_value_(true) {}

  optional& operator=(const optional<T>& other) { value_ = other.value_; has_value_ = other.has_value_; return *this; }
  optional& operator=(optional<T>&& other) { value_ = std::move(other.value_); has_value_ = other.has_value_; return *this; }

  optional& operator=(const T& value) { value_ = value; has_value_ = true; return *this; }
  optional& operator=(T&& value) { value_ = std::forward<T>(value); has_value_ = true; return *this; }

  const T* operator->() const { return &value_; }
  T* operator->() { return &value_; }
  const T& operator*() const { return value_; }
  T& operator*() { return value_; }

  explicit operator bool() const { return has_value_; }
  bool has_value() const { return has_value_; }

  const T& value() const { return value_; }
  T& value() { return value_; }

  T value_or(T&& default_value) const { return has_value_ ? value_ : default_value; }

  void swap(optional& other) { std::swap(value_, other.value_); std::swap(has_value_, other.has_value_); }

  void reset() { has_value_ = false; }

  template<typename... Args>
  T& emplace(Args&&... args) { value_ = T(std::forward<Args>(args)...); return value_; }

 private:

  T value_;
  bool has_value_ = false;

};

template<typename T>
bool operator==(const std::optional<T>& lhs, const std::optional<T>& rhs) {
  return lhs.has_value() == rhs.has_value() && (!lhs.has_value() || lhs.value() == rhs.value());
}

}  // namespace std

#endif  // LOGIC_OPT_UTILS_OPTIONAL_H_

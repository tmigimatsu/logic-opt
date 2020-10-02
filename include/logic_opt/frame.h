/**
 * frame.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 12, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_FRAME_H_
#define LOGIC_OPT_FRAME_H_

#include <string>  // std::string

namespace logic_opt {

/**
 * Relative frame.
 */
class Frame {
 public:
  Frame() = default;

  Frame(const std::string& name) : name_(name) {}

  Frame(const std::string& name, int idx_var)
      : name_(name), idx_var_(idx_var) {}

  /**
   * Whether this frame is represented with an optimization variable.
   */
  bool is_variable() const { return idx_var_ >= 0; }

  /**
   * Frame name.
   */
  const std::string& name() const { return name_; }

  /**
   * Timestep index of this frame's variable in the trajectory.
   */
  int idx_var() const { return idx_var_; }

  /**
   * Set timestep index of this frame's variable in the trajectory.
   */
  void set_idx_var(int idx_var = -1) { idx_var_ = idx_var; }

  bool operator==(const Frame& other) const {
    return name_ == other.name_ && idx_var_ == other.idx_var_;
  }
  bool operator!=(const Frame& other) const { return !(*this == other); }

 protected:
  std::string name_;
  int idx_var_ = -1;
};

}  // namespace logic_opt

#endif  // LOGIC_OPT_FRAME_H_

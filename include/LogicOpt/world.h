/**
 * world.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 12, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_WORLD_H_
#define LOGIC_OPT_WORLD_H_

#include <spatial_dyn/spatial_dyn.h>

#include <map>        // std::map
#include <memory>     // std::unique_ptr, std::shared_ptr
#include <string>     // std::string

#include <ncollide/ncollide3d.h>

#include "LogicOpt/utils/tree.h"

namespace LogicOpt {

class Constraint;
typedef std::vector<std::unique_ptr<Constraint>> Constraints;

class Frame {

 public:

  Frame() {}

  Frame(const std::string& name)
      : name_(name) {}

  Frame(const std::string& name, int idx_var)
      : name_(name), idx_var_(idx_var) {}

  bool is_variable() const { return idx_var_ >= 0; }

  const std::string& name() const { return name_; }

  int idx_var() const { return idx_var_; }

  void set_idx_var(const int idx_var = -1) { idx_var_ = idx_var; }

  bool operator==(const Frame& other) const { return name_ == other.name_ && idx_var_ == other.idx_var_; }
  bool operator!=(const Frame& other) const { return !(*this == other); }

 private:

  std::string name_;
  int idx_var_ = -1;

};

class Object : public spatial_dyn::RigidBody {

 public:

  Object() = default;

  Object(const std::string& name) : spatial_dyn::RigidBody(name) {}

  std::shared_ptr<ncollide3d::shape::Shape> collision;

};

class World {

 public:

  World(const std::shared_ptr<const std::map<std::string, Object>>& objects, size_t T = 1);

  size_t num_timesteps() const { return controller_frames_.size(); }

  const std::shared_ptr<const std::map<std::string, Object>>& objects() const { return objects_; }

  void ReserveTimesteps(size_t T);

  void AttachFrame(const std::string& name_frame, const std::string& name_target, size_t t);

  void DetachFrame(const std::string& name_frame, size_t t);

  const Tree<std::string, Frame>& frames(size_t t) const { return frames_[t]; }

  void set_controller_frames(const std::string& control_frame, const std::string& target_frame,
                             size_t t);

  const std::pair<std::string, std::string>& controller_frames(size_t t) const;
  const std::string& control_frame(size_t t) const;
  const std::string& target_frame(size_t t) const;

  Eigen::Isometry3d T_to_world(const std::string& name_frame,
                               Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const;

  Eigen::Isometry3d T_to_parent(const std::string& name_frame,
                                Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const;

  Eigen::Isometry3d T_to_frame(const std::string& from_frame, const std::string& to_frame,
                               Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const;

  Eigen::Isometry3d T_control_to_target(Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const;

  Eigen::Vector3d Position(const std::string& of_frame, const std::string& in_frame,
                           Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const;

  Eigen::Matrix3d Orientation(const std::string& of_frame, const std::string& in_frame,
                              Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const;

  Eigen::Matrix3Xd PositionJacobian(const std::string& name_frame,
                                    Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const;

  std::array<Eigen::Matrix3d, 3> OrientationAngleAxisJacobians(Eigen::Ref<const Eigen::Vector3d> aa) const;

  std::array<std::optional<Eigen::Matrix3d>, 5> RotationChain(const std::string& name_frame,
                                                              size_t idx_var,
                                                              Eigen::Ref<const Eigen::MatrixXd> X,
                                                              size_t t1, size_t t2) const;

  Eigen::Matrix3d OrientationTraceJacobian(const std::string& name_frame,
                                           size_t idx_var, Eigen::Ref<const Eigen::MatrixXd> X,
                                           size_t t1, size_t t2, double* trace = nullptr) const;

  static const std::string kWorldFrame;

 private:

  const std::shared_ptr<const std::map<std::string, Object>> objects_;

  std::vector<Tree<std::string, Frame>> frames_;

  std::vector<std::pair<std::string, std::string>> controller_frames_;

};

std::ostream& operator<<(std::ostream& os, const World& world);

}  // namespace LogicOpt

#endif  // LOGIC_OPT_WORLD_H_

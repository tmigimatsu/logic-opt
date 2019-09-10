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

#include <algorithm>    // std::max
#include <exception>    // std::out_of_range
#include <map>          // std::map
#include <memory>       // std::unique_ptr, std::shared_ptr
#include <string>       // std::string
#include <type_traits>  // std::conditional_t

#include <ctrl_utils/tree.h>
#include <ncollide_cpp/ncollide.h>

#include "logic_opt/optimization/variables.h"

namespace logic_opt {

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

 protected:

  std::string name_;
  int idx_var_ = -1;

};

template<int Dim>
Eigen::Transform<double, Dim, Eigen::Isometry> ConvertIsometry(const Eigen::Isometry3d& T);

template<int Dim>
class Object : public spatial_dyn::RigidBody {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Object(const std::string& name) : spatial_dyn::RigidBody(name) {}
  Object(const spatial_dyn::RigidBody& rb);

  virtual ~Object() = default;

  std::shared_ptr<typename ncollide<Dim>::shape::Shape> collision;

  template<int Dim_ = Dim>
  const typename std::enable_if_t<Dim_ == 2, Eigen::Isometry2d>&
  T_to_parent() const { return T_to_parent_2d_; }

  template<int Dim_ = Dim>
  const typename std::enable_if_t<Dim_ == 3, Eigen::Isometry3d>&
  T_to_parent() const { return T_to_parent_; }

 protected:

  static std::unique_ptr<typename ncollide<Dim>::shape::Shape>
  MakeCollision(const spatial_dyn::Graphics::Geometry& geometry);

  Eigen::Isometry2d T_to_parent_2d_;

};

typedef Object<3> Object3;
typedef Object<2> Object2;

template<int Dim>
class World {

 public:

  static constexpr size_t kDof = (Dim == 2) ? 3 : 6;
  static const std::string kWorldFrame;  // = "__world";

  using Isometry = Eigen::Transform<double, Dim, Eigen::Isometry>;
  using Rotation = std::conditional_t<Dim == 2, Eigen::Rotation2Dd, Eigen::AngleAxisd>;

  World(const std::shared_ptr<const std::map<std::string, Object<Dim>>>& objects,
        size_t T = 1);

  virtual ~World() = default;

  size_t num_timesteps() const { return controller_frames_.size(); }

  const std::shared_ptr<const std::map<std::string, Object<Dim>>>& objects() const {
    return objects_;
  }

  void ReserveTimesteps(size_t T);

  void AttachFrame(const std::string& name_frame, const std::string& name_target,
                   size_t t, bool fixed = false);

  void DetachFrame(const std::string& name_frame, size_t t);

  const ctrl_utils::Tree<std::string, Frame>& frames(size_t t) const { return frames_[t]; }

  void set_controller_frames(const std::string& control_frame,
                             const std::string& target_frame, size_t t);

  const std::pair<std::string, std::string>& controller_frames(size_t t) const {
    if (t >= controller_frames_.size()) {
      throw std::out_of_range("World::controller_frames(): t (" + std::to_string(t) +
                              ") must be less than " + std::to_string(controller_frames_.size()));
    }
    return controller_frames_[t];
  };

  const std::string& control_frame(size_t t) const {
    if (t >= controller_frames_.size()) {
      throw std::out_of_range("World::controller_frames(): t (" + std::to_string(t) +
                              ") must be less than " + std::to_string(controller_frames_.size()));
    }
    return controller_frames_[t].first;
  };

  const std::string& target_frame(size_t t) const {
    if (t >= controller_frames_.size()) {
      throw std::out_of_range("World::controller_frames(): t (" + std::to_string(t) +
                              ") must be less than " + std::to_string(controller_frames_.size()));
    }
    return controller_frames_[t].second;
  };

  void set_controller(const std::string& controller, size_t t) { controllers_[t] = controller; }

  const std::string& controller(size_t t) const { return controllers_[t]; }

  Isometry T_to_world(const std::string& name_frame,
                      Eigen::Ref<const Eigen::MatrixXd> X,
                      size_t t) const;

  Isometry T_to_parent(const std::string& name_frame,
                       Eigen::Ref<const Eigen::MatrixXd> X,
                       size_t t) const;

  Isometry T_to_frame(const std::string& from_frame,
                      const std::string& to_frame,
                      Eigen::Ref<const Eigen::MatrixXd> X,
                      size_t t) const;

  Isometry T_control_to_target(Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const;

  Eigen::Matrix<double, Dim, 1> Position(const std::string& of_frame,
                                         const std::string& in_frame,
                                         Eigen::Ref<const Eigen::MatrixXd> X,
                                         size_t t) const;

  Eigen::Matrix<double, Dim, Dim> Orientation(const std::string& of_frame,
                                              const std::string& in_frame,
                                              Eigen::Ref<const Eigen::MatrixXd> X,
                                              size_t t) const;

 protected:

  const std::shared_ptr<const std::map<std::string, Object<Dim>>> objects_;

  std::vector<ctrl_utils::Tree<std::string, Frame>> frames_;

  std::vector<std::pair<std::string, std::string>> controller_frames_;

  std::vector<std::string> controllers_;

  static Rotation ExtractRotation(Eigen::Ref<const Eigen::MatrixXd> X, size_t t);

};

typedef World<3> World3;
typedef World<2> World2;

template<int Dim>
std::ostream& operator<<(std::ostream& os, const World<Dim>& world);

/**
 * Implementation
 */

template<int Dim>
Object<Dim>::Object(const spatial_dyn::RigidBody& rb) : spatial_dyn::RigidBody(rb) {
  // Compute 2d transform
  T_to_parent_2d_ = ConvertIsometry<2>(T_to_parent_);

  if (rb.graphics.empty()) return;

  if (rb.graphics.size() == 1) {
    // TODO: Handle graphics.T_to_parent
    collision = MakeCollision(rb.graphics[0].geometry);
    return;
  }

  typename ncollide<Dim>::shape::ShapeVector shapes;
  shapes.reserve(rb.graphics.size());
  for (const spatial_dyn::Graphics& graphics : rb.graphics) {
    shapes.emplace_back(ConvertIsometry<Dim>(graphics.T_to_parent), MakeCollision(graphics.geometry));
  }
  collision = std::make_unique<typename ncollide<Dim>::shape::Compound>(std::move(shapes));
}

template<int Dim>
const std::string World<Dim>::kWorldFrame = "__world";

template<int Dim>
World<Dim>::World(const std::shared_ptr<const std::map<std::string, Object<Dim>>>& objects,
                  size_t T)
    : objects_(objects),
      frames_(std::max(T, static_cast<size_t>(1))),
      controller_frames_(std::max(T, static_cast<size_t>(1)), {"", ""}),
      controllers_(std::max(T, static_cast<size_t>(1))) {

  for (ctrl_utils::Tree<std::string, Frame>& frames_t : frames_) {
    frames_t.insert(kWorldFrame, Frame(kWorldFrame));
  }

  for (const auto& key_val : *objects_) {
    const std::string& name = key_val.first;

    for (ctrl_utils::Tree<std::string, Frame>& frames_t : frames_) {
      frames_t.insert_child(kWorldFrame, name, Frame(name));
    }
  }
}

template<int Dim>
void World<Dim>::ReserveTimesteps(size_t T) {
  if (frames_.size() >= T) return;
  frames_.reserve(T);
  controller_frames_.reserve(T);
  controllers_.reserve(T);

  for (size_t t = frames_.size(); t < T; t++) {
    // Copy kinematic tree from previous timestep
    frames_.push_back(frames_.back());

    controller_frames_.push_back({"", ""});
    controllers_.push_back("");
  }
}

template<int Dim>
void World<Dim>::AttachFrame(const std::string& name_frame, const std::string& name_target,
                             size_t t, bool fixed) {
  if (name_frame == name_target) {
    throw std::runtime_error("World::AttachFrame(): Cannot attach frame " + name_frame + " to itself.");
  }
  if (fixed) {
    for (size_t tt = t; tt < frames_.size(); tt++) {
      frames_[tt].set_parent(name_frame, name_target);
    }
    return;
  }
  // Set frames for all empty preceding timesteps
  for (int tt = t; tt >= 0; tt--) {
    // Check if frame is being controlled
    // if (!control_frame(tt).empty() && frames_[tt].at(control_frame(tt)).idx_var() == tt) break;
    if (!control_frame(tt).empty()) break;

    // Set control variable to current timestep
    frames_[tt].set_parent(name_frame, name_target);
    set_controller_frames(name_frame, name_target, tt);
    frames_[tt].at(name_frame).set_idx_var(tt);
  }

  // Update frame tree for all subsequent timesteps
  for (size_t tt = t + 1; tt < frames_.size(); tt++) {
    frames_[tt].set_parent(name_frame, name_target);
    set_controller_frames("", "", tt);
    frames_[tt].at(name_frame).set_idx_var(t);
  }
}

template<int Dim>
void World<Dim>::DetachFrame(const std::string& name_frame, size_t t) {
  for (size_t tt = t; tt < frames_.size(); tt++) {
    frames_[tt].set_parent(name_frame, kWorldFrame);
  }
}

template<int Dim>
void World<Dim>::set_controller_frames(const std::string& control_frame, const std::string& target_frame,
                                       size_t t) {
  controller_frames_[t].first = control_frame;
  controller_frames_[t].second = target_frame;
}

template<int Dim>
typename World<Dim>::Isometry World<Dim>::T_to_world(const std::string& name_frame,
                                                     Eigen::Ref<const Eigen::MatrixXd> X,
                                                     size_t t) const {
  return T_to_frame(name_frame, kWorldFrame, X, t);
}

template<int Dim>
typename World<Dim>::Isometry World<Dim>::T_to_parent(const std::string& name_frame,
                                                      Eigen::Ref<const Eigen::MatrixXd> X,
                                                      size_t t) const {
  const Frame& frame = frames_[t].at(name_frame);
  return frame.is_variable() ? T_control_to_target(X, frame.idx_var())
                             : objects_->at(frame.name()).T_to_parent();
}

template<int Dim>
typename World<Dim>::Isometry World<Dim>::T_to_frame(const std::string& from_frame,
                                                     const std::string& to_frame,
                                                     Eigen::Ref<const Eigen::MatrixXd> X,
                                                     size_t t) const {
  std::string ancestor_frame;
  std::string descendant_frame;
  Isometry T_result = Isometry::Identity();
  if (frames_[t].is_ancestor(to_frame, from_frame)) {
    ancestor_frame = to_frame;
    descendant_frame = from_frame;
  } else if (frames_[t].is_ancestor(from_frame, to_frame)) {
    ancestor_frame = from_frame;
    descendant_frame = to_frame;
  } else {
    T_result = T_to_world(to_frame, X, t).inverse() * T_to_world(from_frame, X, t);
    return T_result;
  }

  for (const std::pair<std::string, Frame>& key_val : frames_[t].ancestors(descendant_frame)) {
    const std::string& frame = key_val.first;
    if (frame == ancestor_frame) break;
    T_result = T_to_parent(frame, X, t) * T_result;
  }
  if (ancestor_frame == from_frame) {
    T_result = T_result.inverse();
  }
  return T_result;
}

template<int Dim>
typename World<Dim>::Isometry World<Dim>::T_control_to_target(Eigen::Ref<const Eigen::MatrixXd> X,
                                                              size_t t) const {
  const auto pos = X.block<Dim, 1>(0, t);
  return Eigen::Translation<double, Dim>(pos) * ExtractRotation(X, t);
}

template<int Dim>
Eigen::Matrix<double, Dim, 1> World<Dim>::Position(const std::string& of_frame,
                                                   const std::string& in_frame,
                                                   Eigen::Ref<const Eigen::MatrixXd> X,
                                                   size_t t) const {

  Eigen::Matrix<double, Dim, 1> p = Eigen::Matrix<double, Dim, 1>::Zero();
  bool frame_reached = false;
  for (const std::pair<std::string, Frame>& key_val : frames_[t].ancestors(of_frame)) {
    if (key_val.first == in_frame) {
      frame_reached = true;
      break;
    }

    const Frame& frame = key_val.second;
    p = T_to_parent(frame.name(), X, t) * p;
  }

  if (!frame_reached) {
    throw std::invalid_argument("World::Position(): frame \"" + of_frame +
                                "\" must be an descendant of \"" + in_frame + "\"");
  }
  return p;
}

template<int Dim>
Eigen::Matrix<double, Dim, Dim> World<Dim>::Orientation(const std::string& of_frame,
                                                        const std::string& in_frame,
                                                        Eigen::Ref<const Eigen::MatrixXd> X,
                                                        size_t t) const {
  Eigen::Matrix<double, Dim, Dim> R = Eigen::Matrix<double, Dim, Dim>::Identity();
  bool frame_reached = false;
  for (const std::pair<std::string, Frame>& key_val : frames_[t].ancestors(of_frame)) {
    if (key_val.first == in_frame) {
      frame_reached = true;
      break;
    }

    const Frame& frame = key_val.second;
    if (frame.is_variable()) {
      R = ExtractRotation(X, frame.idx_var()) * R;
    } else {
      R = objects_->at(frame.name()).T_to_parent().linear() * R;
    }
  }

  if (!frame_reached) {
    throw std::invalid_argument("World::Orientation(): frame \"" + of_frame +
                                "\" must be an descendant of \"" + in_frame + "\"");
  }
  return R;
}

template<int Dim>
std::ostream& operator<<(std::ostream& os, const World<Dim>& world) {
  for (size_t t = 0; t < world.num_timesteps(); t++) {
    const auto& frame_pair = world.controller_frames(t);
    os << "Frame: " << t << std::endl
       << "  Control: " << frame_pair.first << std::endl
       << "  Target: " << frame_pair.second << std::endl;

    const auto& frame_tree = world.frames(t);
    os << "  Tree:" << std::endl;
    for (const auto& key_val : frame_tree.values()) {
      const std::string& name = key_val.first;
      const Frame& frame = key_val.second;
      const std::optional<std::string>& id_parent = frame_tree.parent(name);
      os << "    " << name << ":" << std::endl;
      if (id_parent) {
         os << "      parent: " << *id_parent << std::endl;
      }
      if (frame.is_variable()) {
        os << "      idx_var: " << frame.idx_var() << std::endl;
      }
    }
  }
  return os;
}

}  // namespace logic_opt

#endif  // LOGIC_OPT_WORLD_H_

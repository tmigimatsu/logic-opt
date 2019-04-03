/**
 * world.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 12, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/world.h"
#include "LogicOpt/constraints.h"

#include <algorithm>  // std::max
#include <cmath>      // std::fabs

#include <ctrl_utils/string.h>

namespace LogicOpt {

const std::string World::kWorldFrame = "__world";

std::unique_ptr<ncollide3d::shape::Shape> MakeCollision(const spatial_dyn::Graphics::Geometry& geometry) {
  switch (geometry.type) {
    case spatial_dyn::Graphics::Geometry::Type::kBox:
      return std::make_unique<ncollide3d::shape::RoundedCuboid>(geometry.scale.array() / 2. - 0.005, 0.005);
    case spatial_dyn::Graphics::Geometry::Type::kCapsule:
      return std::make_unique<ncollide3d::shape::Capsule>(geometry.length / 2., geometry.radius);
    case spatial_dyn::Graphics::Geometry::Type::kSphere:
      return std::make_unique<ncollide3d::shape::Ball>(geometry.radius);
    case spatial_dyn::Graphics::Geometry::Type::kMesh:
      return std::make_unique<ncollide3d::shape::TriMesh>(geometry.mesh);
    default:
      throw std::runtime_error("LogicOpt::Object::MakeCollision(): Geometry type " +
                               ctrl_utils::ToString(geometry.type) + " not implemented yet.");
      break;
  }
}

Object::Object(const spatial_dyn::RigidBody& rb)
    : spatial_dyn::RigidBody(rb) {

  if (rb.graphics.size() == 1) {
    collision = MakeCollision(rb.graphics[0].geometry);
    return;
  }

  ncollide3d::shape::ShapeVector shapes;
  shapes.reserve(rb.graphics.size());
  for (const spatial_dyn::Graphics& graphics : rb.graphics) {
    shapes.emplace_back(graphics.T_to_parent, MakeCollision(graphics.geometry));
  }
  collision = std::make_unique<ncollide3d::shape::Compound>(std::move(shapes));
}

World::World(const std::shared_ptr<const std::map<std::string, Object>>& objects, size_t T)
    : objects_(objects), frames_(std::max(T, static_cast<size_t>(1))),
      controller_frames_(std::max(T, static_cast<size_t>(1))) {

  for (Tree<std::string, Frame>& frames_t : frames_) {
    frames_t.insert(kWorldFrame, Frame(kWorldFrame));
  }

  for (const auto& key_val : *objects_) {
    const std::string& name = key_val.first;

    for (Tree<std::string, Frame>& frames_t : frames_) {
      frames_t.insert_child(kWorldFrame, name, Frame(name));
    }
  }
}

void World::ReserveTimesteps(size_t T) {
  if (frames_.size() >= T) return;
  frames_.reserve(T);

  for (size_t t = frames_.size(); t < T; t++) {
    frames_.push_back(frames_.back());
    controller_frames_.push_back(controller_frames_.back());
  }
}

void World::AttachFrame(const std::string& name_frame, const std::string& name_target, size_t t) {
  for (size_t tt = t; tt < frames_.size(); tt++) {
    frames_[tt].set_parent(name_frame, name_target);
    frames_[tt].at(name_frame).set_idx_var(t);
    set_controller_frames(name_frame, name_target, tt);
  }
}

void World::DetachFrame(const std::string& name_frame, size_t t) {
  for (size_t tt = t; tt < frames_.size(); tt++) {
    frames_[tt].set_parent(name_frame, kWorldFrame);
  }
}

void World::set_controller_frames(const std::string& control_frame, const std::string& target_frame,
                                  size_t t) {
  controller_frames_[t].first = control_frame;
  controller_frames_[t].second = target_frame;
}

const std::pair<std::string, std::string>& World::controller_frames(size_t t) const {
  return controller_frames_[t];
}

const std::string& World::control_frame(size_t t) const {
  return controller_frames_[t].first;
}
const std::string& World::target_frame(size_t t) const {
  return controller_frames_[t].second;
}

Eigen::Isometry3d World::T_to_world(const std::string& name_frame,
                                    Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const {
  return T_to_frame(name_frame, kWorldFrame, X, t);
}

Eigen::Isometry3d World::T_to_parent(const std::string& name_frame,
                                     Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const {
  const Frame& frame = frames_[t].at(name_frame);
  return frame.is_variable() ? T_control_to_target(X, frame.idx_var())
                             : objects_->at(frame.name()).T_to_parent();
}

Eigen::Isometry3d World::T_to_frame(const std::string& from_frame, const std::string& to_frame,
                                    Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const {
  std::string ancestor_frame;
  std::string descendant_frame;
  if (frames_[t].is_ancestor(to_frame, from_frame)) {
    ancestor_frame = to_frame;
    descendant_frame = from_frame;
  } else if (frames_[t].is_ancestor(from_frame, to_frame)) {
    ancestor_frame = from_frame;
    descendant_frame = to_frame;
  } else {
    throw std::invalid_argument("World::T_to_frame(): Frames " + from_frame + " and " +
                                to_frame + " must be in the same tree chain.");
  }

  Eigen::Isometry3d T_result = Eigen::Isometry3d::Identity();
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

Eigen::Isometry3d World::T_control_to_target(Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const {
  auto x = X.col(t);
  auto pos = x.head<3>();
  auto w = x.tail<3>();
  return Eigen::Translation3d(pos) * Eigen::AngleAxisd(w.norm(), w.normalized());
}

Eigen::Matrix3d World::Orientation(const std::string& of_frame, const std::string& in_frame,
                                   Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  bool frame_reached = false;
  for (const std::pair<std::string, Frame>& key_val : frames_[t].ancestors(of_frame)) {
    if (key_val.first == in_frame) {
      frame_reached = true;
      break;
    }

    const Frame& frame = key_val.second;
    if (frame.is_variable()) {
      auto aa = X.col(frame.idx_var()).tail<3>();
      R = Eigen::AngleAxisd(aa.norm(), aa.normalized()) * R;
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

Eigen::Vector3d World::Position(const std::string& of_frame, const std::string& in_frame,
                                Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  bool frame_reached = false;
  for (const std::pair<std::string, Frame>& key_val : frames_[t].ancestors(of_frame)) {
    if (key_val.first == in_frame) {
      frame_reached = true;
      break;
    }

    const Frame& frame = key_val.second;
    if (frame.is_variable()) {
      auto x = X.col(frame.idx_var());
      auto pos = x.head<3>();
      auto aa = x.tail<3>();
      double angle = aa.norm();
      p = Eigen::Translation3d(pos) * Eigen::AngleAxisd(aa.norm(), aa.normalized()) * p;
    } else {
      p = objects_->at(frame.name()).T_to_parent() * p;
    }
  }

  if (!frame_reached) {
    throw std::invalid_argument("World::Position(): frame \"" + of_frame +
                                "\" must be an descendant of \"" + in_frame + "\"");
  }
  return p;
}

std::ostream& operator<<(std::ostream& os, const World& world) {
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

}  // namespace LogicOpt

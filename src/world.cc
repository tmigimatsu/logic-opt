/**
 * world.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 12, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/world.h"

#include <sstream>  // std::stringstream

namespace {

using Isometry = ::spatial_opt::Isometry;

}  // namespace

namespace logic_opt {

const std::string World::kWorldFrame = "__world";

World::World(
    const std::shared_ptr<const std::map<std::string, Object>>& objects,
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

void World::ReserveTimesteps(size_t T) {
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

void World::AttachFrame(const std::string& name_frame,
                        const std::string& name_target, size_t t, bool fixed) {
  if (name_frame == name_target) {
    throw std::runtime_error("World::AttachFrame(): Cannot attach frame " +
                             name_frame + " to itself.");
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
    // if (!control_frame(tt).empty() &&
    // frames_[tt].at(control_frame(tt)).idx_var() == tt) break;
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

void World::DetachFrame(const std::string& name_frame, size_t t) {
  for (size_t tt = t; tt < frames_.size(); tt++) {
    frames_[tt].set_parent(name_frame, kWorldFrame);
  }
}

void World::InitializeTree(const ctrl_utils::Tree<std::string, Frame>& tree,
                           size_t t) {
  ctrl_utils::Tree<std::string, Frame>& tree_t = frames_[t];
  tree_t = tree;

  // Reset idx variables
  for (const std::pair<std::string, Frame>& key_frame : tree_t.nodes()) {
    const std::string& key = key_frame.first;
    tree_t.at(key).set_idx_var();
  }

  // Initialize remaining timesteps;
  for (size_t tt = t + 1; tt < frames_.size(); tt++) {
    frames_[tt] = tree_t;
  }
}

Isometry ComputeTransform(const std::string& from_frame,
                          const std::string& to_frame,
                          const ctrl_utils::Tree<std::string, Frame>& tree,
                          const std::map<std::string, Object>& objects) {
  Isometry T = Isometry::Identity();
  for (const std::pair<std::string, Frame>& key_ancestor :
       tree.ancestors(from_frame)) {
    const std::string& key = key_ancestor.first;
    const Frame& ancestor = key_ancestor.second;
    if (ancestor.is_variable()) {
      throw std::runtime_error("World::set_T_to_world(): Frame " + key +
                               " is variable.");
    } else if (key == to_frame) {
      break;
    }
    T = objects.at(key).T_to_parent() * T;
  }
  return T;
}

void World::set_T_to_world(const std::string& frame, const Isometry& T_to_world,
                           const ctrl_utils::Tree<std::string, Frame>& tree,
                           std::map<std::string, Object>* objects,
                           bool freeze_children_in_world) {
  // Get transform from old frame to parent
  Object& obj = objects->at(frame);
  const Isometry T_old_to_parent = obj.T_to_parent();

  // Compute transform from parent to world
  const std::optional<std::string>& parent = tree.parent(frame);
  if (!parent && frame != kWorldFrame) {
    throw std::runtime_error("World::set_T_to_world(): Frame " + frame +
                             " has no parent.");
  }
  const Isometry T_parent_to_world =
      parent ? ComputeTransform(*parent, kWorldFrame, tree, *objects)
             : Isometry::Identity();

  // Compute and set transform from new frame to parent
  const Isometry T_new_to_parent = T_parent_to_world.inverse() * T_to_world;
  obj.set_T_to_parent(T_new_to_parent);

  if (!freeze_children_in_world) return;

  // Compute transform from old frame to new frame
  const Isometry T_old_to_new = T_new_to_parent.inverse() * T_old_to_parent;

  for (auto& key_child : tree.children(frame)) {
    // Get transform from child to old frame
    Object& child = objects->at(key_child.first);
    const Isometry& T_child_to_old = child.T_to_parent();

    // Compute and set transform from child to new frame
    const Isometry T_child_to_new = T_old_to_new * T_child_to_old;
    child.set_T_to_parent(T_child_to_new);
  }
}

void World::set_T_to_world(const std::map<std::string, Object>& object_poses,
                           const ctrl_utils::Tree<std::string, Frame>& tree,
                           std::map<std::string, Object>* objects,
                           bool freeze_children_in_world) {
  using SearchType = ctrl_utils::Tree<std::string, Frame>::SearchType;
  for (const auto& key_val : tree.nodes(SearchType::kBreadthFirstSearch)) {
    const std::string& frame = key_val.first;

    const bool is_in_object_poses =
        object_poses.find(frame) != object_poses.end();
    const bool is_in_objects = objects->find(frame) != objects->end();
    if (!is_in_object_poses || !is_in_objects) continue;

    const logic_opt::Object& obj = object_poses.at(frame);
    logic_opt::World::set_T_to_world(frame, obj.T_to_parent(), tree, objects,
                                     freeze_children_in_world);
  }
}

void World::set_controller_frames(const std::string& control_frame,
                                  const std::string& target_frame, size_t t) {
  controller_frames_[t].first = control_frame;
  controller_frames_[t].second = target_frame;
}

Isometry World::T_to_world(const std::string& name_frame,
                           Eigen::Ref<const Eigen::MatrixXd> X,
                           size_t t) const {
  return T_to_frame(name_frame, kWorldFrame, X, t);
}

Isometry World::T_to_parent(const std::string& name_frame,
                            Eigen::Ref<const Eigen::MatrixXd> X,
                            size_t t) const {
  const Frame& frame = frames_[t].at(name_frame);
  return frame.is_variable() ? T_control_to_target(X, frame.idx_var())
                             : objects_->at(frame.name()).T_to_parent();
}

Isometry World::T_to_frame(const std::string& from_frame,
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
    T_result =
        T_to_world(to_frame, X, t).inverse() * T_to_world(from_frame, X, t);
    return T_result;
  }

  for (const std::pair<std::string, Frame>& key_val :
       frames_[t].ancestors(descendant_frame)) {
    const std::string& frame = key_val.first;
    if (frame == ancestor_frame) break;
    T_result = T_to_parent(frame, X, t) * T_result;
  }
  if (ancestor_frame == from_frame) {
    T_result = T_result.inverse();
  }
  return T_result;
}

Isometry World::T_control_to_target(Eigen::Ref<const Eigen::MatrixXd> X,
                                    size_t t) const {
  return Isometry(NormalizedQuaternion(X, t), Position(X, t));
}

Eigen::Vector3d World::Position(const std::string& of_frame,
                                const std::string& in_frame,
                                Eigen::Ref<const Eigen::MatrixXd> X,
                                size_t t) const {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  bool frame_reached = false;
  for (const std::pair<std::string, Frame>& key_val :
       frames_[t].ancestors(of_frame)) {
    if (key_val.first == in_frame) {
      frame_reached = true;
      break;
    }

    const Frame& frame = key_val.second;
    p = T_to_parent(frame.name(), X, t) * p;
  }

  if (!frame_reached) {
    throw std::invalid_argument("World::Position(): frame \"" + of_frame +
                                "\" must be an descendant of \"" + in_frame +
                                "\"");
  }
  return p;
}

Eigen::Quaterniond World::Orientation(const std::string& of_frame,
                                      const std::string& in_frame,
                                      Eigen::Ref<const Eigen::MatrixXd> X,
                                      size_t t) const {
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  bool frame_reached = false;
  for (const std::pair<std::string, Frame>& key_val :
       frames_[t].ancestors(of_frame)) {
    if (key_val.first == in_frame) {
      frame_reached = true;
      break;
    }

    const Frame& frame = key_val.second;
    if (frame.is_variable()) {
      q = NormalizedQuaternion(X, frame.idx_var()) * q;
    } else {
      q = objects_->at(frame.name()).T_to_parent().rotation() * q;
    }
  }

  if (!frame_reached) {
    throw std::invalid_argument("World::Orientation(): frame \"" + of_frame +
                                "\" must be an descendant of \"" + in_frame +
                                "\"");
  }
  return q;
}

std::ostream& operator<<(std::ostream& os, const World& world) {
  for (size_t t = 0; t < world.num_timesteps(); t++) {
    const auto& frame_pair = world.controller_frames(t);
    os << "Frame: " << t << std::endl
       << "  Control: " << frame_pair.first << std::endl
       << "  Target: " << frame_pair.second << std::endl;

    const auto& frame_tree = world.frames(t);
    os << "  Tree:" << std::endl;
    frame_tree.printf(
        os,
        [&world](const std::string& key, const Frame& frame) -> std::string {
          std::stringstream ss;
          if (frame.is_variable()) {
            ss << "(idx_var: " << frame.idx_var() << ")\t";
          } else {
            ss << "(constant)\t";
          }
          if (world.objects()->find(key) != world.objects()->end()) {
            ss << world.objects()->at(key).T_to_parent();
          }
          return ss.str();
        });
  }
  return os;
}

}  // namespace logic_opt

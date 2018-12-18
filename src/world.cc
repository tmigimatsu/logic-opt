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

#include <algorithm>  // std::find
#include <iterator>   // std::begin, std::end
#include <limits>     // std::numeric_limits

namespace LogicOpt {

World::World(const std::shared_ptr<const std::map<std::string, SpatialDyn::RigidBody>>& objects,
      size_t T)
    : objects_(objects), frames_(T), controller_frames_(T) {

  for (Tree<std::string, Frame>& frames_t : frames_) {
    frames_t.insert(kWorldFrame, Frame(kWorldFrame));
  }

  for (const auto& key_val : *objects_) {
    const std::string& name = key_val.first;
    const SpatialDyn::RigidBody& object = key_val.second;

    for (Tree<std::string, Frame>& frames_t : frames_) {
      frames_t.insert_child(kWorldFrame, name, Frame(name));
    }
  }
}

void World::AttachFrame(const std::string& name_frame, const std::string& name_target, size_t t) {
  for (size_t tt = t; tt < frames_.size(); tt++) {
    frames_[tt].set_parent(name_frame, name_target);
    frames_[tt].at(name_frame).set_idx_var(t);
  }
  set_controller_frames(name_frame, name_target, t);
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

Eigen::Matrix3Xd World::PositionJacobian(const std::string& name_frame,
                                         Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const {
  Eigen::Matrix3Xd J = Eigen::Matrix3Xd::Zero(3, X.size());
  auto chain = frames_[t].ancestors(name_frame);
  for (auto it = chain.begin(); it != chain.end(); ++it) {
    if (it->first == kWorldFrame) break;
    const Frame& frame = it->second;
    if (!frame.is_variable()) continue;

    auto J_pos = J.block<3,3>(0, 6 * frame.idx_var());
    J_pos = Orientation(*frames_[t].parent(frame.name()), kWorldFrame, X, t);

    auto J_ori = J.block<3,3>(0, 6 * frame.idx_var() + 3);
    J_ori = -Eigen::CrossMatrix(Position(name_frame, frame.name(), X, t));
  }
  return J;
}

// Eigen::Matrix3Xd World::OrientationJacobian(const std::string& name_frame,
//                                             Eigen::Ref<const Eigen::MatrixXd> X, size_t t) {
//   Eigen::Matrix3Xd J = Eigen::Matrix3Xd::Zero(3, X.size());
//   auto chain = frames_[t].ancestors(name_frame);
//   for (auto it = chain.begin(); it != chain.end(); ++it) {
//     if (it->first == kWorldFrame) break;
//     const Frame& frame = it->second;
//     if (!frame.is_variable()) continue;

//     auto J_pos = J.block<3,3>(0, 6 * frame.idx_var());
//     J_pos = Orientation(frames_[t].parent(frame.name()), kWorldFrame, X, t);

//     auto J_ori = J.block<3,3>(0, 6 * frame.idx_var() + 3);
//     J_ori = -Eigen::CrossMatrix(Position(name_frame, frame.name(), X, t));
//   }
//   return J;
// }

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

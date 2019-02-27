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
#include <cmath>      // std::fabs
#include <iterator>   // std::begin, std::end
#include <limits>     // std::numeric_limits

namespace LogicOpt {

const std::string World::kWorldFrame = "__world";

World::World(const std::shared_ptr<const std::map<std::string, Object>>& objects, size_t T)
    : objects_(objects), frames_(T), controller_frames_(T) {

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
    Eigen::Vector3d p = Position(name_frame, frame.name(), X, t);
    auto x_r = X.col(frame.idx_var()).tail<3>();
    double angle = x_r.norm();
    Eigen::Matrix3d R = Eigen::AngleAxisd(angle, x_r.normalized()).toRotationMatrix();
    if (std::fabs(angle) > 0) {
      J_ori = J_pos * (R * ctrl_utils::Eigen::CrossMatrix(p) / -(angle * angle) *
                       (x_r * x_r.transpose() + (R.transpose() - Eigen::Matrix3d::Identity()) *
                        ctrl_utils::Eigen::CrossMatrix(x_r)));
    }
  }
  return J;
}

std::array<Eigen::Matrix3d, 3> World::OrientationAngleAxisJacobians(Eigen::Ref<const Eigen::Vector3d> aa) const {
  std::array<Eigen::Matrix3d, 3> Js;
  double angle = aa.norm();
  if (angle == 0.) {
    for (size_t i = 0; i < Js.size(); i++) Js[i].setZero();
    return Js;
  }

  Eigen::Matrix3d R = Eigen::AngleAxisd(angle, aa.normalized()).toRotationMatrix();
  Eigen::Matrix3d aa_cross = ctrl_utils::Eigen::CrossMatrix(aa);
  Eigen::Matrix3d R_hat = R / (angle * angle);
  for (size_t i = 0; i < Js.size(); i++) {
    Js[i] = (aa[i] * aa_cross +
             ctrl_utils::Eigen::CrossMatrix(aa.cross(Eigen::Vector3d::Unit(i) - R.col(i)))) * R_hat;
  }
  return Js;
}

std::array<std::optional<Eigen::Matrix3d>, 5> World::RotationChain(const std::string& name_frame,
                                                                   size_t idx_var,
                                                                   Eigen::Ref<const Eigen::MatrixXd> X,
                                                                   size_t t1, size_t t2) const {
  // Matrix tuple (A, X_inv, B, X, C)
  std::array<std::optional<Eigen::Matrix3d>, 5> Rs;

  // Get ancestor chains from ee to world at t1 and t2
  std::vector<const std::pair<const std::string, Frame>*> chain1;
  std::vector<const std::pair<const std::string, Frame>*> chain2;
  auto ancestors1 = frames_[t1].ancestors(name_frame);
  auto ancestors2 = frames_[t2].ancestors(name_frame);
  for (auto it = ancestors1.begin(); it != ancestors1.end(); ++it) {
    chain1.push_back(&*it);
  }
  for (auto it = ancestors2.begin(); it != ancestors2.end(); ++it) {
    chain2.push_back(&*it);
  }

  // Find closest common ancestor between ee frames at t1 and t2
  int idx_end1 = chain1.size() - 1;
  int idx_end2 = chain2.size() - 1;
  while (idx_end1 >= 0 && idx_end2 >= 0 && chain1[idx_end1]->second == chain2[idx_end2]->second) {
    --idx_end1;
    --idx_end2;
  }
  ++idx_end1;
  ++idx_end2;
  // for (size_t i = 0; i < idx_end1; i++) {
  //   std::cout << chain1[i]->first << ":" << chain1[i]->second.idx_var();
  //   if (chain1[i]->second.idx_var() == idx_var) std::cout << "!";
  //   std::cout << " ";
  // }
  // if (idx_end1 < chain1.size()) std::cout << "(" << chain1[idx_end1]->first << ")";
  // std::cout << std::endl;
  // for (size_t i = 0; i < idx_end2; i++) {
  //   std::cout << chain2[i]->first << ":" << chain2[i]->second.idx_var();
  //   if (chain2[i]->second.idx_var() == idx_var) std::cout << "!";
  //   std::cout << " ";
  // }
  // if (idx_end2 < chain2.size()) std::cout << "(" << chain2[idx_end2]->first << ")";
  // std::cout << std::endl;

  // Construct A?, X_inv?
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  for (size_t i = 0; i < idx_end1; i++) {
    const Frame& frame = chain1[i]->second;
    if (frame.idx_var() == idx_var && frame.is_variable()) {
      Rs[0] = R.transpose();
      Rs[1] = T_to_parent(frame.name(), X, t1).linear().transpose();
      R.setIdentity();
      continue;
    }
    R = T_to_parent(frame.name(), X, t1).linear() * R;
  }

  // Construct B, X?, C?
  Rs[2] = R.transpose();
  // if (Rs[0]) {
  //   std::cout << *Rs[0] * *Rs[1] * *Rs[2] << std::endl << std::endl;
  // } else {
  //   std::cout << *Rs[2] << std::endl << std::endl;
  // }
  // std::cout << T_to_world(name_frame, X, t1).linear().transpose() << std::endl << std::endl;
  R.setIdentity();
  for (int i = idx_end2 - 1; i >= 0; i--) {
    const Frame& frame = chain2[i]->second;
    if (frame.idx_var() == idx_var && frame.is_variable()) {
      Rs[2] = *Rs[2] * R;
      Rs[3] = T_to_parent(frame.name(), X, t2).linear();
      R.setIdentity();
      continue;
    }
    R = R * T_to_parent(frame.name(), X, t2).linear();
  }

  // Finish B or C
  if (Rs[3]) {
    Rs[4] = R;
  } else {
    Rs[2] = *Rs[2] * R;
  }

  return Rs;
}

Eigen::Matrix3d World::OrientationTraceJacobian(const std::string& name_frame,
                                                size_t idx_var, Eigen::Ref<const Eigen::MatrixXd> X,
                                                size_t t1, size_t t2, double* trace) const {
  std::array<std::optional<Eigen::Matrix3d>, 5> Rs = RotationChain(name_frame, idx_var, X, t1, t2);

  Eigen::Matrix3d J;
  if (Rs[0] && !Rs[4]) {
    const Eigen::Matrix3d A     = *Rs[0];
    const Eigen::Matrix3d X_inv = *Rs[1];
    const Eigen::Matrix3d B     = *Rs[2];
    // std::cout << "A: " << std::endl << A << std::endl;
    // std::cout << "X_inv: " << std::endl << X_inv << std::endl;
    // std::cout << "B: " << std::endl << B << std::endl;
    // std::cout << std::endl << A * X_inv * B << std::endl << std::endl;
    // std::cout << T_to_world(name_frame, X, t1).linear().transpose() * T_to_world(name_frame, X, t2).linear() << std::endl << std::endl;

    Eigen::Matrix3d B_A_Xinv = B * A * X_inv;
    J = -(X_inv * B_A_Xinv).transpose();
    if (trace != nullptr) {
      *trace = (B_A_Xinv).diagonal().sum();
    }
  } else if (!Rs[0] && Rs[4]) {
    const Eigen::Matrix3d B  = *Rs[2];
    const Eigen::Matrix3d X_ = *Rs[3];
    const Eigen::Matrix3d C  = *Rs[4];
    // std::cout << "B: " << std::endl << B << std::endl;
    // std::cout << "X: " << std::endl << X << std::endl;
    // std::cout << "C: " << std::endl << C << std::endl;
    // std::cout << std::endl << B * X_ * C << std::endl << std::endl;
    // std::cout << T_to_world(name_frame, X, t1).linear().transpose() * T_to_world(name_frame, X, t2).linear() << std::endl << std::endl;

    Eigen::Matrix3d C_B = C * B;
    J = (C_B).transpose();
    if (trace != nullptr) {
      *trace = (C_B * X_).diagonal().sum();
    }
  } else if (Rs[0] && Rs[4]) {
    const Eigen::Matrix3d A     = *Rs[0];
    const Eigen::Matrix3d X_inv = *Rs[1];
    const Eigen::Matrix3d B     = *Rs[2];
    const Eigen::Matrix3d X_    = *Rs[3];
    const Eigen::Matrix3d C     = *Rs[4];
    // std::cout << "A: " << std::endl << A << std::endl;
    // std::cout << "X_inv: " << std::endl << X_inv << std::endl;
    // std::cout << "B: " << std::endl << B << std::endl;
    // std::cout << "X: " << std::endl << X << std::endl;
    // std::cout << "C: " << std::endl << C << std::endl;
    // std::cout << std::endl << A * X_inv * B * X_ * C << std::endl << std::endl;
    // std::cout << T_to_world(name_frame, X, t1).linear().transpose() * T_to_world(name_frame, X, t2).linear() << std::endl << std::endl;

    Eigen::Matrix3d A_Xinv = A * X_inv;
    Eigen::Matrix3d B_X_C_A_Xinv = B * X_ * C * A_Xinv;
    J = (C * A_Xinv * B - X_inv * B_X_C_A_Xinv).transpose();
    if (trace != nullptr) {
      *trace = (B_X_C_A_Xinv).diagonal().sum();
    }
  } else {
    J = Eigen::Matrix3d::Zero();
  }

  return J;
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

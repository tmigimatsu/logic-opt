/**
 * collision_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/collision_constraint.h"

#include <algorithm>  // std::find, std::iter_swap, std::max, std::min
#include <exception>  // std::runtime_error
#include <limits>     // std::numeric_limits
#include <sstream>    // std::stringstream
#include <utility>    // std::move

#include <ctrl_utils/math.h>

namespace {

const double kH = 1e-5;
const double kH_ori = 1e-2;

const double kMaxDist = 0.1;

std::string ControlFrame(const logic_opt::World3& world, size_t t) {
  for (int tt = t; tt >= 0; tt--) {
    if (!world.control_frame(tt).empty()) return world.control_frame(tt);
  }
  throw std::runtime_error("CollisionConstraint::ControlFrame(): No control frame found.");
}

std::string TargetFrame(const logic_opt::World3& world, size_t t) {
  for (int tt = t; tt >= 0; tt--) {
    if (!world.target_frame(tt).empty()) return world.target_frame(tt);
  }
  throw std::runtime_error("CollisionConstraint::TargetFrame(): No target frame found.");
}

}  // namespace

namespace logic_opt {

CollisionConstraint::CollisionConstraint(World3& world, size_t t_collision, bool ignore_control_target,
                                         const std::set<std::string>& ignore_obstacles)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_collision, kNumTimesteps,
                      ControlFrame(world, t_collision), TargetFrame(world, t_collision),
                      "constraint_t" + std::to_string(t_collision) + "_collision"),
      ignore_control_target_(ignore_control_target),
      world_(world) {

  // Find the possible collision frames
  const ctrl_utils::Tree<std::string, Frame>& frames = world.frames(t_collision);
  for (const std::pair<std::string, Frame>& key_val : frames.values()) {
    const std::string& frame = key_val.first;
    if (frame == World3::kWorldFrame || !world_.objects()->at(frame).collision ||
        ignore_obstacles.find(frame) != ignore_obstacles.end()) continue;

    // Descendants of the control frame are fixed to the ee and can't collide
    if (frames.is_descendant(frame, control_frame())) {
      ee_frames_.push_back(frame);
    } else {
      object_frames_.push_back(frame);
    }
  }

  contact_ = ComputeError(Eigen::MatrixXd::Zero(kDof, world.num_timesteps()),
                          &ee_closest_, &object_closest_);
}

void CollisionConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                   Eigen::Ref<Eigen::VectorXd> constraints) {
  contact_ = ComputeError(X, &ee_closest_, &object_closest_);

  const double dist = contact_ ? contact_->depth : -kMaxDist;
  constraints(0) = 0.5 * std::abs(dist) * dist;

  Constraint::Evaluate(X, constraints);
}

void CollisionConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                   Eigen::Ref<Eigen::VectorXd> Jacobian) {
  if (!contact_ || ee_closest_.empty() || object_closest_.empty()) {
    Constraint::Jacobian(X, Jacobian);
    return;
  }
  // Jacobian.head<3>() = -std::abs(contact_->depth) * contact_->normal;

  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < kDof; i++) {
    const double h = i < 3 ? kH : kH_ori;
    double& x_it = X_h(i, t_start());
    const double x_it_0 = x_it;
    x_it = x_it_0 + h;
    const double x_err_hp = ComputeDistance(X_h, ee_closest_, object_closest_, kMaxDist);
    x_it = x_it_0 - h;
    const double x_err_hn = ComputeDistance(X_h, ee_closest_, object_closest_, kMaxDist);
    x_it = x_it_0;
    const double dx_h = 0.5 * (std::abs(x_err_hp) * x_err_hp - std::abs(x_err_hn) * x_err_hn) / (2. * h);

    if (contact_->depth > 0 && std::abs(dx_h) > 0.5) {
      // If the contact distance is small, ncollide will clip it to 0, which
      // will make either x_err_hp or x_err_hn 0, and dx_h will become huge. Get
      // around this by leaving the Jacobian element 0.

      std::stringstream ss;
      ss << "CollisionConstraint::Jacobian(): Ill-conditioned J(" << i << ","
         << t_start() << "): " << dx_h << " " << " " << x_err_hp << " " << x_err_hn << " " << std::endl;
      // throw std::runtime_error(ss.str());
    }

    Jacobian(i) = dx_h;
  }
  Constraint::Jacobian(X, Jacobian);
}

void CollisionConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                          Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0  0  0  0  0  0
  // j:  x  y  z wx wy wz
  const size_t var_t = kDof * t_start();
  idx_j.setLinSpaced(kDof, var_t, var_t + kDof - 1);
}

std::optional<ncollide3d::query::Contact>
CollisionConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                  std::string* out_ee_closest,
                                  std::string* out_object_closest) {

  // Maximum distance: deepest penetration
  double max_dist = -std::numeric_limits<double>::infinity();
  std::optional<ncollide3d::query::Contact> max_contact;
  const std::string* ee_closest = nullptr;
  const std::string* object_closest = nullptr;

  // Iterate over ee/object pairs
  for (const std::string& ee_frame : ee_frames_) {
    for (const std::string& object_frame : object_frames_) {
      if (ignore_control_target_ && ee_frame == control_frame() && object_frame == target_frame()) continue;

      // Filter out objects farther than -max_dist
      const double proximity_dist = std::min(std::max(0., -max_dist), kMaxDist);
      std::optional<ncollide3d::query::Contact> contact;
      const double dist = ComputeDistance(X, ee_frame, object_frame, proximity_dist, &contact);
      if (!contact || dist <= max_dist) continue;

      // Update deepest penetration
      max_dist = dist;
      max_contact = contact;
      ee_closest = &ee_frame;
      object_closest = &object_frame;
    }
  }

  // Output closest ee/object pair
  if (max_dist == -std::numeric_limits<double>::infinity()) {
    if (out_ee_closest != nullptr) out_ee_closest->clear();
    if (out_object_closest != nullptr) out_object_closest->clear();
  } else {
    if (out_ee_closest != nullptr) {
      *out_ee_closest = *ee_closest;
    }
    if (out_object_closest != nullptr) {
      *out_object_closest = *object_closest;
    }
  }

  // Reorder closest ee/object for next iteration
  if (ee_closest != nullptr) {
    auto it_ee = std::find(ee_frames_.begin(), ee_frames_.end(), *ee_closest);
    if (it_ee != ee_frames_.begin()) {
      std::iter_swap(it_ee, ee_frames_.begin());
    }
  }
  if (object_closest != nullptr) {
    auto it_object = std::find(object_frames_.begin(), object_frames_.end(), *object_closest);
    if (it_object != object_frames_.begin()) {
      std::iter_swap(it_object, object_frames_.begin());
    }
  }

  return max_contact;
}

double CollisionConstraint::ComputeDistance(Eigen::Ref<const Eigen::MatrixXd> X,
                                            const std::string& ee_frame,
                                            const std::string& object_frame,
                                            double max_dist,
                                            std::optional<ncollide3d::query::Contact>* out_contact) const {

  const Object3& ee = world_.objects()->at(ee_frame);
  const Object3& object = world_.objects()->at(object_frame);
  const Eigen::Isometry3d T_object_to_ee = world_.T_to_frame(object_frame, ee_frame, X, t_start());

  const auto contact = ncollide3d::query::contact(Eigen::Isometry3d::Identity(), *ee.collision,
                                                  T_object_to_ee, *object.collision, max_dist);

  // Depth is positive if penetrating, negative otherwise
  const double dist = contact ? contact->depth : -kMaxDist;

  if (out_contact != nullptr) {
    *out_contact = std::move(contact);
  }
  return dist;
}

}  // namespace logic_opt

/**
 * collision_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/collision_constraint.h"

#include <algorithm>  // std::max, std::min
#include <limits>     // std::numeric_limits

#include <ctrl_utils/math.h>

#define COLLISION_CONSTRAINT_SYMMETRIC_DIFFERENCE

namespace {

#ifdef COLLISION_CONSTRAINT_SYMMETRIC_DIFFERENCE
const double kH[6] = {1e-5, 1e-5, 1e-5, 1e-2, 1e-2, 1e-2};
#else  // COLLISION_CONSTRAINT_SYMMETRIC_DIFFERENCE
const double kH = 1e-4;
#endif  // COLLISION_CONSTRAINT_SYMMETRIC_DIFFERENCE

const double kMaxDist = 100.;

}  // namespace

namespace logic_opt {

CollisionConstraint::CollisionConstraint(World3& world, size_t t_collision)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_collision, kNumTimesteps,
                      world.control_frame(t_collision), world.target_frame(t_collision),
                      "constraint_t" + std::to_string(t_collision) + "_collision"),
      world_(world) {

  // Find the possible collision frames
  const ctrl_utils::Tree<std::string, Frame>& frames = world.frames(t_collision);
  for (const std::pair<std::string, Frame>& key_val : frames.values()) {
    const std::string& frame = key_val.first;
    if (frame == World3::kWorldFrame || !world_.objects()->at(frame).collision) continue;

    // Descendants of the control frame are fixed to the ee and can't collide
    if (frames.is_descendant(frame, control_frame())) {
      ee_frames_.push_back(frame);
    } else {
      objects_.push_back(frame);
    }
  }

  ComputeError(Eigen::MatrixXd::Zero(kDof, world.num_timesteps()),
               &ee_closest_, &object_closest_);
}

void CollisionConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> constraints) {
  x_err_ = ComputeError(X, &ee_closest_, &object_closest_);
  // constraints(0) = x_err_;
  constraints(0) = 0.5 * ctrl_utils::Signum(x_err_) * x_err_ * x_err_;

  Constraint::Evaluate(X, constraints);
}

void CollisionConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                   Eigen::Ref<Eigen::VectorXd> Jacobian) {
  if (ee_closest_.empty() || object_closest_.empty()) {
    Constraint::Jacobian(X, Jacobian);
    return;
  }

  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < kDof; i++) {
    double& x_it = X_h(i, t_start());
    const double x_it_0 = x_it;
    x_it = x_it_0 + kH[i];
    const double x_err_hp = ComputeDistance(X_h, ee_closest_, object_closest_);
    // const double x_err_hp = ComputeError(X_h);
#ifdef COLLISION_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0 - kH[i];
    const double x_err_hn = ComputeDistance(X_h, ee_closest_, object_closest_);
    // const double x_err_hn = ComputeError(X_h);
#endif  // COLLISION_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0;

#ifdef COLLISION_CONSTRAINT_SYMMETRIC_DIFFERENCE
    Jacobian(i) = 0.5 * (ctrl_utils::Signum(x_err_hp) * x_err_hp * x_err_hp - ctrl_utils::Signum(x_err_hn) * x_err_hn * x_err_hn) / (2. * kH[i]);
#else  // COLLISION_CONSTRAINT_SYMMETRIC_DIFFERENCE
    Jacobian(i) = 0.5 * (ctrl_utils::Signum(x_err_hp) * x_err_hp * x_err_hp - ctrl_utils::Signum(x_err_) * x_err_ * x_err_) / kH;
#endif  // COLLISION_CONSTRAINT_SYMMETRIC_DIFFERENCE
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

double CollisionConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                         std::string* ee_closest,
                                         std::string* object_closest) const {

  double max_dist = -std::numeric_limits<double>::infinity();
  ee_closest->clear();
  object_closest->clear();
  for (const std::string& ee_frame : ee_frames_) {
    const Object3& ee = world_.objects()->at(ee_frame);
    for (const std::string& object_frame : objects_) {
      if (ee_frame == control_frame() && object_frame == target_frame()) continue;

      const Object3& object = world_.objects()->at(object_frame);
      const Eigen::Isometry3d T_object_to_ee = world_.T_to_frame(object_frame, ee_frame, X, t_start());

      const double proximity_dist = std::min(std::max(0., -max_dist), kMaxDist);
      // auto proximity = ncollide3d::query::proximity(Eigen::Isometry3d::Identity(), *ee.collision,
      //                                               T_object_to_ee, *object.collision, proximity_dist);
      // if (proximity == ncollide3d::query::Proximity::Disjoint) continue;

      const auto contact = ncollide3d::query::contact(Eigen::Isometry3d::Identity(), *ee.collision,
                                                      T_object_to_ee, *object.collision, proximity_dist);
      if (!contact) continue;

      const double dist = contact->depth;
      if (dist <= max_dist) continue;

      max_dist = dist;
      if (ee_closest != nullptr) *ee_closest = ee_frame;
      if (object_closest != nullptr) *object_closest = object_frame;
    }
  }

  if (max_dist == -std::numeric_limits<double>::infinity()) {
    std::cerr << "NO CONTACT?!" << std::endl;
    max_dist = 0.;
    ee_closest->clear();
    object_closest->clear();
  }

  return max_dist;
}

double CollisionConstraint::ComputeDistance(Eigen::Ref<const Eigen::MatrixXd> X,
                                            const std::string& ee_frame,
                                            const std::string& object_frame) const {

  const Object3& ee = world_.objects()->at(ee_frame);
  const Object3& object = world_.objects()->at(object_frame);
  const Eigen::Isometry3d T_object_to_ee = world_.T_to_frame(object_frame, ee_frame, X, t_start());

  const auto contact = ncollide3d::query::contact(Eigen::Isometry3d::Identity(), *ee.collision,
                                                  T_object_to_ee, *object.collision, kMaxDist);
  return contact ? contact->depth : 0.;
}

}  // namespace logic_opt

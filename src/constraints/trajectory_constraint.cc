/**
 * trajectory_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/trajectory_constraint.h"

#include <algorithm>  // std::find, std::swap_iter, std::max, std::min
#include <exception>  // std::runtime_error
#include <limits>     // std::numeric_limits
#include <sstream>    // std::stringstream
#include <utility>    // std::move

namespace {

const double kH = 1e-5;
const double kH_ori = 1e-2;

const double kMaxDist = 0.01;

std::string ControlFrame(const logic_opt::World3& world, size_t t) {
  for (int tt = t; tt >= 0; tt--) {
    if (!world.control_frame(tt).empty()) return world.control_frame(tt);
  }
  throw std::runtime_error("TrajectoryConstraint::ControlFrame(): No control frame found.");
}

std::string TargetFrame(const logic_opt::World3& world, size_t t) {
  for (int tt = t; tt >= 0; tt--) {
    if (!world.target_frame(tt).empty()) return world.target_frame(tt);
  }
  throw std::runtime_error("TrajectoryConstraint::TargetFrame(): No target frame found.");
}

}  // namespace

namespace logic_opt {

TrajectoryConstraint::TrajectoryConstraint(World3& world, size_t t_trajectory)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_trajectory, kNumTimesteps,
                      ControlFrame(world, t_trajectory), TargetFrame(world, t_trajectory),
                      "constraint_t" + std::to_string(t_trajectory) + "_trajectory"),
      world_(world) {

  // Find the possible collision frames
  const ctrl_utils::Tree<std::string, Frame>& frames = world.frames(t_trajectory);
  for (const std::pair<std::string, Frame>& key_val : frames.values()) {
    const std::string& frame = key_val.first;
    if (frame == World3::kWorldFrame || !world_.objects()->at(frame).collision) continue;

    // Descendants of the control frame are fixed to the ee and can't collide
    if (frames.is_descendant(frame, control_frame())) {
      ee_frames_.push_back(frame);
    } else {
      object_frames_.push_back(frame);
    }
  }

  const Object3& ee = world_.objects()->at(control_frame());
  const ncollide3d::shape::TriMesh trimesh = ee.collision->to_trimesh();
  augmented_ee_points_.reserve(2 * trimesh.num_points());
  for (size_t i = 0; i < trimesh.num_points(); i++) {
    const Eigen::Ref<const Eigen::Vector3d> point = trimesh.point(i);
    augmented_ee_points_.push_back({point(0), point(1), point(2)});
  }
  for (size_t i = 0; i < trimesh.num_points(); i++) {
    augmented_ee_points_.push_back({0., 0., 0.});
  }
}

void TrajectoryConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                    Eigen::Ref<Eigen::VectorXd> constraints) {

  constraints(0) = ComputeError(X, &contact_, &object_closest_);

  Constraint::Evaluate(X, constraints);
}

void TrajectoryConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                    Eigen::Ref<Eigen::VectorXd> Jacobian) {
  if (!contact_ || object_closest_.empty()) {
    Constraint::Jacobian(X, Jacobian);
    return;
  }

  Eigen::MatrixXd X_h = X;
  for (size_t t = 0; t < 2; t++) {
    for (size_t i = 0; i < kDof; i++) {
      const double h = i < 3 ? kH : kH_ori;
      double& x_it = X_h(i, t_start() + t);
      const double x_it_0 = x_it;
      x_it = x_it_0 + h;
      const double x_err_hp = ComputeJacobianError(X_h, control_frame(), object_closest_, kMaxDist);
      x_it = x_it_0 - h;
      const double x_err_hn = ComputeJacobianError(X_h, control_frame(), object_closest_, kMaxDist);
      x_it = x_it_0;
      const double dx_h = (x_err_hp - x_err_hn) / (2. * h);

      if (contact_->depth > 0 && std::abs(dx_h) > 0.5) {
        // If the contact distance is small, ncollide will clip it to 0, which
        // will make either x_err_hp or x_err_hn 0, and dx_h will become huge. Get
        // around this by leaving the Jacobian element 0.

        std::stringstream ss;
        ss << "TrajectoryConstraint::Jacobian(): Ill-conditioned J(" << i << ","
           << t_start() + t << "): " << dx_h << " " << " " << x_err_hp << " " << x_err_hn << " " << std::endl;
        std::cerr << ss.str();
        // throw std::runtime_error(ss.str());
      }

      Jacobian(kDof*t + i) = dx_h;
    }
  }
  Constraint::Jacobian(X, Jacobian);
}

void TrajectoryConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                           Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0  0  0  0  0  0       0       0       0       0       0       0
  // j:  x  y  z wx wy wz  x_next  y_next  z_next wx_next wy_next wz_next
  const size_t var_t = kDof * t_start();
  idx_j.setLinSpaced(2 * kDof, var_t, var_t + 2 * kDof - 1);
}

double TrajectoryConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                          std::optional<ncollide3d::query::Contact>* out_contact,
                                          std::string* out_object_closest) {

  double max_dist = -std::numeric_limits<double>::infinity();
  std::optional<ncollide3d::query::Contact> max_contact;

  const std::string& ee_frame = control_frame();
  const ConvexHull ee_convex_hull = ComputeConvexHull(X, ee_frame);

  const std::string& object_frame = target_frame();
  for (const std::string& object_frame : object_frames_) {
    const double proximity_dist = std::min(std::max(0., -max_dist), kMaxDist);

    std::optional<ncollide3d::query::Contact> contact;
    const double dist = ComputeDistance(X, ee_frame, object_frame, ee_convex_hull, proximity_dist, &contact);
    if (!contact || dist <= max_dist) continue;

    max_dist = contact->depth;
    max_contact = contact;
    if (out_object_closest != nullptr) {
      *out_object_closest = object_frame;
    }
  }

  if (out_contact != nullptr) {
    *out_contact = std::move(max_contact);
  }

  // Reorder closest object for next iteration
  if (out_object_closest != nullptr && max_contact) {
    auto it_object = std::find(object_frames_.begin(), object_frames_.end(), *out_object_closest);
    if (it_object != object_frames_.begin()) {
      std::iter_swap(it_object, object_frames_.begin());
    }
  }

  if (!max_contact) {
    if (out_object_closest != nullptr) out_object_closest->clear();
    max_dist = -kMaxDist;
  }
  return 0.5 * std::abs(max_dist) * max_dist;
}

TrajectoryConstraint::ConvexHull
TrajectoryConstraint::ComputeConvexHull(Eigen::Ref<const Eigen::MatrixXd> X,
                                        const std::string& ee_frame) {

  const Eigen::Isometry3d T_ee_next_to_ee = world_.T_to_frame(target_frame(), ee_frame, X, t_start()) *
                                            world_.T_to_frame(ee_frame, target_frame(), X, t_start() + 1);

  if (augmented_ee_points_.size() == 1) {
    std::vector<std::array<double, 3>> ee_points = augmented_ee_points_.front();
    const size_t num_points = ee_points.size() / 2;
    Eigen::Map<const Eigen::Matrix3Xd> points(ee_points[0].data(), 3, num_points);
    Eigen::Map<Eigen::Matrix3Xd> points_next(ee_points[num_points].data(), 3, num_points);
    for (size_t i = 0; i < num_points; i++) {
      points_next.col(i) = T_ee_next_to_ee * points.col(i);
    }
#ifdef LOGIC_OPT_TRAJECTORY_CONVEX_HULL
    return ncollide3d::shape::ConvexHull(ee_points);
#else  // LOGIC_OPT_TRAJECTORY_CONVEX_HULL
    return ncollide3d::transformation::convex_hull(ee_points);
#endif  // LOGIC_OPT_TRAJECTORY_CONVEX_HULL
  }

  throw std::runtime_error("TrajectoryConstraint::ComputeConvexHull(): Not implemented.");
}

double TrajectoryConstraint::ComputeDistance(Eigen::Ref<const Eigen::MatrixXd> X,
                                             const std::string& ee_frame,
                                             const std::string& object_frame,
                                             const ConvexHull& ee_convex_hull,
                                             double max_dist,
                                             std::optional<ncollide3d::query::Contact>* out_contact) const {
  const Object3& object = world_.objects()->at(object_frame);
  const Eigen::Isometry3d T_object_to_ee = world_.T_to_frame(object_frame, ee_frame, X, t_start());
  // TODO make sure object isn't moving

  const auto contact = ncollide3d::query::contact(Eigen::Isometry3d::Identity(), ee_convex_hull,
                                                  T_object_to_ee, *object.collision, max_dist);

  // Depth is positive if penetrating, negative otherwise
  const auto dist = contact ? contact->depth : -kMaxDist;

  if (out_contact != nullptr) {
    *out_contact = std::move(contact);
  }
  return dist;
}

double TrajectoryConstraint::ComputeJacobianError(Eigen::Ref<const Eigen::MatrixXd> X,
                                                  const std::string& ee_frame,
                                                  const std::string& object_frame,
                                                  double max_dist) {
  const ConvexHull ee_convex_hull = ComputeConvexHull(X, ee_frame);
  const double dist = ComputeDistance(X, ee_frame, object_frame, ee_convex_hull, max_dist);
  return 0.5 * std::abs(dist) * dist;
}

}  // namespace logic_opt

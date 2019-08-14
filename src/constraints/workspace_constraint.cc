/**
 * workspace_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/workspace_constraint.h"

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

const double kWorkspaceRadius = 0.4;

std::string ControlFrame(const logic_opt::World3& world, size_t t) {
  for (int tt = t; tt >= 0; tt--) {
    if (!world.control_frame(tt).empty()) return world.control_frame(tt);
  }
  throw std::runtime_error("WorkspaceConstraint::ControlFrame(): No control frame found.");
}

std::string TargetFrame(const logic_opt::World3& world, size_t t) {
  for (int tt = t; tt >= 0; tt--) {
    if (!world.target_frame(tt).empty()) return world.target_frame(tt);
  }
  throw std::runtime_error("WorkspaceConstraint::TargetFrame(): No target frame found.");
}

}  // namespace

namespace logic_opt {

WorkspaceConstraint::WorkspaceConstraint(World3& world, size_t t_workspace, const std::string& name_ee)
    : FrameConstraint(kNumConstraints, kDof * (t_workspace + 1), t_workspace, kNumTimesteps,
                      ControlFrame(world, t_workspace), TargetFrame(world, t_workspace),
                      "constraint_t" + std::to_string(t_workspace) + "_workspace"),
      name_ee_(name_ee),
      world_(world) {

}

void WorkspaceConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                   Eigen::Ref<Eigen::VectorXd> constraints) {
  constraints(0) = ComputeError(X);

  Constraint::Evaluate(X, constraints);
}

void WorkspaceConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                   Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Eigen::MatrixXd X_h = X;
  for (size_t t = 0; t <= t_start(); t++) {
    for (size_t i = 0; i < kDof; i++) {
      const double h = i < 3 ? kH : kH_ori;
      double& x_it = X_h(i, t);
      const double x_it_0 = x_it;
      x_it = x_it_0 + h;
      const double x_err_hp = ComputeError(X_h);
      x_it = x_it_0 - h;
      const double x_err_hn = ComputeError(X_h);
      x_it = x_it_0;
      const double dx_h = (x_err_hp - x_err_hn) / (2. * h);
      Jacobian(kDof * t + i) = dx_h;
    }
  }
  Constraint::Jacobian(X, Jacobian);
}

void WorkspaceConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                          Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0  0  0  0  0  0
  // j:  x  y  z wx wy wz
  const size_t var_t = kDof * t_start();
  idx_j.setLinSpaced(kDof * (t_start() + 1), 0, kDof * (t_start() + 1) - 1);
}

double WorkspaceConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const {
  const Eigen::Vector3d x_t = world_.Position(name_ee_, world_.kWorldFrame, X, t_start());
  const double dist = x_t.norm() - kWorkspaceRadius;
  return 0.5 * std::abs(dist) * dist;
}

}  // namespace logic_opt

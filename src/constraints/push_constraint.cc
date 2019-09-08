/**
 * push_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/push_constraint.h"

#include <cmath>  // std::exp

#include <ctrl_utils/math.h>

#include "logic_opt/constraints/collision_constraint.h"
#include "logic_opt/constraints/touch_constraint.h"
#include "logic_opt/constraints/workspace_constraint.h"

namespace {

const double kH = 1e-4;
const double kH_ori = 1e-1;

const double kNormalDepthMargin = 0.01;
const double kNormalDotEpsilon = 0.05;

std::vector<std::unique_ptr<logic_opt::Constraint>>
InitializeConstraints(logic_opt::World3& world, size_t t_push, const std::string& name_pusher,
                      const std::string& name_pushee, const std::string& name_target,
                      logic_opt::PushConstraint& push_constraint) {
  using namespace logic_opt;

  world.ReserveTimesteps(t_push + TouchConstraint::kNumTimesteps +
                         PushConstraint::DestinationConstraint::kNumTimesteps);

  std::vector<std::unique_ptr<Constraint>> constraints;
  constraints.emplace_back(new TouchConstraint(world, t_push, name_pusher, name_pushee));

  constraints.emplace_back(new PushConstraint::ContactAreaConstraint(world, t_push,
                                                                     name_pusher, name_pushee,
                                                                     push_constraint));
  constraints.emplace_back(new PushConstraint::AlignmentConstraint(world, t_push + 1));
  constraints.emplace_back(new PushConstraint::DestinationConstraint(world, t_push + 1,
                                                                     name_pushee, name_target,
                                                                     push_constraint));

  constraints.emplace_back(new CollisionConstraint(world, t_push));
  constraints.emplace_back(new CollisionConstraint(world, t_push + 1));
  constraints.emplace_back(new WorkspaceConstraint(world, t_push + 1, "ee"));

  world.set_controller("push_1", t_push);
  world.set_controller("push_2", t_push + 1);

  return constraints;
}

}  // namespace

namespace logic_opt {

PushConstraint::PushConstraint(World3& world, size_t t_push, const std::string& name_pusher,
                               const std::string& name_pushee, const std::string& name_target)
    : MultiConstraint(InitializeConstraints(world, t_push, name_pusher, name_pushee, name_target, *this),
                                            "constraint_t" + std::to_string(t_push) + "_push"),
      name_pusher_(name_pusher),
      name_pushee_(name_pushee),
      world_(world) {
  if (name_pusher == world.kWorldFrame) {
    throw std::invalid_argument("PushConstraint::PushConstraint(): " + world.kWorldFrame +
                                " cannot be the pusher frame.");
  } else if (name_pushee == world.kWorldFrame) {
    throw std::invalid_argument("PushConstraint::PushConstraint(): " + world.kWorldFrame +
                                " cannot be the pushee frame.");
  }
}

PushConstraint::ContactAreaConstraint::ContactAreaConstraint(World3& world, size_t t_contact,
                                                             const std::string& name_control,
                                                             const std::string& name_target,
                                                             PushConstraint& push_constraint)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_contact, kNumTimesteps,
                      name_control, name_target,
                      "constraint_t" + std::to_string(t_contact) + "_push_contact_area"),
      world_(world),
      push_constraint_(push_constraint) {
  world.ReserveTimesteps(t_contact + kNumTimesteps);
  world.AttachFrame(name_control, name_target, t_contact);
}

void PushConstraint::ContactAreaConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> constraints) {
  x_err_ = ComputeError(X);
  constraints(0) = 0.5 * x_err_.squaredNorm() - 1e-6;

  Constraint::Evaluate(X, constraints);
}

void PushConstraint::ContactAreaConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> Jacobian) {
  // Jacobian for pusher position
  Jacobian.head<3>() = -x_err_;

  Eigen::MatrixXd X_h = X;
  for (size_t i = 3; i < kDof; i++) {
    double& x_it = X_h(i, t_start());
    const double x_it_0 = x_it;
    x_it = x_it_0 + kH_ori;
    const double x_err_hp = ComputeError(X_h).squaredNorm();
    x_it = x_it_0 - kH_ori;
    const double x_err_hn = ComputeError(X_h).squaredNorm();
    x_it = x_it_0;

    const double dx_h = 0.5 * (x_err_hp - x_err_hn) / (2. * kH_ori);
    Jacobian(i) = dx_h;
  }
  Constraint::Jacobian(X, Jacobian);
}

void PushConstraint::ContactAreaConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                                            Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0  0  0  0  0  0
  // j:  x  y  z wx wy wz
  const size_t var_t = kDof * t_start();
  idx_j.setLinSpaced(var_t, var_t + kDof - 1);
}

Eigen::Vector3d PushConstraint::ContactAreaConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                                                    Eigen::Vector3d* contact_pusher) const {
  // Compute push direction in object's frame
  const Eigen::Isometry3d T_object_to_target = world_.T_control_to_target(X, t_start() + 1);
  const Eigen::Isometry3d T_object_to_target_prev = world_.T_to_frame(world_.control_frame(t_start() + 1),
                                                                       world_.target_frame(t_start() + 1),
                                                                       X, t_start());
  const Eigen::Vector2d dir_push = (T_object_to_target.linear().transpose() *
                                    (T_object_to_target.translation() -
                                     T_object_to_target_prev.translation())).head<2>().normalized();

  // Compute toi from object origin to object surface opposite of push direction
  const Object3& pusher = world_.objects()->at(push_constraint_.name_pusher_);
  const Object3& object = world_.objects()->at(push_constraint_.name_pushee_);
  const Eigen::Isometry3d T_pusher_to_object = world_.T_control_to_target(X, t_start());
  const ncollide3d::query::Ray ray(Eigen::Vector3d::Zero(), Eigen::Vector3d(-dir_push(0), -dir_push(1), 0.));
  const auto toi_object = object.collision->toi_with_ray(Eigen::Isometry3d::Identity(), ray, false);

  // Point on object opposite of push direction
  const Eigen::Vector3d contact_object = ray.origin() + *toi_object * ray.dir();

  // Closest point on pusher to contact_object
  const auto projection_pusher = pusher.collision->project_point(T_pusher_to_object, contact_object, true);
  if (contact_pusher != nullptr) {
    *contact_pusher = projection_pusher.point;
  }

  // Distance between desired contact point on object and closest point on pusher
  return contact_object - projection_pusher.point;
}

PushConstraint::AlignmentConstraint::AlignmentConstraint(World3& world, size_t t_push)
    : FrameConstraint(kNumConstraints, kLenJacobian,
                      t_push, kNumTimesteps, "", "",
                      "constraint_t" + std::to_string(t_push) + "_push_alignment") {}

void PushConstraint::AlignmentConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                                   Eigen::Ref<Eigen::VectorXd> constraints) {
  const auto& X_t = X.col(t_start()).tail<kNumConstraints>().array();
  constraints = 0.5 * X_t * X_t;
  Constraint::Evaluate(X, constraints);
}

void PushConstraint::AlignmentConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                                   Eigen::Ref<Eigen::VectorXd> Jacobian) {
  const auto& X_t = X.col(t_start()).tail<kNumConstraints>();
  Jacobian = X_t;
  Constraint::Jacobian(X, Jacobian);
}

void PushConstraint::AlignmentConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                                          Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:   0  1  2
  // j:  wx wy wz
  idx_i += Eigen::ArrayXi::LinSpaced(kNumConstraints, 0, kNumConstraints - 1);
  const size_t var_t = kDof * t_start();
  idx_j.setLinSpaced(kNumConstraints, var_t + 3, var_t + kDof - 1);
}

PushConstraint::DestinationConstraint::DestinationConstraint(World3& world, size_t t_push,
                                                             const std::string& name_control,
                                                             const std::string& name_target,
                                                             PushConstraint& push_constraint)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_push, kNumTimesteps,
                      name_control, name_target,
                      "constraint_t" + std::to_string(t_push) + "_push_destination"),
      world_(world) {
  world.AttachFrame(name_control, name_target, t_push);
}

void PushConstraint::DestinationConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> constraints) {
  z_err_ = ComputeError(X);

  // Constrain movement along z-axis to be 0
  constraints(0) = 0.5 * z_err_ * z_err_;
  const double dist = X.block<2,1>(0, t_start()).norm() - kWorkspaceRadius;
  constraints(1) = 0.5 * std::abs(dist) * dist;

  Constraint::Evaluate(X, constraints);
}

void PushConstraint::DestinationConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Jacobian(0) = z_err_;
  const double dist = X.block<2,1>(0, t_start()).norm() - kWorkspaceRadius;
  Jacobian.tail<2>() = std::abs(dist) * X.block<2,1>(0, t_start()).normalized();
  Constraint::Jacobian(X, Jacobian);
}

void PushConstraint::DestinationConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                                            Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0 1 1
  // j:  z x y
  idx_i(1) += 1;
  idx_i(2) += 1;

  const size_t var_t = kDof * t_start();
  idx_j(0) = var_t + 2;
  idx_j(1) = var_t + 0;
  idx_j(2) = var_t + 1;
}

double PushConstraint::DestinationConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const {
  // Compute push direction
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());
  const Eigen::Isometry3d T_control_to_target_prev = world_.T_to_frame(control_frame(), target_frame(),
                                                                       X, t_start() - 1);
  const Eigen::Vector3d dir_push = T_control_to_target.translation() - T_control_to_target_prev.translation();

  return dir_push(2);
}

}  // namespace logic_opt

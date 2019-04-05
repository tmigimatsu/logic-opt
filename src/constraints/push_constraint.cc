/**
 * push_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/push_constraint.h"

#include "LogicOpt/constraints/touch_constraint.h"

namespace {

const size_t kNumContactAreaConstraints = 2;
const size_t kLenContactAreaJacobian = 12;
const size_t kNumContactAreaTimesteps = 1;

const size_t kNumDestinationConstraints = 2;
const size_t kLenDestinationJacobian = 7;
const size_t kNumDestinationTimesteps = 1;

const size_t kNumTimesteps = 2;

const double kH = 1e-4;

std::vector<std::unique_ptr<LogicOpt::Constraint>>
InitializeConstraints(LogicOpt::World& world, size_t t_push, const std::string& name_pusher,
                      const std::string& name_pushee, LogicOpt::PushConstraint& push_constraint) {
  using namespace LogicOpt;

  world.ReserveTimesteps(t_push + kNumTimesteps);

  std::vector<std::unique_ptr<Constraint>> constraints;
  constraints.emplace_back(new TouchConstraint(world, t_push, name_pusher, name_pushee));
  constraints.emplace_back(new PushConstraint::ContactAreaConstraint(world, t_push,
                                                                     name_pusher, name_pushee,
                                                                     push_constraint));

  const std::string name_target = *world.frames(t_push).parent(name_pushee);
  constraints.emplace_back(new PushConstraint::DestinationConstraint(world, t_push + 1,
                                                                     name_pushee, name_target,
                                                                     push_constraint));
  return constraints;
}

}  // namespace

namespace LogicOpt {

PushConstraint::PushConstraint(World& world, size_t t_push, const std::string& name_pusher,
                               const std::string& name_pushee)
    : MultiConstraint(InitializeConstraints(world, t_push, name_pusher, name_pushee, *this),
                                            "constraint_push_t" + std::to_string(t_push)),
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

void PushConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> constraints) {
  
  const Object& control = world_.objects()->at(name_pusher_);
  const Object& target = world_.objects()->at(name_pushee_);

  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());

  contact_ = *ncollide3d::query::contact(Eigen::Isometry3d::Identity(),
                                         *target.collision, T_control_to_target,
                                         *control.collision, 100.0);

  MultiConstraint::Evaluate(X, constraints);
}

void PushConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> Jacobian) {
  
  const Object& control = world_.objects()->at(name_pusher_);
  const Object& target = world_.objects()->at(name_pushee_);

  // Precompute contacts for Jacobian computations
  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < FrameConstraint::kDof; i++) {
    double& x_it = X_h(i, t_start());
    const double x_it_0 = x_it;
    x_it = x_it_0 + kH;
    const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X_h, t_start());
    x_it = x_it_0;

    contact_hp_[i] = *ncollide3d::query::contact(Eigen::Isometry3d::Identity(),
                                                 *target.collision, T_control_to_target,
                                                 *control.collision, 100.0);
  }

  MultiConstraint::Jacobian(X, Jacobian);
}

PushConstraint::ContactAreaConstraint::ContactAreaConstraint(World& world, size_t t_contact,
                                                             const std::string& name_control,
                                                             const std::string& name_target,
                                                             PushConstraint& push_constraint)
    : FrameConstraint(kNumContactAreaConstraints, kLenContactAreaJacobian,
                      t_contact, kNumContactAreaTimesteps, name_control, name_target,
                      "constraint_push_contact_area_t" + std::to_string(t_contact)),
      world_(world),
      push_constraint_(push_constraint) {

  const Object& target = world_.objects()->at(target_frame());
  if (!target.collision) {
    throw std::runtime_error("PushConstraint::ContactAreaConstraint::ContactAreaConstraint(): " +
                             target_frame() + " is missing a collision object.");
  }

  const ncollide3d::bounding_volume::AABB aabb = target.collision->aabb();
  z_max_ = aabb.maxs()(2);
  z_min_ = aabb.mins()(2);
}

void PushConstraint::ContactAreaConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> constraints) {
  // Constrain control com to be inside bounding box height
  z_contact_ = ComputeError(X, push_constraint_.contact_);
  constraints(0) = z_contact_ - z_max_;
  constraints(1) = z_min_ - z_contact_;

  Constraint::Evaluate(X, constraints);
}

void PushConstraint::ContactAreaConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < kDof; i++) {
    double& x_it = X_h(i, t_start() - 1);
    const double x_it_0 = x_it;
    x_it = x_it_0 + kH;
    const double z_contact_h = ComputeError(X_h, push_constraint_.contact_hp_[i]);
    x_it = x_it_0;

    const double dz_h = (z_contact_h - z_contact_) / kH;
    Jacobian(i) = dz_h;
    Jacobian(kDof + i) = -dz_h;
  }
}

void PushConstraint::ContactAreaConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                                            Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0  0  0  0  0  0  1  1  1  1  1  1
  // j:  x  y  z wx wy wz  x  y  z wx wy wz
  idx_i.tail<6>() += 1;

  const size_t var_t = kDof * t_start();
  idx_j.head<6>().setLinSpaced(var_t, var_t + kDof - 1);
  idx_j.tail<6>().setLinSpaced(var_t, var_t + kDof - 1);
}

double PushConstraint::ContactAreaConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                                           const ncollide3d::query::Contact& contact) const {
  const Object& control = world_.objects()->at(control_frame());
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());
  const Eigen::Vector3d point_contact_pusher = T_control_to_target.inverse() * contact.world2;
  return point_contact_pusher(2);
}

PushConstraint::DestinationConstraint::DestinationConstraint(World& world, size_t t_push,
                                                             const std::string& name_control,
                                                             const std::string& name_target,
                                                             PushConstraint& push_constraint)
    : FrameConstraint(kNumDestinationConstraints, kLenDestinationJacobian,
                      t_push, kNumDestinationTimesteps, name_control, name_target,
                      "constraint_push_destination_t" + std::to_string(t_push)),
      world_(world),
      push_constraint_(push_constraint) {
  world.AttachFrame(name_control, name_target, t_push);
}

void PushConstraint::DestinationConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> constraints) {
  xy_dot_normal_ = ComputeError(X, push_constraint_.contact_, &z_err_);

  // Constrain movement along z-axis to be 0
  constraints(0) = 0.5 * z_err_ * z_err_;

  // Constrain contact normal = push normal in xy-plane
  constraints(1) = xy_dot_normal_ - 1.;

  Constraint::Evaluate(X, constraints);
}

void PushConstraint::DestinationConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Jacobian(0) = z_err_;

  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < kDof; i++) {
    double& x_it = X_h(i, t_start() - 1);
    const double x_it_0 = x_it;
    x_it = x_it_0 + kH;
    const double xy_err_h = ComputeError(X_h, push_constraint_.contact_hp_[i]);
    x_it = x_it_0;

    Jacobian(i + 1) = (xy_err_h - xy_dot_normal_) / kH;
  }

  // if (xy_.cwiseEqual(0.).all()) {
  //   Jacobian.tail<2>().fill(0.);
  // } else {
  //   const double xy_norm = xy_.norm();
  //   Jacobian.tail<2>() = normal_ / xy_norm - xy_.dot(normal_) / (xy_norm * xy_norm * xy_norm) * xy_;
  // }
}

void PushConstraint::DestinationConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                                            Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0      1      1      1       1       1       1
  // j:  z x_prev y_prev z_prev wx_prev wy_prev wz_prev
  const size_t var_t = kDof * t_start();
  idx_j(0) = var_t + 2;

  const size_t var_t_prev = var_t - kDof;
  idx_i.tail<kDof>() += 1;
  idx_j.tail<kDof>().setLinSpaced(var_t_prev, var_t - 1);

  // X_{:3,t}
  // for (size_t j = 0; j < 2; j++) {
  //   idx_j(j + kDof + 1) = kDof * t_start() + j;
  // }

}

double PushConstraint::DestinationConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                                           const ncollide3d::query::Contact& contact,
                                                           double* z_err) const {
  // Compute contact normal
  const Object& pusher = world_.objects()->at(push_constraint_.name_pusher_);
  const Eigen::Isometry3d T_pusher_to_object = world_.T_control_to_target(X, t_start() - 1);
  const Eigen::Vector3d point_contact_pusher = T_pusher_to_object.inverse() * contact.world2;
  const Eigen::Vector2d normal_contact = (T_pusher_to_object.linear() *
                                          pusher.collision->normal(point_contact_pusher)
                                         ).head<2>().normalized();

  // Compute push direction
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());
  const Eigen::Isometry3d T_control_to_target_prev = world_.T_to_frame(control_frame(), target_frame(),
                                                                       X, t_start() - 1);
  const Eigen::Vector3d dir_push = T_control_to_target.translation() - T_control_to_target_prev.translation();

  if (z_err != nullptr) {
    *z_err = dir_push(2);
  }

  Eigen::Vector2d normal_push;
  if (normal_contact.isApprox(Eigen::Vector2d::Zero())) {
    normal_push.fill(0.);
    return 0;
  }

  normal_push = dir_push.head<2>();
  return normal_push.normalized().dot(normal_contact);
}

}  // namespace LogicOpt

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

const size_t kNumSupportAreaConstraints = 2;
const size_t kLenSupportAreaJacobian = kNumSupportAreaConstraints;

const size_t kNumNormalConstraints = 1;
const size_t kLenNormalJacobian = 6;

const size_t kNumDestinationConstraints = 3;
const size_t kLenDestinationJacobian = 9;

const size_t kNumTimesteps = 1;

const double kH = 1e-4;

std::vector<std::unique_ptr<LogicOpt::Constraint>>
InitializeConstraints(LogicOpt::World& world, size_t t_push, const std::string& name_pusher,
                      const std::string& name_pushee, LogicOpt::PushConstraint& push_constraint) {
  using namespace LogicOpt;

  std::vector<std::unique_ptr<Constraint>> constraints;
  constraints.emplace_back(new TouchConstraint(world, t_push, name_pusher, name_pushee));
  constraints.emplace_back(new PushConstraint::SupportAreaConstraint(world, t_push,
                                                                     name_pusher, name_pushee,
                                                                     push_constraint));
  constraints.emplace_back(new PushConstraint::NormalConstraint(world, t_push,
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
                                            "constraint_push_t" + std::to_string(t_push)) {
  if (name_pusher == world.kWorldFrame) {
    throw std::invalid_argument("PushConstraint::PushConstraint(): " + world.kWorldFrame +
                                " cannot be the pusher frame.");
  } else if (name_pushee == world.kWorldFrame) {
    throw std::invalid_argument("PushConstraint::PushConstraint(): " + world.kWorldFrame +
                                " cannot be the pushee frame.");
  }
}

PushConstraint::SupportAreaConstraint::SupportAreaConstraint(World& world, size_t t_contact,
                                                             const std::string& name_control,
                                                             const std::string& name_target,
                                                             PushConstraint& push_constraint)
    : FrameConstraint(kNumSupportAreaConstraints, kLenSupportAreaJacobian,
                      t_contact, kNumTimesteps, name_control, name_target,
                      "constraint_push_support_area_t" + std::to_string(t_contact)),
      world_(world),
      push_constraint_(push_constraint) {
  const Object& target = world_.objects()->at(target_frame());
  const ncollide3d::bounding_volume::AABB aabb = target.collision->aabb();
  z_max_ = aabb.maxs()(2);
  z_min_ = aabb.mins()(2);
}

void PushConstraint::SupportAreaConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> constraints) {
  ComputeError(X);
  constraints(0) = z_err_max_;
  constraints(1) = -z_err_min_;

  Constraint::Evaluate(X, constraints);
}

void PushConstraint::SupportAreaConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Jacobian(0) = 1.;
  Jacobian(1) = -1.;
}

void PushConstraint::SupportAreaConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                                            Eigen::Ref<Eigen::ArrayXi> idx_j) {
  idx_i(1) += 1;
  idx_j.fill(kDof * t_start() + 2);
}

Constraint::Type PushConstraint::SupportAreaConstraint::constraint_type(size_t idx_constraint) const {
  return Type::kInequality;
}

void PushConstraint::SupportAreaConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) {
  const Object& control = world_.objects()->at(control_frame());
  const Object& target = world_.objects()->at(target_frame());
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());
  const Eigen::Vector3d com_control = (T_control_to_target * control.inertia().com);

  push_constraint_.contacts_[0] = *ncollide3d::query::contact(Eigen::Isometry3d::Identity(),
                                                              *target.collision,
                                                              T_control_to_target,
                                                              *control.collision, 100.0);
  // Precompute contacts for Jacobian computations
  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < kDof; i++) {
    const double x_ij = X_h(i, t_start());
    X_h(i, t_start()) += kH;
    const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X_h, t_start());
    const Eigen::Vector3d com_control = (T_control_to_target * control.inertia().com);

    push_constraint_.contacts_[i + 1] = *ncollide3d::query::contact(Eigen::Isometry3d::Identity(),
                                                                    *target.collision,
                                                                    T_control_to_target,
                                                                    *control.collision, 100.0);

    X_h(i, t_start()) = x_ij;
  }

  z_err_max_ = com_control(2) - z_max_;//contact->world2(2);
  z_err_min_ = com_control(2) - z_min_;//contact->world2(2);
}

PushConstraint::NormalConstraint::NormalConstraint(World& world, size_t t_contact,
                                                   const std::string& name_control,
                                                   const std::string& name_target,
                                                   PushConstraint& push_constraint)
    : FrameConstraint(kNumNormalConstraints, kLenNormalJacobian,
                      t_contact, kNumTimesteps, name_control, name_target,
                      "constraint_push_normal_t" + std::to_string(t_contact)),
      world_(world),
      push_constraint_(push_constraint) {
  const Object& target = world_.objects()->at(target_frame());
  target_2d_ = target.collision->project_2d();
}

void PushConstraint::NormalConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                                Eigen::Ref<Eigen::VectorXd> constraints) {
  xy_err_ = ComputeError();
  constraints(0) = xy_err_;

  Constraint::Evaluate(X, constraints);
}

void PushConstraint::NormalConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                                Eigen::Ref<Eigen::VectorXd> Jacobian) {
  for (size_t i = 0; i < kLenNormalJacobian; i++) {
    double xy_err_h = ComputeError(i + 1);
    Jacobian(i) = (xy_err_h - xy_err_) / kH;
  }
}

void PushConstraint::NormalConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                                       Eigen::Ref<Eigen::ArrayXi> idx_j) {
  idx_j = Eigen::VectorXi::LinSpaced(kLenNormalJacobian, kDof * t_start(), kDof * t_start() + kLenNormalJacobian - 1).array();
}

Constraint::Type PushConstraint::NormalConstraint::constraint_type(size_t idx_constraint) const {
  return Type::kInequality;
}

double PushConstraint::NormalConstraint::ComputeError(size_t idx_contact) {
  const ncollide3d::query::Contact& contact = push_constraint_.contacts_[idx_contact];
  const auto projection = target_2d_->project_point(Eigen::Isometry2d::Identity(),
                                                    contact.world2.head<2>(), false);
  return -(projection.is_inside ? -1. : 1.) *
         (contact.world2.head<2>() - projection.point).norm();
}

PushConstraint::DestinationConstraint::DestinationConstraint(World& world, size_t t_push,
                                                             const std::string& name_control,
                                                             const std::string& name_target,
                                                             PushConstraint& push_constraint)
    : FrameConstraint(kNumDestinationConstraints, kLenDestinationJacobian,
                      t_push, kNumTimesteps, name_control, name_target,
                      "constraint_push_destination_t" + std::to_string(t_push)),
      world_(world),
      push_constraint_(push_constraint) {
  world.AttachFrame(name_control, name_target, t_push);
}

void PushConstraint::DestinationConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> constraints) {
  xy_dot_normal_ = ComputeError(X);
  constraints(0) = xy_dot_normal_ - 1.;
  constraints(1) = 0.5 * z_err_ * z_err_;

  Constraint::Evaluate(X, constraints);
}

void PushConstraint::DestinationConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> Jacobian) {
  if (xy_.cwiseEqual(0.).all()) {
    Jacobian.head<2>().fill(0.);
  } else {
    const double xy_norm = xy_.norm();
    Jacobian.head<2>() = normal_ / xy_norm - xy_.dot(normal_) / (xy_norm * xy_norm * xy_norm) * xy_;
  }
  Jacobian(2) = z_err_;

  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < kDof; i++) {
    const double x_ij = X_h(i, t_start() - 1);
    X_h(i, t_start() - 1) += kH;
    const double xy_err_h = ComputeError(X_h, i + 1);
    Jacobian(i + 3) = (xy_err_h - xy_dot_normal_) / kH;

    X_h(i, t_start() - 1) = x_ij;
  }
}

void PushConstraint::DestinationConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                                            Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // z_err
  idx_i(2) += 1;

  // X_{:3,t}
  for (size_t j = 0; j < 3; j++) {
    idx_j(j) = kDof * t_start() + j;
  }

  // X_{:,t-1}
  for (size_t j = 0; j < kDof; j++) {
    idx_j(j + 3) = kDof * (t_start() - 1) + j;
  }
}

double PushConstraint::DestinationConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                                           size_t idx_contact) {
  const Object& control = world_.objects()->at(control_frame());
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());
  const Eigen::Isometry3d T_control_to_target_prev = world_.T_to_frame(control_frame(), target_frame(),
                                                                       X, t_start() - 1);

  const ncollide3d::query::Contact& contact = push_constraint_.contacts_[idx_contact];
  normal_ = (contact.depth > 0. ? 1. : -1.) * contact.normal.head<2>();

  const Eigen::Vector3d dx = T_control_to_target.translation() - T_control_to_target_prev.translation();

  z_err_ = dx(2);
  if (normal_.isApprox(Eigen::Vector2d::Zero())) {
    xy_.fill(0.);
    return 0;
  }

  normal_.normalize();
  xy_ = dx.head<2>();
  return xy_.normalized().dot(normal_);
}

}  // namespace LogicOpt

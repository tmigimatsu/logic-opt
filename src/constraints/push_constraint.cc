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

const size_t kNumDestinationConstraints = 2;
const size_t kLenDestinationJacobian = 3;

const size_t kNumTimesteps = 1;

std::vector<std::unique_ptr<LogicOpt::Constraint>>
InitializeConstraints(LogicOpt::World& world, size_t t_push, const std::string& name_pusher,
                      const std::string& name_pushee, LogicOpt::PushConstraint& push_constraint) {
  using namespace LogicOpt;

  std::vector<std::unique_ptr<Constraint>> constraints;
  constraints.emplace_back(new TouchConstraint(world, t_push, name_pusher, name_pushee));
  constraints.emplace_back(new PushConstraint::SupportAreaConstraint(world, t_push,
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
  return Constraint::Type::kInequality;
}

void PushConstraint::SupportAreaConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) {
  const Object& control = world_.objects()->at(control_frame());
  const Object& target = world_.objects()->at(target_frame());
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());
  const Eigen::Vector3d com_control = (T_control_to_target * control.inertia().com);

  auto contact = ncollide3d::query::contact(Eigen::Isometry3d::Identity(), *target.collision,
                                            T_control_to_target, *control.collision, 100.0);

  z_err_max_ = com_control(2) - z_max_;//contact->world2(2);
  z_err_min_ = com_control(2) - z_min_;//contact->world2(2);
  push_constraint_.normal_ = (contact->depth > 0. ? -1. : 1.) * contact->normal;  // in target frame
  std::cout << push_constraint_.normal_.transpose() << std::endl;
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
  ComputeError(X);
  if (xy_.cwiseEqual(0.).all()) {
    constraints(0) = 0.;
  } else {
    constraints(0) = xy_.normalized().dot(normal_) - 1.;
  }
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
}

void PushConstraint::DestinationConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                                            Eigen::Ref<Eigen::ArrayXi> idx_j) {
  idx_i(2) += 1;
  for (size_t j = 0; j < kLenDestinationJacobian; j++) {
    idx_j(j) = kDof * t_start() + j;
  }
}

Constraint::Type PushConstraint::DestinationConstraint::constraint_type(size_t idx_constraint) const {
  return idx_constraint == 0 ? Constraint::Type::kEquality : Constraint::Type::kEquality;
}

void PushConstraint::DestinationConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) {
  const Object& control = world_.objects()->at(control_frame());
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());
  const Eigen::Isometry3d T_control_to_target_prev = world_.T_to_frame(control_frame(), target_frame(),
                                                                       X, t_start() - 1);

  normal_ = (T_control_to_target_prev.linear() * push_constraint_.normal_).head<2>();
  const Eigen::Vector3d dx = T_control_to_target.translation() - T_control_to_target_prev.translation();

  if (normal_.isApprox(Eigen::Vector2d::Zero())) {
    xy_.fill(0.);
  } else {
    xy_ = dx.head<2>();
  }
  z_err_ = dx(2);
}

}  // namespace LogicOpt

/**
 * push_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/push_constraint.h"

#include "logic_opt/constraints/collision_constraint.h"
#include "logic_opt/constraints/touch_constraint.h"

namespace {

const size_t kNumContactAreaConstraints = 3;
const size_t kLenContactAreaJacobian = 12;
const size_t kNumContactAreaTimesteps = 1;

const size_t kNumAlignmentConstraints = 3;
const size_t kLenAlignmentJacobian = 3;
const size_t kNumAlignmentTimesteps = 1;

const size_t kNumDestinationConstraints = 2;
const size_t kLenDestinationJacobian = 7;
const size_t kNumDestinationTimesteps = 1;

const size_t kNumTimesteps = 2;

#ifdef PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
const double kH = 5e-2;
#else  // PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
const double kH = 1e-4;
#endif  // PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE

std::vector<std::unique_ptr<logic_opt::Constraint>>
InitializeConstraints(logic_opt::World3& world, size_t t_push, const std::string& name_pusher,
                      const std::string& name_pushee, logic_opt::PushConstraint& push_constraint) {
  using namespace logic_opt;

  world.ReserveTimesteps(t_push + kNumTimesteps);

  std::vector<std::unique_ptr<Constraint>> constraints;
  constraints.emplace_back(new TouchConstraint(world, t_push, name_pusher, name_pushee));

  const std::string name_target = *world.frames(t_push).parent(name_pushee);
  constraints.emplace_back(new PushConstraint::AlignmentConstraint(world, t_push + 1));
  constraints.emplace_back(new PushConstraint::DestinationConstraint(world, t_push + 1,
                                                                     name_pushee, name_target,
                                                                     push_constraint));
  // constraints.emplace_back(new PushConstraint::ContactAreaConstraint(world, t_push,
  //                                                                    name_pusher, name_pushee,
  //                                                                    push_constraint));

  constraints.emplace_back(new CollisionConstraint(world, t_push + 1));
  return constraints;
}

}  // namespace

namespace logic_opt {

PushConstraint::PushConstraint(World3& world, size_t t_push, const std::string& name_pusher,
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
  
  const Object3& control = world_.objects()->at(name_pusher_);
  const Object3& target = world_.objects()->at(name_pushee_);

  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());

  contact_ = *ncollide3d::query::contact(Eigen::Isometry3d::Identity(),
                                         *target.collision, T_control_to_target,
                                         *control.collision, 100.0);

  MultiConstraint::Evaluate(X, constraints);
}

void PushConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> Jacobian) {
  
  const Object3& control = world_.objects()->at(name_pusher_);
  const Object3& target = world_.objects()->at(name_pushee_);

  // Precompute contacts for Jacobian computations
  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < FrameConstraint::kDof; i++) {
    double& x_it = X_h(i, t_start());
    const double x_it_0 = x_it;
    x_it = x_it_0 + kH;
    const Eigen::Isometry3d T_control_to_target_hp = world_.T_control_to_target(X_h, t_start());
    contact_hp_[i] = *ncollide3d::query::contact(Eigen::Isometry3d::Identity(),
                                                 *target.collision, T_control_to_target_hp,
                                                 *control.collision, 100.0);
#ifdef PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0 - kH;
    const Eigen::Isometry3d T_control_to_target_hn = world_.T_control_to_target(X_h, t_start());
    contact_hn_[i] = *ncollide3d::query::contact(Eigen::Isometry3d::Identity(),
                                                 *target.collision, T_control_to_target_hn,
                                                 *control.collision, 100.0);
#endif  // PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE

    x_it = x_it_0;
  }

  MultiConstraint::Jacobian(X, Jacobian);
}

PushConstraint::ContactAreaConstraint::ContactAreaConstraint(World3& world, size_t t_contact,
                                                             const std::string& name_control,
                                                             const std::string& name_target,
                                                             PushConstraint& push_constraint)
    : FrameConstraint(kNumContactAreaConstraints, kLenContactAreaJacobian,
                      t_contact, kNumContactAreaTimesteps, name_control, name_target,
                      "constraint_push_contact_area_t" + std::to_string(t_contact)),
      world_(world),
      push_constraint_(push_constraint) {

  const Object3& target = world_.objects()->at(target_frame());
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
  z_xy_contact_ = ComputeError(X, push_constraint_.contact_);
  constraints(0) = z_xy_contact_(0) - z_max_;
  constraints(1) = z_min_ - z_xy_contact_(0);
  constraints(2) = z_xy_contact_(1);

  Constraint::Evaluate(X, constraints);
}

void PushConstraint::ContactAreaConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < kDof; i++) {
    double& x_it = X_h(i, t_start() - 1);
    const double x_it_0 = x_it;
    x_it = x_it_0 + kH;
    const Eigen::Vector2d z_xy_contact_h = ComputeError(X_h, push_constraint_.contact_hp_[i]);
    x_it = x_it_0;

    const Eigen::Vector2d dz_xy_h = (z_xy_contact_h - z_xy_contact_) / kH;
    Jacobian(i) = dz_xy_h(0);
    Jacobian(kDof + i) = -dz_xy_h(0);
    Jacobian(2 * kDof + i) = dz_xy_h(0);
  }
}

void PushConstraint::ContactAreaConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                                            Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0  0  0  0  0  0  1  1  1  1  1  1
  // j:  x  y  z wx wy wz  x  y  z wx wy wz
  idx_i.segment<kDof>(kDof) += 1;
  idx_i.tail<kDof>() += 2;

  const size_t var_t = kDof * t_start();
  idx_j.head<kDof>().setLinSpaced(var_t, var_t + kDof - 1);
  idx_j.segment<kDof>(kDof).setLinSpaced(var_t, var_t + kDof - 1);
  idx_j.tail<kDof>().setLinSpaced(var_t, var_t + kDof - 1);
}

Eigen::Vector2d PushConstraint::ContactAreaConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                                                    const ncollide3d::query::Contact& contact) const {
  const Eigen::Vector3d& point_contact = contact.world2;  // in object frame
  const double contact_dot_push = point_contact.head<2>().normalized().dot(push_constraint_.dir_push_object_);
  return Eigen::Vector2d(point_contact(2), contact_dot_push);
}

PushConstraint::AlignmentConstraint::AlignmentConstraint(World3& world, size_t t_push)
    : FrameConstraint(kNumAlignmentConstraints, kLenAlignmentJacobian,
                      t_push, kNumAlignmentTimesteps, "", "",
                      "constraint_push_alignment_t" + std::to_string(t_push)) {}

void PushConstraint::AlignmentConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                                   Eigen::Ref<Eigen::VectorXd> constraints) {
  const auto& X_t = X.col(t_start()).tail<kNumAlignmentConstraints>().array();
  constraints = 0.5 * X_t * X_t;
}

void PushConstraint::AlignmentConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                                   Eigen::Ref<Eigen::VectorXd> Jacobian) {
  const auto& X_t = X.col(t_start()).tail<kNumAlignmentConstraints>();
  Jacobian = X_t;
}

void PushConstraint::AlignmentConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                                          Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:   0  1  2
  // j:  wx wy wz
  idx_i += Eigen::ArrayXi::LinSpaced(kNumAlignmentConstraints, 0, kNumAlignmentConstraints - 1);
  const size_t var_t = kDof * t_start();
  idx_j.setLinSpaced(kNumAlignmentConstraints, var_t + 3, var_t + kDof - 1);
}

PushConstraint::DestinationConstraint::DestinationConstraint(World3& world, size_t t_push,
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
    const double x_err_hp = ComputeError(X_h, push_constraint_.contact_hp_[i]);
#ifdef PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0 - kH;
    const double x_err_hn = ComputeError(X_h, push_constraint_.contact_hn_[i]);
#endif  // PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0;

#ifdef PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    const double dx_h = (x_err_hp - x_err_hn) / (2. * kH);
#else  // PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    const double dx_h = (x_err_hp - xy_dot_normal_) / kH;
#endif  // PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    Jacobian(1 + i) = dx_h;
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
  const Object3& pusher = world_.objects()->at(push_constraint_.name_pusher_);
  const Eigen::Isometry3d T_pusher_to_object = world_.T_control_to_target(X, t_start() - 1);
  const ncollide3d::query::Ray ray(Eigen::Vector3d::Zero(), contact.world2.normalized());
  const auto intersect = pusher.collision->toi_and_normal_with_ray(T_pusher_to_object, ray, false);

  // Compute push direction
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());
  const Eigen::Isometry3d T_control_to_target_prev = world_.T_to_frame(control_frame(), target_frame(),
                                                                       X, t_start() - 1);
  const Eigen::Vector3d dir_push = T_control_to_target.translation() - T_control_to_target_prev.translation();

  push_constraint_.dir_push_object_ = (T_control_to_target.linear().transpose() * dir_push).head<2>().normalized();

  if (z_err != nullptr) {
    *z_err = dir_push(2);
  }

  const Eigen::Vector3d normal_push = Eigen::Vector3d(dir_push(0), dir_push(1), 0.).normalized();
  return normal_push.dot(intersect->normal);
}

}  // namespace logic_opt

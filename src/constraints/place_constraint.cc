/**
 * place_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/place_constraint.h"

#include <algorithm>  // std::max

#include <ncollide/ncollide2d.h>

#include "LogicOpt/constraints/touch_constraint.h"

namespace {

const size_t kNumNormalConstraints = 2;
const size_t kLenNormalJacobian = 2;

#ifdef PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
const size_t kNumSupportAreaConstraints = 2 + 2;
const size_t kLenSupportAreaJacobian = 3 + 6;
#else  // PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
const size_t kNumSupportAreaConstraints = 3;
const size_t kLenSupportAreaJacobian = 3;
#endif  // PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN

const size_t kNumTimesteps = 1;

const double kH = 1e-4;

std::vector<std::unique_ptr<LogicOpt::Constraint>>
InitializeConstraints(LogicOpt::World& world, size_t t_place,
                      const std::string& name_object, const std::string& name_target) {
  using namespace LogicOpt;

  std::vector<std::unique_ptr<Constraint>> constraints;
  constraints.emplace_back(new PlaceConstraint::NormalConstraint(t_place, name_object, name_target));
  constraints.emplace_back(new TouchConstraint(world, t_place, name_object, name_target));
  constraints.emplace_back(new PlaceConstraint::SupportAreaConstraint(world, t_place,
                                                                      name_object, name_target));
  return constraints;
}

}  // namespace

namespace LogicOpt {

PlaceConstraint::PlaceConstraint(World& world, size_t t_place,
                                 const std::string& name_object, const std::string& name_target)
    : MultiConstraint(InitializeConstraints(world, t_place, name_object, name_target),
                                            "constraint_place_t" + std::to_string(t_place)) {

  if (name_object == world.kWorldFrame) {
    throw std::invalid_argument("PlaceConstraint::PlaceConstraint(): " + world.kWorldFrame +
                                " cannot be the object frame.");
  } else if (name_target == world.kWorldFrame) {
    throw std::invalid_argument("PlaceConstraint::PlaceConstraint(): " + world.kWorldFrame +
                                " cannot be the target frame.");
  }
  world.ReserveTimesteps(t_place + kNumTimesteps);
  world.AttachFrame(name_object, name_target, t_place);
}

PlaceConstraint::NormalConstraint::NormalConstraint(size_t t_place, const std::string& name_control,
                                                    const std::string& name_target)
    : FrameConstraint(kNumNormalConstraints, kLenNormalJacobian, t_place, kNumTimesteps,
                      name_control, name_target,
                      "constraint_place_normal_t" + std::to_string(t_place)) {}

void PlaceConstraint::NormalConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                                 Eigen::Ref<Eigen::VectorXd> constraints) {
  Eigen::Vector2d xy = X.block<2,1>(3, t_start());
  constraints = 0.5 * xy.array().square();

  Constraint::Evaluate(X, constraints);
}

void PlaceConstraint::NormalConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                                      Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Eigen::Vector2d xy = X.block<2,1>(3, t_start());
  Jacobian = xy;
}

void PlaceConstraint::NormalConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                                        Eigen::Ref<Eigen::ArrayXi> idx_j) {
  idx_i(1) += 1;
  idx_j(0) = kDof * t_start() + 3;
  idx_j(1) = kDof * t_start() + 4;
}

PlaceConstraint::SupportAreaConstraint::SupportAreaConstraint(World& world, size_t t_place,
                                                              const std::string& name_control,
                                                              const std::string& name_target)
    : FrameConstraint(kNumSupportAreaConstraints, kLenSupportAreaJacobian,
                      t_place, kNumTimesteps, name_control, name_target,
                      "constraint_place_support_area_t" + std::to_string(t_place)),
      world_(world) {
  const Object& control = world_.objects()->at(control_frame());
  const Object& target = world_.objects()->at(target_frame());
  target_2d_ = target.collision->project_2d();
  z_surface_ = target.collision->aabb(Eigen::Isometry3d::Identity()).maxs()(2);
  Eigen::Vector3d half_extents = control.collision->aabb(Eigen::Isometry3d::Identity()).maxs();
  r_object_ = std::max(half_extents(0), half_extents(1));

  const Eigen::Vector3d& com = control.inertia().com;
  ncollide3d::query::Ray ray_x(com, Eigen::Vector3d::UnitX());
  std::optional<double> toi_x = control.collision->toi_with_ray(Eigen::Isometry3d::Identity(), ray_x, true);
  if (!toi_x) {
    ncollide3d::query::Ray ray_xneg(com, -Eigen::Vector3d::UnitX());
    toi_x = control.collision->toi_with_ray(Eigen::Isometry3d::Identity(), ray_xneg, true);
    if (toi_x) *toi_x *= -1.;
  }

  ncollide3d::query::Ray ray_y(com, Eigen::Vector3d::UnitY());
  std::optional<double> toi_y = control.collision->toi_with_ray(Eigen::Isometry3d::Identity(), ray_y, true);
  if (!toi_y) {
    ncollide3d::query::Ray ray_yneg(com, -Eigen::Vector3d::UnitY());
    toi_y = control.collision->toi_with_ray(Eigen::Isometry3d::Identity(), ray_yneg, true);
    if (toi_y) *toi_y *= -1.;
  }

  if (!toi_x || !toi_y) {
    throw std::runtime_error("PlaceConstraint::PlaceConstraint(): Unsupported shape.");
  }
  xy_support_[0] = Eigen::Vector2d(*toi_x, 0.);
  xy_support_[1] = Eigen::Vector2d(0., *toi_y);
}

void PlaceConstraint::SupportAreaConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                                      Eigen::Ref<Eigen::VectorXd> constraints) {
  xy_err_ = ComputeError(X);

#ifdef PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
  constraints(0) = xy_err_(0);
  constraints(1) = -z_err_;
  constraints(2) = xy_err_(1);
  constraints(3) = xy_err_(2);
#else  // PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
  constraints.head<2>() = 0.5 * xy_err_.array().square();
  constraints(2) = -z_err_;
#endif  // PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN

  Constraint::Evaluate(X, constraints);
}

void PlaceConstraint::SupportAreaConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                                      Eigen::Ref<Eigen::VectorXd> Jacobian) {
#ifdef PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < 2; i++) {
    const double x_ij = X_h(i, t_start());
    X_h(i, t_start()) += kH;
    const Eigen::Vector3d xy_err_h = ComputeError(X_h);
    const Eigen::Vector3d J = (xy_err_h - xy_err_) / kH;
    Jacobian(i) = J(0);
    Jacobian(3 + i) = J(1);
    Jacobian(6 + i) = J(2);
    X_h(i, t_start()) = x_ij;
  }
  {
    size_t i = 5;
    const double x_ij = X_h(i, t_start());
    X_h(i, t_start()) += kH;
    const Eigen::Vector3d xy_err_h = ComputeError(X_h);
    const Eigen::Vector3d J = (xy_err_h - xy_err_) / kH;
    Jacobian(5) = J(1);
    Jacobian(8) = J(2);
    X_h(i, t_start()) = x_ij;
  }
#else  // PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
  Jacobian.head<2>() = xy_err_;
#endif  // PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN

  Jacobian(2) = -1.;
}

#ifdef PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
Eigen::Vector3d PlaceConstraint::SupportAreaConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) {
#else  // PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
Eigen::Vector2d PlaceConstraint::SupportAreaConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) {
#endif  // PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
  const Object& control = world_.objects()->at(control_frame());
  const Object& target = world_.objects()->at(target_frame());
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());
  const Eigen::Vector2d com_control = (T_control_to_target * control.inertia().com).head<2>();

  z_err_ = T_control_to_target.translation()(2) - z_surface_;

#ifdef PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
  Eigen::Vector3d error = Eigen::Vector3d::Zero();
  {
    const auto projection = target_2d_->project_point(Eigen::Isometry2d::Identity(), com_control, false);
    error(0) = (projection.is_inside ? -1. : 1.) * (com_control - projection.point).norm();
  }
  for (size_t i = 0; i < 2; i++) {
    if (!xy_support_[i].isZero()) {
      Eigen::Vector3d xyz_support = Eigen::Vector3d::Zero();
      xyz_support.head<2>() = xy_support_[i];
      Eigen::Vector2d xy_support = (T_control_to_target * xyz_support).head<2>();
      const auto projection = target_2d_->project_point(Eigen::Isometry2d::Identity(), xy_support, false);
      error(i + 1) = (projection.is_inside ? -1. : 1.) * (xy_support - projection.point).norm();
    }
  }
  return error;
#else  // PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
  const auto projection = target_2d_->project_point(Eigen::Isometry2d::Identity(), com_control, true);
  return com_control - projection.point;
#endif  // PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
}

#ifdef PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
void PlaceConstraint::SupportAreaConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                                             Eigen::Ref<Eigen::ArrayXi> idx_j) {
  idx_i(2) += 1;
  for (size_t j = 0; j < 3; j++) {
    idx_j(j) = kDof * t_start() + j;
  }
  for (size_t i = 0; i < 2; i++) {
    idx_i(3 * i + 3) += 2 + i;
    idx_i(3 * i + 4) += 2 + i;
    idx_i(3 * i + 5) += 2 + i;
    idx_j(3 * i + 3) = kDof * t_start() + 0;
    idx_j(3 * i + 4) = kDof * t_start() + 1;
    idx_j(3 * i + 5) = kDof * t_start() + 5;
  }
}
#endif  // PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN

Constraint::Type PlaceConstraint::SupportAreaConstraint::constraint_type(size_t idx_constraint) const {
#ifdef PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
  return Type::kInequality;
#else  // PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
  return idx_constraint == 2 ? Type::kInequality : Type::kEquality;
#endif  // PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN
}

}  // namespace LogicOpt

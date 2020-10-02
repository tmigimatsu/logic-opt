/**
 * place_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/place_constraint.h"

#define PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE

#include <spatial_opt/constraints/quaternion_norm_constraint.h>

#include "logic_opt/constraints/collision_constraint.h"
#include "logic_opt/constraints/touch_constraint.h"

namespace {

const size_t kNumNormalConstraints = 2;  // qx = 0, qy = 0
const size_t kLenNormalJacobian = 2;

const size_t kNumSupportAreaConstraints = 4;  // z > 0, COM/2D, x/2D, y/2D
const size_t kLenSupportAreaJacobian = 11;

#ifdef PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE
const double kH = 1e-5;
const double kH_ori = 1e-2;
#else   // PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE
const double kH = 1e-4;
#endif  // PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE

const double kMaxToi = 100.;

std::vector<std::unique_ptr<spatial_opt::Constraint>> InitializeConstraints(
    logic_opt::World& world, size_t t_place, const std::string& name_object,
    const std::string& name_target) {
  using namespace logic_opt;

  if (name_object == world.kWorldFrame) {
    throw std::invalid_argument(
        "PlaceConstraint::PlaceConstraint(): " + world.kWorldFrame +
        " cannot be the object frame.");
  } else if (name_target == world.kWorldFrame) {
    throw std::invalid_argument(
        "PlaceConstraint::PlaceConstraint(): " + world.kWorldFrame +
        " cannot be the target frame.");
  }
  // Handled by TouchConstraint
  // world.ReserveTimesteps(t_place + PlaceConstraint::kNumTimesteps);
  // world.AttachFrame(name_object, name_target, t_place);

  std::vector<std::unique_ptr<spatial_opt::Constraint>> constraints;

  // Constrain wx, wy = 0
  constraints.emplace_back(
      new PlaceConstraint::NormalConstraint(t_place, name_object, name_target));

  constraints.emplace_back(
      new TouchConstraint(world, t_place, name_object, name_target));

  // Constrain com, x-support, y-support of object to be inside 2d support area
  // of target
  constraints.emplace_back(new PlaceConstraint::SupportAreaConstraint(
      world, t_place, name_object, name_target));

  constraints.emplace_back(new CollisionConstraint(world, t_place));

  // Constrain quaternion norm to be unit norm
  constraints.emplace_back(new spatial_opt::QuaternionNormConstraint(t_place));

  return constraints;
}

}  // namespace

namespace logic_opt {

PlaceConstraint::PlaceConstraint(World& world, size_t t_place,
                                 const std::string& name_object,
                                 const std::string& name_target)
    : spatial_opt::MultiConstraint(
          InitializeConstraints(world, t_place, name_object, name_target),
          "constraint_t" + std::to_string(t_place) + "_place") {
  world.set_controller("place", t_place);
}

PlaceConstraint::NormalConstraint::NormalConstraint(
    size_t t_place, const std::string& name_control,
    const std::string& name_target)
    : FrameConstraint(
          kNumNormalConstraints, kLenNormalJacobian, t_place, kNumTimesteps,
          name_control, name_target,
          "constraint_t" + std::to_string(t_place) + "_place_normal") {}

void PlaceConstraint::NormalConstraint::Evaluate(
    Eigen::Ref<const Eigen::MatrixXd> X,
    Eigen::Ref<Eigen::VectorXd> constraints) {
  // Constrain qx, qy = 0
  const auto& wxy = X.block<2, 1>(3, t_start());
  constraints = 0.5 * wxy.array().square();

  Constraint::Evaluate(X, constraints);
}

void PlaceConstraint::NormalConstraint::Jacobian(
    Eigen::Ref<const Eigen::MatrixXd> X, Eigen::Ref<Eigen::VectorXd> Jacobian) {
  const auto& wxy = X.block<2, 1>(3, t_start());

  Jacobian = Eigen::Vector2d::Ones();

  Constraint::Jacobian(X, Jacobian);
}

void PlaceConstraint::NormalConstraint::JacobianIndices(
    Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0  1
  // j: qx qy

  idx_i(1) += 1;

  const size_t var_t = kDof * t_start();
  idx_j(0) = var_t + 3;
  idx_j(1) = var_t + 4;
}

PlaceConstraint::SupportAreaConstraint::SupportAreaConstraint(
    World& world, size_t t_place, const std::string& name_control,
    const std::string& name_target)
    : FrameConstraint(
          kNumSupportAreaConstraints, kLenSupportAreaJacobian, t_place,
          kNumTimesteps, name_control, name_target,
          "constraint_t" + std::to_string(t_place) + "_place_support_area"),
      world_(world) {
  const Object& control = world_.objects()->at(control_frame());
  const Object& target = world_.objects()->at(target_frame());
  target_2d_ = target.collision->project_2d();
  z_surface_ = target.collision->aabb(Eigen::Isometry3d::Identity()).maxs()(2) -
               control.collision->aabb(Eigen::Isometry3d::Identity()).mins()(2);
  if (dynamic_cast<ncollide3d::shape::Compound*>(target.collision.get()) !=
      nullptr) {
    z_surface_ -= 0.04;  // ncollide loosens aabb of compound by 0.04
  }
  if (dynamic_cast<ncollide3d::shape::Compound*>(control.collision.get()) !=
      nullptr) {
    z_surface_ -= 0.04;
  }

  // Project ray from com along x-axis
  const Eigen::Vector3d& com = control.inertia().com;
  const ncollide3d::query::Ray ray_x(com, Eigen::Vector3d::UnitX());
  std::optional<double> toi_x = control.collision->toi_with_ray(
      Eigen::Isometry3d::Identity(), ray_x, true);
  if (!toi_x) {
    // Project ray from com along neg x-axis
    const ncollide3d::query::Ray ray_xneg(com, -Eigen::Vector3d::UnitX());
    toi_x = control.collision->toi_with_ray(Eigen::Isometry3d::Identity(),
                                            ray_xneg, true);
    if (toi_x) *toi_x *= -1.;
  }

  // Project ray from com along y-axis
  const ncollide3d::query::Ray ray_y(com, Eigen::Vector3d::UnitY());
  std::optional<double> toi_y = control.collision->toi_with_ray(
      Eigen::Isometry3d::Identity(), ray_y, true);
  if (!toi_y) {
    // Project ray from com along neg y-axis
    const ncollide3d::query::Ray ray_yneg(com, -Eigen::Vector3d::UnitY());
    toi_y = control.collision->toi_with_ray(Eigen::Isometry3d::Identity(),
                                            ray_yneg, true);
    if (toi_y) *toi_y *= -1.;
  }

  if (!toi_x || !toi_y) {
    throw std::runtime_error(
        "PlaceConstraint::PlaceConstraint(): Unsupported shape.");
  }

  // Constrain these points to be inside the support area
  xy_support_[0] = Eigen::Vector3d(*toi_x, 0., 0.);
  xy_support_[1] = Eigen::Vector3d(0., *toi_y, 0.);
}

void PlaceConstraint::SupportAreaConstraint::Evaluate(
    Eigen::Ref<const Eigen::MatrixXd> X,
    Eigen::Ref<Eigen::VectorXd> constraints) {
  x_err_ = ComputeError(X, &z_err_);

  // Constrain z height to be positive
  constraints(0) = -z_err_;

  // Constrain origin, x-axis, y-axis projections to be inside support area
  constraints.segment<3>(1) = x_err_;

  Constraint::Evaluate(X, constraints);
}

void PlaceConstraint::SupportAreaConstraint::Jacobian(
    Eigen::Ref<const Eigen::MatrixXd> X, Eigen::Ref<Eigen::VectorXd> Jacobian) {
  // Jacobian for positive z
  Jacobian(0) = -1.;

  Eigen::MatrixXd X_h = X;

  // Vary x, y
  for (size_t i = 0; i < 2; i++) {
    double& x_it = X_h(i, t_start());
    const double x_it_0 = x_it;
    x_it = x_it_0 + kH;
    const Eigen::Vector3d x_err_hp = ComputeError(X_h);
#ifdef PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0 - kH;
    const Eigen::Vector3d x_err_hn = ComputeError(X_h);
#endif  // PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0;

#ifdef PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE
    const Eigen::Vector3d dx_h = (x_err_hp - x_err_hn) / (2. * kH);
#else   // PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE
    const Eigen::Vector3d dx_h = (x_err_hp - x_err_) / kH;
#endif  // PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE

    Jacobian(1 + i) = dx_h(0);
    Jacobian(3 + i) = dx_h(1);
    Jacobian(7 + i) = dx_h(2);
  }

  // Vary qz
  {
    double& x_it = X_h(5, t_start());
    const double x_it_0 = x_it;
    x_it = x_it_0 + kH_ori;
    const Eigen::Vector2d x_err_hp = ComputeError(X_h).tail<2>();
#ifdef PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0 - kH_ori;
    const Eigen::Vector2d x_err_hn = ComputeError(X_h).tail<2>();
#endif  // PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0;

#ifdef PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE
    const Eigen::Vector2d dx_h = (x_err_hp - x_err_hn) / (2. * kH_ori);
#else   // PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE
    const Eigen::Vector2d dx_h = (x_err_hp - x_err_.tail<2>()) / kH_ori;
#endif  // PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE

    Jacobian(5) = dx_h(0);
    Jacobian(9) = dx_h(1);
  }

  // Vary qw
  {
    double& x_it = X_h(6, t_start());
    const double x_it_0 = x_it;
    x_it = x_it_0 + kH_ori;
    const Eigen::Vector2d x_err_hp = ComputeError(X_h).tail<2>();
#ifdef PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0 - kH_ori;
    const Eigen::Vector2d x_err_hn = ComputeError(X_h).tail<2>();
#endif  // PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0;

#ifdef PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE
    const Eigen::Vector2d dx_h = (x_err_hp - x_err_hn) / (2. * kH_ori);
#else   // PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE
    const Eigen::Vector2d dx_h = (x_err_hp - x_err_.tail<2>()) / kH_ori;
#endif  // PLACE_CONSTRAINT_SYMMETRIC_DIFFERENCE

    Jacobian(6) = dx_h(0);
    Jacobian(10) = dx_h(1);
  }

  // Angle axis representation
  // i:  0   1  1   2  2  2   3  3  3
  // j:  z   x  y   x  y qz   x  y qz
  //     0   1  2   3  4  5   6  7  8

  // Quaternion representation
  // i:  0   1  1   2  2  2  2   3  3  3  3
  // j:  z   x  y   x  y qz qw   x  y qz qw
  //     0   1  2   3  4  5  6   7  8  9  10
  Constraint::Jacobian(X, Jacobian);
}

Eigen::Vector3d PlaceConstraint::SupportAreaConstraint::ComputeError(
    Eigen::Ref<const Eigen::MatrixXd> X, double* z_err) const {
  const Object& control = world_.objects()->at(control_frame());
  const Object& target = world_.objects()->at(target_frame());
  // const Eigen::Isometry3d T_control_to_target =
  //     world_.T_control_to_target(X, t_start());
  const spatial_opt::Isometry T_control_to_target =
      world_.T_control_to_target(X, t_start());
  const Eigen::Vector2d com_control =
      (T_control_to_target * control.inertia().com).head<2>();

  if (z_err != nullptr) {
    *z_err = T_control_to_target.translation()(2) - z_surface_;
  }

  Eigen::Vector3d error = Eigen::Vector3d::Zero();

  // Signed distance between com and 2d support area
  {
    const auto projection = target_2d_->project_point(
        Eigen::Isometry2d::Identity(), com_control, false);
    const double sign = projection.is_inside ? -1. : 1.;
    // error(0) = sign * (com_control - projection.point).norm();
    error(0) = 0.5 * sign * (com_control - projection.point).squaredNorm();
  }

  // Signed distance between x-support/y-support (for non-convex shapes) and 2d
  // support area
  for (size_t i = 0; i < 2; i++) {
    if (xy_support_[i].isZero()) continue;

    const Eigen::Vector2d xy_support =
        (T_control_to_target * xy_support_[i]).head<2>();
    const auto projection = target_2d_->project_point(
        Eigen::Isometry2d::Identity(), xy_support, false);
    const double sign = projection.is_inside ? -1. : 1.;
    // error(1 + i) = sign * (xy_support - projection.point).norm();
    error(1 + i) = 0.5 * sign * (xy_support - projection.point).squaredNorm();
  }
  return error;
}

void PlaceConstraint::SupportAreaConstraint::JacobianIndices(
    Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0   1  1   2  2  2  2   3  3  3  3
  // j:  z   x  y   x  y qz qw   x  y qz qw
  // Ignore wx wy
  const size_t var_t = kDof * t_start();
  idx_j(0) = var_t + 2;

  idx_i.segment<2>(1) += 1;
  idx_j.segment<2>(1).setLinSpaced(var_t, var_t + 1);

  for (size_t i = 0; i < 2; i++) {
    idx_i.segment<4>(4 * i + 3) += 2 + i;
    idx_j(4 * i + 3) = var_t + 0;  // x
    idx_j(4 * i + 4) = var_t + 1;  // y
    idx_j(4 * i + 5) = var_t + 5;  // qz
    idx_j(4 * i + 6) = var_t + 6;  // qw
  }
}

}  // namespace logic_opt

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

namespace {

#ifdef PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
// const double kH = 5e-2;
const double kH = 1e-4;
const double kH_ori = 1e-1;
#else  // PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
const double kH = 1e-4;
#endif  // PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE

const double kNormalDepthMargin = 0.01;
const double kNormalDotEpsilon = 0.05;

std::vector<std::unique_ptr<logic_opt::Constraint>>
InitializeConstraints(logic_opt::World3& world, size_t t_push, const std::string& name_pusher,
                      const std::string& name_pushee, logic_opt::PushConstraint& push_constraint) {
  using namespace logic_opt;

  world.ReserveTimesteps(t_push + TouchConstraint::kNumTimesteps +
                         PushConstraint::DestinationConstraint::kNumTimesteps);

  std::vector<std::unique_ptr<Constraint>> constraints;
  constraints.emplace_back(new TouchConstraint(world, t_push, name_pusher, name_pushee));

  const std::string name_target = *world.frames(t_push).parent(name_pushee);
  constraints.emplace_back(new PushConstraint::AlignmentConstraint(world, t_push + 1));
  constraints.emplace_back(new PushConstraint::DestinationConstraint(world, t_push + 1,
                                                                     name_pushee, name_target,
                                                                     push_constraint));
  constraints.emplace_back(new PushConstraint::ContactAreaConstraint(world, t_push,
                                                                     name_pusher, name_pushee,
                                                                     push_constraint));

  constraints.emplace_back(new CollisionConstraint(world, t_push));
  constraints.emplace_back(new CollisionConstraint(world, t_push + 1));
  return constraints;
}

double Gaussian(double x, double std_dev) {
  return std::exp(-x*x / (2 * std_dev * std_dev));
}

}  // namespace

namespace logic_opt {

PushConstraint::PushConstraint(World3& world, size_t t_push, const std::string& name_pusher,
                               const std::string& name_pushee)
    : MultiConstraint(InitializeConstraints(world, t_push, name_pusher, name_pushee, *this),
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
  const Object3& target = world_.objects()->at(name_pushee);
  const double radius = target.collision->project_2d()->bounding_sphere().radius();
  const double height = std::abs(target.collision->aabb().mins()(2));  // from com to bottom
  ncollide3d::shape::ShapeVector shapes;
  shapes.emplace_back(Eigen::Translation3d(Eigen::Vector3d(0., 0., -height / 2.)) *
                      Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX()),
                      std::make_unique<ncollide3d::shape::Capsule>(height / 2., radius));
  pushee_cylinder_ = std::make_unique<ncollide3d::shape::Compound>(std::move(shapes));
}

void PushConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> constraints) {

  const Object3& pusher = world_.objects()->at(name_pusher_);
  const Object3& object = world_.objects()->at(name_pushee_);

  const Eigen::Isometry3d T_pusher_to_object = world_.T_control_to_target(X, t_start());

  // Compute contact in pushee's frame
  contact_ = ncollide3d::query::contact(Eigen::Isometry3d::Identity(),
                                        *object.collision, T_pusher_to_object,
                                        // *pushee_cylinder_, T_pusher_to_object,
                                        *pusher.collision, 100.0);
  projection_ = pusher.collision->project_point(T_pusher_to_object, Eigen::Vector3d::Zero(), true);

  MultiConstraint::Evaluate(X, constraints);
}

void PushConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> Jacobian) {
  
  const Object3& control = world_.objects()->at(name_pusher_);
  const Object3& target = world_.objects()->at(name_pushee_);

  // Precompute contacts for Jacobian computations
  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < FrameConstraint::kDof; i++) {
    const double h = i < 3 ? kH : kH_ori;
    double& x_it = X_h(i, t_start());
    const double x_it_0 = x_it;
    x_it = x_it_0 + h;
    const Eigen::Isometry3d T_control_to_target_hp = world_.T_control_to_target(X_h, t_start());
    // contact_hp_[i] = ncollide3d::query::contact(Eigen::Isometry3d::Identity(),
    //                                             *target.collision, T_control_to_target_hp,
    //                                             // *pushee_cylinder_, T_control_to_target_hp,
    //                                             *control.collision, 100.0);
    projection_hp_[i] = control.collision->project_point(T_control_to_target_hp, Eigen::Vector3d::Zero(), true);
#ifdef PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0 - h;
    const Eigen::Isometry3d T_control_to_target_hn = world_.T_control_to_target(X_h, t_start());
    // contact_hn_[i] = ncollide3d::query::contact(Eigen::Isometry3d::Identity(),
    //                                             *target.collision, T_control_to_target_hn,
    //                                             // *pushee_cylinder_, T_control_to_target_hn,
    //                                             *control.collision, 100.0);
    projection_hn_[i] = control.collision->project_point(T_control_to_target_hn, Eigen::Vector3d::Zero(), true);
#endif  // PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE

    x_it = x_it_0;
  }

  MultiConstraint::Jacobian(X, Jacobian);
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
  if (!push_constraint_.contact_ || push_constraint_.projection_.is_inside) {
    std::cerr << name << "::Evaluate(): No contact!" << std::endl;
    Constraint::Evaluate(X, constraints);
    return;
  }

  // Constrain control com to be inside bounding box height
  const double z_contact = ComputeError(X, push_constraint_.projection_);
  constraints(0) = z_contact - z_max_;

  Constraint::Evaluate(X, constraints);
}

void PushConstraint::ContactAreaConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> Jacobian) {
  if (!push_constraint_.contact_) {
    Constraint::Jacobian(X, Jacobian);
    return;
  }

  Eigen::MatrixXd X_h = X;
  for (size_t i = 0; i < kDof; i++) {
    const double h = i < 3 ? kH : kH_ori;

    // TODO: Different step sizes for pos and ori
    double& x_it = X_h(i, t_start());
    const double x_it_0 = x_it;
    x_it = x_it_0 + h;
    const double z_err_hp = ComputeError(X_h, push_constraint_.projection_hp_[i]);
    x_it = x_it_0 - h;
    const double z_err_hn = ComputeError(X_h, push_constraint_.projection_hn_[i]);
    x_it = x_it_0;

    const double dz_h = z_err_hp != 0. && z_err_hn != 0. ? (z_err_hp - z_err_hn) / (2. * h) : 0.;
    Jacobian(i) = dz_h;
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

double PushConstraint::ContactAreaConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                                           const ncollide3d::query::PointProjection& projection) const {
  return !projection.is_inside ? projection.point(2) : 0.;
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
      world_(world),
      push_constraint_(push_constraint) {
  world.AttachFrame(name_control, name_target, t_push);

  const Object3& object = world_.objects()->at(name_control);
  const ncollide3d::bounding_volume::AABB aabb = object.collision->aabb();
  z_max_ = aabb.maxs()(2);
  z_min_ = aabb.mins()(2);
}

void PushConstraint::DestinationConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> constraints) {
  if (!push_constraint_.contact_ || push_constraint_.projection_.is_inside) {
    // throw std::runtime_error(name + "::Evaluate(): No contact!");
    Constraint::Evaluate(X, constraints);
    return;
  }

  // xy_dot_normal_ = ComputeError(X, push_constraint_.contact_.value(), &z_err_);
  xy_dot_normal_ = ComputeError(X, push_constraint_.projection_, &z_err_, &normal_, &xy_);

  // Constrain movement along z-axis to be 0
  constraints(0) = 0.5 * z_err_ * z_err_;

  // Constrain contact normal = push normal in xy-plane
  // constraints(1) = Gaussian(push_constraint_.contact_->depth, kNormalDepthMargin) *
  //                  (1. - kNormalDotEpsilon - xy_dot_normal_);
  constraints(1) = 1. - kNormalDotEpsilon - xy_dot_normal_;

  Constraint::Evaluate(X, constraints);
}

void PushConstraint::DestinationConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                                     Eigen::Ref<Eigen::VectorXd> Jacobian) {
  // if (!push_constraint_.contact_ || std::abs(push_constraint_.contact_->depth) > kNormalDepthMargin) {
  // if (!push_constraint_.contact_) {
  if (!push_constraint_.contact_ || push_constraint_.projection_.is_inside || xy_dot_normal_ == 0.) {
    Constraint::Jacobian(X, Jacobian);
    return;
  }

  Jacobian(0) = z_err_;

  Eigen::MatrixXd X_h = X;
  {
    const Eigen::Vector2d tangent(-normal_(1), normal_(0));
    const Object3& pusher = world_.objects()->at(push_constraint_.name_pusher_);

    Eigen::Ref<Eigen::Vector2d> x_it = X_h.block<2,1>(0, t_start() - 1);
    const Eigen::Vector2d x_it_0 = x_it;

    x_it = x_it_0 + kH * tangent;
    const Eigen::Isometry3d T_control_to_target_hp = world_.T_control_to_target(X_h, push_constraint_.t_start());
    const auto projection_hp = pusher.collision->project_point(T_control_to_target_hp, Eigen::Vector3d::Zero(), true);
    const double x_err_hp = !projection_hp.is_inside ? ComputeError(X_h, projection_hp) : 0.;

    x_it = x_it_0 - kH * tangent;
    const Eigen::Isometry3d T_control_to_target_hn = world_.T_control_to_target(X_h, push_constraint_.t_start());
    const auto projection_hn = pusher.collision->project_point(T_control_to_target_hn, Eigen::Vector3d::Zero(), true);
    const double x_err_hn = !projection_hn.is_inside ? ComputeError(X_h, projection_hn) : 0.;
    x_it = x_it_0;

    const double dx_h = x_err_hp != 0. && x_err_hn != 0. ? ctrl_utils::Clip((x_err_hp - x_err_hn) / (2. * kH), 10.) : 0.;
    Jacobian.segment<2>(1) = -dx_h * tangent;
  }

  // TODO: Try approximating cylinder
  for (size_t i = 5; i < kDof; i++) {
    // if (!push_constraint_.contact_hp_[i]) //continue;
    // { std::cerr << name << "::Jacobian(): No contact_hp!" << std::endl; continue; }

    const double h = i < 3 ? kH : kH_ori;

    double& x_it = X_h(i, push_constraint_.t_start());
    const double x_it_0 = x_it;
    x_it = x_it_0 + h;
    // const double x_err_hp = ComputeError(X_h, push_constraint_.contact_hp_[i].value());
    // const double x_err_hp = ComputeError(X_h, push_constraint_.projection_hp_[i].value());
    if (push_constraint_.projection_hp_[i].is_inside) {
      x_it = x_it_0;
      continue;
    }
    double x_err_hp = i < 6 ? ComputeError(X_h, push_constraint_.projection_hp_[i])
                                  : ComputeError(X_h, push_constraint_.projection_);
    // x_err_hp = xy_dot_normal_ + ctrl_utils::Clip(x_err_hp - xy_dot_normal_, 2*h);
    // if (x_err_hp == 0. || std::abs(x_err_hp - xy_dot_normal_) > kH_ori) continue;
#ifdef PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0 - h;
    if (push_constraint_.projection_hn_[i].is_inside) {
      x_it = x_it_0;
      continue;
    }
    double x_err_hn =
        // push_constraint_.contact_hn_[i] ? ComputeError(X_h, push_constraint_.contact_hn_[i].value())
            i < 6 ? ComputeError(X_h, push_constraint_.projection_hn_[i])
                   : ComputeError(X_h, push_constraint_.projection_);
    // x_err_hn = xy_dot_normal_ + ctrl_utils::Clip(x_err_hn - xy_dot_normal_, 2*h);
    // if (x_err_hn == 0. || std::abs(x_err_hn - xy_dot_normal_) > kH_ori) continue;
#endif  // PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    x_it = x_it_0;

#ifdef PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    const double dx_h = x_err_hp != 0. && x_err_hn != 0. ? ctrl_utils::Clip((x_err_hp - x_err_hn) / (2. * h), 20.) : 0.;
#else  // PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    const double dx_h = x_err_hp != 0. ? (x_err_hp - xy_dot_normal_) / h : 0.;
#endif  // PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
    Jacobian(1 + i) = -dx_h;
    // Jacobian(1 + i) = -Gaussian(push_constraint_.contact_->depth, kNormalDepthMargin) * dx_h;
    // const double dx_z = ctrl_utils::Cross(normal_, xy_);
    // Jacobian(1 + i) = (dx_h * dx_z > 0.) ? -dx_z : 0.;
    // std::cout << dx_h << " " << dx_z << std::endl;
  }
  Jacobian(1 + 5) = -ctrl_utils::Cross(normal_, xy_);

  Constraint::Jacobian(X, Jacobian);
}

void PushConstraint::DestinationConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                                            Eigen::Ref<Eigen::ArrayXi> idx_j) {
  // i:  0 1      1      1      1       1       1
  // j:  z x_prev y_prev z_prev wx_prev wy_prev wz_prev
  const size_t var_t = kDof * t_start();
  idx_j(0) = var_t + 2;

  const size_t var_t_prev = var_t - kDof;
  idx_i.tail<kDof>() += 1;
  idx_j.tail<kDof>().setLinSpaced(var_t_prev, var_t - 1);
}

// double PushConstraint::DestinationConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
//                                                            const ncollide3d::query::Contact& contact,
//                                                            double* z_err) const {
//   // Compute contact normal
//   const Object3& pusher = world_.objects()->at(push_constraint_.name_pusher_);
//   const Eigen::Isometry3d T_pusher_to_object = world_.T_control_to_target(X, t_start() - 1);

//   // Ray at object's contact point, pointing towards pusher's contact point
//   const ncollide3d::query::Ray ray(contact.world1, (contact.world2 - contact.world1).normalized());
//   const auto intersect = pusher.collision->toi_and_normal_with_ray(T_pusher_to_object, ray, false);
//   if (!intersect || intersect->normal.norm() == 0.) {
//     std::cout << "world1: " << contact.world1.transpose() << std::endl;
//     std::cout << "world2: " << contact.world2.transpose() << std::endl;
//     std::cout << "normal: " << contact.normal.transpose() << std::endl;
//     throw std::runtime_error(name + "::ComputeError(): No intersect!");
//   }

//   // Compute push direction
//   const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());
//   const Eigen::Isometry3d T_control_to_target_prev = world_.T_to_frame(control_frame(), target_frame(),
//                                                                        X, t_start() - 1);
//   const Eigen::Vector3d dir_push = T_control_to_target.translation() - T_control_to_target_prev.translation();

//   push_constraint_.dir_push_object_ = (T_control_to_target.linear().transpose() * dir_push).head<2>().normalized();

//   if (z_err != nullptr) {
//     *z_err = dir_push(2);
//   }

//   const Eigen::Vector3d normal_push = Eigen::Vector3d(dir_push(0), dir_push(1), 0.).normalized();
//   return normal_push.dot(intersect->normal);
// }

double PushConstraint::DestinationConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                                           const ncollide3d::query::PointProjection& projection,
                                                           double* z_err, Eigen::Vector2d* normal,
                                                           Eigen::Vector2d* xy) const {
  // Compute push direction
  const Eigen::Isometry3d T_control_to_target = world_.T_control_to_target(X, t_start());
  const Eigen::Isometry3d T_control_to_target_prev = world_.T_to_frame(control_frame(), target_frame(),
                                                                       X, t_start() - 1);
  const Eigen::Vector3d dir_push = T_control_to_target.translation() - T_control_to_target_prev.translation();

  // push_constraint_.dir_push_object_ = (T_control_to_target.linear().transpose() * dir_push).head<2>().normalized();

  if (z_err != nullptr) {
    *z_err = dir_push(2);
  }

  // Compute contact normal
  const Object3& pusher = world_.objects()->at(push_constraint_.name_pusher_);
  const Eigen::Isometry3d T_pusher_to_object = world_.T_control_to_target(X, push_constraint_.t_start());

  // Ray at object's contact point, pointing towards pusher's contact point
  const ncollide3d::query::Ray ray(Eigen::Vector3d::Zero(), projection.point.normalized());
  const auto intersect = pusher.collision->toi_and_normal_with_ray(T_pusher_to_object, ray, true);
  if (!intersect) {
    std::cout << "point: " << projection.point.transpose() << "; " << projection.is_inside << std::endl;
    const auto p2 = pusher.collision->project_point(T_pusher_to_object, Eigen::Vector3d::Zero(), true);
    std::cout << "point2: " << p2.point.transpose() << std::endl;
    // if (normal != nullptr) {
    //   normal->setZero();
    // }
    // return 0.;
    // std::cout << "depth: " << push_constraint_.contact_->depth << std::endl;
    throw std::runtime_error(name + "::ComputeError(): No intersect!");
  }
  const Eigen::Vector3d normal_intersect = Eigen::Vector3d(intersect->normal(0), intersect->normal(1), 0.).normalized();
  // const Eigen::Vector3d normal_intersect = intersect->normal.normalized();
  if (normal_intersect.norm() == 0.) {
    if (normal != nullptr) normal->setZero();
    if (xy != nullptr) xy->setZero();
    return 0.;
  }

  if (normal != nullptr) {
    *normal = normal_intersect.head<2>();
  }

  const Eigen::Vector3d normal_push = Eigen::Vector3d(dir_push(0), dir_push(1), 0.).normalized();
  if (xy != nullptr) {
    *xy = normal_push.head<2>();
  }

  return normal_push.dot(normal_intersect);
}

}  // namespace logic_opt

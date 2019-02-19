/**
 * pick_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/pick_constraint.h"

#include <ctrl_utils/math.h>

namespace {

const size_t kNumConstraints = 3;
const size_t kDof = 6;
const size_t kLenJacobian = 3;//kNumConstraints * kDof;
const size_t kNumTimesteps = 1;

const double kH = 1e-8;

}  // namespace

namespace LogicOpt {

PickConstraint::PickConstraint(World& world, size_t t_pick, const std::string& name_ee,
                               const std::string& name_object, const Eigen::Vector3d& object_offset)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_pick, kNumTimesteps, name_ee, name_object,
                      "constraint_pick_t" + std::to_string(t_pick)),
      dx_des_(object_offset), world_(world) {
  world.AttachFrame(name_ee, name_object, t_pick);
}

void PickConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> constraints) {
  dx_err_ = ComputeError(X);
  constraints = 0.5 * dx_err_.array().square();
  // dx_err_ = ComputeError(X);
  // constraints = dx_err_;
  // std::cout << constraints.transpose() << "; " << world_.T_control_to_target(X, t_start()).translation().transpose() << std::endl;

  Constraint::Evaluate(X, constraints);
}

void PickConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> Jacobian) {
  // ComputeError(X);
  Jacobian = dx_err_;
  // Eigen::Map<Eigen::MatrixXd> J(Jacobian.data(), kNumConstraints, kDof);
  // for (size_t i = 0; i < kDof; i++) {
  //   Eigen::MatrixXd X_h = X;
  //   X_h(i, t_start()) += kH;
  //   Eigen::Vector3d f_x_h = ComputeError(X_h);
  //   J.col(i) = (f_x_h - dx_err_) / kH;
  // }
}

// void PickConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
//                                      Eigen::Ref<Eigen::ArrayXi> idx_j) {
//   for (size_t i = 0; i < kDof; i++) {
//     idx_i.block<3,1>(i * kNumConstraints, 0) +=
//         Eigen::VectorXi::LinSpaced(kNumConstraints, 0, kNumConstraints - 1).array();
//     idx_j.block<3,1>(i * kNumConstraints, 0) = kDof * t_start() + i;
//   }
// }

Eigen::Vector3d PickConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const {
  const Object& ee = world_.objects()->at(control_frame());
  const Object& object = world_.objects()->at(target_frame());
  const Eigen::Isometry3d T_ee_to_object = world_.T_control_to_target(X, t_start());
  const auto& x_ee = T_ee_to_object.translation();

  auto projection = object.collision->project_point(Eigen::Isometry3d::Identity(),
                                                    x_ee, false);
  return x_ee - projection.point;

  // return 0.5 * (projection.is_inside ? -1. : 1.) * (x_ee - projection.point).array().square();

  // auto contact = ncollide3d::query::contact(Eigen::Isometry3d::Identity(), *object.collision,
  //                                           T_ee_to_object, *ee.collision, 1000.);
  // if (!contact) throw std::runtime_error("Contact returned none.");

  // // std::cout << "Contact: " << std::endl
  // //           << "  world1: " << contact->world1.transpose() << std::endl
  // //           << "  world2: " << contact->world2.transpose() << std::endl
  // //           << "  normal: " << contact->normal.transpose() << std::endl
  // //           << "  depth: " << contact->depth << std::endl;
  // double distance_sq = -ctrl_utils::math::Signum(contact->depth) * (contact->depth * contact->depth);
  // return 0.5 * distance_sq * contact->normal;
}

}  // namespace LogicOpt

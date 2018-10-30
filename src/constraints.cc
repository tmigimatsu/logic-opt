/**
 * constraints.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 27, 2018
 * Authors: Toki Migimatsu
 */

#include "constraints.h"

#include <cassert>  // assert

namespace TrajOpt {

void JointPositionConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::VectorXd> constraints) {
  constraints = 0.5 * (Q.col(timestep) - q_des).array().square();
}

void JointPositionConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Jacobian = Q.col(timestep) - q_des;
}

void JointPositionConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                              Eigen::Ref<Eigen::ArrayXi> idx_j) {
  const size_t& dof = len_jacobian;
  for (size_t i = 0; i < len_jacobian; i++) {
    idx_i(i) += i;
    idx_j(i) = dof * timestep + i;
  }
}

void CartesianPoseConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::VectorXd> constraints) {
  ComputeError(Q);
  constraints(0) = 0.5 * x_quat_err_.squaredNorm();
}

void CartesianPoseConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::VectorXd> Jacobian) {
  ComputeError(Q);
  Jacobian = SpatialDyn::Jacobian(ab_).transpose() * x_quat_err_;
}

void CartesianPoseConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> Q) {
  if (q_.size() == ab_.dof() && (q_.array() == Q.col(timestep).array()).all()) return;

  ab_.set_q(Q.col(timestep));
  x_quat_err_.head<3>() = SpatialDyn::Position(ab_) - x_des;
  x_quat_err_.tail<3>() = SpatialDyn::Opspace::OrientationError(SpatialDyn::Orientation(ab_), quat_des);
  q_ = ab_.q();
}

void CartesianPoseConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                              Eigen::Ref<Eigen::ArrayXi> idx_j) {
  for (size_t i = 0; i < len_jacobian; i++) {
    idx_j(i) = ab_.dof() * timestep + i;
  }
}

AboveTableConstraint::AboveTableConstraint(const SpatialDyn::ArticulatedBody& ab,
                                           const SpatialDyn::RigidBody& table,
                                           size_t t_start, size_t num_timesteps)
    : Constraint(num_timesteps, num_timesteps * ab.dof(), Type::INEQUALITY),
      t_start(t_start), num_timesteps(num_timesteps),
      ab_(ab), table_(table) {

  const auto& pos_table = table_.T_to_parent().translation();
  const Eigen::Vector3d& dim_table = table_.graphics.geometry.scale;
  height_table_ = pos_table(2) + 0.5 * dim_table(2);
  area_table_ = {
    pos_table(0) + 0.5 * dim_table(0),
    pos_table(0) - 0.5 * dim_table(0),
    pos_table(1) + 0.5 * dim_table(1),
    pos_table(1) - 0.5 * dim_table(1)
  };
}

void AboveTableConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                                    Eigen::Ref<Eigen::VectorXd> constraints) {
  for (size_t t = t_start; t < t_start + num_timesteps; t++) {
    ab_.set_q(Q.col(t));
    Eigen::Vector3d pos_ee = SpatialDyn::Position(ab_);

    if (pos_ee(0) > area_table_[0] || pos_ee(0) < area_table_[1] ||
        pos_ee(1) > area_table_[2] || pos_ee(1) < area_table_[3]) {
      constraints(t) = 0.;
    } else {
      // double dz = height_table_ - pos_ee(2);
      // constraints(t) = 1. / 3. * dz * dz * dz;
      double dz = std::min(0., pos_ee(2) - height_table_);
      constraints(t) = 0.5 * dz * dz;
    }
  }
}

void AboveTableConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                    Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Eigen::Map<Eigen::MatrixXd> JacobianMatrix(&Jacobian(0), ab_.dof(), num_timesteps);
  for (size_t t = t_start; t < t_start + num_timesteps; t++) {
    ab_.set_q(Q.col(t));
    Eigen::Vector3d pos_ee = SpatialDyn::Position(ab_);

    if (pos_ee(0) > area_table_[0] || pos_ee(0) < area_table_[1] ||
        pos_ee(1) > area_table_[2] || pos_ee(1) < area_table_[3]) {
      JacobianMatrix.col(t).setZero();
    } else {
      Eigen::VectorXd J_z = SpatialDyn::LinearJacobian(ab_).row(2);
      // double dz = height_table_ - pos_ee(2);
      // JacobianMatrix.col(t) = dz * dz * J_z;
      double dz = std::min(0., pos_ee(2) - height_table_);
      JacobianMatrix.col(t) = dz * J_z.transpose();
    }
  }
}

void AboveTableConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                           Eigen::Ref<Eigen::ArrayXi> idx_j) {
  for (size_t t = 0; t < num_timesteps; t++) {
    for (size_t i = 0; i < ab_.dof(); i++) {
      idx_i(t * ab_.dof() + i) += t;
      idx_j(t * ab_.dof() + i) = (t + t_start) * ab_.dof() + i;
    }
  }
}

}  // namespace TrajOpt

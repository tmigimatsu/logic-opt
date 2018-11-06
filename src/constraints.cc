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

void Constraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                          Eigen::Ref<Eigen::VectorXd> constraints) {
  if (!log.is_open()) return;
  log << constraints.transpose() << std::endl;
}

void JointPositionConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::VectorXd> constraints) {
  constraints = 0.5 * (Q.col(t_goal) - q_des).array().square();
  Constraint::Evaluate(Q, constraints);
}

void JointPositionConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Jacobian = Q.col(t_goal) - q_des;
}

void JointPositionConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                              Eigen::Ref<Eigen::ArrayXi> idx_j) {
  const size_t& dof = len_jacobian;
  for (size_t i = 0; i < len_jacobian; i++) {
    idx_i(i) += i;
    idx_j(i) = dof * t_goal + i;
  }
}

void JointPositionConstraint::Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                      Eigen::Ref<const Eigen::VectorXd> lambda,
                                      Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {
  const size_t& dof = Q.rows();
  for (size_t i = 0; i < dof; i++) {
    Hessian.coeffRef(t_goal * dof + i, t_goal * dof + i) += lambda(i);
  }
}

void JointPositionConstraint::HessianStructure(Eigen::SparseMatrix<bool>& Hessian,
                                               size_t T) {
  const size_t& dof = len_jacobian;
  for (size_t i = 0; i < dof; i++) {
    if (Hessian.coeff(t_goal * dof + i, t_goal * dof + i)) continue;
    Hessian.insert(t_goal * dof + i, t_goal * dof + i) = true;
  }
}

void CartesianPoseConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::VectorXd> constraints) {
  ComputeError(Q);
  if (num_constraints == 1) {
    constraints(0) = (0.5 * x_quat_err_.array().square()).sum();
  } else {
    constraints = 0.5 * x_quat_err_.array().square();
  }
  Constraint::Evaluate(Q, constraints);
}

void CartesianPoseConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Eigen::Map<Eigen::MatrixXd> J(&Jacobian(0), num_constraints, ab_.dof());

  ComputeError(Q);
  if (num_constraints == 1) {
    J = (SpatialDyn::Jacobian(ab_).array().colwise() * x_quat_err_.array()).colwise().sum();
  } else {
    J = SpatialDyn::Jacobian(ab_).array().colwise() * x_quat_err_.array();
  }
}

void CartesianPoseConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> Q) {
  if (q_.size() == ab_.dof() && (q_.array() == Q.col(t_goal).array()).all()) return;

  ab_.set_q(Q.col(t_goal));
  x_quat_err_.head<3>() = SpatialDyn::Position(ab_) - x_des;
  x_quat_err_.tail<3>() = SpatialDyn::Opspace::OrientationError(SpatialDyn::Orientation(ab_), quat_des);
  q_ = ab_.q();
}

void CartesianPoseConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                              Eigen::Ref<Eigen::ArrayXi> idx_j) {
  Eigen::Map<Eigen::MatrixXi> Idx_i(&idx_i(0), num_constraints, ab_.dof());
  Eigen::Map<Eigen::MatrixXi> Idx_j(&idx_j(0), num_constraints, ab_.dof());

  Idx_i.colwise() += Eigen::VectorXi::LinSpaced(num_constraints, 0, num_constraints - 1);
  Idx_j.rowwise() = Eigen::VectorXi::LinSpaced(ab_.dof(), t_goal * ab_.dof(),
                                               (t_goal + 1) * ab_.dof() - 1).transpose();
}

void CartesianPoseConstraint::Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                      Eigen::Ref<const Eigen::VectorXd> lambda,
                                      Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {
  const size_t& dof = Q.rows();
  Eigen::MatrixXd H(dof, dof);
  Eigen::TensorMap<Eigen::Tensor2d> tensor_H(&H(0, 0), dof, dof);

  ComputeError(Q);
  const Eigen::Matrix6Xd& J = SpatialDyn::Jacobian(ab_);
  H = J.transpose() * (lambda.asDiagonal() * J);

  Eigen::Vector6d dx = lambda.array() * x_quat_err_.array();
  Eigen::TensorMap<Eigen::Tensor1d> tensor_dx(&dx(0), 6);

  Eigen::array<Eigen::IndexPair<int>, 1> product_dims = { Eigen::IndexPair<int>(2, 0) };
  tensor_H += SpatialDyn::Hessian(ab_).contract(tensor_dx, product_dims);

  for (size_t j = 0; j < dof; j++) {
    for (size_t i = 0; i <= j; i++) {
      Hessian.coeffRef(i + t_goal * dof, j + t_goal * dof) += H(i, j);
    }
  }
}

void CartesianPoseConstraint::HessianStructure(Eigen::SparseMatrix<bool>& Hessian,
                                               size_t T) {
  for (size_t j = 0; j < ab_.dof(); j++) {
    for (size_t i = 0; i <= j; i++) {
      if (Hessian.coeff(i + t_goal * ab_.dof(), j + t_goal * ab_.dof())) continue;
      Hessian.insert(i + t_goal * ab_.dof(), j + t_goal * ab_.dof()) = true;
    }
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
    Eigen::Vector3d pos_ee = SpatialDyn::Position(ab_, Q.col(t));

    if (pos_ee(0) > area_table_[0] || pos_ee(0) < area_table_[1] ||
        pos_ee(1) > area_table_[2] || pos_ee(1) < area_table_[3]) {
      constraints(t) = 0.;
    } else {
      double dz = std::min(0., pos_ee(2) - height_table_);
      constraints(t) = 0.5 * dz * dz;
    }
  }
  Constraint::Evaluate(Q, constraints);
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

void PickConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                              Eigen::Ref<Eigen::VectorXd> constraints) {
  ComputeError(Q);
  constraints = 0.5 * x_err_.array().square();
  Constraint::Evaluate(Q, constraints);
}

void PickConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                              Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Eigen::Map<Eigen::MatrixXd> J(&Jacobian(0), 3, ab_.dof());

  ComputeError(Q);
  J = SpatialDyn::LinearJacobian(ab_).array().colwise() * x_err_.array();
}

void PickConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> Q) {
  ab_.set_q(Q.col(t_pick));
  x_err_ = SpatialDyn::Position(ab_, -1, ee_offset) - object_.T_to_parent().translation();
}

void PickConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                     Eigen::Ref<Eigen::ArrayXi> idx_j) {
  Eigen::Map<Eigen::MatrixXi> Idx_i(&idx_i(0), num_constraints, ab_.dof());
  Eigen::Map<Eigen::MatrixXi> Idx_j(&idx_j(0), num_constraints, ab_.dof());

  Idx_i.colwise() += Eigen::VectorXi::LinSpaced(num_constraints, 0, num_constraints - 1);
  Idx_j.rowwise() = Eigen::VectorXi::LinSpaced(ab_.dof(), t_pick * ab_.dof(),
                                               (t_pick + 1) * ab_.dof() - 1).transpose();
}

void PlaceConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                               Eigen::Ref<Eigen::VectorXd> constraints) {
  ComputeError(Q);
  constraints = 0.5 * x_quat_err_.array().square();
  Constraint::Evaluate(Q, constraints);
}

void PlaceConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                               Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Eigen::Map<Eigen::MatrixXd> J(&Jacobian(0), 6, ab_.dof());

  ComputeError(Q);
  J = SpatialDyn::Jacobian(ab_).array().colwise() * x_quat_err_.array();
}

void PlaceConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> Q) {
  ab_.set_q(Q.col(t_pick));
  Eigen::Isometry3d T_pick_ee_to_world = ab_.T_to_world(-1);
  const Eigen::Isometry3d& T_pick_object_to_world = object_.T_to_parent();
  Eigen::Isometry3d T_object_to_ee = T_pick_ee_to_world.inverse() * T_pick_object_to_world;
  Eigen::Isometry3d T_ee_to_object = T_pick_object_to_world.inverse() * T_pick_ee_to_world;

  ab_.set_q(Q.col(t_place));
  x_quat_err_.head<3>() = SpatialDyn::Position(ab_, -1) - (x_des + T_ee_to_object.translation());

  Eigen::Quaterniond quat_ee = SpatialDyn::Orientation(ab_);
  Eigen::Quaterniond quat_ee_des(T_ee_to_object.linear() * quat_des);
  x_quat_err_.tail<3>() = SpatialDyn::Opspace::OrientationError(quat_ee, quat_ee_des);
}

void PlaceConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                      Eigen::Ref<Eigen::ArrayXi> idx_j) {
  Eigen::Map<Eigen::MatrixXi> Idx_i(&idx_i(0), num_constraints, ab_.dof());
  Eigen::Map<Eigen::MatrixXi> Idx_j(&idx_j(0), num_constraints, ab_.dof());

  Idx_i.colwise() += Eigen::VectorXi::LinSpaced(num_constraints, 0, num_constraints - 1);
  Idx_j.rowwise() = Eigen::VectorXi::LinSpaced(ab_.dof(), t_place * ab_.dof(),
                                               (t_place + 1) * ab_.dof() - 1).transpose();
}


}  // namespace TrajOpt

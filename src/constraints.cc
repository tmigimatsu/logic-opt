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

  // Position
  switch (layout) {
    case Layout::SCALAR:  // Combined position, orientation scalar
      constraints(0) = 0.5 * x_quat_err_.squaredNorm();
      break;
    case Layout::POS_SCALAR:
    case Layout::SCALAR_SCALAR:
    case Layout::SCALAR_VECTOR:  // Position scalar
      constraints(0) = 0.5 * x_quat_err_.head<3>().squaredNorm();
      break;
    case Layout::POS_VECTOR:
    case Layout::VECTOR_SCALAR:
    case Layout::VECTOR_VECTOR:  // Position vector
      constraints.head(3) = 0.5 * x_quat_err_.head<3>().array().square();
      break;
    default: break;
  }

  // Orientation
  switch (layout) {
    case Layout::ORI_SCALAR:
    case Layout::SCALAR_SCALAR:
    case Layout::VECTOR_SCALAR:  // Orientation scalar
      constraints(constraints.size() - 1) = 0.5 * x_quat_err_.tail<3>().squaredNorm();
      break;
    case Layout::ORI_VECTOR:
    case Layout::SCALAR_VECTOR:
    case Layout::VECTOR_VECTOR:  // Orientation vector
      constraints.tail(3) = 0.5 * x_quat_err_.tail<3>().array().square();
      break;
    default: break;
  }
  Constraint::Evaluate(Q, constraints);
}

void CartesianPoseConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Eigen::Map<Eigen::MatrixXd> J(&Jacobian(0), num_constraints, ab_.dof());

  ComputeError(Q);
  const Eigen::Matrix6Xd& J_x = SpatialDyn::Jacobian(ab_, -1, ee_offset);

  // Position
  switch (layout) {
    case Layout::SCALAR:  // Combined position, orientation scalar
      J = x_quat_err_.transpose() * J_x;
      break;
    case Layout::POS_SCALAR:
    case Layout::SCALAR_SCALAR:
    case Layout::SCALAR_VECTOR:  // Position scalar
      J.row(0) = x_quat_err_.head<3>().transpose() * J_x.topRows<3>();
      break;
    case Layout::POS_VECTOR:
    case Layout::VECTOR_SCALAR:
    case Layout::VECTOR_VECTOR:  // Position vector
      J.topRows(3) = J_x.topRows<3>().array().colwise() * x_quat_err_.head<3>().array();
      break;
    default: break;
  }

  // Orientation
  switch (layout) {
    case Layout::ORI_SCALAR:
    case Layout::SCALAR_SCALAR:
    case Layout::VECTOR_SCALAR:  // Orientation scalar
      J.row(J.rows() - 1) = x_quat_err_.tail<3>().transpose() * J_x.bottomRows<3>();
      break;
    case Layout::ORI_VECTOR:
    case Layout::SCALAR_VECTOR:
    case Layout::VECTOR_VECTOR:  // Orientation vector
      J.bottomRows(3) = J_x.bottomRows<3>().array().colwise() * x_quat_err_.tail<3>().array();
      break;
    default: break;
  }
}

void CartesianPoseConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> Q) {
  if (q_.size() == ab_.dof() && (q_.array() == Q.col(t_goal).array()).all()) return;

  ab_.set_q(Q.col(t_goal));
  if (layout != Layout::ORI_SCALAR && layout != Layout::ORI_VECTOR) {
    x_quat_err_.head<3>() = SpatialDyn::Position(ab_, -1, ee_offset) - x_des;
  }
  if (layout != Layout::POS_SCALAR && layout != Layout::POS_VECTOR) {
    x_quat_err_.tail<3>() = SpatialDyn::Opspace::OrientationError(SpatialDyn::Orientation(ab_), quat_des);
  }
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
  Eigen::Vector6d lambda_6d = Eigen::Vector6d::Zero();

  // Position
  switch (layout) {
    case Layout::SCALAR:  // Combined position, orientation scalar
      lambda_6d.fill(lambda(0));
      break;
    case Layout::POS_SCALAR:
    case Layout::SCALAR_SCALAR:
    case Layout::SCALAR_VECTOR:  // Position scalar
      lambda_6d.head<3>().fill(lambda(0));
      break;
    case Layout::POS_VECTOR:
    case Layout::VECTOR_SCALAR:
    case Layout::VECTOR_VECTOR:  // Position vector
      lambda_6d.head<3>() = lambda.head(3);
      break;
    default: break;
  }

  // Orientation
  switch (layout) {
    case Layout::ORI_SCALAR:
    case Layout::SCALAR_SCALAR:
    case Layout::VECTOR_SCALAR:  // Orientation scalar
      lambda_6d.tail<3>().fill(lambda(lambda.size() - 1));
      break;
    case Layout::ORI_VECTOR:
    case Layout::SCALAR_VECTOR:
    case Layout::VECTOR_VECTOR:  // Orientation vector
      lambda_6d.tail<3>() = lambda.tail(3);
      break;
    default: break;
  }

  ComputeError(Q);
  const Eigen::Matrix6Xd& J = SpatialDyn::Jacobian(ab_, -1, ee_offset);
  H = J.transpose() * (lambda_6d.asDiagonal() * J);

  Eigen::Vector6d dx = lambda_6d.array() * x_quat_err_.array();
  Eigen::TensorMap<Eigen::Tensor1d> tensor_dx(&dx(0), 6);

  Eigen::array<Eigen::IndexPair<int>, 1> product_dims = { Eigen::IndexPair<int>(2, 0) };
  Eigen::Tensor2d tH = SpatialDyn::Hessian(ab_).contract(tensor_dx, product_dims);
  // std::cout << "H JTJ2: " << std::endl << tensor_H << std::endl << std::endl;
  // std::cout << "H dJ: " << std::endl << tH << std::endl << std::endl;
  // std::cout << "dx: " << dx.transpose() << std::endl;
  tensor_H += SpatialDyn::Hessian(ab_, -1, ee_offset).contract(tensor_dx, product_dims);

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

size_t CartesianPoseConstraint::NumConstraints(CartesianPoseConstraint::Layout l) {
  switch (l) {
    case Layout::SCALAR:
    case Layout::POS_SCALAR:
    case Layout::ORI_SCALAR:
      return 1;
    case Layout::SCALAR_SCALAR:
      return 2;
    case Layout::POS_VECTOR:
    case Layout::ORI_VECTOR:
      return 3;
    case Layout::VECTOR_SCALAR:
    case Layout::SCALAR_VECTOR:
      return 4;
    case Layout::VECTOR_VECTOR: return 6;
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
  x_des = object_.T_to_parent().translation();
  CartesianPoseConstraint::Evaluate(Q, constraints);
}

void PickConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                              Eigen::Ref<Eigen::VectorXd> Jacobian) {
  x_des = object_.T_to_parent().translation();
  CartesianPoseConstraint::Jacobian(Q, Jacobian);
}

void PickConstraint::Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                             Eigen::Ref<const Eigen::VectorXd> lambda,
                             Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {
  x_des = object_.T_to_parent().translation();
  CartesianPoseConstraint::Hessian(Q, lambda, Hessian);
}

void PlaceConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                               Eigen::Ref<Eigen::VectorXd> constraints) {
  ComputePlacePose(Q);
  CartesianPoseConstraint::Evaluate(Q, constraints);
}

void PlaceConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                               Eigen::Ref<Eigen::VectorXd> Jacobian) {
  ComputePlacePose(Q);
  CartesianPoseConstraint::Jacobian(Q, Jacobian);
}

void PlaceConstraint::Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                              Eigen::Ref<const Eigen::VectorXd> lambda,
                              Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {
  ComputePlacePose(Q);
  CartesianPoseConstraint::Hessian(Q, lambda, Hessian);
}

void PlaceConstraint::ComputePlacePose(Eigen::Ref<const Eigen::MatrixXd> Q) {
  ab_.set_q(Q.col(t_pick));

  Eigen::Isometry3d T_pick_ee_to_world = ab_.T_to_world(-1);
  const Eigen::Isometry3d& T_pick_object_to_world = object_.T_to_parent();
  Eigen::Isometry3d T_ee_to_object = T_pick_object_to_world.inverse() * T_pick_ee_to_world;

  x_des = x_des_place + T_ee_to_object.translation();
  quat_des = T_ee_to_object.linear() * quat_des_place;

  ab_.set_q(Q.col(t_goal));
}

}  // namespace TrajOpt

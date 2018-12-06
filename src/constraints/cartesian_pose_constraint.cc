/**
 * cartesian_pose_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/cartesian_pose_constraint.h"

namespace LogicOpt {

void CartesianPoseConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::VectorXd> constraints) {
  ComputeError(Q);

  // Position
  switch (layout_) {
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
  switch (layout_) {
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
  Eigen::Map<Eigen::MatrixXd> J(&Jacobian(0), num_constraints_, ab_.dof());

  ComputeError(Q);
  const Eigen::Matrix6Xd& J_x = SpatialDyn::Jacobian(ab_, -1, ee_offset_);

  // Position
  switch (layout_) {
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
  switch (layout_) {
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
  if (q_.size() == ab_.dof() && (q_.array() == Q.col(t_start_).array()).all()) return;

  ab_.set_q(Q.col(t_start_));
  if (layout_ != Layout::ORI_SCALAR && layout_ != Layout::ORI_VECTOR) {
    x_quat_err_.head<3>() = SpatialDyn::Position(ab_, -1, ee_offset_) - x_des_;
  }
  if (layout_ != Layout::POS_SCALAR && layout_ != Layout::POS_VECTOR) {
    x_quat_err_.tail<3>() = SpatialDyn::Opspace::OrientationError(SpatialDyn::Orientation(ab_),
                                                                  quat_des_);
  }
  q_ = ab_.q();
}

void CartesianPoseConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                              Eigen::Ref<Eigen::ArrayXi> idx_j) {
  Eigen::Map<Eigen::MatrixXi> Idx_i(&idx_i(0), num_constraints_, ab_.dof());
  Eigen::Map<Eigen::MatrixXi> Idx_j(&idx_j(0), num_constraints_, ab_.dof());

  Idx_i.colwise() += Eigen::VectorXi::LinSpaced(num_constraints_, 0, num_constraints_ - 1);
  Idx_j.rowwise() = Eigen::VectorXi::LinSpaced(ab_.dof(), t_start_ * ab_.dof(),
                                               (t_start_ + 1) * ab_.dof() - 1).transpose();
}

void CartesianPoseConstraint::Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                      Eigen::Ref<const Eigen::VectorXd> lambda,
                                      Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {
  const size_t& dof = Q.rows();
  Eigen::MatrixXd H(dof, dof);
  Eigen::TensorMap<Eigen::Tensor2d> tensor_H(&H(0, 0), dof, dof);
  Eigen::Vector6d lambda_6d = Eigen::Vector6d::Zero();

  // Position
  switch (layout_) {
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
  switch (layout_) {
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
  const Eigen::Matrix6Xd& J = SpatialDyn::Jacobian(ab_, -1, ee_offset_);
  H = J.transpose() * (lambda_6d.asDiagonal() * J);

  Eigen::Vector6d dx = lambda_6d.array() * x_quat_err_.array();
  Eigen::TensorMap<Eigen::Tensor1d> tensor_dx(&dx(0), 6);

  Eigen::array<Eigen::IndexPair<int>, 1> product_dims = { Eigen::IndexPair<int>(2, 0) };
  Eigen::Tensor2d tH = SpatialDyn::Hessian(ab_).contract(tensor_dx, product_dims);
  tensor_H += SpatialDyn::Hessian(ab_, -1, ee_offset_).contract(tensor_dx, product_dims);

  for (size_t j = 0; j < dof; j++) {
    for (size_t i = 0; i <= j; i++) {
      Hessian.coeffRef(i + t_start_ * dof, j + t_start_ * dof) += H(i, j);
    }
  }
}

void CartesianPoseConstraint::HessianStructure(Eigen::SparseMatrix<bool>& Hessian) {
  for (size_t j = 0; j < ab_.dof(); j++) {
    for (size_t i = 0; i <= j; i++) {
      if (Hessian.coeff(i + t_start_ * ab_.dof(), j + t_start_ * ab_.dof())) continue;
      Hessian.insert(i + t_start_ * ab_.dof(), j + t_start_ * ab_.dof()) = true;
    }
  }
}

size_t CartesianPoseConstraint::NumConstraints(CartesianPoseConstraint::Layout layout) {
  switch (layout) {
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

}  // namespace LogicOpt

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
  } else if (num_constraints == 4) {
    constraints.head<3>() = 0.5 * x_quat_err_.head<3>().array().square();
    constraints(3) = 0.5 * x_quat_err_.tail<3>().squaredNorm();
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
  } else if (num_constraints == 4) {
    Eigen::Matrix6Xd J_x = SpatialDyn::Jacobian(ab_);
    J.topRows(3) = J_x.topRows<3>().array().colwise() * x_quat_err_.head<3>().array();
    J.row(3) = x_quat_err_.tail<3>().transpose() * J_x.bottomRows<3>();
  } else {
    // Eigen::Quaterniond quat = SpatialDyn::Orientation(ab_);
    // // Eigen::AngleAxisd aa(quat);
    // // Eigen::AngleAxisd aa_des(quat_des);

    // // double angle = x_quat_err_.tail<3>().norm();
    // // Eigen::Vector3d axis = x_quat_err_.tail<3>() / angle;

    // // double q_dot_qd = quat.coeffs().dot(quat_des.coeffs());
    // // double d_arccos = -1 / std::sqrt(1 - (q_dot_qd * q_dot_qd));
    // // Eigen::Vector4d grad_dist = 2 * d_arccos * quat_des.coeffs();
    // // Eigen::Matrix<double,4,3> E_quat = SpatialDyn::Opspace::QuaternionJacobian(quat);
    // // Eigen::Matrix3d E_r1 = axis * (grad_dist.transpose() * E_quat);
    // // Eigen::Matrix3d E_r2 = -angle * aa_des.axis().crossMatrix() * aa.axis().crossMatrix();
    // // Eigen::Matrix3d E_r = E_r1 - E_r2;
    // // std::cout << "E_r1: " << std::endl << E_r1 << std::endl << std::endl;
    // // std::cout << "E_r2: " << std::endl << E_r2 << std::endl << std::endl;
    // // std::cout << "E_r: " << std::endl << E_r << std::endl << std::endl;

    // // Eigen::Matrix6Xd J_x = SpatialDyn::Jacobian(ab_);
    // // J_x.bottomRows<3>() = E_r * J_x.bottomRows<3>();
    // // std::cout << "J_x: " << std::endl << J_x << std::endl << std::endl;

    // double angle = x_quat_err_.tail<3>().norm() / 2;
    // Eigen::Vector3d axis = x_quat_err_.tail<3>() / (2 * angle);
    // std::cout << "AA: " << axis.transpose() << "; " << angle << std::endl;

    // // double theta_tan = angle / std::tan(angle);
    // // Eigen::Matrix3d dlog_err = theta_tan * Eigen::Matrix3d::Identity() +
    // //                            (1 - theta_tan) * aa_err.axis() * aa_err.axis().transpose() +
    // //                            (angle * aa_err.axis()).crossMatrix();
    // // Eigen::Matrix<double,4,3> E_quat = SpatialDyn::Opspace::QuaternionJacobian(quat);
    // // Eigen::Matrix3d E_r = dlog_err * E_quat. topRows<3>();
    // // std::cout << "E_r: " << std::endl << E_r << std::endl << std::endl;

    // // Eigen::Matrix3d E_r = Eigen::Matrix3d::Identity() + (0.5 * angle * axis).crossMatrix() +
    // //                       (1 - angle / std::tan(angle)) * axis.doubleCrossMatrix();
    // // std::cout << "E_r: " << std::endl << E_r << std::endl << std::endl;

    // Eigen::Matrix3d E_r1 = axis * axis.transpose();
    // Eigen::Matrix3d E_r2 = 0.5 * angle * (axis.crossMatrix() +
    //                        std::sin(angle) / (1 - std::cos(angle)) * axis.doubleCrossMatrix());
    // // Eigen::Matrix3d E_r = E_r2 - E_r1;
    // Eigen::Quaterniond quat_err(0, x_quat_err_(3), x_quat_err_(4), x_quat_err_(5));
    // Eigen::Matrix3d E_r = SpatialDyn::Opspace::QuaternionJacobian(quat_err).bottomRows<3>();
    // // Eigen::Matrix3d E_r2 = angle * (1 / std::tan(angle) * (Eigen::Matrix3d::Identity() - E_r1) +
    // //                                 axis.crossMatrix());
    // // Eigen::Matrix3d E_r = E_r2 + E_r1;
    // // Eigen::Matrix3d E_r = axis * axis.transpose() -
    // //                       0.5 * angle * (axis.crossMatrix() +
    // //                       std::sin(angle) / (1 - std::cos(angle)) * axis.doubleCrossMatrix());

    // std::cout << "E_r1: " << std::endl << E_r1 << std::endl << std::endl;
    // std::cout << "E_r2: " << std::endl << E_r2 << std::endl << std::endl;
    // std::cout << "E_r: " << std::endl << E_r << std::endl << std::endl;

    // Eigen::Matrix6Xd J_x = SpatialDyn::Jacobian(ab_);
    // J_x.bottomRows<3>() = E_r * J_x.bottomRows<3>();
    // std::cout << "J_x: " << std::endl << J_x << std::endl << std::endl;

    // // Eigen::Quaterniond quat_err(0, x_quat_err_(3), x_quat_err_(4), x_quat_err_(5));
    // // double angle = x_quat_err_.tail<3>().norm();
    // // Eigen::Vector3d axis = x_quat_err_.tail<3>() / angle;
    // // Eigen::AngleAxisd aa(angle, axis);
    // // Eigen::Matrix<double,4,3> E_aa = SpatialDyn::Opspace::AngleAxisJacobian(aa);
    // // std::cout << angle << "; " << axis.transpose() << std::endl;
    // // std::cout << E_aa << std::endl << std::endl;
    // // Eigen::Matrix<double,3,3> E_r = angle * E_aa.bottomRows<3>() + axis * E_aa.row(0);
    // // // Eigen::Matrix<double,3,3> E_r = SpatialDyn::Opspace::QuaternionJacobian(quat_err).bottomRows<3>();
    // // std::cout << E_r << std::endl << std::endl;
    // // Eigen::Matrix6Xd J_x = SpatialDyn::Jacobian(ab_);
    // // J_x.bottomRows<3>() = E_r * J_x.bottomRows<3>();
    // // std::cout << J_x << std::endl << std::endl;
    // // J_x.bottomRows<3>() = J_x.bottomRows<3>().colwise().cross(-x_quat_err_.tail<3>());

    // J = J_x.array().colwise() * x_quat_err_.array();
    J = SpatialDyn::Jacobian(ab_).array().colwise() * x_quat_err_.array();
    // std::cout << "J: " << std::endl << J << std::endl << std::endl;
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

  Eigen::Vector6d lambda_6d;
  if (num_constraints == 1) {
    lambda_6d.fill(lambda(0));
  } else if (lambda.size() == 4) {
    lambda_6d.head<3>() = lambda.head<3>();
    lambda_6d.tail<3>().fill(lambda(3));
  } else {
    lambda_6d = lambda;
  }
  std::cout << lambda_6d.transpose() << std::endl;

  ComputeError(Q);
  const Eigen::Matrix6Xd& J = SpatialDyn::Jacobian(ab_);
  H = J.transpose() * (lambda_6d.asDiagonal() * J);
  // std::cout << "H JTJ: " << std::endl << H << std::endl << std::endl;

  Eigen::Vector6d dx = lambda_6d.array() * x_quat_err_.array();
  Eigen::TensorMap<Eigen::Tensor1d> tensor_dx(&dx(0), 6);

  Eigen::array<Eigen::IndexPair<int>, 1> product_dims = { Eigen::IndexPair<int>(2, 0) };
  Eigen::Tensor2d tH = SpatialDyn::Hessian(ab_).contract(tensor_dx, product_dims);
  // std::cout << "H JTJ2: " << std::endl << tensor_H << std::endl << std::endl;
  // std::cout << "H dJ: " << std::endl << tH << std::endl << std::endl;
  // std::cout << "dx: " << dx.transpose() << std::endl;
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
  J = SpatialDyn::LinearJacobian(ab_, -1, ee_offset).array().colwise() * x_err_.array();
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

void PickConstraint::Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                             Eigen::Ref<const Eigen::VectorXd> lambda,
                             Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {
  const size_t& dof = Q.rows();
  Eigen::MatrixXd H(dof, dof);
  Eigen::TensorMap<Eigen::Tensor2d> tensor_H(&H(0, 0), dof, dof);

  ComputeError(Q);
  Eigen::Matrix3Xd J = SpatialDyn::LinearJacobian(ab_, -1, ee_offset);
  H = J.transpose() * (lambda.asDiagonal() * J);

  Eigen::Vector3d dx = lambda.array() * x_err_.array();
  Eigen::TensorMap<Eigen::Tensor1d> tensor_dx(&dx(0), 3);

  Eigen::array<Eigen::IndexPair<int>, 1> product_dims = { Eigen::IndexPair<int>(2, 0) };
  Eigen::array<int, 3> offsets = { 0, 0, 0 };
  Eigen::array<int, 3> extents = { static_cast<int>(dof), static_cast<int>(dof), 3 };
  tensor_H += SpatialDyn::Hessian(ab_).slice(offsets, extents).contract(tensor_dx, product_dims);

  for (size_t j = 0; j < dof; j++) {
    for (size_t i = 0; i <= j; i++) {
      Hessian.coeffRef(i + t_pick * dof, j + t_pick * dof) += H(i, j);
    }
  }
}

void PickConstraint::HessianStructure(Eigen::SparseMatrix<bool>& Hessian, size_t T) {
  for (size_t j = 0; j < ab_.dof(); j++) {
    for (size_t i = 0; i <= j; i++) {
      if (Hessian.coeff(i + t_pick * ab_.dof(), j + t_pick * ab_.dof())) continue;
      Hessian.insert(i + t_pick * ab_.dof(), j + t_pick * ab_.dof()) = true;
    }
  }
}


void PlaceConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                               Eigen::Ref<Eigen::VectorXd> constraints) {
  ComputeError(Q);
  constraints.head<3>() = 0.5 * x_quat_err_.head<3>().array().square();
  constraints(3) = 0.5 * x_quat_err_.tail<3>().squaredNorm();
  Constraint::Evaluate(Q, constraints);
}

void PlaceConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                               Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Eigen::Map<Eigen::MatrixXd> J(&Jacobian(0), num_constraints, ab_.dof());

  ComputeError(Q);
  Eigen::Matrix6Xd J_x = SpatialDyn::Jacobian(ab_);
  J.topRows(3) = J_x.topRows<3>().array().colwise() * x_quat_err_.head<3>().array();
  J.row(3) = 2 * x_quat_err_.tail<3>().transpose() * J_x.bottomRows<3>();
}

void PlaceConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> Q) {
  ab_.set_q(Q.col(t_pick));
  Eigen::Isometry3d T_pick_ee_to_world = ab_.T_to_world(-1);
  const Eigen::Isometry3d& T_pick_object_to_world = object_.T_to_parent();
  Eigen::Isometry3d T_ee_to_object = T_pick_object_to_world.inverse() * T_pick_ee_to_world;

  ab_.set_q(Q.col(t_place));
  x_quat_err_.head<3>() = SpatialDyn::Position(ab_) - (x_des + T_ee_to_object.translation());

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

void PlaceConstraint::Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                              Eigen::Ref<const Eigen::VectorXd> lambda,
                              Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {
  const size_t& dof = Q.rows();
  Eigen::MatrixXd H(dof, dof);
  Eigen::TensorMap<Eigen::Tensor2d> tensor_H(&H(0, 0), dof, dof);

  Eigen::Vector6d lambda_6d;
  lambda_6d.head<3>() = lambda.head<3>();
  lambda_6d.tail<3>().fill(2 * lambda(3));

  ComputeError(Q);
  const Eigen::Matrix6Xd& J = SpatialDyn::Jacobian(ab_);
  H = J.transpose() * (lambda_6d.asDiagonal() * J);

  Eigen::Vector6d dx = lambda_6d.array() * x_quat_err_.array();
  Eigen::TensorMap<Eigen::Tensor1d> tensor_dx(&dx(0), 6);

  Eigen::array<Eigen::IndexPair<int>, 1> product_dims = { Eigen::IndexPair<int>(2, 0) };
  tensor_H += SpatialDyn::Hessian(ab_).contract(tensor_dx, product_dims);

  for (size_t j = 0; j < dof; j++) {
    for (size_t i = 0; i <= j; i++) {
      Hessian.coeffRef(i + t_place * dof, j + t_place * dof) += H(i, j);
    }
  }
}

void PlaceConstraint::HessianStructure(Eigen::SparseMatrix<bool>& Hessian,
                                       size_t T) {
  for (size_t j = 0; j < ab_.dof(); j++) {
    for (size_t i = 0; i <= j; i++) {
      if (Hessian.coeff(i + t_place * ab_.dof(), j + t_place * ab_.dof())) continue;
      Hessian.insert(i + t_place * ab_.dof(), j + t_place * ab_.dof()) = true;
    }
  }
}

}  // namespace TrajOpt

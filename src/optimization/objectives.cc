/**
 * objectives.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 26, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/optimization/objectives.h"

#include <cassert>  // assert

namespace LogicOpt {

void Objective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) {
  if (!log.is_open()) return;
  log << objective << std::endl;
}

void MinNormObjective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) {
  Eigen::Map<const Eigen::VectorXd> x(&X.coeff(0, 0), X.size());
  objective += coeff_ * 0.5 * x.squaredNorm();

  Objective::Evaluate(X, objective);
}

void MinNormObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> X,
                                Eigen::Ref<Eigen::MatrixXd> Gradient) {
  Gradient = X;
}

void LinearVelocityObjective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) {
  Eigen::Vector3d x_t = world_.Position(name_ee_, world_.kWorldFrame, X, 0);
  for (size_t t = 0; t < X.cols() - 1; t++) {
    Eigen::Vector3d x_next = world_.Position(name_ee_, world_.kWorldFrame, X, t+1);

    // 0.5 * || x_{t+1} - x_{t} ||^2
    objective += coeff_ * 0.5 * (x_next - x_t).squaredNorm();

    x_t = x_next;
  }
  Objective::Evaluate(X, objective);
}

void LinearVelocityObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> X,
                                       Eigen::Ref<Eigen::MatrixXd> Gradient) {
  Eigen::Vector3d dx_prev = Eigen::Vector3d::Zero();
  Eigen::Map<Eigen::VectorXd> gradient(&Gradient(0, 0), Gradient.size());

  Eigen::Vector3d x_t = world_.Position(name_ee_, world_.kWorldFrame, X, 0);
  for (size_t t = 0; t < X.cols() - 1; t++) {
    Eigen::Matrix3Xd J_t = world_.PositionJacobian(name_ee_, X, t);

    Eigen::Vector3d x_next = world_.Position(name_ee_, world_.kWorldFrame, X, t+1);
    Eigen::Vector3d dx_t = x_next - x_t;

    // J_{:,t} = J_t^T * ((x_{t} - x_{t-1}) - (x_{t+1} - x_{t}))
    gradient += coeff_ * (J_t.transpose() * (dx_prev - dx_t));

    x_t = x_next;
    dx_prev = dx_t;
  }
}

#if 0
void JointPositionObjective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) {
  for (size_t t = 0; t < Q.cols(); t++) {
    // 0.5 * || q_{t} - q_des ||^2
    objective += coeff * 0.5 * (Q.col(t) - q_des).squaredNorm();
  }
  Objective::Evaluate(Q, objective);
}

void JointPositionObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> Q,
                                      Eigen::Ref<Eigen::MatrixXd> Gradient) {
  for (size_t t = 0; t < Q.cols(); t++) {
    // J_{:,t} = (q_{t} - q_des)
    Gradient.col(t) += coeff * (Q.col(t) - q_des);
  }
}

void JointVelocityObjective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) {
  const size_t T = Q.cols();
  for (size_t t = 0; t < T - 1; t++) {
    // 0.5 * || q_{t+1} - q_{t} ||^2
    objective += coeff * 0.5 * (Q.col(t+1) - Q.col(t)).squaredNorm();
  }
  Objective::Evaluate(Q, objective);
}

void JointVelocityObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> Q,
                                      Eigen::Ref<Eigen::MatrixXd> Gradient) {
  const size_t T = Q.cols();
  Eigen::VectorXd dq_prev = Eigen::VectorXd::Zero(Q.rows());
  for (size_t t = 0; t < T - 1; t++) {
    Eigen::VectorXd dq_t = Q.col(t+1) - Q.col(t);

    // J_{:,t} = (q_{t} - q_{t-1}) - (q_{t+1} - q_{t})
    Gradient.col(t) += coeff * (dq_prev - dq_t);

    dq_prev = dq_t;
  }

  // J_{:,T} = q_{T} - q_{T-1}
  Gradient.col(T - 1) += coeff * dq_prev;
}

void JointVelocityObjective::Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                                     double hessian_coeff, Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {
  const size_t dof = Q.rows();
  const size_t T = Q.cols();

  size_t idx_hessian = 0;
  for (size_t t = 0; t < T; t++) {
    for (size_t i = 0; i < dof; i++) {
      double value = (t > 0 && t < T - 1) ? 2. : 1.;
      Hessian.coeffRef(t * dof + i, t * dof + i) += coeff * hessian_coeff * value;
    }
  }

  for (size_t i = 0; i < (T - 1) * dof; i++) {
    Hessian.coeffRef(i, i + dof) += coeff * hessian_coeff * -1.;
  }
}

void JointVelocityObjective::HessianStructure(Eigen::SparseMatrix<bool>& Hessian, size_t T) {
  const size_t dof = Hessian.cols() / T;
  for (size_t t = 0; t < T; t++) {
    for (size_t i = 0; i < dof; i++) {
      if (Hessian.coeff(t * dof + i, t * dof + i)) continue;
      Hessian.insert(t * dof + i, t * dof + i) = true;
    }
  }

  for (size_t i = 0; i < (T - 1) * dof; i++) {
    if (Hessian.coeff(i, i + dof)) continue;
    Hessian.insert(i, i + dof) = true;
  }
}

void JointAccelerationObjective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                                          double& objective) {
  const size_t T = Q.cols();
  assert(T >= 2);

  // 0.5 * || q_{2} - 2 * q_{1} + q_{1} ||^2
  objective += coeff * 0.5 * (Q.col(1) - Q.col(0)).squaredNorm();
  for (size_t t = 1; t < T - 1; t++) {
    // 0.5 * || q_{t+1} - 2 * q_{t} + q_{t-1} ||^2
    objective += coeff * 0.5 * (Q.col(t+1) - 2 * Q.col(t) + Q.col(t-1)).squaredNorm();
  }
  // 0.5 * || q_{T} - 2 * q_{T} + q_{T-1} ||^2
  objective += coeff * 0.5 * (Q.col(T-2) - Q.col(T-1)).squaredNorm();
  Objective::Evaluate(Q, objective);
}

void JointAccelerationObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> Q,
                                          Eigen::Ref<Eigen::MatrixXd> Gradient) {
  double T = Q.cols();
  assert(T >= 2);

  Eigen::VectorXd dq_prev = Q.col(1) - Q.col(0);
  Eigen::VectorXd dq_t = Q.col(2) - 2 * Q.col(1) + Q.col(0);

  // J_{:,t} = dq_{1} - dq_{0}
  Gradient.col(0) += coeff * (dq_t - dq_prev);
  for (size_t t = 1; t < T - 2; t++) {
    Eigen::VectorXd dq_next = Q.col(t+2) - 2 * Q.col(t+1) + Q.col(t);

    // J_{:,t} = dq_{t+1} - 2 * dq_{t} + dq_{t-1}
    Gradient.col(t) += coeff * (dq_next - 2 * dq_t + dq_prev);

    dq_prev = dq_t;
    dq_t = dq_next;
  }
  Eigen::VectorXd dq_next = Q.col(T-2) - Q.col(T-1);

  // J_{:,T-1} = dq_{T} - 2 * dq_{T-1} + dq_{T-2}
  Gradient.col(T-2) += coeff * (dq_next - 2 * dq_t + dq_prev);

  // J_{:,T} = dq_{T-1} - dq_{T}
  Gradient.col(T-1) += coeff * (dq_t - dq_next);
}

void LinearVelocityObjective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) {
  Eigen::Vector3d x_t = SpatialDyn::Position(ab_, Q.col(0));
  for (size_t t = 0; t < Q.cols() - 1; t++) {
    Eigen::Vector3d x_next = SpatialDyn::Position(ab_, Q.col(t+1));

    // 0.5 * || x_{t+1} - x_{t} ||^2
    objective += coeff * 0.5 * (x_next - x_t).squaredNorm();

    x_t = x_next;
  }
  Objective::Evaluate(Q, objective);
}

void LinearVelocityObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> Q,
                                       Eigen::Ref<Eigen::MatrixXd> Gradient) {
  Eigen::Vector3d dx_prev = Eigen::Vector3d::Zero();

  // ab.set_q(Q.col(0));
  Eigen::Vector3d x_t = SpatialDyn::Position(ab_, Q.col(0));
  for (size_t t = 0; t < Q.cols() - 1; t++) {
    Eigen::Matrix3Xd J_t = SpatialDyn::LinearJacobian(ab_, Q.col(t));

    // ab.set_q(Q.col(t+1));
    Eigen::Vector3d x_next = SpatialDyn::Position(ab_, Q.col(t+1));
    Eigen::Vector3d dx_t = x_next - x_t;

    // J_{:,t} = J_t^T * ((x_{t} - x_{t-1}) - (x_{t+1} - x_{t}))
    Gradient.col(t) += coeff * (J_t.transpose() * (dx_prev - dx_t));

    x_t = x_next;
    dx_prev = dx_t;
  }

  // J_{:,T} = J_T^T * (x_{T} - x_{T-1})
  Gradient.col(Q.cols()-1) += coeff * (SpatialDyn::LinearJacobian(ab_, Q.col(Q.cols()-1)).transpose() * dx_prev);
}

void AngularVelocityObjective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) {
  Eigen::Quaterniond quat_t = SpatialDyn::Orientation(ab_, Q.col(0));
  for (size_t t = 0; t < Q.cols() - 1; t++) {
    Eigen::Quaterniond quat_next = SpatialDyn::Orientation(ab_, Q.col(t+1));

    // 0.5 * || x_{t+1} - x_{t} ||^2
    objective += coeff * 0.5 * SpatialDyn::Opspace::OrientationError(quat_next, quat_t).squaredNorm();

    quat_t = quat_next;
  }
  Objective::Evaluate(Q, objective);
}

void AngularVelocityObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> Q,
                                        Eigen::Ref<Eigen::MatrixXd> Gradient) {
  Eigen::Vector3d w_prev = Eigen::Vector3d::Zero();

  Eigen::Quaterniond quat_t = SpatialDyn::Orientation(ab_, Q.col(0));
  for (size_t t = 0; t < Q.cols() - 1; t++) {
    Eigen::Matrix3Xd J_t = SpatialDyn::AngularJacobian(ab_, Q.col(t));

    // ab.set_q(Q.col(t+1));
    Eigen::Quaterniond quat_next = SpatialDyn::Orientation(ab_, Q.col(t+1));
    Eigen::Vector3d w_t = SpatialDyn::Opspace::OrientationError(quat_next, quat_t);

    // J_{:,t} = J_t^T * (x_{t-1} - x_{t-1}) - (x_{t+1} - x_{t}))
    Gradient.col(t) += coeff * (J_t.transpose() * (w_prev - w_t));

    quat_t = quat_next;
    w_prev = w_t;
  }

  // J_{:,T} = J_T^T * (x_{T} - x_{T-1})
  Gradient.col(Q.cols()-1) += coeff * (SpatialDyn::AngularJacobian(ab_, Q.col(Q.cols()-1)).transpose() * w_prev);
}

void LinearVelocityCartesianObjective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) {
  Eigen::Vector3d x_t = skeleton_.frames[0]->T_to_parent() * X.col(0).head<3>();
  for (size_t t = 0; t < X.cols() - 1; t++) {
    Eigen::Vector3d x_next = skeleton_.frames[t]->T_to_parent() * X.col(t+1).head<3>();

    // 0.5 * || x_{t+1} - x_{t} ||^2
    objective += coeff * 0.5 * (x_next - x_t).squaredNorm();

    x_t = x_next;
  }
  Objective::Evaluate(X, objective);
}

void LinearVelocityCartesianObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> X,
                                                Eigen::Ref<Eigen::MatrixXd> Gradient) {
  Eigen::Vector3d dx_prev = Eigen::Vector3d::Zero();

  Eigen::Vector3d x_t = skeleton_.frames[0]->T_to_parent() * X.col(0).head<3>();
  for (size_t t = 0; t < X.cols() - 1; t++) {
    const Eigen::Isometry3d& T_t = skeleton_.frames[t]->T_to_parent();
    const Eigen::Isometry3d& T_next = skeleton_.frames[t+1]->T_to_parent();
    Eigen::Vector3d x_next = T_next * X.col(t+1).head<3>();
    Eigen::Vector3d dx_t = x_next - x_t;

    // J_{:,t} = J_t^T * ((x_{t} - x_{t-1}) - (x_{t+1} - x_{t}))
    Gradient.col(t) += coeff * (T_t.linear().transpose() * (dx_t - dx_prev));

    x_t = x_next;
    dx_prev = dx_t;
  }

  // J_{:,T} = J_T^T * (x_{T} - x_{T-1})
  const Eigen::Isometry3d& T_last = skeleton_.frames.back()->T_to_parent();
  Gradient.col(X.cols()-1) -= coeff * (T_last.linear().transpose() * dx_prev);
}

void AngularVelocityCartesianObjective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) {
  const Eigen::Ref<const Eigen::Vector3d> dquat_t = X.col(0).tail<3>();
  Eigen::Quaterniond quat_t(skeleton_.frames[0]->T_to_parent().linear());
  if (dquat_t.squaredNorm() > 0) {
    quat_t = Eigen::AngleAxisd(dquat_t.norm(), dquat_t.normalized()) * quat_t;
  }
  for (size_t t = 0; t < X.cols() - 1; t++) {
    const Eigen::Ref<const Eigen::Vector3d> dquat_next = X.col(0).tail<3>();
    Eigen::Quaterniond quat_next(skeleton_.frames[t]->T_to_parent().linear());
    if (dquat_next.squaredNorm() > 0) {
      quat_next = Eigen::AngleAxisd(dquat_next.norm(), dquat_next.normalized()) * quat_next;
    }

    // 0.5 * || x_{t+1} - x_{t} ||^2
    objective += coeff * 0.5 * SpatialDyn::Opspace::OrientationError(quat_next, quat_t).squaredNorm();

    quat_t = quat_next;
  }
  Objective::Evaluate(X, objective);
}

void AngularVelocityCartesianObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> X,
                                                 Eigen::Ref<Eigen::MatrixXd> Gradient) {
  Eigen::Vector3d w_prev = Eigen::Vector3d::Zero();

  const Eigen::Ref<const Eigen::Vector3d> dquat_t = X.col(0).tail<3>();
  Eigen::Quaterniond quat_t(skeleton_.frames[0]->T_to_parent().linear());
  if (dquat_t.squaredNorm() > 0) {
    quat_t = Eigen::AngleAxisd(dquat_t.norm(), dquat_t.normalized()) * quat_t;
  }

  for (size_t t = 0; t < X.cols() - 1; t++) {
    const Eigen::Ref<const Eigen::Vector3d> dquat_next = X.col(0).tail<3>();
    const Eigen::Isometry3d& T_t = skeleton_.frames[t]->T_to_parent();
    const Eigen::Isometry3d& T_next = skeleton_.frames[t+1]->T_to_parent();
    Eigen::Quaterniond quat_next(T_next.linear());
    if (dquat_next.squaredNorm() > 0) {
      quat_next = Eigen::AngleAxisd(dquat_next.norm(), dquat_next.normalized()) * quat_next;
    }

    // ab.set_q(Q.col(t+1));
    Eigen::Vector3d w_t = SpatialDyn::Opspace::OrientationError(quat_next, quat_t);

    // J_{:,t} = J_t^T * (x_{t-1} - x_{t-1}) - (x_{t+1} - x_{t}))
    Gradient.col(t) += coeff * (T_t.linear().transpose() * (w_prev - w_t));

    quat_t = quat_next;
    w_prev = w_t;
  }

  // J_{:,T} = J_T^T * (x_{T} - x_{T-1})
  const Eigen::Isometry3d& T_last = skeleton_.frames.back()->T_to_parent();
  Gradient.col(X.cols()-1) += coeff * (T_last.linear().transpose() * w_prev);
}
#endif

}  // namespace LogicOpt

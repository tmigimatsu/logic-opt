/**
 * objectives.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 26, 2018
 * Authors: Toki Migimatsu
 */

#include "objectives.h"

#include <cassert>  // assert

namespace TrajOpt {

void JointPositionObjective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) {
  for (size_t t = 0; t < Q.cols(); t++) {
    // 0.5 * || q_{t} - q_des ||^2
    objective += coeff * 0.5 * (Q.col(t) - q_des).squaredNorm();
  }
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
                                     double hessian_coeff, Eigen::Ref<Eigen::VectorXd> Hessian) {
  const size_t dof = Q.rows();
  const size_t T = Q.cols();

  size_t idx_hessian = 0;
  for (size_t t = 0; t < T; t++) {
    Eigen::Map<Eigen::MatrixXd> H(&Hessian(idx_hessian), dof, dof);

    double value = (t > 0 && t < T - 1) ? 2. : 1.;
    H.diagonal().array() += coeff * hessian_coeff * value;

    idx_hessian += dof * dof;
  }
  Hessian.segment(idx_hessian, Hessian.size() - idx_hessian).array() += coeff * hessian_coeff * -1.;
}

// void JointVelocityObjective::HessianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
//                                             Eigen::Ref<Eigen::ArrayXi> idx_j) {
//   size_t idx_hessian = 0;

//   idx_i.segment(idx_hessian, dof_) = Eigen::ArrayXi::LinSpaced(dof_, 0, dof_ - 1);
//   idx_j.segment(idx_hessian, dof_) = Eigen::ArrayXi::LinSpaced(dof_, 0, dof_ - 1);
//   idx_hessian += dof_;

//   idx_i.segment(idx_hessian, dof_) = Eigen::ArrayXi::LinSpaced(dof_, dof_, 2 * dof_ - 1);
//   idx_j.segment(idx_hessian, dof_) = Eigen::ArrayXi::LinSpaced(dof_, 0, dof_ - 1);
//   idx_hessian += dof_;

//   for (size_t t = 1; t < T_ - 1; t++) {
//     idx_i.segment(idx_hessian, dof_) = Eigen::ArrayXi::LinSpaced(dof_, (t - 1) * dof_, t * dof_ - 1);
//     idx_j.segment(idx_hessian, dof_) = Eigen::ArrayXi::LinSpaced(dof_, t * dof_, (t + 1) * dof_ - 1);
//     idx_hessian += dof_;

//     idx_i.segment(idx_hessian, dof_) = Eigen::ArrayXi::LinSpaced(dof_, t * dof_, (t + 1) * dof_ - 1);
//     idx_j.segment(idx_hessian, dof_) = Eigen::ArrayXi::LinSpaced(dof_, t * dof_, (t + 1) * dof_ - 1);
//     idx_hessian += dof_;

//     idx_i.segment(idx_hessian, dof_) = Eigen::ArrayXi::LinSpaced(dof_, (t + 1) * dof_, (t + 2) * dof_ - 1);
//     idx_j.segment(idx_hessian, dof_) = Eigen::ArrayXi::LinSpaced(dof_, t * dof_, (t + 1) * dof_ - 1);
//     idx_hessian += dof_;
//   }

//   idx_i.segment(idx_hessian, dof_) = Eigen::ArrayXi::LinSpaced(dof_, (T_ - 2) * dof_, (T_ - 1) * dof_ - 1);
//   idx_j.segment(idx_hessian, dof_) = Eigen::ArrayXi::LinSpaced(dof_, (T_ - 1) * dof_, T_ * dof_ - 1);
//   idx_hessian += dof_;

//   idx_i.segment(idx_hessian, dof_) = Eigen::ArrayXi::LinSpaced(dof_, (T_ - 1) * dof_, T_ * dof_ - 1);
//   idx_j.segment(idx_hessian, dof_) = Eigen::ArrayXi::LinSpaced(dof_, (T_ - 1) * dof_, T_ * dof_ - 1);
//   idx_hessian += dof_;
// }

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
    objective += coeff * 0.5 * SpatialDyn::Opspace::OrientationError(quat_t, quat_next).squaredNorm();

    quat_t = quat_next;
  }
}

void AngularVelocityObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> Q,
                                        Eigen::Ref<Eigen::MatrixXd> Gradient) {
  Eigen::Vector3d w_prev = Eigen::Vector3d::Zero();

  Eigen::Quaterniond quat_t = SpatialDyn::Orientation(ab_, Q.col(0));
  for (size_t t = 0; t < Q.cols() - 1; t++) {
    Eigen::Matrix3Xd J_t = SpatialDyn::AngularJacobian(ab_, Q.col(t));

    // ab.set_q(Q.col(t+1));
    Eigen::Quaterniond quat_next = SpatialDyn::Orientation(ab_, Q.col(t+1));
    Eigen::Vector3d w_t = SpatialDyn::Opspace::OrientationError(quat_t, quat_next);

    // J_{:,t} = J_t^T * (x_{t-1} - x_{t-1}) - (x_{t+1} - x_{t}))
    Gradient.col(t) += coeff * (J_t.transpose() * (w_prev + w_t));

    quat_t = quat_next;
    w_prev = w_t;
  }

  // J_{:,T} = J_T^T * (x_{T} - x_{T-1})
  Gradient.col(Q.cols()-1) += coeff * (SpatialDyn::AngularJacobian(ab_, Q.col(Q.cols()-1)).transpose() * w_prev);
}

}  // namespace TrajOpt

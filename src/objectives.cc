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
namespace Objective {

double JointPosition(Eigen::Ref<const Eigen::MatrixXd> Q,
                     Eigen::Ref<const Eigen::VectorXd> q_des) {
  double obj = 0.;
  for (size_t t = 0; t < Q.cols(); t++) {
    // 0.5 * || q_{t} - q_des ||^2
    obj += 0.5 * (Q.col(t) - q_des).squaredNorm();
  }
  return obj;
}

void JointPositionGradient(Eigen::Ref<const Eigen::MatrixXd> Q,
                           Eigen::Ref<const Eigen::VectorXd> q_des,
                           Eigen::Ref<Eigen::MatrixXd> Grad,
                           double coeff) {

  for (size_t t = 0; t < Q.cols(); t++) {
    // J_{:,t} = (q_{t} - q_des)
    Grad.col(t) += coeff * (Q.col(t) - q_des);
  }
  // Grad.eval();
}

double JointVelocity(Eigen::Ref<const Eigen::MatrixXd> Q) {
  double obj = 0.;
  for (size_t t = 0; t < Q.cols() - 1; t++) {
    // 0.5 * || q_{t+1} - q_{t} ||^2
    obj += 0.5 * (Q.col(t+1) - Q.col(t)).squaredNorm();
  }
  return obj;
}

void JointVelocityGradient(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::MatrixXd> Grad,
                           double coeff) {

  Eigen::VectorXd dq_prev = Eigen::VectorXd::Zero(Q.rows());
  for (size_t t = 0; t < Q.cols() - 1; t++) {
    Eigen::VectorXd dq_t = Q.col(t+1) - Q.col(t);

    // J_{:,t} = (q_{t} - q_{t-1}) - (q_{t+1} - q_{t})
    Grad.col(t) += coeff * (dq_prev - dq_t);

    dq_prev = dq_t;
  }

  // J_{:,T} = q_{T} - q_{T-1}
  Grad.col(Q.cols()-1) += coeff * dq_prev;
}

double JointAcceleration(Eigen::Ref<const Eigen::MatrixXd> Q) {
  double T = Q.cols();
  assert(T >= 2);

  // 0.5 * || q_{2} - 2 * q_{1} + q_{1} ||^2
  double obj = 0;//0.5 * (Q.col(1) - Q.col(0)).squaredNorm();
  for (size_t t = 1; t < T - 1; t++) {
    // 0.5 * || q_{t+1} - 2 * q_{t} + q_{t-1} ||^2
    obj += 0.5 * (Q.col(t+1) - 2 * Q.col(t) + Q.col(t-1)).squaredNorm();
  }
  // 0.5 * || q_{T} - 2 * q_{T} + q_{T-1} ||^2
  // obj += 0.5 * (Q.col(T-2) - Q.col(T-1)).squaredNorm();
  return obj;
}

void JointAccelerationGradient(Eigen::Ref<const Eigen::MatrixXd> Q,
                               Eigen::Ref<Eigen::MatrixXd> Grad,
                               double coeff) {
  double T = Q.cols();
  assert(T >= 2);

  Eigen::VectorXd dq_prev = Eigen::VectorXd::Zero(Q.rows());//Q.col(1) - Q.col(0);
  Eigen::VectorXd dq_t = Q.col(2) - 2 * Q.col(1) + Q.col(0);

  // J_{:,t} = dq_{1} - dq_{0}
  Grad.col(0) += coeff * (dq_t - dq_prev);
  for (size_t t = 1; t < T - 2; t++) {
    Eigen::VectorXd dq_next = Q.col(t+2) - 2 * Q.col(t+1) + Q.col(t);

    // J_{:,t} = dq_{t+1} - 2 * dq_{t} + dq_{t-1}
    Grad.col(t) += coeff * (dq_next - 2 * dq_t + dq_prev);

    dq_prev = dq_t;
    dq_t = dq_next;
  }
  Eigen::VectorXd dq_next = Eigen::VectorXd::Zero(Q.rows());//Q.col(T-2) - Q.col(T-1);

  // J_{:,T-1} = dq_{T} - 2 * dq_{T-1} + dq_{T-2}
  Grad.col(T-2) += coeff * (dq_next - 2 * dq_t + dq_prev);

  // J_{:,T} = dq_{T-1} - dq_{T}
  Grad.col(T-1) += coeff * (dq_t - dq_next);
}

double LinearVelocity(const SpatialDyn::ArticulatedBody& ab,
                      Eigen::Ref<const Eigen::MatrixXd> Q) {
  double obj = 0.;
  Eigen::Vector3d x_t = SpatialDyn::Position(ab, Q.col(0));
  for (size_t t = 0; t < Q.cols() - 1; t++) {
    Eigen::Vector3d x_next = SpatialDyn::Position(ab, Q.col(t+1));

    // 0.5 * || x_{t+1} - x_{t} ||^2
    obj += 0.5 * (x_next - x_t).squaredNorm();

    x_t = x_next;
  }
  return obj;
}

void LinearVelocityGradient(SpatialDyn::ArticulatedBody& ab,
                            Eigen::Ref<const Eigen::MatrixXd> Q,
                            Eigen::Ref<Eigen::MatrixXd> Grad,
                            double coeff) {
  Eigen::Vector3d dx_prev = Eigen::Vector3d::Zero();

  ab.set_q(Q.col(0));
  Eigen::Vector3d x_t = SpatialDyn::Position(ab);
  for (size_t t = 0; t < Q.cols() - 1; t++) {
    Eigen::Matrix3Xd J_t = SpatialDyn::LinearJacobian(ab);

    ab.set_q(Q.col(t+1));
    Eigen::Vector3d x_next = SpatialDyn::Position(ab);
    Eigen::Vector3d dx_t = x_next - x_t;

    // J_{:,t} = J_t^T * ((x_{t} - x_{t-1}) - (x_{t+1} - x_{t}))
    Grad.col(t) += coeff * (J_t.transpose() * (dx_prev - dx_t));

    x_t = x_next;
    dx_prev = dx_t;
  }

  // J_{:,T} = J_T^T * (x_{T} - x_{T-1})
  Grad.col(Q.cols()-1) += coeff * (SpatialDyn::LinearJacobian(ab).transpose() * dx_prev);
}

double AngularVelocity(const SpatialDyn::ArticulatedBody& ab,
                       Eigen::Ref<const Eigen::MatrixXd> Q) {
  double obj = 0.;
  Eigen::Quaterniond quat_t = SpatialDyn::Orientation(ab, Q.col(0));
  for (size_t t = 0; t < Q.cols() - 1; t++) {
    Eigen::Quaterniond quat_next = SpatialDyn::Orientation(ab, Q.col(t+1));

    // 0.5 * || x_{t+1} - x_{t} ||^2
    obj += 0.5 * SpatialDyn::Opspace::OrientationError(quat_t, quat_next).squaredNorm();

    quat_t = quat_next;
  }
  return obj;
}

void AngularVelocityGradient(SpatialDyn::ArticulatedBody& ab,
                             Eigen::Ref<const Eigen::MatrixXd> Q,
                             Eigen::Ref<Eigen::MatrixXd> Grad,
                             double coeff) {
  Eigen::Vector3d w_prev = Eigen::Vector3d::Zero();

  ab.set_q(Q.col(0));
  Eigen::Quaterniond quat_t = SpatialDyn::Orientation(ab);
  for (size_t t = 0; t < Q.cols() - 1; t++) {
    Eigen::Matrix3Xd J_t = SpatialDyn::AngularJacobian(ab);

    ab.set_q(Q.col(t+1));
    Eigen::Quaterniond quat_next = SpatialDyn::Orientation(ab);
    Eigen::Vector3d w_t = SpatialDyn::Opspace::OrientationError(quat_t, quat_next);

    // J_{:,t} = J_t^T * (x_{t-1} - x_{t-1}) - (x_{t+1} - x_{t}))
    Grad.col(t) += coeff * (J_t.transpose() * (w_prev + w_t));

    quat_t = quat_next;
    w_prev = w_t;
  }

  // J_{:,T} = J_T^T * (x_{T} - x_{T-1})
  Grad.col(Q.cols()-1) += coeff * (SpatialDyn::AngularJacobian(ab).transpose() * w_prev);
}

}  // namespace Objective
}  // namespace TrajOpt

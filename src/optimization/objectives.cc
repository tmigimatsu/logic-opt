/**
 * objectives.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 26, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/optimization/objectives.h"

#include <algorithm>  // std::max
#include <cmath>      // std::acos, std::sqrt
#include <cassert>    // assert

namespace LogicOpt {

void Objective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) {
  if (!log.is_open()) return;
  log << objective << std::endl;
}

void MinNormObjective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) {
  Eigen::Map<const Eigen::VectorXd> x(X.data(), X.size());
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
  Eigen::Map<Eigen::VectorXd> gradient(Gradient.data(), Gradient.size());

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

  // J_{:,T} = J_T^T * ((x_{T} - x_{T-1})
  Eigen::Matrix3Xd J_T = world_.PositionJacobian(name_ee_, X, X.cols() - 1);
  gradient += coeff_ * J_T.transpose() * dx_prev;
}

void AngularVelocityObjective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) {
  Eigen::Quaterniond quat_t(world_.Orientation(name_ee_, world_.kWorldFrame, X, 0));
  for (size_t t = 0; t < X.cols() - 1; t++) {
    Eigen::Quaterniond quat_next = spatial_dyn::opspace::NearQuaternion(
                                       world_.Orientation(name_ee_, world_.kWorldFrame, X, t+1), quat_t);

    // 0.5 * || 2 log(R_{t}^{-1} R_{t+1}) ||^2
    objective += coeff_ * 0.5 * spatial_dyn::opspace::OrientationError(quat_next, quat_t).squaredNorm();

    quat_t = quat_next;
  }
  Objective::Evaluate(X, objective);
}

void AngularVelocityObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> X,
                                        Eigen::Ref<Eigen::MatrixXd> Gradient) {
  Eigen::Vector3d w_prev = Eigen::Vector3d::Zero();
  Eigen::Map<Eigen::VectorXd> gradient(Gradient.data(), Gradient.size());

  for (int idx_var = 0; idx_var < X.cols(); idx_var++) {
    auto x_r = X.col(idx_var).tail<3>();
    std::array<Eigen::Matrix3d, 3> dRs = world_.OrientationAngleAxisJacobians(x_r);
    for (size_t t = std::max(0, idx_var - 1); t < X.cols() - 1; t++) {
      double trace = 0.;
      Eigen::Matrix3d dTrace = world_.OrientationTraceJacobian(name_ee_, idx_var, X, t, t+1, &trace);
      double factor = (trace - 1.) / 2.;
      if (factor != 1.) {
        factor = -std::acos(factor) / (2. * std::sqrt(1 - factor * factor));
        for (size_t i = 0; i < 3; i++) {
          Gradient(3 + i, idx_var) += coeff_ * factor * (dTrace.transpose() * dRs[i]).diagonal().sum();
        }
      }
    }
  }
}

}  // namespace LogicOpt

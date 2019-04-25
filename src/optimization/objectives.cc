/**
 * objectives.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 26, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/optimization/objectives.h"

#include <algorithm>  // std::max
#include <cmath>      // std::acos, std::sqrt
#include <cassert>    // assert

namespace logic_opt {

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

Eigen::Matrix3Xd PositionJacobian(const logic_opt::World3& world,
                                  const std::string& name_frame,
                                  Eigen::Ref<const Eigen::MatrixXd> X,
                                  size_t t) {
  Eigen::Matrix3Xd J_t = Eigen::Matrix3Xd::Zero(3, X.size());

  auto chain = world.frames(t).ancestors(name_frame);
  for (auto it = chain.begin(); it != chain.end(); ++it) {
    if (it->first == world.kWorldFrame) break;
    const Frame& frame = it->second;
    if (!frame.is_variable()) continue;

    auto J_pos = J_t.block<3,3>(0, 6 * frame.idx_var());
    const std::string name_parent = *world.frames(t).parent(frame.name());
    J_pos = world.Orientation(name_parent, world.kWorldFrame, X, t);

    if (frame.name() == name_frame) continue;
    auto J_ori = J_t.block<3,3>(0, 6 * frame.idx_var() + 3);
    Eigen::Vector3d p = world.Position(name_frame, frame.name(), X, t);
    auto x_r = X.col(frame.idx_var()).tail<3>();
    const Eigen::AngleAxisd aa(x_r.norm(), x_r.normalized());
    J_ori = J_pos * spatial_dyn::opspace::ExpCoordsJacobian(aa, p);
  }
  return J_t;
}

void LinearVelocityObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> X,
                                       Eigen::Ref<Eigen::MatrixXd> Gradient) {
  Eigen::Vector3d dx_prev = Eigen::Vector3d::Zero();
  Eigen::Map<Eigen::VectorXd> gradient(Gradient.data(), Gradient.size());

  Eigen::Vector3d x_t = world_.Position(name_ee_, world_.kWorldFrame, X, 0);
  for (size_t t = 0; t < X.cols() - 1; t++) {
    Eigen::Matrix3Xd J_t = PositionJacobian(world_, name_ee_, X, t);

    Eigen::Vector3d x_next = world_.Position(name_ee_, world_.kWorldFrame, X, t+1);
    Eigen::Vector3d dx_t = x_next - x_t;

    // J_{:,t} = J_t^T * ((x_{t} - x_{t-1}) - (x_{t+1} - x_{t}))
    gradient += coeff_ * (J_t.transpose() * (dx_prev - dx_t));

    x_t = x_next;
    dx_prev = dx_t;
  }

  // J_{:,T} = J_T^T * ((x_{T} - x_{T-1})
  Eigen::Matrix3Xd J_T = PositionJacobian(world_, name_ee_, X, X.cols() - 1);
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

std::array<std::optional<Eigen::Matrix3d>, 3> RotationChain(const logic_opt::World3& world,
                                                            const std::string& name_frame,
                                                            size_t idx_var,
                                                            Eigen::Ref<const Eigen::MatrixXd> X,
                                                            size_t t1, size_t t2) {
  // Matrix tuple (A, X_inv, B, X, C)
  std::array<std::optional<Eigen::Matrix3d>, 3> Rs;

  // Get ancestor chains from ee to world at t1 and t2
  std::vector<const std::pair<const std::string, Frame>*> chain1;
  std::vector<const std::pair<const std::string, Frame>*> chain2;
  auto ancestors1 = world.frames(t1).ancestors(name_frame);
  auto ancestors2 = world.frames(t2).ancestors(name_frame);
  for (auto it = ancestors1.begin(); it != ancestors1.end(); ++it) {
    chain1.push_back(&*it);
  }
  for (auto it = ancestors2.begin(); it != ancestors2.end(); ++it) {
    chain2.push_back(&*it);
  }

  // Find closest common ancestor between ee frames at t1 and t2
  int idx_end1 = chain1.size() - 1;
  int idx_end2 = chain2.size() - 1;
  while (idx_end1 >= 0 && idx_end2 >= 0 && chain1[idx_end1]->second == chain2[idx_end2]->second) {
    --idx_end1;
    --idx_end2;
  }

  // Construct A?
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  for (size_t i = 0; i <= idx_end1; i++) {
    const Frame& frame = chain1[i]->second;
    if (frame.idx_var() == idx_var && frame.is_variable()) {
      Rs[0] = R.transpose();
      R.setIdentity();
      continue;
    }
    R = world.T_to_parent(frame.name(), X, t1).linear() * R;
  }

  // Construct B, C?
  Rs[1] = R.transpose();
  size_t idx = 1;
  R.setIdentity();
  for (int i = idx_end2; i >= 0; i--) {
    const Frame& frame = chain2[i]->second;
    if (frame.idx_var() == idx_var && frame.is_variable()) {
      Rs[1] = *Rs[1] * R;
      R.setIdentity();
      idx = 2;
      continue;
    }
    R = R * world.T_to_parent(frame.name(), X, t2).linear();
  }

  // Finish B or C
  Rs[idx] = (idx == 2) ? R : *Rs[idx] * R;

  return Rs;
}

void ComputeOrientationTrace(const Eigen::Matrix3d& R,
                             const std::array<std::optional<Eigen::Matrix3d>, 3>& Rs,
                             Eigen::Matrix3d* Phi, Eigen::Matrix3d* dTrPhi_dR) {

  if (Rs[0] && !Rs[2]) {
    const Eigen::Matrix3d A     = *Rs[0];
    const Eigen::Matrix3d B     = *Rs[1];

    const Eigen::Matrix3d A_Rinv = A * R.transpose();
    *Phi = A_Rinv * B;
    *dTrPhi_dR = -(R.transpose() * B * A_Rinv).transpose();
  } else if (!Rs[0] && Rs[2]) {
    const Eigen::Matrix3d B  = *Rs[1];
    const Eigen::Matrix3d C  = *Rs[2];

    *Phi = B * R * C;
    *dTrPhi_dR = (C * B).transpose();
  } else if (Rs[0] && Rs[2]) {
    const Eigen::Matrix3d A     = *Rs[0];
    const Eigen::Matrix3d B     = *Rs[1];
    const Eigen::Matrix3d C     = *Rs[2];

    const Eigen::Matrix3d A_Rinv = A * R.transpose();
    const Eigen::Matrix3d B_R_C = B * R * C;
    *Phi = A_Rinv * B_R_C;
    *dTrPhi_dR = (C * A_Rinv * B - R.transpose() * B_R_C * A_Rinv).transpose();
  }
  // *dTrPhi_dR += -dTrPhi_dR->transpose() + Eigen::Matrix3d(dTrPhi_dR->diagonal().asDiagonal());
}

void AngularVelocityObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> X,
                                        Eigen::Ref<Eigen::MatrixXd> Gradient) {

  for (int idx_var = 0; idx_var < X.cols(); idx_var++) {
    const auto x_r = X.col(idx_var).tail<3>();
    const Eigen::AngleAxisd aa(x_r.norm(), x_r.normalized());
    const Eigen::Matrix3d R = spatial_dyn::opspace::Exp(x_r);
    const Eigen::Matrix<double,9,3> dR_dw = spatial_dyn::opspace::ExpCoordsJacobian(aa);

    for (size_t t = std::max(0, idx_var - 1); t < X.cols() - 1; t++) {
      const std::array<std::optional<Eigen::Matrix3d>, 3> Rs = RotationChain(world_, name_ee_, idx_var, X, t, t+1);
      if (!Rs[0] && !Rs[2]) continue;

      Eigen::Matrix3d Phi;
      Eigen::Matrix3d dTrPhi_dR;
      ComputeOrientationTrace(R, Rs, &Phi, &dTrPhi_dR);

      const Eigen::Vector3d g = spatial_dyn::opspace::NormLogExpCoordsGradient(Phi, dR_dw, dTrPhi_dR);
      Gradient.block<3,1>(3, idx_var) += coeff_ * g;
    }
  }
}

}  // namespace logic_opt

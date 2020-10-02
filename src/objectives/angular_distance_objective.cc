/**
 * angular_distance_objective.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 26, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/objectives/angular_distance_objective.h"

#include <iostream>  // std::cout
#include <sstream>   // std::stringstream

#include "logic_opt/objectives/linear_distance_objective.h"

namespace logic_opt {

// Quaternion representation
std::array<std::optional<Eigen::Quaterniond>, 3> RotationChain(
    const World& world, const std::string& name_frame, size_t idx_var,
    Eigen::Ref<const Eigen::MatrixXd> X, size_t t1, size_t t2) {
  // Matrix tuple (A, X_inv, B, X, C)
  std::array<std::optional<Eigen::Quaterniond>, 3> Qs;

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
  // Iterate starting from world frame
  while (idx_end1 >= 0 && idx_end2 >= 0 &&
         chain1[idx_end1]->second == chain2[idx_end2]->second) {
    --idx_end1;
    --idx_end2;
  }

  // Construct A?
  // A: rotation from variable frame to ee frame at t1
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

  // Walk from ee at t1 to variable frame
  for (size_t i = 0; i <= idx_end1; i++) {
    const Frame& frame = chain1[i]->second;
    if (frame.idx_var() == idx_var && frame.is_variable()) {
      Qs[0] = q;
      q.setIdentity();
      continue;
    }
    q = q * world.T_to_parent(frame.name(), X, t1).rotation().conjugate();
  }

  // Construct B, C?
  // B: rotation from variable frame to next variable frame
  // C: rotation from variable frame to ee at t2
  Qs[1] = q;
  size_t idx = 1;
  q.setIdentity();
  for (int i = idx_end2; i >= 0; i--) {
    const Frame& frame = chain2[i]->second;
    if (frame.idx_var() == idx_var && frame.is_variable()) {
      Qs[1] = *Qs[1] * q;
      q.setIdentity();
      idx = 2;
      continue;
    }
    q = q * world.T_to_parent(frame.name(), X, t2).rotation();
  }

  // Finish B or C
  Qs[idx] = (idx == 2) ? q : *Qs[idx] * q;
  return Qs;
}

std::string PrintRotationChain(
    const std::array<std::optional<Eigen::Quaterniond>, 3> Qs) {
  std::stringstream ss;
  for (size_t i = 0; i < Qs.size(); i++) {
    ss << "Qs[" << i << "]: ";
    if (Qs[i]) {
      ss << Qs[i]->coeffs().transpose();
    }
    ss << std::endl;
  }
  return ss.str();
}

bool TestRotationChain(
    const std::array<std::optional<Eigen::Quaterniond>, 3> Qs,
    const Eigen::Quaterniond& q, const Eigen::Quaterniond& quat_ref) {
  Eigen::Quaterniond quat;
  if (Qs[0] && Qs[1] && Qs[2]) {
    const Eigen::Quaterniond& a = *Qs[0];
    const Eigen::Quaterniond& b = *Qs[1];
    const Eigen::Quaterniond& c = *Qs[2];
    quat = a * q.conjugate() * b * q * c;
  } else if (Qs[0] && Qs[1]) {
    const Eigen::Quaterniond& a = *Qs[0];
    const Eigen::Quaterniond& b = *Qs[1];
    quat = a * q.conjugate() * b;
  } else if (Qs[1] && Qs[2]) {
    const Eigen::Quaterniond& b = *Qs[1];
    const Eigen::Quaterniond& c = *Qs[2];
    quat = b * q * c;
  } else {
    std::stringstream ss;
    ss << "Invalid Qs: " << PrintRotationChain(Qs);
    throw std::runtime_error(ss.str());
  }

  const double is_equal = quat.isApprox(quat_ref);
  if (!is_equal) {
    std::cout << "ERROR: " << std::endl
              << "quat: " << quat.coeffs().transpose() << std::endl
              << " does not match " << std::endl
              << "quat_ref: " << quat_ref.coeffs().transpose() << std::endl
              << std::endl
              << PrintRotationChain(Qs) << "q: " << q.coeffs().transpose()
              << std::endl;
  }
  return is_equal;
}

/**
 * Computes the square quaternion norm of the given quaternion,
 * representing the cosine of the quaternion's angle.
 *
 * This is equivalent to q_conj.dot(q) = cos(theta).
 */
double SquaredNorm(const Eigen::Quaterniond& q) {
  return q.w() * q.w() - q.vec().dot(q.vec());
}

void AngularDistanceObjective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                        double& objective) {
  Eigen::Quaterniond quat_t =
      world_.Orientation(name_ee_, world_.kWorldFrame, X, 0);

  for (size_t t = 0; t < X.cols() - 1; t++) {
    Eigen::Quaterniond quat_next =
        world_.Orientation(name_ee_, world_.kWorldFrame, X, t + 1);
    const Eigen::Quaterniond quat_err = quat_t.conjugate() * quat_next;

    // The general formula for the angle of a quaternion is:
    //
    //     theta = arccos(q_conj.dot(q))
    //
    // The arccos breaks down if q is near identity or sometimes if ||q|| > 1.
    // To get around this, we approximate arccos with a line:
    //
    //     theta ~= pi/2 (1 - q_conj.dot(q))
    const double theta = M_PI / 2. * (1. - SquaredNorm(quat_err));

    objective += coeff_ * 0.5 * theta * theta;

    quat_t = quat_next;
  }
  Objective::Evaluate(X, objective);
}

Eigen::Matrix4Xd AngularDistanceObjective::OrientationJacobian(
    const World& world, const std::string& name_frame,
    Eigen::Ref<const Eigen::MatrixXd> X, size_t t) {
  Eigen::Matrix4Xd J_t = Eigen::Matrix4Xd::Zero(4, X.size());

  // Iterate over all frames that might affect the rotation distance between t
  // and t+1.
  for (size_t idx_var = 0; idx_var <= t + 1; idx_var++) {
    const std::array<std::optional<Eigen::Quaterniond>, 3> Qs =
        RotationChain(world, name_frame, idx_var, X, t, t + 1);

    // Ignore if frame variable doesn't affect orientation distance between
    // timesteps.
    if (!Qs[0] && !Qs[2]) continue;

    // Since the quaternion variables are not necessarily unit norm, we compute
    // the orientation using the normalized quaternion `q_hat`, and then add
    // `dqhat_dq` in the chain rule.
    auto J_quat = J_t.block<4, 4>(0, world.kDof * idx_var + 3);
    const Eigen::Map<const Eigen::Quaterniond> q = world.Quaternion(X, idx_var);
    if (q.norm() == 0.) {
      std::cout << "X: " << X << std::endl;
      throw std::domain_error(
          "AngularDistanceObjective::OrientationJacobian(): Cannot normalize "
          "zero norm quaternion.");
    }
    const Eigen::Quaterniond q_hat = q.normalized();

    J_quat = QuaternionJacobian(Qs, q_hat) * dqhat_dq(q);
  }
  return J_t;
}

inline Eigen::MatrixXd AngularDistanceObjectiveGradient(
    Eigen::Ref<const Eigen::MatrixXd> X, const World& world,
    const std::string& name_ee) {
  Eigen::MatrixXd Gradient =
      Eigen::MatrixXd::Zero(world.kDof, world.num_timesteps());
  Eigen::Map<Eigen::VectorXd> gradient(Gradient.data(), Gradient.size());

  // Iterate over all timesteps except last one
  Eigen::Quaterniond quat_t =
      world.Orientation(name_ee, world.kWorldFrame, X, 0);
  for (size_t t = 0; t < X.cols() - 1; t++) {
    // Compute Jacobian wrt all frame variables from 0 to t + 1. The resulting
    // matrix is [4 x 7T].
    const Eigen::Matrix4Xd J_t =
        AngularDistanceObjective::OrientationJacobian(world, name_ee, X, t);

    const Eigen::Quaterniond quat_next =
        world.Orientation(name_ee, world.kWorldFrame, X, t + 1);
    const Eigen::Quaterniond quat_err = quat_t.conjugate() * quat_next;

    // The general formula for the angle of a quaternion is:
    //
    //     theta = arccos(q_conj.dot(q))
    //
    // The arccos breaks down if q is near identity or sometimes if ||q|| > 1.
    // To get around this, we approximate arccos with a line:
    //
    //     theta ~= pi/2 (1 - q_conj.dot(q))
    const double theta = M_PI / 2. * (1. - SquaredNorm(quat_err));
    const Eigen::VectorXd dtheta_dq =
        -M_PI * J_t.transpose() * quat_err.conjugate().coeffs();

    // Objective is 0.5 * theta^2.
    gradient += theta * dtheta_dq;

    quat_t = quat_next;
  }

  return Gradient;
}

// Quaternion representation
void AngularDistanceObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> X,
                                        Eigen::Ref<Eigen::MatrixXd> Gradient) {
  Gradient += coeff_ * AngularDistanceObjectiveGradient(X, world_, name_ee_);
  Objective::Gradient(X, Gradient);

  // for (int idx_var = 0; idx_var < X.cols(); idx_var++) {
  //   const Eigen::Map<const Eigen::Quaterniond> q =
  //       world_.Quaternion(X, idx_var);
  //   if (q.norm() == 0.) {
  //     throw std::domain_error(
  //         "AngularDistanceObjective::Gradient(): Cannot normalize zero norm "
  //         "quaternion.");
  //   }
  //   const Eigen::Quaterniond q_hat = q.normalized();

  //   for (size_t t = std::max(0, idx_var - 1); t < X.cols() - 1; t++) {
  //     const std::array<std::optional<Eigen::Quaterniond>, 3> Qs =
  //         RotationChain(world_, name_ee_, idx_var, X, t, t + 1);

  //     if (!Qs[0] && !Qs[2]) continue;

  //     const Eigen::Quaterniond quat_t =
  //         world_.Orientation(name_ee_, world_.kWorldFrame, X, t);
  //     const Eigen::Quaterniond quat_next =
  //         world_.Orientation(name_ee_, world_.kWorldFrame, X, t + 1);
  //     const Eigen::Quaterniond quat_err = quat_t.conjugate() * quat_next;

  //     // Verify that quat_err = Qs[0] * q_hat.conjugate() * Qs[1] * q_hat *
  //     Qs[2]
  //     // if (!TestRotationChain(Qs, q_hat, quat_err)) {
  //     //   throw std::runtime_error(
  //     //       "AngularDistanceObjective::Gradient(): Quaternions don't
  //     match.");
  //     // }

  //     // The general formula for the angle of a quaternion is:
  //     //
  //     //     theta = arccos(q_conj.dot(q))
  //     //
  //     // The arccos breaks down if q is near identity or sometimes if ||q||
  //     > 1.
  //     // To get around this, we approximate arccos with a line:
  //     //
  //     //     theta ~= pi/2 (1 - q_conj.dot(q))
  //     const double theta = M_PI / 2. * (1. - SquaredNorm(quat_err));

  //     // dtheta_dq [1 x 4] = -pi * wq * dwq_dq [1 x 4] + pi * vq' * dvq_dq [3
  //     x 4]
  //     // dtheta_dq [1 x 4] = -pi * q_conj' * dq_dq [4 x 4]
  //     // dtheta_dq [4 x 1] = -pi * dq_dq' [4 x 4] * q_conj
  //     const Eigen::Vector4d dtheta_dq = -M_PI *
  //                                       dqhat_dq(q).transpose() *
  //                                       QuaternionJacobian(Qs,
  //                                       q_hat).transpose() *
  //                                       quat_err.conjugate().coeffs();

  //     // d/dq 0.5 * theta^2 = theta * dtheta_dq
  //     world_.Quaternion(Gradient, idx_var).coeffs() +=
  //         coeff_ * theta * dtheta_dq;
  //   }
  // }
  // if (!Gradient.isApprox(coeff_ * AngularDistanceObjectiveGradient(X, world_,
  // name_ee_))) {
  //   std::cout << Gradient << std::endl;
  //   std::cout << coeff_ * AngularDistanceObjectiveGradient(X, world_,
  //   name_ee_) << std::endl; throw std::runtime_error("");
  // } else {
  //   std::cout << (Gradient - coeff_ * AngularDistanceObjectiveGradient(X,
  //   world_, name_ee_)).array().abs().sum() << std::endl;
  // }

  // Objective::Gradient(X, Gradient);
}

Eigen::Quaterniond AngularDistanceObjective::aqinvb(
    const Eigen::Quaterniond& q, const Eigen::Quaterniond& a,
    const Eigen::Quaterniond& b) {
  const double wq = q.w();
  const double wa = a.w();
  const double wb = b.w();
  const Eigen::Ref<const Eigen::Vector3d> Vq = q.vec();
  const Eigen::Ref<const Eigen::Vector3d> Va = a.vec();
  const Eigen::Ref<const Eigen::Vector3d> Vb = b.vec();

  Eigen::Quaterniond aqb;
  aqb.w() = wa * wb * wq + wa * Vb.dot(Vq) + wb * Va.dot(Vq) - wq * Va.dot(Vb) -
            Va.transpose() * Vb.cross(Vq);

  aqb.vec() = -wa * wb * Vq + wa * wq * Vb + wb * wq * Va - Va.dot(Vb) * Vq +
              Va.dot(Vq) * Vb + Vb.dot(Vq) * Va + wa * Vb.cross(Vq) -
              wb * Va.cross(Vq) + wq * Va.cross(Vb);
  return aqb;
}

Eigen::Quaterniond AngularDistanceObjective::bqc(const Eigen::Quaterniond& q,
                                                 const Eigen::Quaterniond& b,
                                                 const Eigen::Quaterniond& c) {
  const double wq = q.w();
  const double wb = b.w();
  const double wc = c.w();
  const Eigen::Ref<const Eigen::Vector3d> Vq = q.vec();
  const Eigen::Ref<const Eigen::Vector3d> Vb = b.vec();
  const Eigen::Ref<const Eigen::Vector3d> Vc = c.vec();

  Eigen::Quaterniond bqc;
  bqc.w() = wb * wc * wq - wb * Vc.dot(Vq) - wc * Vb.dot(Vq) - wq * Vb.dot(Vc) +
            Vb.transpose() * Vc.cross(Vq);

  bqc.vec() = wb * wc * Vq + wb * wq * Vc + wc * wq * Vb + Vb.dot(Vc) * Vq -
              Vb.dot(Vq) * Vc - Vc.dot(Vq) * Vb - wb * Vc.cross(Vq) +
              wc * Vb.cross(Vq) + wq * Vb.cross(Vc);
  return bqc;
}

Eigen::Quaterniond AngularDistanceObjective::aqinvbqc(
    const Eigen::Quaterniond& q, const Eigen::Quaterniond& a,
    const Eigen::Quaterniond& b, const Eigen::Quaterniond& c) {
  const double wq = q.w();
  const double wa = a.w();
  const double wb = b.w();
  const double wc = c.w();
  const Eigen::Ref<const Eigen::Vector3d> Vq = q.vec();
  const Eigen::Ref<const Eigen::Vector3d> Va = a.vec();
  const Eigen::Ref<const Eigen::Vector3d> Vb = b.vec();
  const Eigen::Ref<const Eigen::Vector3d> Vc = c.vec();

  Eigen::Quaterniond aqbqc;
  aqbqc.w() = wa * wb * wc * wq * wq + wa * wb * wc * Vq.dot(Vq) -
              wa * wq * wq * Vb.dot(Vc) - wb * wq * wq * Va.dot(Vc) -
              wc * wq * wq * Va.dot(Vb) + wa * Vb.dot(Vc) * Vq.dot(Vq) -
              2 * wa * Vb.dot(Vq) * Vc.dot(Vq) - wb * Va.dot(Vc) * Vq.dot(Vq) +
              wc * Va.dot(Vb) * Vq.dot(Vq) - 2 * wc * Va.dot(Vq) * Vb.dot(Vq) +
              2 * wq * Va.dot(Vb) * Vc.dot(Vq) -
              2 * wq * Va.dot(Vq) * Vb.dot(Vc) +
              2 * wa * wq * Vb.transpose() * Vc.cross(Vq) -
              2 * wc * wq * Va.transpose() * Vb.cross(Vq) -
              wq * wq * Va.transpose() * Vb.cross(Vc) +
              2 * Vb.dot(Vq) * Va.transpose() * Vc.cross(Vq) +
              Vq.dot(Vq) * Va.transpose() * Vb.cross(Vc);

  aqbqc.vec() =
      wa * wb * wq * wq * Vc + wa * wc * wq * wq * Vb + wb * wc * wq * wq * Va +
      wa * wb * Vq.dot(Vq) * Vc + 2 * wa * wc * Vb.dot(Vq) * Vq -
      wa * wc * Vq.dot(Vq) * Vb + 2 * wa * wq * Vb.dot(Vc) * Vq -
      2 * wa * wq * Vc.dot(Vq) * Vb + wb * wc * Vq.dot(Vq) * Va -
      2 * wc * wq * Va.dot(Vb) * Vq + 2 * wc * wq * Va.dot(Vq) * Vb -
      wq * wq * Va.dot(Vb) * Vc + wq * wq * Va.dot(Vc) * Vb -
      wq * wq * Vb.dot(Vc) * Va + Va.dot(Vb) * Vq.dot(Vq) * Vc +
      2 * Va.dot(Vc) * Vb.dot(Vq) * Vq - Va.dot(Vc) * Vq.dot(Vq) * Vb -
      2 * Va.dot(Vq) * Vb.dot(Vq) * Vc + Vb.dot(Vc) * Vq.dot(Vq) * Va -
      2 * Vb.dot(Vq) * Vc.dot(Vq) * Va + 2 * wa * wc * wq * Vb.cross(Vq) +
      wa * wq * wq * Vb.cross(Vc) + wb * wq * wq * Va.cross(Vc) +
      wc * wq * wq * Va.cross(Vb) - 2 * wa * Vb.dot(Vq) * Vc.cross(Vq) -
      wa * Vq.dot(Vq) * Vb.cross(Vc) + wb * Vq.dot(Vq) * Va.cross(Vc) +
      2 * wc * Vb.dot(Vq) * Va.cross(Vq) - wc * Vq.dot(Vq) * Va.cross(Vb) +
      2 * wq * Va.dot(Vb) * Vc.cross(Vq) - 2 * wq * Va.dot(Vc) * Vb.cross(Vq) +
      2 * wq * Va.dot(Vq) * Vb.cross(Vc) + 2 * wq * Vb.dot(Vc) * Va.cross(Vq) -
      2 * wq * Vc.dot(Vq) * Va.cross(Vb);
  return aqbqc;
}

// dq_dq (4 x 4) = [dv_dq (3 x 4); dw_dq (1 x 4)]
Eigen::Matrix4d AngularDistanceObjective::QuaternionJacobian(
    const std::array<std::optional<Eigen::Quaterniond>, 3>& Qs,
    const Eigen::Quaterniond& q) {
  Eigen::Matrix4d dq_dq;
  if (Qs[0] && Qs[1] && Qs[2]) {
    const Eigen::Quaterniond& a = *Qs[0];
    const Eigen::Quaterniond& b = *Qs[1];
    const Eigen::Quaterniond& c = *Qs[2];
    dq_dq << daqinvbqcv_dq(q, a, b, c), daqinvbqcw_dq(q, a, b, c);
  } else if (Qs[0] && Qs[1]) {
    const Eigen::Quaterniond& a = *Qs[0];
    const Eigen::Quaterniond& b = *Qs[1];
    dq_dq << daqinvbv_dq(q, a, b), daqinvbw_dq(q, a, b);
  } else if (Qs[1] && Qs[2]) {
    const Eigen::Quaterniond& b = *Qs[1];
    const Eigen::Quaterniond& c = *Qs[2];
    dq_dq << dbqcv_dq(q, b, c), dbqcw_dq(q, b, c);
  } else {
    throw std::runtime_error("QuaternionJacobian(): Invalid Qs.");
  }

  return dq_dq;
}

Eigen::Matrix<double, 1, 4> AngularDistanceObjective::daqinvbw_dq(
    const Eigen::Quaterniond& q, const Eigen::Quaterniond& a,
    const Eigen::Quaterniond& b) {
  const double wq = q.w();
  const double wa = a.w();
  const double wb = b.w();
  const Eigen::Ref<const Eigen::Vector3d> Vq = q.vec();
  const Eigen::Ref<const Eigen::Vector3d> Va = a.vec();
  const Eigen::Ref<const Eigen::Vector3d> Vb = b.vec();
  Eigen::Matrix<double, 1, 4> dw_dq;

  dw_dq << wa * Vb.transpose() + wb * Va.transpose() -
               Va.transpose() * ctrl_utils::CrossMatrix(Vb),

      wa * wb - Va.dot(Vb);

  return dw_dq;
}

Eigen::Matrix<double, 3, 4> AngularDistanceObjective::daqinvbv_dq(
    const Eigen::Quaterniond& q, const Eigen::Quaterniond& a,
    const Eigen::Quaterniond& b) {
  const double wq = q.w();
  const double wa = a.w();
  const double wb = b.w();
  const Eigen::Ref<const Eigen::Vector3d> Vq = q.vec();
  const Eigen::Ref<const Eigen::Vector3d> Va = a.vec();
  const Eigen::Ref<const Eigen::Vector3d> Vb = b.vec();
  Eigen::Matrix<double, 3, 4> dv_dq;

  dv_dq << wa * ctrl_utils::CrossMatrix(Vb) - wb * ctrl_utils::CrossMatrix(Va) +
               Va * Vb.transpose() + Vb * Va.transpose() -
               a.coeffs().dot(b.coeffs()) * Eigen::Matrix3d::Identity(),

      wa * Vb + wb * Va + Va.cross(Vb);

  return dv_dq;
}

Eigen::Matrix<double, 1, 4> AngularDistanceObjective::dbqcw_dq(
    const Eigen::Quaterniond& q, const Eigen::Quaterniond& b,
    const Eigen::Quaterniond& c) {
  const double wq = q.w();
  const double wb = b.w();
  const double wc = c.w();
  const Eigen::Ref<const Eigen::Vector3d> Vq = q.vec();
  const Eigen::Ref<const Eigen::Vector3d> Vb = b.vec();
  const Eigen::Ref<const Eigen::Vector3d> Vc = c.vec();
  Eigen::Matrix<double, 1, 4> dw_dq;

  dw_dq << -wb * Vc.transpose() - wc * Vb.transpose() +
               Vb.transpose() * ctrl_utils::CrossMatrix(Vc),

      wb * wc - Vb.dot(Vc);

  return dw_dq;
}

Eigen::Matrix<double, 3, 4> AngularDistanceObjective::dbqcv_dq(
    const Eigen::Quaterniond& q, const Eigen::Quaterniond& b,
    const Eigen::Quaterniond& c) {
  const double wq = q.w();
  const double wb = b.w();
  const double wc = c.w();
  const Eigen::Ref<const Eigen::Vector3d> Vq = q.vec();
  const Eigen::Ref<const Eigen::Vector3d> Vb = b.vec();
  const Eigen::Ref<const Eigen::Vector3d> Vc = c.vec();
  Eigen::Matrix<double, 3, 4> dv_dq;

  dv_dq << -wb * ctrl_utils::CrossMatrix(Vc) +
               wc * ctrl_utils::CrossMatrix(Vb) - Vb * Vc.transpose() -
               Vc * Vb.transpose() +
               b.coeffs().dot(c.coeffs()) * Eigen::Matrix3d::Identity(),

      wb * Vc + wc * Vb + Vb.cross(Vc);

  return dv_dq;
}

Eigen::Matrix<double, 1, 4> AngularDistanceObjective::daqinvbqcw_dq(
    const Eigen::Quaterniond& q, const Eigen::Quaterniond& a,
    const Eigen::Quaterniond& b, const Eigen::Quaterniond& c) {
  const double wq = q.w();
  const double wa = a.w();
  const double wb = b.w();
  const double wc = c.w();
  const Eigen::Ref<const Eigen::Vector3d> Vq = q.vec();
  const Eigen::Ref<const Eigen::Vector3d> Va = a.vec();
  const Eigen::Ref<const Eigen::Vector3d> Vb = b.vec();
  const Eigen::Ref<const Eigen::Vector3d> Vc = c.vec();
  // const Eigen::Matrix3d vb_x = ctrl_utils::CrossMatrix(vb);
  // const Eigen::Matrix3d vc_x = ctrl_utils::CrossMatrix(vc);
  Eigen::Matrix<double, 1, 4> dw_dq;

  dw_dq << 2 * wa * wb * wc * Vq.transpose() +
               2 * wa * Vb.dot(Vc) * Vq.transpose() -
               2 * wa * Vb.dot(Vq) * Vc.transpose() -
               2 * wa * Vc.dot(Vq) * Vb.transpose() -
               2 * wb * Va.dot(Vc) * Vq.transpose() +
               2 * wc * Va.dot(Vb) * Vq.transpose() -
               2 * wc * Va.dot(Vq) * Vb.transpose() -
               2 * wc * Vb.dot(Vq) * Va.transpose() +
               2 * wq * Va.dot(Vb) * Vc.transpose() -
               2 * wq * Vb.dot(Vc) * Va.transpose() +
               2 * wa * wq * Vb.transpose() * ctrl_utils::CrossMatrix(Vc) -
               2 * wc * wq * Va.transpose() * ctrl_utils::CrossMatrix(Vb) +
               2 * Vb.dot(Vq) * Va.transpose() * ctrl_utils::CrossMatrix(Vc) +
               2 * Va.transpose() * Vb.cross(Vc) * Vq.transpose() +
               2 * Va.transpose() * Vc.cross(Vq) * Vb.transpose(),

      2 * wa * wb * wc * wq - 2 * wa * wq * Vb.dot(Vc) -
          2 * wb * wq * Va.dot(Vc) - 2 * wc * wq * Va.dot(Vb) +
          2 * Va.dot(Vb) * Vc.dot(Vq) - 2 * Va.dot(Vq) * Vb.dot(Vc) +
          2 * wa * Vb.transpose() * Vc.cross(Vq) -
          2 * wc * Va.transpose() * Vb.cross(Vq) -
          2 * wq * Va.transpose() * Vb.cross(Vc);

  return dw_dq;
}

Eigen::Matrix<double, 3, 4> AngularDistanceObjective::daqinvbqcv_dq(
    const Eigen::Quaterniond& q, const Eigen::Quaterniond& a,
    const Eigen::Quaterniond& b, const Eigen::Quaterniond& c) {
  const double wq = q.w();
  const double wa = a.w();
  const double wb = b.w();
  const double wc = c.w();
  const Eigen::Ref<const Eigen::Vector3d> Vq = q.vec();
  const Eigen::Ref<const Eigen::Vector3d> Va = a.vec();
  const Eigen::Ref<const Eigen::Vector3d> Vb = b.vec();
  const Eigen::Ref<const Eigen::Vector3d> Vc = c.vec();
  Eigen::Matrix<double, 3, 4> dv_dq;

  dv_dq << -2 * wa * wq * Vb * Vc.transpose() +
               2 * wc * wq * Vb * Va.transpose() +
               2 * a.coeffs().dot(b.coeffs()) * Vc * Vq.transpose() -
               2 * a.coeffs().dot(c.coeffs()) * Vb * Vq.transpose() +
               2 * a.coeffs().dot(c.coeffs()) * Vq * Vb.transpose() +
               2 * b.coeffs().dot(c.coeffs()) * Va * Vq.transpose() +
               2 * (wa * wc * wq - wq * Va.dot(Vc)) *
                   ctrl_utils::CrossMatrix(Vb) +
               2 * (-wa * Vb.dot(Vq) + wq * Va.dot(Vb)) *
                   ctrl_utils::CrossMatrix(Vc) +
               2 * (wc * Vb.dot(Vq) + wq * Vb.dot(Vc)) *
                   ctrl_utils::CrossMatrix(Va) -
               2 * Va.dot(Vq) * Vc * Vb.transpose() -
               2 * Vb.dot(Vq) * Va * Vc.transpose() -
               2 * Vb.dot(Vq) * Vc * Va.transpose() -
               2 * Vc.dot(Vq) * Va * Vb.transpose() -
               2 * wa * Vb.cross(Vc) * Vq.transpose() -
               2 * wa * Vc.cross(Vq) * Vb.transpose() +
               2 * wb * Va.cross(Vc) * Vq.transpose() -
               2 * wc * Va.cross(Vb) * Vq.transpose() +
               2 * wc * Va.cross(Vq) * Vb.transpose() -
               2 * wq * Va.cross(Vb) * Vc.transpose() +
               2 * wq * Vb.cross(Vc) * Va.transpose() +
               2 *
                   (wa * wc * Vb.dot(Vq) + wa * wq * Vb.dot(Vc) -
                    wc * wq * Va.dot(Vb) + Va.dot(Vc) * Vb.dot(Vq)) *
                   Eigen::Matrix3d::Identity(),

      2 * wa * wb * wq * Vc + 2 * wa * wc * wq * Vb + 2 * wb * wc * wq * Va +
          2 * wa * Vb.dot(Vc) * Vq - 2 * wa * Vc.dot(Vq) * Vb -
          2 * wc * Va.dot(Vb) * Vq + 2 * wc * Va.dot(Vq) * Vb -
          2 * wq * Va.dot(Vb) * Vc + 2 * wq * Va.dot(Vc) * Vb -
          2 * wq * Vb.dot(Vc) * Va + 2 * wa * wc * Vb.cross(Vq) +
          2 * wa * wq * Vb.cross(Vc) + 2 * wb * wq * Va.cross(Vc) +
          2 * wc * wq * Va.cross(Vb) + 2 * Va.dot(Vb) * Vc.cross(Vq) -
          2 * Va.dot(Vc) * Vb.cross(Vq) + 2 * Va.dot(Vq) * Vb.cross(Vc) +
          2 * Vb.dot(Vc) * Va.cross(Vq) - 2 * Vc.dot(Vq) * Va.cross(Vb);

  return dv_dq;
}

Eigen::Matrix4d AngularDistanceObjective::dqhat_dq(
    const Eigen::Quaterniond& q) {
  return LinearDistanceObjective::dqhat_dq(q);
}

}  // namespace logic_opt

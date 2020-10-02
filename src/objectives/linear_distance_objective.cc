/**
 * linear_distance_objective.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 26, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/objectives/linear_distance_objective.h"

#include <exception>  // std::domain_error

namespace logic_opt {

Eigen::Vector3d LinearDistanceObjective::qxqinv(const Eigen::Quaterniond& q,
                                                const Eigen::Vector3d& x) {
  const double wq = q.w();
  const Eigen::Ref<const Eigen::Vector3d> Vq = q.vec();

  return (wq * wq - Vq.dot(Vq)) * x + 2 * Vq.dot(x) * Vq + 2 * wq * Vq.cross(x);
}

Eigen::Matrix3d LinearDistanceObjective::dqxqinv_dx(
    const Eigen::Quaterniond& q) {
  const double wq = q.w();
  const Eigen::Ref<const Eigen::Vector3d> Vq = q.vec();

  return 2 * wq * ctrl_utils::CrossMatrix(Vq) + 2 * Vq * Vq.transpose() +
         (wq * wq - Vq.dot(Vq)) * Eigen::Matrix3d::Identity();
}

Eigen::Matrix<double, 3, 4> LinearDistanceObjective::dqxqinv_dq(
    const Eigen::Quaterniond& q, const Eigen::Vector3d& x) {
  const double wq = q.w();
  const Eigen::Ref<const Eigen::Vector3d> Vq = q.vec();

  Eigen::Matrix<double, 3, 4> dv_dq;
  dv_dq << -2 * wq * ctrl_utils::CrossMatrix(x) + 2 * Vq * x.transpose() -
               2 * x * Vq.transpose() +
               2 * Vq.dot(x) * Eigen::Matrix3d::Identity(),

      2 * wq * x + 2 * Vq.cross(x);
  return dv_dq;
}

Eigen::Matrix4d LinearDistanceObjective::dqhat_dq(const Eigen::Quaterniond& q) {
  if (q.norm() == 0.) {
    throw std::domain_error(
        "dqhat_dq_impl(): Cannot normalize zero norm quaternion.");
  }
  const Eigen::Quaterniond q_hat = q.normalized();
  const double q_norm = q.coeffs().norm();
  return 1 / q_norm *
         (Eigen::Matrix4d::Identity() -
          q_hat.coeffs() * q_hat.coeffs().transpose());
}

inline double LinearDistanceObjectiveEvaluate(
    Eigen::Ref<const Eigen::MatrixXd> X, const std::string& name_ee,
    const World& world) {
  double objective = 0.;
  Eigen::Vector3d x_t = world.Position(name_ee, world.kWorldFrame, X, 0);
  for (size_t t = 0; t < X.cols() - 1; t++) {
    const Eigen::Vector3d x_next =
        world.Position(name_ee, world.kWorldFrame, X, t + 1);

    // 0.5 * || x_{t+1} - x_{t} ||^2
    objective += 0.5 * (x_next - x_t).squaredNorm();

    x_t = x_next;
  }
  return objective;
}

void LinearDistanceObjective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                       double& objective) {
  objective += coeff_ * LinearDistanceObjectiveEvaluate(X, name_ee_, world_);
  Objective::Evaluate(X, objective);
}

Eigen::Matrix3Xd LinearDistanceObjective::PositionJacobian(
    const World& world, const std::string& name_frame,
    Eigen::Ref<const Eigen::MatrixXd> X, size_t t) {
  Eigen::Matrix3Xd J_t = Eigen::Matrix3Xd::Zero(3, X.size());

  // Iterate over all ancestors of the given end-effector frame. The Jacobian is
  // nonzero for variable frames only.
  auto chain = world.frames(t).ancestors(name_frame);
  for (auto it = chain.begin(); it != chain.end(); ++it) {
    if (it->first == world.kWorldFrame) break;
    const Frame& frame = it->second;
    if (!frame.is_variable()) continue;

    // Position variables affect the end-effector position with an identity
    // mapping. However, the position variables are oriented with their parent's
    // frame, so the identity Jacobian needs to be transformed to the world
    // frame with `R_parent`.
    auto J_pos = J_t.block<3, 3>(0, world.kDof * frame.idx_var());
    const std::string& name_parent = *world.frames(t).parent(frame.name());
    const Eigen::Matrix3d R_parent =
        world.Orientation(name_parent, world.kWorldFrame, X, t)
            .toRotationMatrix();
    J_pos = R_parent;

    // The rotation variable of the end-effector frame has no affect on the
    // end-effector's position.
    if (frame.name() == name_frame) continue;

    // Assuming the end-effector's position is fixed in the current frame at
    // `pos_in_frame`, the quaternion variable affects the end-effector's
    // position with `dqxqinv_dq`. This Jacobian, computed in the parent frame's
    // frame, needs to be transformed to the world frame with `R_parent`. Since
    // the quaternion variables are not necessarily unit norm, we compute the
    // orientation using the normalized quaternion `q_hat`, and then add
    // `dqhat_dq` in the chain rule.
    auto J_quat = J_t.block<3, 4>(0, world.kDof * frame.idx_var() + 3);
    const Eigen::Map<const Eigen::Quaterniond> q =
        world.Quaternion(X, frame.idx_var());
    if (q.norm() == 0.) {
      throw std::domain_error(
          "LinearDistanceObjective::PositionJacobian(): Cannot normalize zero "
          "norm quaternion.");
    }
    const Eigen::Quaterniond q_hat = q.normalized();
    const Eigen::Vector3d pos_in_frame =
        world.Position(name_frame, frame.name(), X, t);

    J_quat = R_parent * dqxqinv_dq(q_hat, pos_in_frame) * dqhat_dq(q);
  }
  return J_t;
}

// Eigen::Matrix2Xd PositionJacobian(const logic_opt::World2& world,
//                                   const std::string& name_frame,
//                                   Eigen::Ref<const Eigen::MatrixXd> X,
//                                   size_t t) {
//   Eigen::Matrix2Xd J_t = Eigen::Matrix2Xd::Zero(2, X.size());

//   auto chain = world.frames(t).ancestors(name_frame);
//   for (auto it = chain.begin(); it != chain.end(); ++it) {
//     if (it->first == world.kWorldFrame) break;
//     const Frame& frame = it->second;
//     if (!frame.is_variable()) continue;

//     auto J_pos = J_t.block<2, 2>(0, 3 * frame.idx_var());
//     const std::string& name_parent = *world.frames(t).parent(frame.name());
//     J_pos = world.Orientation(name_parent, world.kWorldFrame, X, t);

//     if (frame.name() == name_frame) continue;
//     auto J_ori = J_t.block<2, 1>(0, 3 * frame.idx_var() + 2);
//     const Eigen::Vector2d p = world.Position(name_frame, frame.name(), X, t);
//     J_ori = J_pos * Eigen::Vector2d(-p(1), p(0));
//   }
//   return J_t;
// }

inline Eigen::MatrixXd LinearDistanceObjectiveGradient(
    Eigen::Ref<const Eigen::MatrixXd> X, const World& world,
    const std::string& name_ee) {
  Eigen::MatrixXd Gradient =
      Eigen::MatrixXd::Zero(world.kDof, world.num_timesteps());
  Eigen::Map<Eigen::VectorXd> gradient(Gradient.data(), Gradient.size());

  Eigen::Vector3d x_t = world.Position(name_ee, world.kWorldFrame, X, 0);
  Eigen::Vector3d dx_prev = Eigen::Vector3d::Zero();
  for (size_t t = 0; t < X.cols() - 1; t++) {
    const auto J_t =
        LinearDistanceObjective::PositionJacobian(world, name_ee, X, t);

    const auto x_next = world.Position(name_ee, world.kWorldFrame, X, t + 1);
    const Eigen::Vector3d dx_t = x_next - x_t;

    // J_{:,t} = J_t^T * ((x_{t} - x_{t-1}) - (x_{t+1} - x_{t}))
    gradient += J_t.transpose() * (dx_prev - dx_t);

    x_t = x_next;
    dx_prev = dx_t;
  }

  // J_{:,T} = J_T^T * ((x_{T} - x_{T-1})
  const auto J_T = LinearDistanceObjective::PositionJacobian(world, name_ee, X,
                                                             X.cols() - 1);
  gradient += J_T.transpose() * dx_prev;

  return Gradient;  // (7xT) x 1
}

void LinearDistanceObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> X,
                                       Eigen::Ref<Eigen::MatrixXd> Gradient) {
  Gradient += coeff_ * LinearDistanceObjectiveGradient(X, world_, name_ee_);
  Objective::Gradient(X, Gradient);
}

}  // namespace logic_opt

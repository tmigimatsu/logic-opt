/**
 * nlopt.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 25, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_NLOPT_H_
#define TRAJ_OPT_NLOPT_H_

#include <SpatialDyn/SpatialDyn.h>

#include "constraints.h"
#include "objectives.h"
#include "joint_variables.h"

#include <vector>    // std::vector

namespace TrajOpt {
namespace Nlopt {

typedef std::vector<double> OptimizationData;

std::vector<Eigen::VectorXd> Trajectory(const JointVariables& variables,
                                        const Objectives& objectives,
                                        const Constraints& constraints,
                                        OptimizationData* data = nullptr);

}  // namespace Nlopt
}  // namespace TrajOpt

// double JointVelocityObjective(const std::vector<double>& var, std::vector<double>& grad, void* data) {
//   OptimizationData d = *reinterpret_cast<OptimizationData*>(data);
//   SpatialDyn::ArticulatedBody& ab = d.ab;
//   const Eigen::VectorXd& q_0 = d.q_0;
//   const size_t T = d.T;

//   Eigen::Map<const Eigen::MatrixXd> Q(&var[0], ab.dof(), T);

//   if (!grad.empty()) {
//     Eigen::Map<Eigen::MatrixXd> Grad(&grad[0], ab.dof(), T);
//     for (size_t t = 0; t < T; t++) {
//       if (t < T - 1) {
//         Grad.col(t) = Q.col(t) - Q.col(t+1);
//       } else {
//         Grad.col(t).setZero();
//       }

//       if (t > 0) {
//         Grad.col(t) += Q.col(t) - Q.col(t-1);
//       }
//     }
//   }

//   double obj = 0.;
//   for (size_t t = 0; t < T - 1; t++) {
//     obj += 0.5 * (Q.col(t+1) - Q.col(t)).squaredNorm();
//   }

//   double pos_coeff = 0.01;
//   if (!grad.empty()) {
//     Eigen::Map<Eigen::MatrixXd> Grad(&grad[0], ab.dof(), T);
//     for (size_t t = 0; t < T; t++) {
//       Grad.col(t) += pos_coeff * (Q.col(t) - q_0);
//     }
//   }

//   for (size_t t = 0; t < T; t++) {
//     obj += pos_coeff * 0.5 * (Q.col(t) - q_0).squaredNorm();
//   }

//   return obj;
// }

// double TaskVelocityObjective(const std::vector<double>& var, std::vector<double>& grad, void* data) {
//   OptimizationData d = *reinterpret_cast<OptimizationData*>(data);
//   SpatialDyn::ArticulatedBody& ab = d.ab;
//   const size_t T = d.T;

//   Eigen::Map<const Eigen::MatrixXd> Q(&var[0], ab.dof(), T);
//   Eigen::Map<Eigen::MatrixXd> Grad(&grad[0], ab.dof(), T);

//   std::vector<Eigen::Vector6d> dx_ws(T-1);
//   Eigen::VectorXd grad_prev;
//   for (size_t t = 0; t < T; t++) {
//     ab.set_q(Q.col(t));
//     Eigen::MatrixXd J;
//     if (!grad.empty()) J = SpatialDyn::Jacobian(ab);

//     if (t < T - 1) {
//       Eigen::Vector3d x_t = SpatialDyn::Position(ab);
//       Eigen::Quaterniond quat_t = SpatialDyn::Orientation(ab);

//       ab.set_q(Q.col(t+1));
//       dx_ws[t].head<3>() = SpatialDyn::Position(ab) - x_t;
//       dx_ws[t].tail<3>().setZero();// = SpatialDyn::Opspace::OrientationError(quat_t, SpatialDyn::Orientation(ab));
//       if (!grad.empty()) {
//         if (t > 0) {
//           Grad.col(t) = J.transpose() * (dx_ws[t-1] - dx_ws[t]);
//         } else {
//           Grad.col(t) = -J.transpose() * dx_ws[t];
//         }
//         // dx_ws[t] = J * (Q.col(t+1) - Q.col(t));
//         // grad_prev = J.transpose() * dx_ws[t];
//         // Grad.col(t) -= grad_prev;
//       }
//     } else if (!grad.empty()) {
//       Grad.col(t) = J.transpose() * dx_ws[t-1];
//     }

//   }

//   double obj = 0.;
//   for (size_t t = 0; t < T - 1; t++) {
//     obj += 0.5 * dx_ws[t].squaredNorm();
//   }
//   return obj;
// }

// double JointPositionConstraint(const std::vector<double>& var, std::vector<double>& grad, void* data) {
//   OptimizationData d = *reinterpret_cast<OptimizationData*>(data);
//   SpatialDyn::ArticulatedBody& ab = d.ab;
//   const Eigen::VectorXd& q_des = d.q_des;
//   const size_t T = d.T;

//   Eigen::Map<const Eigen::MatrixXd> Q(&var[0], ab.dof(), T);

//   if (!grad.empty()) {
//     Eigen::Map<Eigen::MatrixXd> Grad(&grad[0], ab.dof(), T);
//     Grad.leftCols(T - 1).setZero();
//     Grad.col(T - 1) = Q.col(T - 1) - q_des;
//   }

//   return 0.5 * (Q.col(T - 1) - q_des).squaredNorm();
// }


// double TaskGoalConstraint(const std::vector<double>& var, std::vector<double>& grad, void* data) {
//   OptimizationData d = *reinterpret_cast<OptimizationData*>(data);
//   SpatialDyn::ArticulatedBody& ab = d.ab;
//   const Eigen::Vector3d& x_des = d.x_des;
//   const Eigen::Quaterniond& quat_des = d.quat_des;
//   const size_t T = d.T;

//   Eigen::Map<const Eigen::MatrixXd> Q(&var[0], ab.dof(), T);

//   ab.set_q(Q.col(T - 1));
//   Eigen::Vector6d x_quat_err;
//   x_quat_err.head<3>() = SpatialDyn::Position(ab) - x_des;
//   x_quat_err.tail<3>() = SpatialDyn::Opspace::OrientationError(SpatialDyn::Orientation(ab), quat_des);

//   if (!grad.empty()) {
//     Eigen::Map<Eigen::MatrixXd> Grad(&grad[0], ab.dof(), T);
//     Grad.leftCols(T - 1).setZero();
//     Grad.col(T - 1) = SpatialDyn::Jacobian(ab).transpose() * x_quat_err;
//   }
//   std::cout << "x_err: " << x_quat_err.transpose() << std::endl;

//   return 0.5 * x_quat_err.squaredNorm();
// }

// double AboveTableConstraint(const std::vector<double>& var, std::vector<double>& grad, void* data) {
//   OptimizationData d = *reinterpret_cast<OptimizationData*>(data);
//   SpatialDyn::ArticulatedBody& ab = d.ab;
//   const size_t T = d.T;

//   const SpatialDyn::RigidBody& table = d.world_objects["table"];
//   const auto& pos_table = table.T_to_parent().translation();
//   const Eigen::Vector3d& dim_table = table.graphics.geometry.scale;
//   double height_table = pos_table(2) + 0.5 * dim_table(2);
//   std::array<double, 4> area_table = {
//     pos_table(0) + 0.5 * dim_table(0),
//     pos_table(0) - 0.5 * dim_table(0),
//     pos_table(1) + 0.5 * dim_table(1),
//     pos_table(1) - 0.5 * dim_table(1)
//   };

//   Eigen::Map<const Eigen::MatrixXd> Q(&var[0], ab.dof(), T);
//   Eigen::Map<Eigen::MatrixXd> Grad(&grad[0], ab.dof(), T);

//   Eigen::VectorXd z_err(T);
//   for (size_t t = 0; t < T; t++) {
//     ab.set_q(Q.col(t));
//     Eigen::Vector3d pos_ee = SpatialDyn::Position(ab);
//     z_err(t) = std::min(0., pos_ee(2) - height_table);
//     if (z_err(t) != 0.) {
//       if (!(pos_ee(0) <= area_table[0] && pos_ee(0) >= area_table[1] &&
//             pos_ee(1) <= area_table[2] && pos_ee(1) >= area_table[3])) {
//         z_err(t) = 0.;
//       }
//     }

//     if (!grad.empty()) {
//       if (z_err(t) == 0.) {
//         Grad.col(t).setZero();
//       } else {
//         Grad.col(t) = SpatialDyn::Jacobian(ab).row(2).transpose() * z_err(t);
//       }
//     }
//   }
//   // std::cout << "z_err: " << z_err.transpose() << std::endl;
//   // std::cout << "|Z|: " << z_err.sum() << std::endl;
//   // if (!grad.empty()) {
//   //   std::cout << "G: " << std::endl << Grad << std::endl << std::endl;
//   // }

//   // return -z_err.sum();
//   return 0.5 * z_err.squaredNorm();
// }

// double InitConstraint(const std::vector<double>& var, std::vector<double>& grad, void* data) {
//   OptimizationData d = *reinterpret_cast<OptimizationData*>(data);
//   SpatialDyn::ArticulatedBody& ab = d.ab;
//   const Eigen::VectorXd& q_0 = d.q_0;
//   const size_t T = d.T;

//   Eigen::Map<const Eigen::MatrixXd> Q(&var[0], ab.dof(), T);

//   if (!grad.empty()) {
//     Eigen::Map<Eigen::MatrixXd> Grad(&grad[0], ab.dof(), T);
//     Grad.col(0) = Q.col(0) - q_0;
//     Grad.rightCols(T - 1).setZero();
//   }

//   return 0.5 * (Q.col(0) - q_0).squaredNorm();
// }


#endif  // TRAJ_OPT_NLOPT_H_

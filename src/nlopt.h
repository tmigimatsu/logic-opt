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
#include <nlopt.hpp>

#include <array>     // std::array
#include <iostream>  // std::cout
#include <map>       // std::map
#include <string>    // std::string
#include <vector>    // std::vector

struct OptimizationData {
  SpatialDyn::ArticulatedBody ab;
  std::map<std::string, SpatialDyn::RigidBody> world_objects;
  Eigen::Vector3d x_des;
  Eigen::Quaterniond quat_des;
  Eigen::VectorXd q_0;
  Eigen::VectorXd q_des;
  size_t T;

  OptimizationData(const SpatialDyn::ArticulatedBody& ab,
                   const std::map<std::string, SpatialDyn::RigidBody>& world_objects,
                   const Eigen::Vector3d& x_des, const Eigen::Quaterniond& quat_des,
                   const Eigen::VectorXd& q_0, size_t T)
      : ab(ab), world_objects(world_objects), x_des(x_des), quat_des(quat_des),
        q_0(q_0), T(T) {};

  OptimizationData(const SpatialDyn::ArticulatedBody& ab, const Eigen::VectorXd& q_des,
                   const Eigen::VectorXd& q_0)
      : ab(ab), q_des(q_des), q_0(q_0) {};
};

double JointVelocityObjective(const std::vector<double>& var, std::vector<double>& grad, void* data) {
  OptimizationData d = *reinterpret_cast<OptimizationData*>(data);
  SpatialDyn::ArticulatedBody& ab = d.ab;
  const Eigen::VectorXd& q_0 = d.q_0;
  const size_t T = d.T;

  Eigen::Map<const Eigen::MatrixXd> Q(&var[0], ab.dof(), T);

  if (!grad.empty()) {
    Eigen::Map<Eigen::MatrixXd> Grad(&grad[0], ab.dof(), T);
    for (size_t t = 0; t < T; t++) {
      if (t < T - 1) {
        Grad.col(t) = Q.col(t) - Q.col(t+1);
      } else {
        Grad.col(t).setZero();
      }

      if (t > 0) {
        Grad.col(t) += Q.col(t) - Q.col(t-1);
      }
    }
  }

  double obj = 0.;
  for (size_t t = 0; t < T - 1; t++) {
    obj += 0.5 * (Q.col(t+1) - Q.col(t)).squaredNorm();
  }

  double pos_coeff = 0.01;
  if (!grad.empty()) {
    Eigen::Map<Eigen::MatrixXd> Grad(&grad[0], ab.dof(), T);
    for (size_t t = 0; t < T; t++) {
      Grad.col(t) += pos_coeff * (Q.col(t) - q_0);
    }
  }

  for (size_t t = 0; t < T; t++) {
    obj += pos_coeff * 0.5 * (Q.col(t) - q_0).squaredNorm();
  }

  return obj;
}

double TaskVelocityObjective(const std::vector<double>& var, std::vector<double>& grad, void* data) {
  OptimizationData d = *reinterpret_cast<OptimizationData*>(data);
  SpatialDyn::ArticulatedBody& ab = d.ab;
  const size_t T = d.T;

  Eigen::Map<const Eigen::MatrixXd> Q(&var[0], ab.dof(), T);
  Eigen::Map<Eigen::MatrixXd> Grad(&grad[0], ab.dof(), T);

  std::vector<Eigen::Vector6d> dx_ws(T-1);
  Eigen::VectorXd grad_prev;
  for (size_t t = 0; t < T; t++) {
    ab.set_q(Q.col(t));
    Eigen::MatrixXd J;
    if (!grad.empty()) J = SpatialDyn::Jacobian(ab);

    if (t < T - 1) {
      Eigen::Vector3d x_t = SpatialDyn::Position(ab);
      Eigen::Quaterniond quat_t = SpatialDyn::Orientation(ab);

      ab.set_q(Q.col(t+1));
      dx_ws[t].head<3>() = SpatialDyn::Position(ab) - x_t;
      dx_ws[t].tail<3>().setZero();// = SpatialDyn::Opspace::OrientationError(quat_t, SpatialDyn::Orientation(ab));
      if (!grad.empty()) {
        if (t > 0) {
          Grad.col(t) = J.transpose() * (dx_ws[t-1] - dx_ws[t]);
        } else {
          Grad.col(t) = -J.transpose() * dx_ws[t];
        }
        // dx_ws[t] = J * (Q.col(t+1) - Q.col(t));
        // grad_prev = J.transpose() * dx_ws[t];
        // Grad.col(t) -= grad_prev;
      }
    } else if (!grad.empty()) {
      Grad.col(t) = J.transpose() * dx_ws[t-1];
    }

  }

  double obj = 0.;
  for (size_t t = 0; t < T - 1; t++) {
    obj += 0.5 * dx_ws[t].squaredNorm();
  }
  return obj;
}

double JointPositionConstraint(const std::vector<double>& var, std::vector<double>& grad, void* data) {
  OptimizationData d = *reinterpret_cast<OptimizationData*>(data);
  SpatialDyn::ArticulatedBody& ab = d.ab;
  const Eigen::VectorXd& q_des = d.q_des;
  const size_t T = d.T;

  Eigen::Map<const Eigen::MatrixXd> Q(&var[0], ab.dof(), T);

  if (!grad.empty()) {
    Eigen::Map<Eigen::MatrixXd> Grad(&grad[0], ab.dof(), T);
    Grad.leftCols(T - 1).setZero();
    Grad.col(T - 1) = Q.col(T - 1) - q_des;
  }

  return 0.5 * (Q.col(T - 1) - q_des).squaredNorm();
}


double TaskGoalConstraint(const std::vector<double>& var, std::vector<double>& grad, void* data) {
  OptimizationData d = *reinterpret_cast<OptimizationData*>(data);
  SpatialDyn::ArticulatedBody& ab = d.ab;
  const Eigen::Vector3d& x_des = d.x_des;
  const Eigen::Quaterniond& quat_des = d.quat_des;
  const size_t T = d.T;

  Eigen::Map<const Eigen::MatrixXd> Q(&var[0], ab.dof(), T);

  ab.set_q(Q.col(T - 1));
  Eigen::Vector6d x_quat_err;
  x_quat_err.head<3>() = SpatialDyn::Position(ab) - x_des;
  x_quat_err.tail<3>() = SpatialDyn::Opspace::OrientationError(SpatialDyn::Orientation(ab), quat_des);

  if (!grad.empty()) {
    Eigen::Map<Eigen::MatrixXd> Grad(&grad[0], ab.dof(), T);
    Grad.leftCols(T - 1).setZero();
    Grad.col(T - 1) = SpatialDyn::Jacobian(ab).transpose() * x_quat_err;
  }
  std::cout << "x_err: " << x_quat_err.transpose() << std::endl;

  return 0.5 * x_quat_err.squaredNorm();
}

double AboveTableConstraint(const std::vector<double>& var, std::vector<double>& grad, void* data) {
  OptimizationData d = *reinterpret_cast<OptimizationData*>(data);
  SpatialDyn::ArticulatedBody& ab = d.ab;
  const size_t T = d.T;

  const SpatialDyn::RigidBody& table = d.world_objects["table"];
  const auto& pos_table = table.T_to_parent().translation();
  const Eigen::Vector3d& dim_table = table.graphics.geometry.scale;
  double height_table = pos_table(2) + 0.5 * dim_table(2);
  std::array<double, 4> area_table = {
    pos_table(0) + 0.5 * dim_table(0),
    pos_table(0) - 0.5 * dim_table(0),
    pos_table(1) + 0.5 * dim_table(1),
    pos_table(1) - 0.5 * dim_table(1)
  };

  Eigen::Map<const Eigen::MatrixXd> Q(&var[0], ab.dof(), T);
  Eigen::Map<Eigen::MatrixXd> Grad(&grad[0], ab.dof(), T);

  Eigen::VectorXd z_err(T);
  for (size_t t = 0; t < T; t++) {
    ab.set_q(Q.col(t));
    Eigen::Vector3d pos_ee = SpatialDyn::Position(ab);
    z_err(t) = std::min(0., pos_ee(2) - height_table);
    if (z_err(t) != 0.) {
      if (!(pos_ee(0) <= area_table[0] && pos_ee(0) >= area_table[1] &&
            pos_ee(1) <= area_table[2] && pos_ee(1) >= area_table[3])) {
        z_err(t) = 0.;
      }
    }

    if (!grad.empty()) {
      if (z_err(t) == 0.) {
        Grad.col(t).setZero();
      } else {
        Grad.col(t) = SpatialDyn::Jacobian(ab).row(2).transpose() * z_err(t);
      }
    }
  }
  // std::cout << "z_err: " << z_err.transpose() << std::endl;
  // std::cout << "|Z|: " << z_err.sum() << std::endl;
  // if (!grad.empty()) {
  //   std::cout << "G: " << std::endl << Grad << std::endl << std::endl;
  // }

  // return -z_err.sum();
  return 0.5 * z_err.squaredNorm();
}

double InitConstraint(const std::vector<double>& var, std::vector<double>& grad, void* data) {
  OptimizationData d = *reinterpret_cast<OptimizationData*>(data);
  SpatialDyn::ArticulatedBody& ab = d.ab;
  const Eigen::VectorXd& q_0 = d.q_0;
  const size_t T = d.T;

  Eigen::Map<const Eigen::MatrixXd> Q(&var[0], ab.dof(), T);

  if (!grad.empty()) {
    Eigen::Map<Eigen::MatrixXd> Grad(&grad[0], ab.dof(), T);
    Grad.col(0) = Q.col(0) - q_0;
    Grad.rightCols(T - 1).setZero();
  }

  return 0.5 * (Q.col(0) - q_0).squaredNorm();
}

std::vector<Eigen::VectorXd> NloptTaskSpaceTrajectory(const SpatialDyn::ArticulatedBody& ab,
                                                      const std::map<std::string, SpatialDyn::RigidBody>& world_objects,
                                                      const Eigen::Vector3d& x_des,
                                                      const Eigen::Quaterniond& quat_des,
                                                      size_t T) {
  nlopt::opt opt(nlopt::algorithm::LD_SLSQP, ab.dof() * T);
  // nlopt::opt opt(nlopt::algorithm::AUGLAG, ab.dof() * T);
  // nlopt::opt local_opt(nlopt::algorithm::LD_SLSQP, ab.dof() * T);
  // local_opt.set_ftol_abs(0.001);
  // opt.set_local_optimizer(local_opt);

  OptimizationData data(ab, world_objects, x_des, quat_des, ab.q(), T);
  // OptimizationData data(ab, q_des, ab.q());
  opt.set_min_objective(JointVelocityObjective, &data);
  // opt.set_min_objective(TaskVelocityObjective, &data);
  // opt.add_equality_constraint(JointPositionConstraint, &data);
  opt.add_equality_constraint(TaskGoalConstraint, &data);
  opt.add_equality_constraint(InitConstraint, &data);
  opt.add_inequality_constraint(AboveTableConstraint, &data);

  // Joint limits
  std::vector<double> q_min(ab.dof() * T);
  std::vector<double> q_max(ab.dof() * T);
  Eigen::Map<Eigen::MatrixXd> Q_min(&q_min[0], ab.dof(), T);
  Eigen::Map<Eigen::MatrixXd> Q_max(&q_max[0], ab.dof(), T);
  for (size_t i = 0; i < ab.dof(); i++) {
    Q_min.row(i).fill(ab.rigid_bodies(i).joint().q_min());
    Q_max.row(i).fill(ab.rigid_bodies(i).joint().q_max());
  }
  opt.set_lower_bounds(q_min);
  opt.set_upper_bounds(q_max);

  opt.set_xtol_abs(0.001);

  // nlopt::opt local_opt(nlopt::algorithm::LD_MMA, ab.dof() * T);
  // local_opt.set_xtol_abs(0.0001);
  // opt.set_local_optimizer(local_opt);

  std::vector<double> opt_vars(ab.dof() * T, 1.);
  double opt_val;
  // opt.optimize(opt_vars, opt_val);

  // opt.set_min_objective(TaskVelocityObjective, &data);
  // opt.add_inequality_constraint(AboveTableConstraint, &data);
  // opt.set_xtol_abs(0.000001);
  nlopt::result result = opt.optimize(opt_vars, opt_val);
  switch (result) {
    case nlopt::SUCCESS: std::cout << "SUCCESS" << std::endl; break;
    case nlopt::STOPVAL_REACHED: std::cout << "STOPVAL_REACHED" << std::endl; break;
    case nlopt::FTOL_REACHED: std::cout << "FTOL_REACHED" << std::endl; break;
    case nlopt::XTOL_REACHED: std::cout << "XTOL_REACHED" << std::endl; break;
    case nlopt::MAXEVAL_REACHED: std::cout << "MAXEVAL_REACHED" << std::endl; break;
    case nlopt::MAXTIME_REACHED: std::cout << "MAXTIME_REACHED" << std::endl; break;
    default: std::cout << "UNKNOWN" << std::endl;
  }

  Eigen::Map<Eigen::MatrixXd> Q(&opt_vars[0], ab.dof(), T);
  std::vector<Eigen::VectorXd> q_des_traj(T);
  for (size_t t = 0; t < T; t++) {
    q_des_traj[t] = Q.col(t);
    std::cout << q_des_traj[t].transpose() << std::endl;
  }
  return q_des_traj;
}

#endif  // TRAJ_OPT_NLOPT_H_

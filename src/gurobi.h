/**
 * gurobi.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 25, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_GUROBI_H_
#define TRAJ_OPT_GUROBI_H_

#include <SpatialDyn/SpatialDyn.h>
#include <gurobi_c++.h>

#include <vector>    // std::vector


std::vector<Eigen::VectorXd> JointSpaceTrajectory(SpatialDyn::ArticulatedBody& ab,
                                                  Eigen::VectorXd& q_des,
                                                  size_t T) {
  // Set up Gurobi
  GRBEnv env;
  GRBModel model(env);

  std::vector<std::vector<GRBVar>> var_q(T, std::vector<GRBVar>(ab.dof()));
  for (size_t t = 0; t < T; t++) {
    for (size_t i = 0; i < ab.dof(); i++) {
      var_q[t][i] = model.addVar(ab.rigid_bodies(i).joint().q_min(), ab.rigid_bodies(i).joint().q_max(), 0., GRB_CONTINUOUS);
    }
  }

  GRBQuadExpr objective;
  for (size_t t = 0; t < T - 2; t++) {
    for (size_t i = 0; i < ab.dof(); i++) {
      GRBLinExpr var_ddq = var_q[t+2][i] - 2 * var_q[t+1][i] + var_q[t][i];
      objective += var_ddq * var_ddq;
    }
  }
  for (size_t i = 0; i < ab.dof(); i++) {
    GRBLinExpr var_ddq = q_des(i) - 2 * var_q[T - 1][i] + var_q[T - 2][i];
    objective += var_ddq * var_ddq;
  }
  for (size_t i = 0; i < ab.dof(); i++) {
    GRBLinExpr var_ddq = -q_des(i) + var_q[T - 1][i];
    objective += var_ddq * var_ddq;
  }
  model.setObjective(objective);

  for (size_t i = 0; i < ab.dof(); i++) {
    model.addConstr(var_q[0][i], GRB_EQUAL, 0);
    model.addConstr(var_q[T-1][i], GRB_EQUAL, q_des(i));
  }

  model.optimize();

  std::vector<Eigen::VectorXd> q_des_traj(T, Eigen::VectorXd(ab.dof()));
  for (size_t t = 0; t < T; t++) {
    for (size_t i = 0; i < ab.dof(); i++) {
      q_des_traj[t](i) = var_q[t][i].get(GRB_DoubleAttr_X);
    }
  }
  return q_des_traj;
}

#endif  // TRAJ_OPT_GUROBI_H_

/**
 * ipopt.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 25, 2018
 * Authors: Toki Migimatsu
 */

#include "ipopt.h"

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#include <cassert>    // assert
#include <exception>  // std::runtime_error

namespace TrajOpt {
namespace Ipopt {

static std::vector<Eigen::VectorXd> Optimize(NonlinearProgram* my_nlp) {
  ::Ipopt::SmartPtr<::Ipopt::TNLP> nlp = my_nlp;
  ::Ipopt::SmartPtr<::Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  // Set solver options
  app->Options()->SetStringValue("linear_solver", "ma57");
  app->Options()->SetStringValue("hessian_approximation", "limited-memory");

  ::Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != ::Ipopt::Solve_Succeeded) {
    throw std::runtime_error("JointSpaceTrajectory(): Error during Ipopt initialization.");
  }

  status = app->OptimizeTNLP(nlp);
  if (status != ::Ipopt::Solve_Succeeded) {
    if (my_nlp->status_ == "DIVERGING_ITERATES" ||
        my_nlp->status_ == "RESTORATION_FAILURE" ||
        my_nlp->status_ == "ERROR_IN_STEP_COMPUTATION" ||
        my_nlp->status_ == "INVALID_NUMBER_DETECTED" ||
        my_nlp->status_ == "INTERNAL_ERROR") {
      throw std::runtime_error("JointSpaceTrajectory(): Ipopt optimization failed.");
    }
  }

  return my_nlp->trajectory_;
}

std::vector<Eigen::VectorXd> Trajectory(const SpatialDyn::ArticulatedBody& ab,
                                        const std::map<std::string,
                                                       SpatialDyn::RigidBody>& world_objects,
                                        const Eigen::VectorXd& q_des,
                                        size_t T) {
  return Optimize(new NonlinearProgram(ab, q_des, T));
}

std::vector<Eigen::VectorXd> Trajectory(const SpatialDyn::ArticulatedBody& ab,
                                        const std::map<std::string,
                                                       SpatialDyn::RigidBody>& world_objects,
                                        const Eigen::Vector3d& x_des,
                                        const Eigen::Quaterniond& quat_des,
                                        size_t T) {
  return Optimize(new NonlinearProgram(ab, x_des, quat_des, T));
}

bool NonlinearProgram::get_nlp_info(int& n, int& m, int& nnz_jac_g,
                         int& nnz_h_lag, IndexStyleEnum& index_style) {
  n = ab_.dof() * T_;

  m = 0;
  nnz_jac_g = 0;
  for (std::unique_ptr<Constraint>& c : constraints_) {
    m += c->num_constraints;
    nnz_jac_g += c->len_jacobian;
  }

  nnz_h_lag = 0;

  index_style = TNLP::C_STYLE;

  return true;
}

bool NonlinearProgram::get_bounds_info(int n, double* x_l, double* x_u,
                                       int m, double* g_l, double* g_u) {

  Eigen::Map<Eigen::MatrixXd> Q_min(x_l, ab_.dof(), T_);
  Eigen::Map<Eigen::MatrixXd> Q_max(x_u, ab_.dof(), T_);
  Eigen::Map<Eigen::VectorXd> G_min(g_l, m);
  Eigen::Map<Eigen::VectorXd> G_max(g_u, m);

  for (size_t i = 0; i < ab_.dof(); i++) {
    Q_min.row(i).fill(ab_.rigid_bodies(i).joint().q_min());
    Q_max.row(i).fill(ab_.rigid_bodies(i).joint().q_max());
  }

  G_min.setZero();
  G_max.setZero();

  return true;
}

bool NonlinearProgram::get_starting_point(int n, bool init_x, double* x,
                                          bool init_z, double* z_L, double* z_U,
                                          int m, bool init_lambda, double* lambda) {
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);

  Eigen::Map<Eigen::MatrixXd> Q(x, ab_.dof(), T_);

  for (size_t i = 0; i < ab_.dof(); i++) {
    Q.row(i).fill(q_0_(i));
  }

  return true;
}

bool NonlinearProgram::eval_f(int n, const double* x, bool new_x, double& obj_value) {

  Eigen::Map<const Eigen::MatrixXd> Q(x, ab_.dof(), T_);

  obj_value = 0.;
  for (std::unique_ptr<Objective>& o : objectives_) {
    o->Evaluate(Q, obj_value);
  }

  return true;
}

bool NonlinearProgram::eval_grad_f(int n, const double* x, bool new_x, double* grad_f) {

  Eigen::Map<const Eigen::MatrixXd> Q(x, ab_.dof(), T_);
  Eigen::Map<Eigen::MatrixXd> Grad(grad_f, ab_.dof(), T_);

  Grad.setZero();
  for (std::unique_ptr<Objective>& o : objectives_) {
    o->Gradient(Q, Grad);
  }

  return true;
}

bool NonlinearProgram::eval_g(int n, const double* x, bool new_x, int m, double* g) {

  Eigen::Map<const Eigen::MatrixXd> Q(x, ab_.dof(), T_);

  size_t idx_constraint = 0;
  for (std::unique_ptr<Constraint>& c : constraints_) {
    Eigen::Map<Eigen::VectorXd> g_c(g + idx_constraint, c->num_constraints);
    g_c.setZero();
    c->Evaluate(Q, g_c);

    idx_constraint += c->num_constraints;
  }

  return true;
}

bool NonlinearProgram::eval_jac_g(int n, const double* x, bool new_x,
                                  int m, int nele_jac, int* iRow, int *jCol, double* values) {

  if (x != nullptr) {  // values != nullptr
    Eigen::Map<const Eigen::MatrixXd> Q(x, ab_.dof(), T_);

    size_t idx_jacobian = 0;
    for (std::unique_ptr<Constraint>& c : constraints_) {
      Eigen::Map<Eigen::VectorXd> J_c(values + idx_jacobian, c->len_jacobian);
      J_c.setZero();
      c->Jacobian(Q, J_c);

      idx_jacobian += c->len_jacobian;
    }
  }

  if (iRow != nullptr) {  // jCol != nullptr
    size_t idx_jacobian = 0;
    for (std::unique_ptr<Constraint>& c : constraints_) {
      Eigen::Map<Eigen::ArrayXi> i_c(iRow + idx_jacobian, c->len_jacobian);
      Eigen::Map<Eigen::ArrayXi> j_c(jCol + idx_jacobian, c->len_jacobian);
      i_c.fill(idx_jacobian);
      j_c.setZero();
      c->JacobianIndices(i_c, j_c);

      idx_jacobian += c->len_jacobian;
    }
  }

  return true;
}

bool NonlinearProgram::eval_h(int n, const double* x, bool new_x, double obj_factor,
            int m, const double* lambda, bool new_lambda,
            int nele_hess, int* iRow, int* jCol, double* values) {
  std::cout << "EVAL H?!" << std::endl;
  return false;
}

void NonlinearProgram::finalize_solution(::Ipopt::SolverReturn status,
                                         int n, const double* x, const double* z_L, const double* z_U,
                                         int m, const double* g, const double* lambda,
                                         double obj_value, const ::Ipopt::IpoptData* ip_data,
                                         ::Ipopt::IpoptCalculatedQuantities* ip_cq) {

  switch (status) {
    case ::Ipopt::SUCCESS: status_ = "SUCCESS"; break;
    case ::Ipopt::MAXITER_EXCEEDED: status_ = "MAXITER_EXCEEDED"; break;
    case ::Ipopt::CPUTIME_EXCEEDED: status_ = "CPUTIME_EXCEEDED"; break;
    case ::Ipopt::STOP_AT_TINY_STEP: status_ = "STOP_AT_TINY_STEP"; break;
    case ::Ipopt::STOP_AT_ACCEPTABLE_POINT: status_ = "STOP_AT_ACCEPTABLE_POINT"; break;
    case ::Ipopt::LOCAL_INFEASIBILITY: status_ = "LOCAL_INFEASIBILITY"; break;
    case ::Ipopt::DIVERGING_ITERATES: status_ = "DIVERGING_ITERATES"; break;
    case ::Ipopt::RESTORATION_FAILURE: status_ = "RESTORATION_FAILURE"; break;
    case ::Ipopt::ERROR_IN_STEP_COMPUTATION: status_ = "ERROR_IN_STEP_COMPUTATION"; break;
    case ::Ipopt::INVALID_NUMBER_DETECTED: status_ = "INVALID_NUMBER_DETECTED"; break;
    case ::Ipopt::INTERNAL_ERROR: status_ = "INTERNAL_ERROR"; break;
    default: status_ = "UNKNOWN"; break;
  }

  Eigen::Map<const Eigen::MatrixXd> Q(x, ab_.dof(), T_);

  trajectory_.resize(T_);
  for (size_t t = 0; t < T_; t++) {
    trajectory_[t] = Q.col(t);
  }

  std::cout << status_ << ": " << obj_value << std::endl << std::endl;
}

}  // namespace Ipopt
}  // namespace TrajOpt

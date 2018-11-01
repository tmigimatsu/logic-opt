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

#include <cstring>    // std::memcpy
#include <exception>  // std::runtime_error
#include <iostream>   // std::cout
#include <limits>     // std::numeric_limits
#include <string>     // std::string

namespace TrajOpt {
namespace Ipopt {

class NonlinearProgram : public ::Ipopt::TNLP {

 public:

  NonlinearProgram(const JointVariables& variables, const Objectives& objectives,
                   const Constraints& constraints, std::vector<Eigen::VectorXd>& trajectory_result,
                   OptimizationData* data = nullptr)
      : variables_(variables), objectives_(objectives),
        constraints_(constraints), trajectory_(trajectory_result), data_(data) {}

  virtual bool get_nlp_info(int& n, int& m, int& nnz_jac_g,
                            int& nnz_h_lag, IndexStyleEnum& index_style) override;

  virtual bool get_bounds_info(int n, double* x_l, double* x_u,
                               int m, double* g_l, double* g_u) override;

  virtual bool get_starting_point(int n, bool init_x, double* x,
                                  bool init_z, double* z_L, double* z_U,
                                  int m, bool init_lambda, double* lambda) override;

  virtual bool eval_f(int n, const double* x, bool new_x, double& obj_value) override;

  virtual bool eval_grad_f(int n, const double* x, bool new_x, double* grad_f) override;

  virtual bool eval_g(int n, const double* x, bool new_x, int m, double* g) override;

  virtual bool eval_jac_g(int n, const double* x, bool new_x,
                          int m, int nele_jac, int* iRow, int* jCol, double* values) override;

  virtual bool eval_h(int n, const double* x, bool new_x, double obj_factor,
                      int m, const double* lambda, bool new_lambda,
                      int nele_hess, int* iRow, int* jCol, double* values) override;

  virtual void finalize_solution(::Ipopt::SolverReturn status,
                                 int n, const double* x, const double* z_L, const double* z_U,
                                 int m, const double* g, const double* lambda,
                                 double obj_value, const ::Ipopt::IpoptData* ip_data,
                                 ::Ipopt::IpoptCalculatedQuantities* ip_cq) override;

 private:

  const JointVariables& variables_;
  const Objectives& objectives_;
  const Constraints& constraints_;
  OptimizationData* data_;
  std::vector<Eigen::VectorXd>& trajectory_;

};

std::vector<Eigen::VectorXd> Trajectory(const JointVariables& variables,
                                        const Objectives& objectives,
                                        const Constraints& constraints,
                                        OptimizationData* data) {

  std::vector<Eigen::VectorXd> trajectory_result;
  NonlinearProgram* my_nlp = new NonlinearProgram(variables, objectives, constraints, trajectory_result, data);
  ::Ipopt::SmartPtr<::Ipopt::TNLP> nlp = my_nlp;
  ::Ipopt::SmartPtr<::Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  // Set solver options
  app->Options()->SetStringValue("linear_solver", "ma57");
  // app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  size_t n = variables.T * variables.dof;
  if (data != nullptr && data->x.size() == n && data->z_L.size() == n && data->z_U.size() == n) {
    app->Options()->SetStringValue("warm_start_init_point", "yes");
  }
  app->Options()->SetNumericValue("max_cpu_time", 600.);
  app->Options()->SetIntegerValue("max_iter", 100000);
  // app->Options()->SetNumericValue("acceptable_tol", 1e-6);
  // app->Options()->SetIntegerValue("acceptable_iter", 4);

  ::Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != ::Ipopt::Solve_Succeeded) {
    throw std::runtime_error("JointSpaceTrajectory(): Error during Ipopt initialization.");
  }

  status = app->OptimizeTNLP(nlp);
  std::string str_status;
  if (status != ::Ipopt::Solve_Succeeded) {
    if (str_status == "DIVERGING_ITERATES" ||
        str_status == "RESTORATION_FAILURE" ||
        str_status == "ERROR_IN_STEP_COMPUTATION" ||
        str_status == "INVALID_NUMBER_DETECTED" ||
        str_status == "INTERNAL_ERROR") {
      throw std::runtime_error("JointSpaceTrajectory(): Ipopt optimization failed.");
    }
  }

  return trajectory_result;
}

bool NonlinearProgram::get_nlp_info(int& n, int& m, int& nnz_jac_g,
                                    int& nnz_h_lag, IndexStyleEnum& index_style) {
  n = variables_.dof * variables_.T;

  m = 0;
  nnz_jac_g = 0;
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    m += c->num_constraints;
    nnz_jac_g += c->len_jacobian;
  }

  nnz_h_lag = variables_.dof * variables_.dof * variables_.T +
              2 * (variables_.T - 1) * variables_.dof;

  index_style = TNLP::C_STYLE;

  return true;
}

bool NonlinearProgram::get_bounds_info(int n, double* x_l, double* x_u,
                                       int m, double* g_l, double* g_u) {

  Eigen::Map<Eigen::MatrixXd> Q_min(x_l, variables_.dof, variables_.T);
  Eigen::Map<Eigen::MatrixXd> Q_max(x_u, variables_.dof, variables_.T);

  Q_min.colwise() = variables_.q_min;
  Q_max.colwise() = variables_.q_max;

  size_t idx_constraint = 0;
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    Eigen::Map<Eigen::VectorXd> G_min(g_l + idx_constraint, c->num_constraints);
    Eigen::Map<Eigen::VectorXd> G_max(g_u + idx_constraint, c->num_constraints);

    if (c->type == Constraint::Type::EQUALITY) {
      G_min.setZero();
    } else {
      G_min.fill(-std::numeric_limits<double>::infinity());
    }
    G_max.setZero();

    idx_constraint += c->num_constraints;
  }

  return true;
}

bool NonlinearProgram::get_starting_point(int n, bool init_x, double* x,
                                          bool init_z, double* z_L, double* z_U,
                                          int m, bool init_lambda, double* lambda) {

  if (data_ != nullptr && data_->x.size() == n && data_->z_L.size() == n && data_->z_U.size() == n) {

    // Copy multipliers for warm restart
    if (init_x) {
      std::memcpy(x, &data_->x[0], n * sizeof(double));
    }
    if (init_z) {
      std::memcpy(z_L, &data_->z_L[0], n * sizeof(double));
      std::memcpy(z_U, &data_->z_U[0], n * sizeof(double));
    }

  } else {

    // Initialize variables
    if (init_x) {
      Eigen::Map<Eigen::MatrixXd> Q(x, variables_.dof, variables_.T);
      if (variables_.q_0.cols() == variables_.T) {
        Q = variables_.q_0;
      } else {
        Q.colwise() = variables_.q_0.col(0);
      }
    }
    if (init_z) {
      std::cout << "INIT Z??" << std::endl;
    }

  }

  if (init_lambda) {
    std::cout << "INIT LAMBDA??" << std::endl;
  }

  return true;
}

bool NonlinearProgram::eval_f(int n, const double* x, bool new_x, double& obj_value) {

  Eigen::Map<const Eigen::MatrixXd> Q(x, variables_.dof, variables_.T);

  obj_value = 0.;
  for (const std::unique_ptr<Objective>& o : objectives_) {
    o->Evaluate(Q, obj_value);
  }

  return true;
}

bool NonlinearProgram::eval_grad_f(int n, const double* x, bool new_x, double* grad_f) {

  Eigen::Map<const Eigen::MatrixXd> Q(x, variables_.dof, variables_.T);
  Eigen::Map<Eigen::MatrixXd> Grad(grad_f, variables_.dof, variables_.T);

  Grad.setZero();
  for (const std::unique_ptr<Objective>& o : objectives_) {
    o->Gradient(Q, Grad);
  }

  return true;
}

bool NonlinearProgram::eval_g(int n, const double* x, bool new_x, int m, double* g) {

  Eigen::Map<const Eigen::MatrixXd> Q(x, variables_.dof, variables_.T);

  size_t idx_constraint = 0;
  for (const std::unique_ptr<Constraint>& c : constraints_) {
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
    Eigen::Map<const Eigen::MatrixXd> Q(x, variables_.dof, variables_.T);

    size_t idx_jacobian = 0;
    for (const std::unique_ptr<Constraint>& c : constraints_) {
      Eigen::Map<Eigen::VectorXd> J_c(values + idx_jacobian, c->len_jacobian);
      J_c.setZero();
      c->Jacobian(Q, J_c);

      idx_jacobian += c->len_jacobian;
    }
  }

  if (iRow != nullptr) {  // jCol != nullptr
    size_t idx_jacobian = 0;
    size_t idx_constraint = 0;
    for (const std::unique_ptr<Constraint>& c : constraints_) {
      Eigen::Map<Eigen::ArrayXi> i_c(iRow + idx_jacobian, c->len_jacobian);
      Eigen::Map<Eigen::ArrayXi> j_c(jCol + idx_jacobian, c->len_jacobian);
      i_c.fill(idx_constraint);
      j_c.setZero();
      c->JacobianIndices(i_c, j_c);

      idx_jacobian += c->len_jacobian;
      idx_constraint += c->num_constraints;
    }
  }

  return true;
}

bool NonlinearProgram::eval_h(int n, const double* x, bool new_x, double obj_factor,
                              int m, const double* lambda, bool new_lambda,
                              int nele_hess, int* iRow, int* jCol, double* values) {

  if (x != nullptr) {  // values != nullptr
    Eigen::Map<const Eigen::MatrixXd> Q(x, variables_.dof, variables_.T);
    Eigen::Map<Eigen::VectorXd> H(values, nele_hess);
    H.setZero();

    for (const std::unique_ptr<Objective>& o : objectives_) {
      o->Hessian(Q, obj_factor, H);
    }

    size_t idx_constraint = 0;
    for (const std::unique_ptr<Constraint>& c : constraints_) {
      Eigen::Map<const Eigen::VectorXd> Lambda(lambda + idx_constraint, c->num_constraints);
      c->Hessian(Q, Lambda, H);

      idx_constraint += c->num_constraints;
    }

    // Eigen::MatrixXd Hessian(variables_.dof * variables_.T, variables_.dof * variables_.T);
    // Hessian.setZero();
    // for (size_t i = 0; i < nele_hess; i++) {
    //   Hessian(iRow_[i], jCol_[i]) = H(i);
    //   // Hessian(iRow_[i], jCol_[i]) = std::stoi(std::to_string(iRow_[i]) + std::to_string(jCol_[i]));
    // }
    // std::cout << std::endl << Hessian << std::endl << std::endl;
  }

  if (iRow != nullptr) {  // jCol != nullptr
    size_t idx_hessian = 0;
    for (size_t t = 0; t < variables_.T; t++) {
      // dof x dof block diagonals of Hessian
      Eigen::Map<Eigen::MatrixXi> idx_i(iRow + idx_hessian, variables_.dof, variables_.dof);
      Eigen::Map<Eigen::MatrixXi> idx_j(jCol + idx_hessian, variables_.dof, variables_.dof);

      idx_i.colwise() = Eigen::VectorXi::LinSpaced(variables_.dof, t * variables_.dof,
                                                   (t + 1) * variables_.dof - 1);
      idx_j.rowwise() = Eigen::VectorXi::LinSpaced(variables_.dof, t * variables_.dof,
                                                   (t + 1) * variables_.dof - 1).transpose();

      idx_hessian += variables_.dof * variables_.dof;
    }

    size_t len_offdiag = (variables_.T - 1) * variables_.dof;
    {
      // Upper off-diagonal of Hessian
      Eigen::Map<Eigen::VectorXi> idx_i(iRow + idx_hessian, len_offdiag);
      Eigen::Map<Eigen::VectorXi> idx_j(jCol + idx_hessian, len_offdiag);

      idx_i.setLinSpaced(len_offdiag, 0, len_offdiag - 1);
      idx_j.setLinSpaced(len_offdiag, variables_.dof, variables_.dof + len_offdiag - 1);

      idx_hessian += len_offdiag;
    }
    {
      // Lower off-diagonal of Hessian
      Eigen::Map<Eigen::VectorXi> idx_i(iRow + idx_hessian, len_offdiag);
      Eigen::Map<Eigen::VectorXi> idx_j(jCol + idx_hessian, len_offdiag);

      idx_i.setLinSpaced(len_offdiag, variables_.dof, variables_.dof + len_offdiag - 1);
      idx_j.setLinSpaced(len_offdiag, 0, len_offdiag - 1);
    }
    iRow_.resize(nele_hess);
    jCol_.resize(nele_hess);
    for (size_t i = 0; i < nele_hess; i++) {
      iRow_[i] = iRow[i];
      jCol_[i] = jCol[i];
    }

    // Eigen::MatrixXd H(variables_.dof * variables_.T, variables_.dof * variables_.T);
    // for (size_t i = 0; i < nele_hess; i++) {
    //   H(iRow[i], jCol[i]) = std::stoi(std::to_string(iRow[i]) + std::to_string(jCol[i]));
    // }
    // std::cout << H << std::endl;
  }

  return true;
}

void NonlinearProgram::finalize_solution(::Ipopt::SolverReturn status,
                                         int n, const double* x, const double* z_L, const double* z_U,
                                         int m, const double* g, const double* lambda,
                                         double obj_value, const ::Ipopt::IpoptData* ip_data,
                                         ::Ipopt::IpoptCalculatedQuantities* ip_cq) {

  std::string str_status;
  switch (status) {
    case ::Ipopt::SUCCESS: str_status = "SUCCESS"; break;
    case ::Ipopt::MAXITER_EXCEEDED: str_status = "MAXITER_EXCEEDED"; break;
    case ::Ipopt::CPUTIME_EXCEEDED: str_status = "CPUTIME_EXCEEDED"; break;
    case ::Ipopt::STOP_AT_TINY_STEP: str_status = "STOP_AT_TINY_STEP"; break;
    case ::Ipopt::STOP_AT_ACCEPTABLE_POINT: str_status = "STOP_AT_ACCEPTABLE_POINT"; break;
    case ::Ipopt::LOCAL_INFEASIBILITY: str_status = "LOCAL_INFEASIBILITY"; break;
    case ::Ipopt::DIVERGING_ITERATES: str_status = "DIVERGING_ITERATES"; break;
    case ::Ipopt::RESTORATION_FAILURE: str_status = "RESTORATION_FAILURE"; break;
    case ::Ipopt::ERROR_IN_STEP_COMPUTATION: str_status = "ERROR_IN_STEP_COMPUTATION"; break;
    case ::Ipopt::INVALID_NUMBER_DETECTED: str_status = "INVALID_NUMBER_DETECTED"; break;
    case ::Ipopt::INTERNAL_ERROR: str_status = "INTERNAL_ERROR"; break;
    default: str_status = "UNKNOWN"; break;
  }

  Eigen::Map<const Eigen::MatrixXd> Q(x, variables_.dof, variables_.T);

  trajectory_.resize(variables_.T);
  for (size_t t = 0; t < variables_.T; t++) {
    trajectory_[t] = Q.col(t);
  }

  // Save multipliers for future warm starts
  if (data_ != nullptr) {
    data_->x = std::vector<double>(x, x + n);
    data_->z_L = std::vector<double>(z_L, z_L + n);
    data_->z_U = std::vector<double>(z_U, z_U + n);
  }

  std::cout << str_status << ": " << obj_value << std::endl << std::endl;
}

}  // namespace Ipopt
}  // namespace TrajOpt

/**
 * ipopt.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 25, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/optimization/ipopt.h"

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#include <csignal>    // std::sig_atomic_t
#include <cstring>    // std::memcpy
#include <exception>  // std::runtime_error
#include <iostream>   // std::cout
#include <limits>     // std::numeric_limits
#include <vector>     // std::vector

namespace {

volatile std::sig_atomic_t g_runloop = true;

}  // namespace

namespace LogicOpt {

Ipopt::Ipopt(const YAML::Node& options) {
  if (!options.IsMap()) throw std::invalid_argument("Ipopt(): Invalid YAML options.");

  if (options["derivative_test"]) options_.derivative_test = options["derivative_test"].as<bool>();
  if (options["use_hessian"]) options_.use_hessian         = options["use_hessian"].as<bool>();
  if (options["max_cpu_time"]) options_.max_cpu_time       = options["max_cpu_time"].as<double>();
  if (options["max_iter"]) options_.max_iter               = options["max_iter"].as<size_t>();
  if (options["acceptable_tol"]) options_.acceptable_tol   = options["acceptable_tol"].as<double>();
  if (options["acceptable_iter"]) options_.acceptable_iter = options["acceptable_iter"].as<size_t>();
}

void Ipopt::Terminate() {
  g_runloop = false;
}

class IpoptNonlinearProgram : public ::Ipopt::TNLP {

 public:

  IpoptNonlinearProgram(const JointVariables& variables, const Objectives& objectives,
                        const Constraints& constraints, Eigen::MatrixXd& trajectory_result,
                        Ipopt::OptimizationData* data = nullptr)
      : variables_(variables), objectives_(objectives),
        constraints_(constraints), trajectory_(trajectory_result), data_(data) {
    ConstructHessian();
  }

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

  virtual bool intermediate_callback(::Ipopt::AlgorithmMode mode, int iter, double obj_value,
                                     double inf_pr, double inf_du, double mu, double d_norm,
                                     double regularization_size, double alpha_du, double alpha_pr,
                                     int ls_trials, const ::Ipopt::IpoptData* ip_data,
                                     ::Ipopt::IpoptCalculatedQuantities* ip_cq) override;

  void OpenLogger(const std::string& filepath);
  void CloseLogger();

 private:

  void ConstructHessian();

  const JointVariables& variables_;
  const Objectives& objectives_;
  const Constraints& constraints_;
  Eigen::SparseMatrix<bool> H_;

  Ipopt::OptimizationData* data_;
  Eigen::MatrixXd& trajectory_;

};

Eigen::MatrixXd Ipopt::Trajectory(const JointVariables& variables, const Objectives& objectives,
                                  const Constraints& constraints, Optimizer::OptimizationData* data) {

  Eigen::MatrixXd trajectory_result;
  Ipopt::OptimizationData* ipopt_data = dynamic_cast<Ipopt::OptimizationData*>(data);
  IpoptNonlinearProgram* my_nlp = new IpoptNonlinearProgram(variables, objectives, constraints,
                                                            trajectory_result, ipopt_data);
  if (!options_.logdir.empty()) {
    my_nlp->OpenLogger(options_.logdir);
  }
  ::Ipopt::SmartPtr<::Ipopt::TNLP> nlp = my_nlp;
  ::Ipopt::SmartPtr<::Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  // Set solver options
  app->Options()->SetStringValue("linear_solver", "ma57");
  if (!options_.use_hessian) {
    app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  }
  if (options_.derivative_test) {
    app->Options()->SetStringValue("derivative_test", "second-order");
  }
  size_t n = variables.dof * variables.T;
  if (ipopt_data != nullptr && ipopt_data->x.size() == n && ipopt_data->z_L.size() == n &&
      ipopt_data->z_U.size() == n) {
    app->Options()->SetStringValue("warm_start_init_point", "yes");
  }
  app->Options()->SetNumericValue("max_cpu_time", options_.max_cpu_time);
  app->Options()->SetIntegerValue("max_iter", options_.max_iter);
  app->Options()->SetNumericValue("acceptable_tol", options_.acceptable_tol);
  app->Options()->SetIntegerValue("acceptable_iter", options_.acceptable_iter);

  ::Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != ::Ipopt::Solve_Succeeded) {
    throw std::runtime_error("JointSpaceTrajectory(): Error during Ipopt initialization.");
  }

  status = app->OptimizeTNLP(nlp);
  my_nlp->CloseLogger();

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

bool IpoptNonlinearProgram::get_nlp_info(int& n, int& m, int& nnz_jac_g,
                                         int& nnz_h_lag, IndexStyleEnum& index_style) {
  n = variables_.dof * variables_.T;

  m = 0;
  nnz_jac_g = 0;
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    m += c->num_constraints();
    nnz_jac_g += c->len_jacobian();
  }

  nnz_h_lag = H_.nonZeros();

  index_style = TNLP::C_STYLE;

  return true;
}

bool IpoptNonlinearProgram::get_bounds_info(int n, double* x_l, double* x_u,
                                            int m, double* g_l, double* g_u) {

  Eigen::Map<Eigen::MatrixXd> Q_min(x_l, variables_.dof, variables_.T);
  Eigen::Map<Eigen::MatrixXd> Q_max(x_u, variables_.dof, variables_.T);

  Q_min.colwise() = variables_.q_min;
  Q_max.colwise() = variables_.q_max;

  size_t idx_constraint = 0;
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    Eigen::Map<Eigen::VectorXd> G_min(g_l + idx_constraint, c->num_constraints());
    Eigen::Map<Eigen::VectorXd> G_max(g_u + idx_constraint, c->num_constraints());

    for (size_t i = 0; i < c->num_constraints(); i++) {
      G_min(i) = c->constraint_type(i) == Constraint::Type::EQUALITY ? 0. : -std::numeric_limits<double>::infinity();
    }
    G_max.setZero();

    idx_constraint += c->num_constraints();
  }

  return true;
}

bool IpoptNonlinearProgram::get_starting_point(int n, bool init_x, double* x,
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

bool IpoptNonlinearProgram::eval_f(int n, const double* x, bool new_x, double& obj_value) {

  Eigen::Map<const Eigen::MatrixXd> Q(x, variables_.dof, variables_.T);

  obj_value = 0.;
  for (const std::unique_ptr<Objective>& o : objectives_) {
    o->Evaluate(Q, obj_value);
  }

  return true;
}

bool IpoptNonlinearProgram::eval_grad_f(int n, const double* x, bool new_x, double* grad_f) {

  Eigen::Map<const Eigen::MatrixXd> Q(x, variables_.dof, variables_.T);
  Eigen::Map<Eigen::MatrixXd> Grad(grad_f, variables_.dof, variables_.T);

  Grad.setZero();
  for (const std::unique_ptr<Objective>& o : objectives_) {
    o->Gradient(Q, Grad);
  }

  return true;
}

bool IpoptNonlinearProgram::eval_g(int n, const double* x, bool new_x, int m, double* g) {

  Eigen::Map<const Eigen::MatrixXd> Q(x, variables_.dof, variables_.T);

  size_t idx_constraint = 0;
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    Eigen::Map<Eigen::VectorXd> g_c(g + idx_constraint, c->num_constraints());
    g_c.setZero();
    c->Evaluate(Q, g_c);

    idx_constraint += c->num_constraints();
  }

  return true;
}

bool IpoptNonlinearProgram::eval_jac_g(int n, const double* x, bool new_x,
                                  int m, int nele_jac, int* iRow, int *jCol, double* values) {

  if (x != nullptr) {  // values != nullptr
    Eigen::Map<const Eigen::MatrixXd> Q(x, variables_.dof, variables_.T);

    size_t idx_jacobian = 0;
    for (const std::unique_ptr<Constraint>& c : constraints_) {
      Eigen::Map<Eigen::VectorXd> J_c(values + idx_jacobian, c->len_jacobian());
      J_c.setZero();
      c->Jacobian(Q, J_c);

      idx_jacobian += c->len_jacobian();
    }
  }

  if (iRow != nullptr) {  // jCol != nullptr
    size_t idx_jacobian = 0;
    size_t idx_constraint = 0;
    for (const std::unique_ptr<Constraint>& c : constraints_) {
      Eigen::Map<Eigen::ArrayXi> i_c(iRow + idx_jacobian, c->len_jacobian());
      Eigen::Map<Eigen::ArrayXi> j_c(jCol + idx_jacobian, c->len_jacobian());
      i_c.fill(idx_constraint);
      j_c.setZero();
      c->JacobianIndices(i_c, j_c);

      idx_jacobian += c->len_jacobian();
      idx_constraint += c->num_constraints();
    }
  }

  return true;
}

void IpoptNonlinearProgram::ConstructHessian() {
  H_.resize(variables_.dof * variables_.T, variables_.dof * variables_.T);
  for (const std::unique_ptr<Objective>& o : objectives_) {
    o->HessianStructure(H_, variables_.T);
  }
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    c->HessianStructure(H_);
  }
  H_.makeCompressed();
}

bool IpoptNonlinearProgram::eval_h(int n, const double* x, bool new_x, double obj_factor,
                                   int m, const double* lambda, bool new_lambda,
                                   int nele_hess, int* iRow, int* jCol, double* values) {

  if (x != nullptr) {  // values != nullptr
    Eigen::Map<const Eigen::MatrixXd> Q(x, variables_.dof, variables_.T);
    Eigen::Map<Eigen::VectorXd> H_vec(values, nele_hess);
    H_vec.setZero();
    Eigen::Map<Eigen::SparseMatrix<double>> H(variables_.dof * variables_.T,
                                              variables_.dof * variables_.T,
                                              nele_hess,
                                              H_.outerIndexPtr(), H_.innerIndexPtr(), values);

    // Compute objective Hessians only if obj_factor is nonzero
    if (obj_factor != 0.) {
      for (const std::unique_ptr<Objective>& o : objectives_) {
        o->Hessian(Q, obj_factor, H);
      }
    }

    size_t idx_constraint = 0;
    for (const std::unique_ptr<Constraint>& c : constraints_) {
      Eigen::Map<const Eigen::VectorXd> Lambda(lambda + idx_constraint, c->num_constraints());

      // Skip Hessian computation if lambda for this constraint is 0
      if ((Lambda.array() == 0.).all()) continue;

      c->Hessian(Q, Lambda, H);

      idx_constraint += c->num_constraints();
    }
  }

  if (iRow != nullptr) {  // jCol != nullptr

    // Convert compressed col indices format to explicit col indices
    size_t idx_hessian = 0;
    for (size_t j = 0; j < variables_.dof * variables_.T; j++) {
      size_t nnz = H_.outerIndexPtr()[j+1] - H_.outerIndexPtr()[j];
      Eigen::Map<Eigen::VectorXi> idx_j(jCol + idx_hessian, nnz);
      idx_j.fill(j);
      idx_hessian += nnz;
    }

    // Copy row indices
    std::memcpy(iRow, H_.innerIndexPtr(), sizeof(int) * nele_hess);

  }

  return true;
}

void IpoptNonlinearProgram::finalize_solution(::Ipopt::SolverReturn status,
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

  trajectory_ = Q;

  // Save multipliers for future warm starts
  if (data_ != nullptr) {
    data_->x = std::vector<double>(x, x + n);
    data_->z_L = std::vector<double>(z_L, z_L + n);
    data_->z_U = std::vector<double>(z_U, z_U + n);
  }

  Eigen::Map<const Eigen::VectorXd> Lambda(lambda, m);

  std::cout << str_status << ": " << obj_value << std::endl << std::endl;
  std::cout << "lambda: " << Lambda.transpose() << std::endl << std::endl;
}

bool IpoptNonlinearProgram::intermediate_callback(::Ipopt::AlgorithmMode mode, int iter, double obj_value,
                                                  double inf_pr, double inf_du, double mu, double d_norm,
                                                  double regularization_size, double alpha_du, double alpha_pr,
                                                  int ls_trials, const ::Ipopt::IpoptData* ip_data,
                                                  ::Ipopt::IpoptCalculatedQuantities* ip_cq) {
  return g_runloop;
}


void IpoptNonlinearProgram::OpenLogger(const std::string& filepath) {
  for (const std::unique_ptr<Objective>& o : objectives_) {
    o->log.open(filepath + o->name + ".log");
  }
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    c->log.open(filepath + c->name + ".log");
  }
}

void IpoptNonlinearProgram::CloseLogger() {
  for (const std::unique_ptr<Objective>& o : objectives_) {
    o->log.close();
  }
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    c->log.close();
  }
}

}  // namespace LogicOpt

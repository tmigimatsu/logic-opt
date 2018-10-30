/**
 * nlopt.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 29, 2018
 * Authors: Toki Migimatsu
 */

#include "nlopt.h"

#include <nlopt.hpp>

#include <functional>  // std::function

namespace TrajOpt {
namespace Nlopt {

struct NonlinearProgram {

  NonlinearProgram(const JointVariables& variables, const Objectives& objectives,
                   const Constraints& constraints, OptimizationData* data = nullptr)
      : variables(variables), objectives(objectives), constraints(constraints),
        constraint_gradient_map(constraints.size()) {}

  const JointVariables& variables;
  const Objectives& objectives;
  const Constraints& constraints;
  std::vector<Eigen::ArrayXi> constraint_gradient_map;

};

nlopt::vfunc CompileObjectives() {
  return [](const std::vector<double>& x, std::vector<double>& grad, void* data) -> double {
    NonlinearProgram& nlp = *reinterpret_cast<NonlinearProgram*>(data);

    Eigen::Map<const Eigen::MatrixXd> Q(&x[0], nlp.variables.dof, nlp.variables.T);

    double obj = 0.;
    for (const std::unique_ptr<Objective>& objective : nlp.objectives) {
      objective->Evaluate(Q, obj);
    }

    if (!grad.empty()) {
      Eigen::Map<Eigen::MatrixXd> Gradient(&grad[0], nlp.variables.dof, nlp.variables.T);
      Gradient.setZero();

      for (const std::unique_ptr<Objective>& objective : nlp.objectives) {
        objective->Gradient(Q, Gradient);
      }
    }

    return obj;
  };
}

nlopt::vfunc CompileConstraint(NonlinearProgram& nlp, size_t idx_constraint) {
  const std::unique_ptr<Constraint>& constraint = nlp.constraints[idx_constraint];
  Eigen::ArrayXi idx_i = Eigen::ArrayXi::Zero(constraint->len_jacobian);
  nlp.constraint_gradient_map[idx_constraint] = Eigen::ArrayXi::Zero(constraint->len_jacobian);
  constraint->JacobianIndices(idx_i, nlp.constraint_gradient_map[idx_constraint]);

  return [](const std::vector<double>& x, std::vector<double>& grad, void* data) {

    std::pair<NonlinearProgram*, size_t>& nlp_i = *reinterpret_cast<std::pair<NonlinearProgram*, size_t>*>(data);
    NonlinearProgram& nlp = *nlp_i.first;
    size_t& idx_constraint = nlp_i.second;
    const std::unique_ptr<Constraint>& constraint = nlp.constraints[idx_constraint];
    Eigen::ArrayXi& idx_j = nlp.constraint_gradient_map[idx_constraint];

    Eigen::Map<const Eigen::MatrixXd> Q(&x[0], nlp.variables.dof, nlp.variables.T);
    Eigen::Map<Eigen::VectorXd> Gradient(&grad[0], grad.size());

    Eigen::VectorXd g = Eigen::VectorXd::Zero(constraint->num_constraints);
    constraint->Evaluate(Q, g);

    Eigen::VectorXd Jacobian = Eigen::VectorXd::Zero(constraint->len_jacobian);
    constraint->Jacobian(Q, Jacobian);

    Gradient.setZero();
    for (size_t i = 0; i < constraint->len_jacobian; i++) {
      Gradient(idx_j[i]) += Jacobian(i);
    }

    return g.sum();
  };
}

std::vector<Eigen::VectorXd> Trajectory(const JointVariables& variables,
                                        const Objectives& objectives,
                                        const Constraints& constraints,
                                        OptimizationData* data) {

  NonlinearProgram nlp(variables, objectives, constraints, data);
  nlopt::opt opt(nlopt::algorithm::LD_SLSQP, variables.dof * variables.T);
  // nlopt::opt opt(nlopt::algorithm::AUGLAG, ab.dof() * T);
  // nlopt::opt local_opt(nlopt::algorithm::LD_SLSQP, ab.dof() * T);
  // local_opt.set_ftol_abs(0.001);
  // opt.set_local_optimizer(local_opt);

  // Objective
  opt.set_min_objective(CompileObjectives(), &nlp);

  // Constraints
  std::vector<nlopt::vfunc> nlopt_constraints;
  std::vector<std::pair<NonlinearProgram*, size_t>> nlopt_constraint_data;
  nlopt_constraints.reserve(nlp.constraints.size());
  for (size_t i = 0; i < nlp.constraints.size(); i++) {
    nlopt_constraints.push_back(CompileConstraint(nlp, i));
    nlopt_constraint_data.emplace_back(&nlp, i);
  }
  for (size_t i = 0; i < nlp.constraints.size(); i++) {
    opt.add_equality_constraint(nlopt_constraints[i], &nlopt_constraint_data[i]);
  }

  // Joint limits
  std::vector<double> q_min(variables.dof * variables.T);
  std::vector<double> q_max(variables.dof * variables.T);
  Eigen::Map<Eigen::MatrixXd> Q_min(&q_min[0], variables.dof, variables.T);
  Eigen::Map<Eigen::MatrixXd> Q_max(&q_max[0], variables.dof, variables.T);
  Q_min.colwise() = variables.q_min;
  Q_max.colwise() = variables.q_max;
  opt.set_lower_bounds(q_min);
  opt.set_upper_bounds(q_max);

  opt.set_xtol_abs(0.001);

  // nlopt::opt local_opt(nlopt::algorithm::LD_MMA, ab.dof() * T);
  // local_opt.set_xtol_abs(0.0001);
  // opt.set_local_optimizer(local_opt);

  // Variable initialization
  std::vector<double> local_opt_vars;
  std::vector<double>& opt_vars = (data != nullptr) ? *data : local_opt_vars;
  if (opt_vars.size() != variables.dof * variables.T) {
    opt_vars.resize(variables.dof * variables.T);
    Eigen::Map<Eigen::MatrixXd> Q_0(&opt_vars[0], variables.dof, variables.T);
    Q_0.colwise() = variables.q_0;
  }

  // Optimize
  double opt_val;
  nlopt::result result = opt.optimize(opt_vars, opt_val);

  // opt.set_min_objective(TaskVelocityObjective, &data);
  // opt.add_inequality_constraint(AboveTableConstraint, &data);
  // opt.set_xtol_abs(0.000001);
  switch (result) {
    case nlopt::SUCCESS: std::cout << "SUCCESS" << std::endl; break;
    case nlopt::STOPVAL_REACHED: std::cout << "STOPVAL_REACHED" << std::endl; break;
    case nlopt::FTOL_REACHED: std::cout << "FTOL_REACHED" << std::endl; break;
    case nlopt::XTOL_REACHED: std::cout << "XTOL_REACHED" << std::endl; break;
    case nlopt::MAXEVAL_REACHED: std::cout << "MAXEVAL_REACHED" << std::endl; break;
    case nlopt::MAXTIME_REACHED: std::cout << "MAXTIME_REACHED" << std::endl; break;
    default: std::cout << "UNKNOWN" << std::endl;
  }

  Eigen::Map<Eigen::MatrixXd> Q(&opt_vars[0], variables.dof, variables.T);
  std::vector<Eigen::VectorXd> q_des_traj(variables.T);
  for (size_t t = 0; t < variables.T; t++) {
    q_des_traj[t] = Q.col(t);
  }
  return q_des_traj;
}

} // namespace Nlopt
} // namespace TrajOpt

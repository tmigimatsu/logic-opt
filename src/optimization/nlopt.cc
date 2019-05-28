/**
 * nlopt.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 29, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/optimization/nlopt.h"

#include <nlopt.hpp>

#include <fstream>     // std::ofstream
#include <functional>  // std::function
#include <iostream>    // std::cout
#include <limits>      // std::numeric_limits
#include <vector>      // std::vector

namespace logic_opt {

struct NloptNonlinearProgram {

  NloptNonlinearProgram(const Variables& variables, const Objectives& objectives,
                        const Constraints& constraints)
      : variables(variables), objectives(objectives), constraints(constraints),
        constraint_gradient_map(constraints.size()),
        constraint_cache(constraints.size()) {}

  void OpenLogger(const std::string& filepath);
  void CloseLogger();

  const Variables& variables;
  const Objectives& objectives;
  const Constraints& constraints;
  std::vector<Eigen::ArrayXi> constraint_gradient_map;

  struct ConstraintCache {
    Eigen::MatrixXd X;
    Eigen::VectorXd constraint;
    Eigen::VectorXd Jacobian;
    Eigen::ArrayXi idx_i;
    Eigen::ArrayXi idx_j;
  };
  std::vector<ConstraintCache> constraint_cache;

  struct ConstraintData {
    ConstraintData(NloptNonlinearProgram& nlp, size_t idx_constraint, size_t idx_vector)
        : nlp(nlp), idx_constraint(idx_constraint), idx_vector(idx_vector) {}

    NloptNonlinearProgram& nlp;
    size_t idx_constraint;
    size_t idx_vector;
  };

  // std::ofstream log_vars_;

};

nlopt::vfunc CompileObjectives() {
  return [](const std::vector<double>& x, std::vector<double>& grad, void* data) -> double {
    NloptNonlinearProgram& nlp = *reinterpret_cast<NloptNonlinearProgram*>(data);

    Eigen::Map<const Eigen::MatrixXd> X(&x[0], nlp.variables.dof, nlp.variables.T);

    // if (nlp.log_vars_.is_open()) {
    //   Eigen::Map<const Eigen::VectorXd> X_vec(&x[0], X.size());
    //   nlp.log_vars_ << X_vec.transpose() << std::endl;
    // }

    double obj = 0.;
    for (const std::unique_ptr<Objective>& objective : nlp.objectives) {
      objective->Evaluate(X, obj);
    }

    if (!grad.empty()) {
      Eigen::Map<Eigen::MatrixXd> Gradient(&grad[0], nlp.variables.dof, nlp.variables.T);
      Gradient.setZero();

      for (const std::unique_ptr<Objective>& objective : nlp.objectives) {
        objective->Gradient(X, Gradient);
      }
    }

    return obj;
  };
}

nlopt::vfunc CompileConstraint(NloptNonlinearProgram& nlp, size_t idx_constraint) {
  const std::unique_ptr<Constraint>& constraint = nlp.constraints[idx_constraint];
  Eigen::ArrayXi idx_i = Eigen::ArrayXi::Zero(constraint->len_jacobian());
  nlp.constraint_gradient_map[idx_constraint] = Eigen::ArrayXi::Zero(constraint->len_jacobian());
  constraint->JacobianIndices(idx_i, nlp.constraint_gradient_map[idx_constraint]);

  // Capturing variables is not allowed since nlopt::vfunc is a C function pointer
  return [](const std::vector<double>& x, std::vector<double>& grad, void* data) {

    NloptNonlinearProgram::ConstraintData& constraint_data = *reinterpret_cast<NloptNonlinearProgram::ConstraintData*>(data);
    NloptNonlinearProgram& nlp = constraint_data.nlp;
    const size_t& idx_constraint = constraint_data.idx_constraint;

    const std::unique_ptr<Constraint>& constraint = nlp.constraints[idx_constraint];
    Eigen::ArrayXi& idx_j = nlp.constraint_gradient_map[idx_constraint];

    Eigen::Map<const Eigen::MatrixXd> X(&x[0], nlp.variables.dof, nlp.variables.T);

    Eigen::VectorXd g = Eigen::VectorXd::Zero(constraint->num_constraints());
    constraint->Evaluate(X, g);

    if (!grad.empty()) {
      Eigen::Map<Eigen::VectorXd> Gradient(&grad[0], grad.size());

      Eigen::VectorXd Jacobian = Eigen::VectorXd::Zero(constraint->len_jacobian());
      constraint->Jacobian(X, Jacobian);

      Gradient.setZero();
      for (size_t i = 0; i < constraint->len_jacobian(); i++) {
        Gradient(idx_j[i]) += Jacobian(i);
      }
    }

    return g.sum();
  };
}

void CompileConstraintVector(NloptNonlinearProgram& nlp, size_t idx_constraint,
                             std::vector<nlopt::vfunc>& nlopt_constraints,
                             std::vector<NloptNonlinearProgram::ConstraintData>& nlopt_constraint_data) {
  const std::unique_ptr<Constraint>& constraint = nlp.constraints[idx_constraint];
  NloptNonlinearProgram::ConstraintCache& constraint_cache = nlp.constraint_cache[idx_constraint];

  constraint_cache.X.resize(nlp.variables.dof, nlp.variables.T);
  constraint_cache.X.fill(std::numeric_limits<double>::infinity());
  constraint_cache.constraint = Eigen::VectorXd::Zero(constraint->num_constraints());
  constraint_cache.Jacobian = Eigen::VectorXd::Zero(constraint->len_jacobian());
  constraint_cache.idx_i = Eigen::ArrayXi::Zero(constraint->len_jacobian());
  constraint_cache.idx_j = Eigen::ArrayXi::Zero(constraint->len_jacobian());
  constraint->JacobianIndices(constraint_cache.idx_i, constraint_cache.idx_j);

  for (size_t i = 0; i < constraint->num_constraints(); i++) {

    nlopt_constraints.push_back([](const std::vector<double>& x, std::vector<double>& grad, void* data) -> double {
      NloptNonlinearProgram::ConstraintData& constraint_data = *reinterpret_cast<NloptNonlinearProgram::ConstraintData*>(data);
      NloptNonlinearProgram& nlp = constraint_data.nlp;
      const size_t& idx_constraint = constraint_data.idx_constraint;
      const size_t& idx_vector = constraint_data.idx_vector;

      const std::unique_ptr<Constraint>& constraint = nlp.constraints[idx_constraint];
      NloptNonlinearProgram::ConstraintCache& constraint_cache = nlp.constraint_cache[idx_constraint];

      Eigen::Map<const Eigen::MatrixXd> X(&x[0], nlp.variables.dof, nlp.variables.T);
      if (X != constraint_cache.X) {
        constraint_cache.X = X;
        constraint_cache.constraint.setZero();
        constraint_cache.Jacobian.setZero();
        constraint->Evaluate(constraint_cache.X, constraint_cache.constraint);
        constraint->Jacobian(constraint_cache.X, constraint_cache.Jacobian);
      }

      if (!grad.empty()) {
        Eigen::Map<Eigen::VectorXd> Gradient(&grad[0], grad.size());
        Gradient.setZero();

        for (size_t i = 0; i < constraint->len_jacobian(); i++) {
          if (constraint_cache.idx_i[i] != idx_vector) continue;
          Gradient(constraint_cache.idx_j[i]) += constraint_cache.Jacobian(i);
        }
      }

      return constraint_cache.constraint(idx_vector);
    });

    nlopt_constraint_data.emplace_back(nlp, idx_constraint, i);
  }
}

Eigen::MatrixXd Nlopt::Trajectory(const Variables& variables, const Objectives& objectives,
                                  const Constraints& constraints,
                                  Optimizer::OptimizationData* data,
                                  const std::function<void(int, const Eigen::MatrixXd&)>& iteration_callback) {

  NloptNonlinearProgram nlp(variables, objectives, constraints);
  if (!options_.logdir.empty()) {
    nlp.OpenLogger(options_.logdir);
  }
  // nlopt::opt opt(nlopt::algorithm::LD_SLSQP, variables.dof * variables.T);
  // nlopt::opt opt(nlopt::algorithm::LD_MMA, variables.dof * variables.T);
  nlopt::opt opt(nlopt::algorithm::AUGLAG, variables.dof * variables.T);
  nlopt::opt local_opt(nlopt::algorithm::LD_MMA, variables.dof * variables.T);
  // nlopt::opt local_opt(nlopt::algorithm::LD_TNEWTON_PRECOND, variables.dof * variables.T);
  local_opt.set_xtol_abs(0.01);
  opt.set_local_optimizer(local_opt);

  // Objective
  opt.set_min_objective(CompileObjectives(), &nlp);

  // Compile constraints
  std::vector<nlopt::vfunc> nlopt_constraints;
  std::vector<NloptNonlinearProgram::ConstraintData> nlopt_constraint_data;

  // Find total number of constraints
  size_t num_nlopt_constraints = 0;
  for (const std::unique_ptr<Constraint>& c : constraints) {
    num_nlopt_constraints += c->num_constraints();
  }
  nlopt_constraints.reserve(num_nlopt_constraints);
  nlopt_constraint_data.reserve(num_nlopt_constraints);

  for (size_t i = 0; i < nlp.constraints.size(); i++) {
    // Append vector of constraint lambda functions
    CompileConstraintVector(nlp, i, nlopt_constraints, nlopt_constraint_data);
  }
  
  // Add constraints
  const double kTolerance = 1e-10;
  for (size_t i = 0; i < num_nlopt_constraints; i++) {
    const size_t& idx_constraint = nlopt_constraint_data[i].idx_constraint;
    const size_t& idx_vector = nlopt_constraint_data[i].idx_vector;
    if (constraints[idx_constraint]->constraint_type(idx_vector) == Constraint::Type::kEquality) {
      opt.add_equality_constraint(nlopt_constraints[i], &nlopt_constraint_data[i], kTolerance);
    } else {
      opt.add_inequality_constraint(nlopt_constraints[i], &nlopt_constraint_data[i], kTolerance);
    }
  }

  // Joint limits
  std::vector<double> q_min(variables.dof * variables.T);
  std::vector<double> q_max(variables.dof * variables.T);
  Eigen::Map<Eigen::MatrixXd> X_min(&q_min[0], variables.dof, variables.T);
  Eigen::Map<Eigen::MatrixXd> X_max(&q_max[0], variables.dof, variables.T);
  X_min.colwise() = variables.x_min;
  X_max.colwise() = variables.x_max;
  opt.set_lower_bounds(q_min);
  opt.set_upper_bounds(q_max);

  opt.set_xtol_abs(0.0001);
  opt.set_maxtime(60);

  // Variable initialization
  std::vector<double> local_opt_vars;
  OptimizationData* nlopt_data = dynamic_cast<OptimizationData*>(data);
  std::vector<double>& opt_vars = (nlopt_data != nullptr) ? nlopt_data->vars : local_opt_vars;
  if (opt_vars.size() != variables.dof * variables.T) {
    opt_vars.resize(variables.dof * variables.T);
    Eigen::Map<Eigen::MatrixXd> X_0(&opt_vars[0], variables.dof, variables.T);
    if (variables.X_0.cols() == variables.T) {
      X_0 = variables.X_0;
    } else {
      X_0.colwise() = variables.X_0.col(0);
    }
  }

  // Optimize
  double opt_val;
  nlopt::result result;
  try {
    result = opt.optimize(opt_vars, opt_val);
  } catch (const std::exception& e) {
    std::cout << "NLopt Error: " << e.what() << std::endl;
  }
  nlp.CloseLogger();

  std::string str_status;
  switch (result) {
    case nlopt::SUCCESS: str_status = "SUCCESS"; break;
    case nlopt::STOPVAL_REACHED: str_status = "STOPVAL_REACHED"; break;
    case nlopt::FTOL_REACHED: str_status = "FTOL_REACHED"; break;
    case nlopt::XTOL_REACHED: str_status = "XTOL_REACHED"; break;
    case nlopt::MAXEVAL_REACHED: str_status = "MAXEVAL_REACHED"; break;
    case nlopt::MAXTIME_REACHED: str_status = "MAXTIME_REACHED"; break;
    default: str_status = "UNKNOWN";
  }

  Eigen::Map<Eigen::MatrixXd> X(&opt_vars[0], variables.dof, variables.T);
  Eigen::MatrixXd q_des_traj = X;

  std::cout << str_status << ": " << opt_val << std::endl << std::endl;
  return q_des_traj;
}

void NloptNonlinearProgram::OpenLogger(const std::string& filepath) {
  // log_vars_.open(filepath + "vars.log");
  // for (const std::unique_ptr<Objective>& o : objectives) {
  //   o->log_objective.open(filepath + o->name + "_objective.log");
  //   o->log_gradient.open(filepath + o->name + "_gradient.log");
  // }
  // for (const std::unique_ptr<Constraint>& c : constraints) {
  //   c->log_constraint.open(filepath + c->name + "_constraint.log");
  //   c->log_jacobian.open(filepath + c->name + "_jacobian.log");
  // }
}

void NloptNonlinearProgram::CloseLogger() {
  // log_vars_.close();
  // for (const std::unique_ptr<Objective>& o : objectives) {
  //   o->log_objective.close();
  //   o->log_gradient.close();
  // }
  // for (const std::unique_ptr<Constraint>& c : constraints) {
  //   c->log_constraint.close();
  //   c->log_jacobian.close();
  // }
}

} // namespace logic_opt

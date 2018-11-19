/**
 * nlopt.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 29, 2018
 * Authors: Toki Migimatsu
 */

#include "TrajOpt/nlopt.h"

#include <nlopt.hpp>

#include <functional>  // std::function
#include <limits>      // std::numeric_limits
#include <vector>      // std::vector

namespace TrajOpt {
namespace Nlopt {

struct NonlinearProgram {

  NonlinearProgram(const JointVariables& variables, const Objectives& objectives,
                   const Constraints& constraints, OptimizationData* data = nullptr)
      : variables(variables), objectives(objectives), constraints(constraints),
        constraint_gradient_map(constraints.size()),
        constraint_cache(constraints.size()) {}

  void OpenLogger(const std::string& filepath);
  void CloseLogger();

  const JointVariables& variables;
  const Objectives& objectives;
  const Constraints& constraints;
  std::vector<Eigen::ArrayXi> constraint_gradient_map;

  struct ConstraintCache {
    Eigen::MatrixXd Q;
    Eigen::VectorXd constraint;
    Eigen::VectorXd Jacobian;
    Eigen::ArrayXi idx_i;
    Eigen::ArrayXi idx_j;
  };
  std::vector<ConstraintCache> constraint_cache;

  struct ConstraintData {
    ConstraintData(NonlinearProgram& nlp, size_t idx_constraint, size_t idx_vector)
        : nlp(nlp), idx_constraint(idx_constraint), idx_vector(idx_vector) {}

    NonlinearProgram& nlp;
    size_t idx_constraint;
    size_t idx_vector;
  };

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
  Eigen::ArrayXi idx_i = Eigen::ArrayXi::Zero(constraint->len_jacobian());
  nlp.constraint_gradient_map[idx_constraint] = Eigen::ArrayXi::Zero(constraint->len_jacobian());
  constraint->JacobianIndices(idx_i, nlp.constraint_gradient_map[idx_constraint]);

  // Capturing variables is not allowed since nlopt::vfunc is a C function pointer
  return [](const std::vector<double>& x, std::vector<double>& grad, void* data) {

    NonlinearProgram::ConstraintData& constraint_data = *reinterpret_cast<NonlinearProgram::ConstraintData*>(data);
    NonlinearProgram& nlp = constraint_data.nlp;
    const size_t& idx_constraint = constraint_data.idx_constraint;

    const std::unique_ptr<Constraint>& constraint = nlp.constraints[idx_constraint];
    Eigen::ArrayXi& idx_j = nlp.constraint_gradient_map[idx_constraint];

    Eigen::Map<const Eigen::MatrixXd> Q(&x[0], nlp.variables.dof, nlp.variables.T);

    Eigen::VectorXd g = Eigen::VectorXd::Zero(constraint->num_constraints());
    constraint->Evaluate(Q, g);

    if (!grad.empty()) {
      Eigen::Map<Eigen::VectorXd> Gradient(&grad[0], grad.size());

      Eigen::VectorXd Jacobian = Eigen::VectorXd::Zero(constraint->len_jacobian());
      constraint->Jacobian(Q, Jacobian);

      Gradient.setZero();
      for (size_t i = 0; i < constraint->len_jacobian(); i++) {
        Gradient(idx_j[i]) += Jacobian(i);
      }
    }

    return g.sum();
  };
}

void CompileConstraintVector(NonlinearProgram& nlp, size_t idx_constraint,
                             std::vector<nlopt::vfunc>& nlopt_constraints,
                             std::vector<NonlinearProgram::ConstraintData>& nlopt_constraint_data) {
  const std::unique_ptr<Constraint>& constraint = nlp.constraints[idx_constraint];
  NonlinearProgram::ConstraintCache& constraint_cache = nlp.constraint_cache[idx_constraint];

  constraint_cache.Q.resize(nlp.variables.dof, nlp.variables.T);
  constraint_cache.Q.fill(std::numeric_limits<double>::infinity());
  constraint_cache.constraint = Eigen::VectorXd::Zero(constraint->num_constraints());
  constraint_cache.Jacobian = Eigen::VectorXd::Zero(constraint->len_jacobian());
  constraint_cache.idx_i = Eigen::ArrayXi::Zero(constraint->len_jacobian());
  constraint_cache.idx_j = Eigen::ArrayXi::Zero(constraint->len_jacobian());
  constraint->JacobianIndices(constraint_cache.idx_i, constraint_cache.idx_j);

  for (size_t i = 0; i < constraint->num_constraints(); i++) {

    nlopt_constraints.push_back([](const std::vector<double>& x, std::vector<double>& grad, void* data) -> double {
      NonlinearProgram::ConstraintData& constraint_data = *reinterpret_cast<NonlinearProgram::ConstraintData*>(data);
      NonlinearProgram& nlp = constraint_data.nlp;
      const size_t& idx_constraint = constraint_data.idx_constraint;
      const size_t& idx_vector = constraint_data.idx_vector;

      const std::unique_ptr<Constraint>& constraint = nlp.constraints[idx_constraint];
      NonlinearProgram::ConstraintCache& constraint_cache = nlp.constraint_cache[idx_constraint];

      Eigen::Map<const Eigen::MatrixXd> Q(&x[0], nlp.variables.dof, nlp.variables.T);
      if (Q != constraint_cache.Q) {
        constraint_cache.Q = Q;
        constraint_cache.constraint.setZero();
        constraint_cache.Jacobian.setZero();
        constraint->Evaluate(constraint_cache.Q, constraint_cache.constraint);
        constraint->Jacobian(constraint_cache.Q, constraint_cache.Jacobian);
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

Eigen::MatrixXd Trajectory(const JointVariables& variables, const Objectives& objectives,
                           const Constraints& constraints, OptimizationData* data,
                           const std::string& logdir) {

  NonlinearProgram nlp(variables, objectives, constraints, data);
  if (!logdir.empty()) {
    nlp.OpenLogger(logdir);
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
  std::vector<NonlinearProgram::ConstraintData> nlopt_constraint_data;

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
    if (constraints[idx_constraint]->constraint_type(idx_vector) == Constraint::Type::EQUALITY) {
      opt.add_equality_constraint(nlopt_constraints[i], &nlopt_constraint_data[i], kTolerance);
    } else {
      opt.add_inequality_constraint(nlopt_constraints[i], &nlopt_constraint_data[i], kTolerance);
    }
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

  opt.set_xtol_abs(0.0001);
  opt.set_maxtime(60);

  // Variable initialization
  std::vector<double> local_opt_vars;
  std::vector<double>& opt_vars = (data != nullptr) ? *data : local_opt_vars;
  if (opt_vars.size() != variables.dof * variables.T) {
    opt_vars.resize(variables.dof * variables.T);
    Eigen::Map<Eigen::MatrixXd> Q_0(&opt_vars[0], variables.dof, variables.T);
    if (variables.q_0.cols() == variables.T) {
      Q_0 = variables.q_0;
    } else {
      Q_0.colwise() = variables.q_0.col(0);
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

  Eigen::Map<Eigen::MatrixXd> Q(&opt_vars[0], variables.dof, variables.T);
  Eigen::MatrixXd q_des_traj = Q;

  std::cout << str_status << ": " << opt_val << std::endl << std::endl;
  return q_des_traj;
}

void NonlinearProgram::OpenLogger(const std::string& filepath) {
  for (const std::unique_ptr<Objective>& o : objectives) {
    o->log.open(filepath + o->name + ".log");
  }
  for (const std::unique_ptr<Constraint>& c : constraints) {
    c->log.open(filepath + c->name + ".log");
  }
}

void NonlinearProgram::CloseLogger() {
  for (const std::unique_ptr<Objective>& o : objectives) {
    o->log.close();
  }
  for (const std::unique_ptr<Constraint>& c : constraints) {
    c->log.close();
  }
}

} // namespace Nlopt
} // namespace TrajOpt

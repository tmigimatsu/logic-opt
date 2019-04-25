/**
 * multi_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/multi_constraint.h"

#include <algorithm>  // std::max, std::min
#include <limits>     // std::numeric_limits

namespace logic_opt {

MultiConstraint::MultiConstraint(std::vector<std::unique_ptr<Constraint>>&& constraints,
                                 const std::string& name_constraint)
    : Constraint(NumConstraints(constraints), LenJacobian(constraints),
                 TStart(constraints), NumTimesteps(constraints), name_constraint),
      constraints_(std::move(constraints)) {}

void MultiConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                               Eigen::Ref<Eigen::VectorXd> constraints) {
  size_t idx_constraint = 0;
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    // Call evaluate on subconstraints
    try {
      c->Evaluate(Q, constraints.segment(idx_constraint, c->num_constraints()));
    } catch (const std::exception& e) {
      std::cerr << "Constraint(" << c->name << ")::Evaluate(): " << e.what() << std::endl;
      throw e;
    }

    idx_constraint += c->num_constraints();
  }
}

void MultiConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                               Eigen::Ref<Eigen::VectorXd> Jacobian) {
  size_t idx_jacobian = 0;
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    // Call Jacobian on subconstraints
    try {
      c->Jacobian(Q, Jacobian.segment(idx_jacobian, c->len_jacobian()));
    } catch (const std::exception& e) {
      std::cerr << "Constraint(" << c->name << ")::Jacobian(): " << e.what() << std::endl;
      throw e;
    }

    idx_jacobian += c->len_jacobian();
  }
}

void MultiConstraint::JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                      Eigen::Ref<Eigen::ArrayXi> idx_j) {
  size_t idx_jacobian = 0;
  size_t idx_constraint = 0;
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    // Call JacobianIndices on subconstraints
    Eigen::Map<Eigen::ArrayXi> idx_i_t(&idx_i(idx_jacobian), c->len_jacobian());
    Eigen::Map<Eigen::ArrayXi> idx_j_t(&idx_j(idx_jacobian), c->len_jacobian());
    idx_i_t += idx_constraint;
    try {
      c->JacobianIndices(idx_i_t, idx_j_t);
    } catch (const std::exception& e) {
      std::cerr << "Constraint(" << c->name << ")::JacobianIndices(): " << e.what() << std::endl;
      throw e;
    }

    idx_jacobian += c->len_jacobian();
    idx_constraint += c->num_constraints();
  }
}

void MultiConstraint::Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                              Eigen::Ref<const Eigen::VectorXd> lambda,
                              Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {
  size_t idx_constraint = 0;
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    // Call Hessian on subconstraints
    Eigen::Map<const Eigen::VectorXd> lambda_t(&lambda.coeffRef(idx_constraint), c->num_constraints());
    try {
      c->Hessian(Q, lambda_t, Hessian);
    } catch (const std::exception& e) {
      std::cerr << "Constraint(" << c->name << ")::Hessian(): " << e.what() << std::endl;
      throw e;
    }

    idx_constraint += c->num_constraints();
  }
}

void MultiConstraint::HessianStructure(Eigen::SparseMatrix<bool>& Hessian) {
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    // Call HessianStructure on subconstraints
    try {
      c->HessianStructure(Hessian);
    } catch (const std::exception& e) {
      std::cerr << "Constraint(" << c->name << ")::HessianStructure(): " << e.what() << std::endl;
      throw e;
    }
  }
}

Constraint::Type MultiConstraint::constraint_type(size_t idx_constraint) const {
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    // Iterate through subconstraints until proper index found
    if (idx_constraint < c->num_constraints()) {
      // Return constraint_type from subconstraint
      return c->constraint_type(idx_constraint);
    }

    idx_constraint -= c->num_constraints();
  }
  throw std::out_of_range("MultiConstraint::constraint_type(): Constraint index out of range.");
}

size_t MultiConstraint::NumConstraints(const std::vector<std::unique_ptr<Constraint>>& constraints) {
  size_t num_constraints = 0;
  for (const std::unique_ptr<Constraint>& constraint : constraints) {
    num_constraints += constraint->num_constraints();
  }
  return num_constraints;
}

size_t MultiConstraint::LenJacobian(const std::vector<std::unique_ptr<Constraint>>& constraints) {
  size_t len_jacobian = 0;
  for (const std::unique_ptr<Constraint>& constraint : constraints) {
    len_jacobian += constraint->len_jacobian();
  }
  return len_jacobian;
}

size_t MultiConstraint::TStart(const std::vector<std::unique_ptr<Constraint>>& constraints) {
  size_t t_start = std::numeric_limits<size_t>::max();
  for (const std::unique_ptr<Constraint>& constraint : constraints) {
    t_start = std::min(t_start, constraint->t_start());
  }
  return t_start;
}

size_t MultiConstraint::NumTimesteps(const std::vector<std::unique_ptr<Constraint>>& constraints) {
  size_t t_start = std::numeric_limits<size_t>::max();
  size_t t_end = 0;
  for (const std::unique_ptr<Constraint>& constraint : constraints) {
    t_start = std::min(t_start, constraint->t_start());
    t_end = std::max(t_end, constraint->t_start() + constraint->num_timesteps());
  }
  return t_end - t_start;
}

}  // namespace logic_opt

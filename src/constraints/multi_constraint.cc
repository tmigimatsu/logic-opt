/**
 * multi_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/multi_constraint.h"

namespace LogicOpt {

void MultiConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                               Eigen::Ref<Eigen::VectorXd> constraints) {
  size_t idx_constraint = 0;
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    // Call evaluate on subconstraints
    c->Evaluate(Q, constraints.segment(idx_constraint, c->num_constraints()));

    idx_constraint += c->num_constraints();
  }
}

void MultiConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                               Eigen::Ref<Eigen::VectorXd> Jacobian) {
  size_t idx_jacobian = 0;
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    // Call Jacobian on subconstraints
    c->Jacobian(Q, Jacobian.segment(idx_jacobian, c->len_jacobian()));

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
    c->JacobianIndices(idx_i_t, idx_j_t);

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
    c->Hessian(Q, lambda_t, Hessian);

    idx_constraint += c->num_constraints();
  }
}

void MultiConstraint::HessianStructure(Eigen::SparseMatrix<bool>& Hessian) {
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    // Call HessianStructure on subconstraints
    c->HessianStructure(Hessian);
  }
}

void MultiConstraint::Simulate(World& world, Eigen::Ref<const Eigen::MatrixXd> Q) {
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    // Call Simulate on subconstraints
    c->Simulate(world, Q);
  }
}

void MultiConstraint::RegisterSimulationStates(World& world) {
  for (const std::unique_ptr<Constraint>& c : constraints_) {
    // Call RegisterSimulationStates on subconstraints
    c->RegisterSimulationStates(world);
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

}  // namespace LogicOpt

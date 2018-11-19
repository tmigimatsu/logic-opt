/**
 * multi_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_MULTI_CONSTRAINT_H_
#define TRAJ_OPT_MULTI_CONSTRAINT_H_

#include "TrajOpt/constraints/constraint.h"

namespace TrajOpt {

/**
 * Concatenation of multiple constraints.
 */
class MultiConstraint : virtual public Constraint {

 public:

  MultiConstraint() : Constraint(0, 0, 0) {}

  virtual ~MultiConstraint() {}

  // Optimization methods
  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                               Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

  virtual void HessianStructure(Eigen::SparseMatrix<bool>& Hessian) override;

  // Simulation methods
  virtual void Simulate(World& world, Eigen::Ref<const Eigen::MatrixXd> Q) override;

  virtual void RegisterSimulationStates(World& world) override;

  // Constraint properties
  virtual Type constraint_type(size_t idx_constraint) const override;

 protected:

  std::vector<std::unique_ptr<Constraint>> constraints_;  // Vector of constraints

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_MULTI_CONSTRAINT_H_

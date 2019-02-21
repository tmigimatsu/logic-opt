/**
 * multi_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_MULTI_CONSTRAINT_H_
#define LOGIC_OPT_MULTI_CONSTRAINT_H_

#include <memory>  // std::unique_ptr
#include <vector>  // std::vector

#include "LogicOpt/constraints/constraint.h"

namespace LogicOpt {

/**
 * Concatenation of multiple constraints.
 */
class MultiConstraint : public Constraint {

 public:

  MultiConstraint(std::vector<std::unique_ptr<Constraint>>&& constraints,
                  const std::string& name_constraint);

  virtual ~MultiConstraint() = default;

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

  // Constraint properties
  virtual Type constraint_type(size_t idx_constraint) const override;

 protected:

  static size_t NumConstraints(const std::vector<std::unique_ptr<Constraint>>& constraints);

  static size_t LenJacobian(const std::vector<std::unique_ptr<Constraint>>& constraints);

  static size_t TStart(const std::vector<std::unique_ptr<Constraint>>& constraints);

  static size_t NumTimesteps(const std::vector<std::unique_ptr<Constraint>>& constraints);

  std::vector<std::unique_ptr<Constraint>> constraints_;  // Vector of constraints

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_MULTI_CONSTRAINT_H_

/**
 * workspace_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: December 13, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_WORKSPACE_CONSTRAINT_H_
#define LOGIC_OPT_WORKSPACE_CONSTRAINT_H_

#include <set>     // std::set
#include <vector>  // std::vector

#include "logic_opt/constraints/constraint.h"

namespace logic_opt {

class WorkspaceConstraint : virtual public FrameConstraint {

 public:

  static constexpr size_t kNumConstraints = 1;
  // static constexpr size_t kLenJacobian = logic_opt::FrameConstraint::kDof;
  static constexpr size_t kNumTimesteps = 1;

  WorkspaceConstraint(World3& world, size_t t_workspace, const std::string& name_ee);

  virtual ~WorkspaceConstraint() = default;

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                               Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  virtual Type constraint_type(size_t idx_constraint) const { return Type::kInequality; }

 protected:

  virtual double ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const;

  const std::string name_ee_;
  const World3& world_;

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_WORKSPACE_CONSTRAINT_H_

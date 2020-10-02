/**
 * workspace_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: December 13, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_CONSTRAINTS_WORKSPACE_CONSTRAINT_H_
#define LOGIC_OPT_CONSTRAINTS_WORKSPACE_CONSTRAINT_H_

#include "logic_opt/constraints/frame_constraint.h"
#include "logic_opt/world.h"

namespace logic_opt {

class WorkspaceConstraint : virtual public FrameConstraint {
 public:
  static const size_t kNumConstraints = 1;
  static const size_t kNumTimesteps = 1;

  WorkspaceConstraint(World& world, size_t t_workspace,
                      const std::string& name_ee);

  virtual ~WorkspaceConstraint() = default;

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                               Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  virtual Type constraint_type(size_t idx_constraint) const override {
    return Type::kInequality;
  }

 protected:
  virtual double ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const;

  const std::string name_ee_;
  const World& world_;
};

}  // namespace logic_opt

#endif  // LOGIC_OPT_CONSTRAINTS_WORKSPACE_CONSTRAINT_H_

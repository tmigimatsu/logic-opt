/**
 * touch_constraint.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 20, 2019
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_TOUCH_CONSTRAINT_H_
#define LOGIC_OPT_TOUCH_CONSTRAINT_H_

#include "logic_opt/constraints/constraint.h"

namespace logic_opt {

class TouchConstraint : virtual public FrameConstraint {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  static constexpr size_t kDof = FrameVariables<3>::kDof;  // TODO: Use FrameConstraint kDof
  static constexpr size_t kNumConstraints = 1;
  static constexpr size_t kLenJacobian = kDof * kNumConstraints;
  static constexpr size_t kNumTimesteps = 1;

  TouchConstraint(World3& world, size_t t_touch, const std::string& name_control,
                  const std::string& name_target);

  virtual ~TouchConstraint() = default;

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                               Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  virtual Type constraint_type(size_t idx_constraint) const { return Type::kEquality; }

 protected:

  std::optional<ncollide3d::query::Contact> ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const;

  std::optional<ncollide3d::query::Contact> contact_;

  const World3& world_;

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_TOUCH_CONSTRAINT_H_

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

#include "LogicOpt/constraints/constraint.h"

namespace LogicOpt {

class TouchConstraint : virtual public FrameConstraint {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  TouchConstraint(World& world, size_t t_touch, const std::string& name_control,
                  const std::string& name_target);

  virtual ~TouchConstraint() = default;

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                               Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  virtual Type constraint_type(size_t idx_constraint) const { return Type::kInequality; }

 protected:

  virtual double ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const;

  double x_err_ = 0.;

  const World& world_;

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_TOUCH_CONSTRAINT_H_

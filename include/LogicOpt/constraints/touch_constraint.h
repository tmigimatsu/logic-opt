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

 protected:

  virtual Eigen::Vector3d ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const;

  Eigen::Vector3d x_err_ = Eigen::Vector3d::Zero();

  const World& world_;

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_TOUCH_CONSTRAINT_H_

/**
 * pick_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: December 13, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_PICK_CONSTRAINT_H_
#define LOGIC_OPT_PICK_CONSTRAINT_H_

#include "LogicOpt/constraints/constraint.h"

namespace LogicOpt {

class PickConstraint : virtual public FrameConstraint {

 public:

  PickConstraint(World& world, size_t t_pick, const std::string& name_ee, const std::string& name_object,
                 const Eigen::Vector3d& object_offset = Eigen::Vector3d::Zero());

  virtual ~PickConstraint() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

 protected:

  virtual void ComputeError(Eigen::Ref<const Eigen::MatrixXd> X);

  Eigen::Vector3d dx_des_;
  Eigen::Vector3d dx_err_ = Eigen::Vector3d::Zero();

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_PICK_CONSTRAINT_H_

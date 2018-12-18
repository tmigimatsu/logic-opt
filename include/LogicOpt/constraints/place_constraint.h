/**
 * place_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_PLACE_CONSTRAINT_H_
#define LOGIC_OPT_PLACE_CONSTRAINT_H_

#include "LogicOpt/constraints/constraint.h"

namespace LogicOpt {

class PlaceConstraint : virtual public FrameConstraint {

 public:

  PlaceConstraint(World& world, size_t t_place, const std::string& name_object, const std::string& name_target);

  virtual ~PlaceConstraint() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

 protected:

  virtual void ComputeError(Eigen::Ref<const Eigen::MatrixXd> X);

  Eigen::Vector3d dx_des_ = Eigen::Vector3d::Zero();  // z, wx, wy
  Eigen::Vector3d dx_err_ = Eigen::Vector3d::Zero();  // z, wx, wy

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_PLACE_CONSTRAINT_H_

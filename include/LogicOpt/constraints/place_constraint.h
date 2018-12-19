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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PlaceConstraint(World& world, size_t t_place, const std::string& name_object, const std::string& name_target);

  virtual ~PlaceConstraint() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  virtual Type constraint_type(size_t idx_constraint) const override;

 protected:

  virtual void ComputeError(Eigen::Ref<const Eigen::MatrixXd> X);

  Eigen::Vector3d dx_des_ = Eigen::Vector3d::Zero();  // z, wx, wy
  Eigen::Matrix<double,7,1> dx_err_ = Eigen::Matrix<double,7,1>::Zero();  // z, wx, wy

  Eigen::Array2d x_limits_ = Eigen::Array2d::Zero();  // x_min, x_max
  Eigen::Array2d y_limits_ = Eigen::Array2d::Zero();  // y_min, y_max

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_PLACE_CONSTRAINT_H_

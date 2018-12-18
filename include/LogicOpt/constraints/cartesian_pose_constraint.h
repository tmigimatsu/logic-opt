/**
 * cartesian_pose_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_CARTESIAN_POSE_CONSTRAINT_H_
#define LOGIC_OPT_CARTESIAN_POSE_CONSTRAINT_H_

#include "LogicOpt/constraints/constraint.h"

namespace LogicOpt {

class CartesianPoseConstraint : virtual public FrameConstraint {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CartesianPoseConstraint(World& world, size_t t_goal,
                          const std::string& control_frame, const std::string& target_frame,
                          const Eigen::Vector3d& x_des, const Eigen::Quaterniond& quat_des);

  CartesianPoseConstraint(World& world, size_t t_goal,
                          const std::string& control_frame, const std::string& target_frame,
                          const Eigen::Vector6d& dx_des);

  virtual ~CartesianPoseConstraint() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

 protected:

  virtual void ComputeError(Eigen::Ref<const Eigen::MatrixXd> X);

  Eigen::Vector6d dx_des_;
  Eigen::Vector6d dx_err_ = Eigen::Vector6d::Zero();

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_CARTESIAN_POSE_CONSTRAINT_H_

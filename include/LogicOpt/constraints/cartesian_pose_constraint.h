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

  template<typename Derived>
  CartesianPoseConstraint(World& world, size_t t_goal,
                          const std::string& control_frame, const std::string& target_frame,
                          const Eigen::Vector3d& x_des, const Eigen::RotationBase<Derived,3>& ori_des)
      : FrameConstraint(6, 6, t_goal, 1, control_frame, target_frame,
                        "constraint_cart_pos_t" + std::to_string(t_goal)) {
    Eigen::AngleAxisd aa(ori_des.derived());
    dx_des_ << x_des, aa.angle() * aa.axis();
    world.ReserveTimesteps(t_goal + 1);
    world.AttachFrame(control_frame_, target_frame_, t_goal);
  }

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

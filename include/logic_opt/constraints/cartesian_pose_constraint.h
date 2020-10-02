/**
 * cartesian_pose_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_CONSTRAINTS_CARTESIAN_POSE_CONSTRAINT_H_
#define LOGIC_OPT_CONSTRAINTS_CARTESIAN_POSE_CONSTRAINT_H_

#include <ctrl_utils/euclidian.h>

#include "logic_opt/world.h"
#include "logic_opt/constraints/frame_constraint.h"

namespace logic_opt {

class CartesianPoseConstraint : virtual public FrameConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const size_t kNumConstraints = kDof;
  static const size_t kLenJacobian = kDof;
  static const size_t kNumTimesteps = 1;

  CartesianPoseConstraint(World& world, size_t t_goal,
                          const std::string& control_frame,
                          const std::string& target_frame,
                          const Eigen::Vector3d& x_des,
                          const Eigen::Quaterniond& quat_des);

  template <typename Derived>
  CartesianPoseConstraint(World& world, size_t t_goal,
                          const std::string& control_frame,
                          const std::string& target_frame,
                          const Eigen::Vector3d& x_des,
                          const Eigen::RotationBase<Derived, 3>& ori_des);

  CartesianPoseConstraint(World& world, size_t t_goal,
                          const std::string& control_frame,
                          const std::string& target_frame,
                          const spatial_opt::Isometry& T_des);

  CartesianPoseConstraint(World& world, size_t t_goal,
                          const std::string& control_frame,
                          const std::string& target_frame,
                          const Eigen::Isometry3d& T_des);

  CartesianPoseConstraint(World& world, size_t t_goal,
                          const std::string& control_frame,
                          const std::string& target_frame,
                          const Eigen::Vector3d& x_des);

  virtual ~CartesianPoseConstraint() = default;

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  const Eigen::Vectord<kDof>& desired_pose() const { return x_des_; }

 protected:
  Eigen::Vectord<kDof> x_des_;
  Eigen::Vectord<kDof> x_err_ = Eigen::Vectord<kDof>::Zero();
};

/**
 * Implementation
 */

template <typename Derived>
CartesianPoseConstraint::CartesianPoseConstraint(
    World& world, size_t t_goal, const std::string& control_frame,
    const std::string& target_frame, const Eigen::Vector3d& x_des,
    const Eigen::RotationBase<Derived, 3>& ori_des)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_goal, kNumTimesteps,
                      control_frame, target_frame,
                      "constraint_t" + std::to_string(t_goal) + "_cart_pos") {
  x_des_ << x_des, Eigen::Quaterniond(ori_des.matrix()).coeffs();
  world.ReserveTimesteps(t_goal + kNumTimesteps);
  world.AttachFrame(control_frame_, target_frame_, t_goal);
  world.set_controller("cart_pos", t_goal);
}

}  // namespace logic_opt

#endif  // LOGIC_OPT_CONSTRAINTS_CARTESIAN_POSE_CONSTRAINT_H_

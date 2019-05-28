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

#include "logic_opt/constraints/constraint.h"

namespace logic_opt {

template<int Dim>
class CartesianPoseConstraint : virtual public FrameConstraint {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr size_t kDof = FrameVariables<Dim>::kDof;  // TODO: Use FrameConstraint::kDof
  static constexpr size_t kNumConstraints = kDof;
  static constexpr size_t kLenJacobian = kDof;
  static constexpr size_t kNumTimesteps = 1;

  template<typename Derived>
  CartesianPoseConstraint(World<Dim>& world, size_t t_goal,
                          const std::string& control_frame, const std::string& target_frame,
                          const Eigen::Vectord<Dim>& x_des,
                          const Eigen::RotationBase<Derived,Dim>& ori_des);

  CartesianPoseConstraint(World<Dim>& world, size_t t_goal,
                          const std::string& control_frame, const std::string& target_frame,
                          const Eigen::Vectord<kDof>& x_des);

  virtual ~CartesianPoseConstraint() = default;

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

 protected:

  using RotationVariable = std::conditional_t<Dim == 2, double, Eigen::Vector3d>;

  template<typename Derived>
  static RotationVariable ToRotationVariable(const Eigen::RotationBase<Derived,Dim>& ori);

  Eigen::Vectord<kDof> x_des_;
  Eigen::Vectord<kDof> x_err_ = Eigen::Vectord<kDof>::Zero();

};


/**
 * Implementation
 */

template<int Dim>
template<typename Derived>
CartesianPoseConstraint<Dim>::CartesianPoseConstraint(World<Dim>& world, size_t t_goal,
                                                      const std::string& control_frame,
                                                      const std::string& target_frame,
                                                      const Eigen::Vectord<Dim>& x_des,
                                                      const Eigen::RotationBase<Derived,Dim>& ori_des)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_goal, kNumTimesteps,
                      control_frame, target_frame,
                      "constraint_t" + std::to_string(t_goal) + "_cart_pos") {
  x_des_ << x_des, ToRotationVariable(ori_des);
  world.ReserveTimesteps(t_goal + kNumTimesteps);
  world.AttachFrame(control_frame_, target_frame_, t_goal);
}

template<int Dim>
CartesianPoseConstraint<Dim>::CartesianPoseConstraint(World<Dim>& world, size_t t_goal,
                                                      const std::string& control_frame,
                                                      const std::string& target_frame,
                                                      const Eigen::Vectord<kDof>& x_des)
    : FrameConstraint(kNumConstraints, kLenJacobian, t_goal, kNumTimesteps,
                      control_frame, target_frame,
                      "constraint_t" + std::to_string(t_goal) + "_cart_pos"),
      x_des_(x_des) {
  world.ReserveTimesteps(t_goal + kNumTimesteps);
  world.AttachFrame(control_frame_, target_frame_, t_goal);
}

template<int Dim>
void CartesianPoseConstraint<Dim>::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                            Eigen::Ref<Eigen::VectorXd> constraints) {
  x_err_ = X.col(t_start_) - x_des_;
  constraints = 0.5 * x_err_.array().square();
  Constraint::Evaluate(X, constraints);
}

template<int Dim>
void CartesianPoseConstraint<Dim>::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                                            Eigen::Ref<Eigen::VectorXd> Jacobian) {
  Jacobian = x_err_;
  Constraint::Jacobian(X, Jacobian);
}

template<>
template<typename Derived>
Eigen::Vector3d CartesianPoseConstraint<3>::ToRotationVariable(const Eigen::RotationBase<Derived,3>& ori) {
  Eigen::AngleAxisd aa(ori.derived());
  return aa.angle() * aa.axis();
}

template<>
template<typename Derived>
double CartesianPoseConstraint<2>::ToRotationVariable(const Eigen::RotationBase<Derived,2>& ori) {
  return ori.derived().angle();
}

}  // namespace logic_opt

#endif  // LOGIC_OPT_CARTESIAN_POSE_CONSTRAINT_H_

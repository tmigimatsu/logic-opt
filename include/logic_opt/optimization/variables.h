/**
 * joint_variables.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 29, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_JOINT_VARIABLES_H_
#define LOGIC_OPT_JOINT_VARIABLES_H_

#include <spatial_dyn/spatial_dyn.h>

namespace logic_opt {

struct Variables {

  Variables(size_t dof, size_t T, Eigen::Ref<const Eigen::MatrixXd> x_0,
            Eigen::Ref<const Eigen::VectorXd> x_min, Eigen::Ref<const Eigen::VectorXd> x_max)
      : dof(dof), T(T), X_0(x_0), x_min(x_min), x_max(x_max) {}

  virtual ~Variables() = default;

  const size_t dof;
  const size_t T;

  Eigen::MatrixXd X_0;          // Initial configuration
  const Eigen::VectorXd x_min;  // Lower limit
  const Eigen::VectorXd x_max;  // Upper limit

};

struct JointVariables : public Variables {

  JointVariables(const spatial_dyn::ArticulatedBody& ab, size_t T)
      : Variables(ab.dof(), T, ab.q(),
                  ab.Map([](const spatial_dyn::RigidBody& rb) { return rb.joint().q_min(); }),
                  ab.Map([](const spatial_dyn::RigidBody& rb) { return rb.joint().q_max(); })) {}

  JointVariables(const spatial_dyn::ArticulatedBody& ab, size_t T, const Eigen::MatrixXd& Q_0)
      : Variables(ab.dof(), T, Q_0,
                  ab.Map([](const spatial_dyn::RigidBody& rb) { return rb.joint().q_min(); }),
                  ab.Map([](const spatial_dyn::RigidBody& rb) { return rb.joint().q_max(); })) {}

};

template<int Dim>
struct FrameVariables : public Variables {

  static constexpr size_t kDof = (Dim == 2) ? 3 : 6;
  static constexpr double kMaxPos = 0.5;
  static constexpr double kMaxOri = M_PI;

  FrameVariables(size_t T);

};

template<>
inline FrameVariables<3>::FrameVariables(size_t T)
    : Variables(kDof, T, Eigen::Vector6d::Zero(),
                Eigen::Vector6d(-kMaxPos, -kMaxPos, 0., -kMaxOri, -kMaxOri, -kMaxOri),
                Eigen::Vector6d(kMaxPos, kMaxPos, 0.5, kMaxOri, kMaxOri, kMaxOri)) {}

template<>
inline FrameVariables<2>::FrameVariables(size_t T)
    : Variables(kDof, T, Eigen::Vector3d::Zero(),
                Eigen::Vector3d(-kMaxPos, -kMaxPos, -kMaxOri),
                Eigen::Vector3d(kMaxPos, kMaxPos, kMaxOri)) {}

template<int Dim>
constexpr size_t FrameVariables<Dim>::kDof;

template<int Dim>
constexpr double FrameVariables<Dim>::kMaxPos;

template<int Dim>
constexpr double FrameVariables<Dim>::kMaxOri;

}  // namespace logic_opt

#endif  // LOGIC_OPT_JOINT_VARIABLES_H_

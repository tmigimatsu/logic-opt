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

#include <SpatialDyn/SpatialDyn.h>

namespace LogicOpt {

struct Variables {

  Variables(size_t dof, size_t T, Eigen::Ref<const Eigen::MatrixXd> x_0,
            Eigen::Ref<const Eigen::VectorXd> x_min, Eigen::Ref<const Eigen::VectorXd> x_max)
      : dof(dof), T(T) {}

  virtual ~Variables() = default;

  const size_t dof;
  const size_t T;

  const Eigen::MatrixXd x_0;    // Initial configuration
  const Eigen::VectorXd x_min;  // Lower limit
  const Eigen::VectorXd x_max;  // Upper limit

};

struct JointVariables : public Variables {

  JointVariables(const SpatialDyn::ArticulatedBody& ab, size_t T)
      : Variables(ab.dof(), T, ab.q(),
                  ab.Map([](const SpatialDyn::RigidBody& rb) { return rb.joint().q_min(); }),
                  ab.Map([](const SpatialDyn::RigidBody& rb) { return rb.joint().q_max(); })) {}

  JointVariables(const SpatialDyn::ArticulatedBody& ab, size_t T, const Eigen::MatrixXd& Q_0)
      : Variables(ab.dof(), T, Q_0,
                  ab.Map([](const SpatialDyn::RigidBody& rb) { return rb.joint().q_min(); }),
                  ab.Map([](const SpatialDyn::RigidBody& rb) { return rb.joint().q_max(); })) {}

};

struct FrameVariables : public Variables {

  FrameVariables(size_t T)
      : Variables(6, T, Eigen::Vector6d::Zero(),
                  Eigen::Vector6d(1., 1., 1., -2.*M_PI, -2.*M_PI, -2*M_PI),
                  Eigen::Vector6d(1., 1., 1.,  2.*M_PI,  2.*M_PI,  2*M_PI)) {}

};


}  // namespace LogicOpt

#endif  // LOGIC_OPT_JOINT_VARIABLES_H_

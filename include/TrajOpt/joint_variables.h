/**
 * joint_variables.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 29, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_JOINT_VARIABLES_H_
#define TRAJ_OPT_JOINT_VARIABLES_H_

#include <SpatialDyn/SpatialDyn.h>

namespace TrajOpt {

struct JointVariables {

  JointVariables(const SpatialDyn::ArticulatedBody& ab, size_t T)
      : dof(ab.dof()), T(T), q_0(ab.q()),
        q_min(ab.Map([](const SpatialDyn::RigidBody& rb) { return rb.joint().q_min(); })),
        q_max(ab.Map([](const SpatialDyn::RigidBody& rb) { return rb.joint().q_max(); })) {}

  JointVariables(const SpatialDyn::ArticulatedBody& ab, size_t T, const Eigen::MatrixXd& Q_0)
      : dof(ab.dof()), T(T), q_0(Q_0),
        q_min(ab.Map([](const SpatialDyn::RigidBody& rb) { return rb.joint().q_min(); })),
        q_max(ab.Map([](const SpatialDyn::RigidBody& rb) { return rb.joint().q_max(); })) {}

  const size_t dof;  // Degrees of freedom for each timestep
  const size_t T;    // Number of timestep

  const Eigen::MatrixXd q_0;    // Initial joint configuration
  const Eigen::VectorXd q_min;  // Lower joint limit
  const Eigen::VectorXd q_max;  // Upper joint limit

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_JOINT_VARIABLES_H_

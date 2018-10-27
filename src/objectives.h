/**
 * objectives.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 26, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_OBJECTIVES_H_
#define TRAJ_OPT_OBJECTIVES_H_

#include <SpatialDyn/SpatialDyn.h>

namespace TrajOpt {
namespace Objective {

// Joint position
double JointPosition(Eigen::Ref<const Eigen::MatrixXd> Q,
                     Eigen::Ref<const Eigen::VectorXd> q_des);

void JointPositionGradient(Eigen::Ref<const Eigen::MatrixXd> Q,
                           Eigen::Ref<const Eigen::VectorXd> q_des,
                           Eigen::Ref<Eigen::MatrixXd> Grad,
                           double coeff = 1.);

// Joint velocity
double JointVelocity(Eigen::Ref<const Eigen::MatrixXd> Q);

void JointVelocityGradient(Eigen::Ref<const Eigen::MatrixXd> Q,
                           Eigen::Ref<Eigen::MatrixXd> Grad,
                           double coeff = 1.);

// Joint acceleration
double JointAcceleration(Eigen::Ref<const Eigen::MatrixXd> Q);

void JointAccelerationGradient(Eigen::Ref<const Eigen::MatrixXd> Q,
                               Eigen::Ref<Eigen::MatrixXd> Grad,
                               double coeff = 1.);

// Linear velocity
double LinearVelocity(const SpatialDyn::ArticulatedBody& ab,
                      Eigen::Ref<const Eigen::MatrixXd> Q);

void LinearVelocityGradient(SpatialDyn::ArticulatedBody& ab,
                            Eigen::Ref<const Eigen::MatrixXd> Q,
                            Eigen::Ref<Eigen::MatrixXd> Grad,
                            double coeff = 1.);

// Angular velocity (doesn't work)
// TODO: Derive correct jacobian
double AngularVelocity(const SpatialDyn::ArticulatedBody& ab,
                       Eigen::Ref<const Eigen::MatrixXd> Q);

void AngularVelocityGradient(SpatialDyn::ArticulatedBody& ab,
                             Eigen::Ref<const Eigen::MatrixXd> Q,
                             Eigen::Ref<Eigen::MatrixXd> Grad,
                             double coeff = 1.);

}  // namespace Objective
}  // namespace TrajOpt

#endif  // TRAJ_OPT_OBJECTIVES_H_

/**
 * cartesian_pose_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/constraints/cartesian_pose_constraint.h"

namespace logic_opt {

// template<>
// void CartesianPoseConstraint<3>::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
//                                           Eigen::Ref<Eigen::VectorXd> constraints) {
//   x_err_ = X.col(t_start_) - x_des_;
//   constraints = 0.5 * x_err_.array().square();
//   Constraint::Evaluate(X, constraints);
// }

// template<>
// void CartesianPoseConstraint<3>::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
//                                        Eigen::Ref<Eigen::VectorXd> Jacobian) {
//   Jacobian = x_err_;
// }

}  // namespace logic_opt

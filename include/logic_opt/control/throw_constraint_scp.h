/**
 * throw_constraint_scp.h
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 4, 2019
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_OPT_CONTROL_THROW_CONSTRAINT_SCP_H_
#define SPATIAL_OPT_CONTROL_THROW_CONSTRAINT_SCP_H_

#include <utility>  // std::pair

#include <spatial_dyn/spatial_dyn.h>

namespace spatial_opt {

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ThrowConstraintScp(const spatial_dyn::ArticulatedBody& ab,
                                                               Eigen::Ref<const Eigen::VectorXd> q_start,
                                                               Eigen::Ref<const Eigen::Vector3d> x_target,
                                                               const Eigen::Vector3d& ee_offset = Eigen::Vector3d::Zero());

}  // namespace spatial_opt

#endif  // SPATIAL_OPT_CONTROL_THROW_CONSTRAINT_SCP_H_

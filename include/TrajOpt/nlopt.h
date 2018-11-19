/**
 * nlopt.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 25, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_NLOPT_H_
#define TRAJ_OPT_NLOPT_H_

#include "TrajOpt/constraints.h"
#include "TrajOpt/objectives.h"
#include "TrajOpt/joint_variables.h"

#include <SpatialDyn/SpatialDyn.h>

#include <string>     // std::string

namespace TrajOpt {
namespace Nlopt {

typedef std::vector<double> OptimizationData;

Eigen::MatrixXd Trajectory(const JointVariables& variables, const Objectives& objectives,
                           const Constraints& constraints, OptimizationData* data = nullptr,
                           const std::string& logdir = "");

}  // namespace Nlopt
}  // namespace TrajOpt

#endif  // TRAJ_OPT_NLOPT_H_

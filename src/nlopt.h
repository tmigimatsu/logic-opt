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

#include <SpatialDyn/SpatialDyn.h>

#include "constraints.h"
#include "objectives.h"
#include "joint_variables.h"

#include <vector>    // std::vector

namespace TrajOpt {
namespace Nlopt {

typedef std::vector<double> OptimizationData;

std::vector<Eigen::VectorXd> Trajectory(const JointVariables& variables,
                                        const Objectives& objectives,
                                        const Constraints& constraints,
                                        OptimizationData* data = nullptr);

}  // namespace Nlopt
}  // namespace TrajOpt

#endif  // TRAJ_OPT_NLOPT_H_

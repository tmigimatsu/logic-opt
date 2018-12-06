/**
 * nlopt.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 25, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_NLOPT_H_
#define LOGIC_OPT_NLOPT_H_

#include "LogicOpt/constraints.h"
#include "LogicOpt/objectives.h"
#include "LogicOpt/joint_variables.h"

#include <SpatialDyn/SpatialDyn.h>

#include <string>     // std::string

namespace LogicOpt {
namespace Nlopt {

typedef std::vector<double> OptimizationData;

Eigen::MatrixXd Trajectory(const JointVariables& variables, const Objectives& objectives,
                           const Constraints& constraints, OptimizationData* data = nullptr,
                           const std::string& logdir = "");

}  // namespace Nlopt
}  // namespace LogicOpt

#endif  // LOGIC_OPT_NLOPT_H_

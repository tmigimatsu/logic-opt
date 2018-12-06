/**
 * ipopt.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 25, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_IPOPT_H_
#define LOGIC_OPT_IPOPT_H_

#include "LogicOpt/constraints.h"
#include "LogicOpt/objectives.h"
#include "LogicOpt/joint_variables.h"

#include <string>     // std::string

namespace LogicOpt {
namespace Ipopt {

struct OptimizationData {
  std::vector<double> x;
  std::vector<double> z_L;
  std::vector<double> z_U;
};

Eigen::MatrixXd Trajectory(const JointVariables& variables, const Objectives& objectives,
                           const Constraints& constraints, OptimizationData* data = nullptr,
                           const std::string& logdir = "", bool with_hessian = false);

}  // namespace Ipopt
}  // namespace LogicOpt

#endif  // LOGIC_OPT_IPOPT_H_

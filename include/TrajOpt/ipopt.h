/**
 * ipopt.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 25, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_IPOPT_H_
#define TRAJ_OPT_IPOPT_H_

#include "TrajOpt/constraints.h"
#include "TrajOpt/objectives.h"
#include "TrajOpt/joint_variables.h"

#include <string>     // std::string

namespace TrajOpt {
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
}  // namespace TrajOpt

#endif  // TRAJ_OPT_IPOPT_H_

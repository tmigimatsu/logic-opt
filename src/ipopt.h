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

#include "constraints.h"
#include "objectives.h"
#include "joint_variables.h"

#include <vector>      // std::vector

namespace TrajOpt {
namespace Ipopt {

struct OptimizationData {
  std::vector<double> x;
  std::vector<double> z_L;
  std::vector<double> z_U;
};

std::vector<Eigen::VectorXd> Trajectory(const JointVariables& variables,
                                        const Objectives& objectives,
                                        const Constraints& constraints,
                                        OptimizationData* data = nullptr);

}  // namespace Ipopt
}  // namespace TrajOpt

#endif  // TRAJ_OPT_IPOPT_H_

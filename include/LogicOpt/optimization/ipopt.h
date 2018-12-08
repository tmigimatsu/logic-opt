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

#include <csignal>  // std::sig_atomic_t
#include <string>   // std::string

#include "LogicOpt/constraints.h"
#include "LogicOpt/optimization/objectives.h"
#include "LogicOpt/optimization/joint_variables.h"

namespace LogicOpt {

namespace Ipopt {

void Terminate();

struct OptimizationData {
  std::vector<double> x;
  std::vector<double> z_L;
  std::vector<double> z_U;
};

struct Options {
  bool derivative_test = false;
  bool use_hessian = false;
  double max_cpu_time = 600.;
  size_t max_iter = 10000;
  double acceptable_tol = 1e-6;
  size_t acceptable_iter = 15;
};

Eigen::MatrixXd Trajectory(const JointVariables& variables, const Objectives& objectives,
                           const Constraints& constraints, OptimizationData* data = nullptr,
                           const std::string& logdir = "", const Options& options = Options());

}  // namespace Ipopt
}  // namespace LogicOpt

#endif  // LOGIC_OPT_IPOPT_H_

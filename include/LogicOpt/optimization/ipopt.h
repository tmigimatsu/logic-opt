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

#include <yaml-cpp/yaml.h>

#include "LogicOpt/optimization/optimizer.h"

namespace LogicOpt {

class Ipopt : public Optimizer {

 public:

  struct OptimizationData : public Optimizer::OptimizationData {
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
    size_t print_level = 5;
    std::string logdir;
  };

  Ipopt() {}
  Ipopt(const Options& options) : options_(options) {}
  Ipopt(const YAML::Node& options);

  virtual Eigen::MatrixXd Trajectory(const Variables& variables, const Objectives& objectives,
                                     const Constraints& constraints,
                                     Optimizer::OptimizationData* data = nullptr,
                                     const IterationCallbackT& iteration_callback = IterationCallbackT{}) override;

  static void Terminate();

 private:

  Options options_;

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_IPOPT_H_

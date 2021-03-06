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

#include "logic_opt/optimization/optimizer.h"

namespace logic_opt {

class Ipopt : public Optimizer {

 public:

  struct OptimizationData : public Optimizer::OptimizationData {
    std::vector<double> x;
    std::vector<double> z_L;
    std::vector<double> z_U;
    std::vector<double> lambda;
  };

  struct Options {
    bool derivative_test = false;
    bool use_hessian = false;
    double max_cpu_time = 600.;
    size_t max_iter = 10000;
    double tol = 1e-3;
    double acceptable_tol = 1e-2;
    size_t acceptable_iter = 10;
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

  const std::string& status() const { return status_; }

 private:

  Options options_;
  std::string status_;

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_IPOPT_H_

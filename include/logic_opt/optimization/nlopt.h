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

#include "logic_opt/optimization/optimizer.h"

namespace logic_opt {

class Nlopt : public Optimizer {

 public:

  struct OptimizationData : public Optimizer::OptimizationData {
    std::vector<double> vars;
  };

  struct Options {
    std::string logdir;
  };

  Nlopt() {}
  Nlopt(const Options& options) : options_(options) {}

  virtual Eigen::MatrixXd Trajectory(const Variables& variables, const Objectives& objectives,
                                     const Constraints& constraints,
                                     Optimizer::OptimizationData* data = nullptr,
                                     const IterationCallbackT& iteration_callback = IterationCallbackT{}) override;

 private:

  Options options_;

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_NLOPT_H_

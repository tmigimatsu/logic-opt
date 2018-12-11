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

#include "LogicOpt/optimization/optimizer.h"

namespace LogicOpt {

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

  virtual Eigen::MatrixXd Trajectory(const JointVariables& variables, const Objectives& objectives,
                                     const Constraints& constraints,
                                     Optimizer::OptimizationData* data = nullptr) override;

 private:

  Options options_;

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_NLOPT_H_

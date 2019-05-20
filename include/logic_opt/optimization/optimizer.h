/**
 * optimizer.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: December 10, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_OPTIMIZER_H_
#define LOGIC_OPT_OPTIMIZER_H_

#include <functional>  // std::function
#include <string>      // std::string

#include "logic_opt/optimization/constraints.h"
#include "logic_opt/optimization/objectives.h"
#include "logic_opt/optimization/variables.h"

namespace logic_opt {

class Optimizer {

 public:

  using IterationCallbackT = std::function<void(int, const Eigen::MatrixXd&)>;

  struct OptimizationData {
    virtual ~OptimizationData() = default;
  };

  virtual ~Optimizer() = default;

  virtual Eigen::MatrixXd Trajectory(const Variables& variables, const Objectives& objectives,
                                     const Constraints& constraints,
                                     OptimizationData* data = nullptr,
                                     const IterationCallbackT& iteration_callback = IterationCallbackT{}) = 0;

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_OPTIMIZER_H_

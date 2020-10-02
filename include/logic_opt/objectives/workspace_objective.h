/**
 * workspace_objective.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 26, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_OBJECTIVES_WORKSPACE_OBJECTIVE_H_
#define LOGIC_OPT_OBJECTIVES_WORKSPACE_OBJECTIVE_H_

#include <spatial_opt/objective.h>

#include "logic_opt/world.h"

namespace logic_opt {

class WorkspaceObjective : virtual public spatial_opt::Objective {
 public:
  WorkspaceObjective(const World& world, const std::string& name_ee,
                     double coeff = 1.)
      : spatial_opt::Objective(coeff, "objective_workspace"),
        world_(world),
        name_ee_(name_ee) {}

  virtual ~WorkspaceObjective() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::MatrixXd> Gradient) override;

 protected:
  static constexpr double kWorkspaceRadius = 0.4;
  static constexpr double kBaseRadius = 0.2;

  const World& world_;
  const std::string name_ee_;
};

}  // namespace logic_opt

#endif  // LOGIC_OPT_OBJECTIVES_WORKSPACE_OBJECTIVE_H_

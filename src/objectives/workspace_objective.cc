/**
 * workspace_objective.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 26, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/objectives/workspace_objective.h"

#include <cmath>  // std::exp

#include "logic_opt/objectives/linear_distance_objective.h"

namespace logic_opt {

void WorkspaceObjective::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                                  double& objective) {
  double o = 0.;
  for (size_t t = 0; t < X.cols(); t++) {
    const Eigen::Vector3d x_t =
        world_.Position(name_ee_, world_.kWorldFrame, X, t);
    const double radius = x_t.norm();
    if (radius > kWorkspaceRadius) {
      o += std::exp(radius - kWorkspaceRadius);
    } else if (radius < kBaseRadius) {
      o += std::exp(kBaseRadius - radius);
    }
  }
  objective += coeff_ * o;
}

void WorkspaceObjective::Gradient(Eigen::Ref<const Eigen::MatrixXd> X,
                                  Eigen::Ref<Eigen::MatrixXd> Gradient) {
  Eigen::Map<Eigen::VectorXd> gradient(Gradient.data(), Gradient.size());
  for (size_t t = 0; t < X.cols(); t++) {
    const Eigen::Vector3d x_t =
        world_.Position(name_ee_, world_.kWorldFrame, X, t);
    const double radius = x_t.norm();
    if (radius > kWorkspaceRadius) {
      const auto J_t =
          LinearDistanceObjective::PositionJacobian(world_, name_ee_, X, t);
      gradient += coeff_ * std::exp(radius - kWorkspaceRadius) *
                  J_t.transpose() * x_t.normalized();
    } else if (radius < kBaseRadius) {
      const auto J_t =
          LinearDistanceObjective::PositionJacobian(world_, name_ee_, X, t);
      gradient += coeff_ * std::exp(kBaseRadius - radius) * J_t.transpose() *
                  x_t.normalized();
    }
    if (radius <= kWorkspaceRadius) continue;

    // gradient += coeff_ * ctrl_utils::Power(radius - kWorkspaceRadius, 9) *
    //             J_t.transpose() * x_t.normalized();
  }
}

}  // namespace logic_opt

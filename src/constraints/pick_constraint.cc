/**
 * pick_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/constraints/pick_constraint.h"

namespace LogicOpt {

PickConstraint::PickConstraint(World& world, size_t t_pick, const std::string& name_ee,
                               const std::string& name_object, const Eigen::Vector3d& object_offset)
    : FrameConstraint(3, 3, t_pick, 1, name_ee, name_object,
                      "constraint_pick_t" + std::to_string(t_pick)),
      dx_des_(object_offset) {
  world.AttachFrame(name_ee, name_object, t_pick);
}

void PickConstraint::Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> constraints) {
  ComputeError(X);
  constraints = 0.5 * dx_err_.array().square();
  Constraint::Evaluate(X, constraints);
}

void PickConstraint::Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                              Eigen::Ref<Eigen::VectorXd> Jacobian) {
  ComputeError(X);
  Jacobian = dx_err_;
}

void PickConstraint::ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) {
  dx_err_ = X.col(t_start_).head<3>() - dx_des_;
}

}  // namespace LogicOpt

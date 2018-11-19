/**
 * joint_position_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_JOINT_POSITION_CONSTRAINT_H_
#define TRAJ_OPT_JOINT_POSITION_CONSTRAINT_H_

#include "TrajOpt/constraints/constraint.h"

namespace TrajOpt {

class JointPositionConstraint : virtual public Constraint {

 public:

  JointPositionConstraint(const SpatialDyn::ArticulatedBody& ab, size_t t_goal,
                          Eigen::Ref<const Eigen::VectorXd> q_des)
      : Constraint(ab.dof(), ab.dof(), t_goal, 1,
                   "constraint_joint_pos_t" + std::to_string(t_goal)), q_des_(q_des) {}

  virtual ~JointPositionConstraint() {}

  // Optimization methods
  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

  virtual void HessianStructure(Eigen::SparseMatrix<bool>& Hessian) override;

 protected:

  Eigen::VectorXd q_des_;

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_JOINT_POSITION_CONSTRAINT_H_

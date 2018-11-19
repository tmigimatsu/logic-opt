/**
 * cartesian_pose_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_CARTESIAN_POSE_CONSTRAINT_H_
#define TRAJ_OPT_CARTESIAN_POSE_CONSTRAINT_H_

#include "TrajOpt/constraints/constraint.h"

namespace TrajOpt {

class CartesianPoseConstraint : virtual public Constraint {

 public:

  enum class Layout {
    // Position only
    POS_SCALAR,
    POS_VECTOR,

    // Orientation only
    ORI_SCALAR,
    ORI_VECTOR,

    // Combined
    SCALAR,
    SCALAR_SCALAR,
    VECTOR_SCALAR,
    SCALAR_VECTOR,
    VECTOR_VECTOR
  };

  CartesianPoseConstraint(const World& world, size_t t_goal,
                          const Eigen::Vector3d& x_des, const Eigen::Quaterniond& quat_des,
                          const Eigen::Vector3d& ee_offset = Eigen::Vector3d::Zero(),
                          Layout layout = Layout::VECTOR_SCALAR)
      : Constraint(NumConstraints(layout), NumConstraints(layout) * world.ab.dof(), t_goal, 1,
                   "constraint_cart_pos_t" + std::to_string(t_goal)),
        ab_(world.ab), x_des_(x_des), quat_des_(quat_des), ee_offset_(ee_offset), layout_(layout) {}

  virtual ~CartesianPoseConstraint() {}

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

  virtual void ComputeError(Eigen::Ref<const Eigen::MatrixXd> Q);

  static size_t NumConstraints(Layout l);

  SpatialDyn::ArticulatedBody& ab_;

  Eigen::Vector3d x_des_;
  Eigen::Quaterniond quat_des_;
  Eigen::Vector3d ee_offset_;
  const Layout layout_;

  Eigen::VectorXd q_;
  Eigen::Vector6d x_quat_err_ = Eigen::Vector6d::Zero();

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_CARTESIAN_POSE_CONSTRAINT_H_

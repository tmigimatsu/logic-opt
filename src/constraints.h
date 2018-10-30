/**
 * constraints.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 26, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_CONSTRAINTS_H_
#define TRAJ_OPT_CONSTRAINTS_H_

#include <SpatialDyn/SpatialDyn.h>

#include <array>       // std::array
#include <memory>      // std::unique_ptr
#include <vector>      // std::vector

namespace TrajOpt {

class Constraint {

 public:
  enum class Type { EQUALITY, INEQUALITY };

  Constraint(size_t num_constraints, size_t len_jacobian, Type type = Type::EQUALITY)
      : num_constraints(num_constraints), len_jacobian(len_jacobian), type(type) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::VectorXd> constraints) = 0;
  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::VectorXd> Jacobian) = 0;
  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) = 0;

  const size_t num_constraints;
  const size_t len_jacobian;
  const Type type;

};

typedef std::vector<std::unique_ptr<Constraint>> Constraints;

class JointPositionConstraint : public Constraint {

 public:
  JointPositionConstraint(const SpatialDyn::ArticulatedBody& ab, size_t timestep,
                          Eigen::Ref<const Eigen::VectorXd> q_des)
      : Constraint(ab.dof(), ab.dof(), Type::EQUALITY), timestep(timestep), q_des(q_des) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::VectorXd> constraints) override;
  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::VectorXd> Jacobian) override;
  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  const size_t timestep;
  const Eigen::VectorXd q_des;

};

class CartesianPoseConstraint : public Constraint {

 public:
  CartesianPoseConstraint(const SpatialDyn::ArticulatedBody& ab, size_t timestep,
                          const Eigen::Vector3d& x_des, const Eigen::Quaterniond& quat_des)
      : Constraint(1, ab.dof(), Type::EQUALITY), timestep(timestep),
        x_des(x_des), quat_des(quat_des), ab_(ab) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::VectorXd> constraints) override;
  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::VectorXd> Jacobian) override;
  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  const size_t timestep;
  const Eigen::Vector3d x_des;
  const Eigen::Quaterniond quat_des;

 private:
  void ComputeError(Eigen::Ref<const Eigen::MatrixXd> Q);

  SpatialDyn::ArticulatedBody ab_;
  Eigen::VectorXd q_;
  Eigen::Vector6d x_quat_err_;

};

class AboveTableConstraint : public Constraint {

 public:
  AboveTableConstraint(const SpatialDyn::ArticulatedBody& ab, const SpatialDyn::RigidBody& table,
                       size_t t_start, size_t num_timesteps = 1);

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::VectorXd> constraints) override;
  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::VectorXd> Jacobian) override;
  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  const size_t t_start;
  const size_t num_timesteps;

 private:
  void ComputeError(Eigen::Ref<const Eigen::MatrixXd> Q);

  SpatialDyn::ArticulatedBody ab_;
  SpatialDyn::RigidBody table_;
  double height_table_;
  std::array<double, 4> area_table_;

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_CONSTRAINTS_H_

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

#include <array>    // std::array
#include <fstream>  // std::ofstream
#include <memory>   // std::unique_ptr
#include <vector>   // std::vector

#define SCALAR_CARTESIAN_POSE 1

namespace TrajOpt {

class Constraint {

 public:
  enum class Type { EQUALITY, INEQUALITY };

  Constraint(size_t num_constraints, size_t len_jacobian, Type type = Type::EQUALITY,
             const std::string& name = "")
      : num_constraints(num_constraints), len_jacobian(len_jacobian), type(type), name(name) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints);

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) = 0;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) = 0;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {};

  virtual void HessianStructure(Eigen::SparseMatrix<bool>& Hessian, size_t T) {};

  const size_t num_constraints;
  const size_t len_jacobian;
  const Type type;

  const std::string name;
  std::ofstream log;

};

typedef std::vector<std::unique_ptr<Constraint>> Constraints;

class JointPositionConstraint : public Constraint {

 public:
  JointPositionConstraint(const SpatialDyn::ArticulatedBody& ab, size_t t_goal,
                          Eigen::Ref<const Eigen::VectorXd> q_des)
      : Constraint(ab.dof(), ab.dof(), Type::EQUALITY, "constraint_joint_pos_t" + std::to_string(t_goal)),
        t_goal(t_goal), q_des(q_des) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

  virtual void HessianStructure(Eigen::SparseMatrix<bool>& Hessian, size_t T) override;

  const size_t t_goal;
  const Eigen::VectorXd q_des;

};

class CartesianPoseConstraint : public Constraint {

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

  CartesianPoseConstraint(const SpatialDyn::ArticulatedBody& ab, size_t t_goal,
                          const Eigen::Vector3d& x_des, const Eigen::Quaterniond& quat_des,
                          const Eigen::Vector3d& ee_offset = Eigen::Vector3d::Zero(),
                          Layout layout = Layout::VECTOR_SCALAR,
                          const std::string& name = "")
      : Constraint(NumConstraints(layout), NumConstraints(layout) * ab.dof(), Type::EQUALITY,
                   name.empty() ? "constraint_cart_pos_t" + std::to_string(t_goal) : name),
        t_goal(t_goal), layout(layout), x_des(x_des), quat_des(quat_des), ee_offset(ee_offset), ab_(ab) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

  virtual void HessianStructure(Eigen::SparseMatrix<bool>& Hessian, size_t T) override;

  Eigen::Vector3d x_des;
  Eigen::Quaterniond quat_des;
  const Eigen::Vector3d ee_offset;
  const size_t t_goal;
  const Layout layout;

 protected:
  static size_t NumConstraints(Layout l);

  SpatialDyn::ArticulatedBody ab_;
  Eigen::VectorXd q_;
  Eigen::Vector6d x_quat_err_ = Eigen::Vector6d::Zero();

 private:
  void ComputeError(Eigen::Ref<const Eigen::MatrixXd> Q);

};

class AboveTableConstraint : public Constraint {

 public:
  AboveTableConstraint(const SpatialDyn::ArticulatedBody& ab, const SpatialDyn::RigidBody& table,
                       size_t t_start, size_t num_timesteps = 1);

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;
  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;
  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  const size_t t_start;
  const size_t num_timesteps;

 protected:

  SpatialDyn::ArticulatedBody ab_;
  SpatialDyn::RigidBody table_;
  double height_table_;
  std::array<double, 4> area_table_;

};

class PickConstraint : public CartesianPoseConstraint {

 public:
  PickConstraint(const SpatialDyn::ArticulatedBody& ab, size_t t_pick,
                 const SpatialDyn::RigidBody& object,
                 const Eigen::Vector3d& ee_offset = Eigen::Vector3d::Zero(),
                 Layout layout = Layout::POS_VECTOR)
      : CartesianPoseConstraint(ab, t_pick, Eigen::Vector3d::Zero(),
                                Eigen::Quaterniond::Identity(), ee_offset, layout,
                                "constraint_pick_t" + std::to_string(t_pick)),
        object_(object) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

 protected:

  const SpatialDyn::RigidBody& object_;

};

class PlaceConstraint : public CartesianPoseConstraint {

 public:
  PlaceConstraint(const SpatialDyn::ArticulatedBody& ab, size_t t_pick, size_t t_place,
                  const SpatialDyn::RigidBody& object,
                  const Eigen::Vector3d& x_des, const Eigen::Quaterniond& quat_des,
                  const Eigen::Vector3d& ee_offset = Eigen::Vector3d::Zero(),
                  Layout layout = Layout::VECTOR_SCALAR)
      : CartesianPoseConstraint(ab, t_place, x_des, quat_des, ee_offset, layout,
                                "constraint_place_t" + std::to_string(t_place)),
        x_des_place(x_des), quat_des_place(quat_des), t_pick(t_pick), object_(object) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

  const Eigen::Vector3d x_des_place;
  const Eigen::Quaterniond quat_des_place;
  const size_t t_pick;

 protected:
  void ComputePlacePose(Eigen::Ref<const Eigen::MatrixXd> Q);

  const SpatialDyn::RigidBody& object_;

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_CONSTRAINTS_H_

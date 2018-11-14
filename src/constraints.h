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

#include "world.h"

#include <SpatialDyn/SpatialDyn.h>

#include <array>    // std::array
#include <fstream>  // std::ofstream
#include <memory>   // std::unique_ptr
#include <vector>   // std::vector

namespace TrajOpt {

class Constraint {

 public:

  enum class Type { EQUALITY, INEQUALITY };

  Constraint(size_t num_constraints, size_t len_jacobian, size_t t_start, size_t num_timesteps = 0,
             std::vector<Type>&& types = std::vector<Type>(), const std::string& name = "")
      : num_constraints(num_constraints), len_jacobian(len_jacobian),
        t_start(t_start), num_timesteps(num_timesteps), types(types), name(name) {}

  virtual ~Constraint() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints);

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) = 0;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) = 0;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {}

  virtual void HessianStructure(Eigen::SparseMatrix<bool>& Hessian, size_t T) {}

  virtual void Simulate(World& world, Eigen::Ref<const Eigen::MatrixXd> Q) {}

  virtual void InterpolateSimulation(const World& world, Eigen::Ref<const Eigen::VectorXd> q, double t,
                                     std::map<std::string, World::ObjectState>& object_states) const {}

  virtual void RegisterSimulationStates(World& world) {}

  const size_t num_constraints;
  const size_t len_jacobian;
  const size_t t_start;
  const size_t num_timesteps;
  const std::vector<Type> types;

  const std::string name;
  std::ofstream log;

};

typedef std::vector<std::unique_ptr<Constraint>> Constraints;

class JointPositionConstraint : virtual public Constraint {

 public:

  JointPositionConstraint(const SpatialDyn::ArticulatedBody& ab, size_t t_goal,
                          Eigen::Ref<const Eigen::VectorXd> q_des)
      : Constraint(ab.dof(), ab.dof(), t_goal, 1, std::vector<Type>(ab.dof(), Type::EQUALITY),
                   "constraint_joint_pos_t" + std::to_string(t_goal)), q_des(q_des) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

  virtual void HessianStructure(Eigen::SparseMatrix<bool>& Hessian, size_t T) override;

  const Eigen::VectorXd q_des;

};

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

  CartesianPoseConstraint(const SpatialDyn::ArticulatedBody& ab, size_t t_goal,
                          const Eigen::Vector3d& x_des, const Eigen::Quaterniond& quat_des,
                          const Eigen::Vector3d& ee_offset = Eigen::Vector3d::Zero(),
                          Layout layout = Layout::VECTOR_SCALAR)
      : Constraint(NumConstraints(layout), NumConstraints(layout) * ab.dof(), t_goal, 1,
                   std::vector<Type>(NumConstraints(layout), Type::EQUALITY),
                   "constraint_cart_pos_t" + std::to_string(t_goal)),
        layout(layout), x_des(x_des), quat_des(quat_des), ee_offset(ee_offset), ab_(ab) {}

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
  const Layout layout;

 protected:

  virtual void ComputeError(Eigen::Ref<const Eigen::MatrixXd> Q);

  static size_t NumConstraints(Layout l);

  SpatialDyn::ArticulatedBody ab_;
  Eigen::VectorXd q_;
  Eigen::Vector6d x_quat_err_ = Eigen::Vector6d::Zero();

};

class PickConstraint : virtual public Constraint, protected CartesianPoseConstraint {

 public:

  PickConstraint(const World& world, size_t t_pick, const std::string& name_object,
                 const Eigen::Vector3d& ee_offset = Eigen::Vector3d::Zero(),
                 Layout layout = Layout::POS_VECTOR);

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

  virtual void Simulate(World& world, Eigen::Ref<const Eigen::MatrixXd> Q) override;

  virtual void RegisterSimulationStates(World& world) override;

  virtual void InterpolateSimulation(const World& world, Eigen::Ref<const Eigen::VectorXd> q, double t,
                                     std::map<std::string, World::ObjectState>& object_states) const override;

  const std::string name_object;

 protected:

  const World& world_;

};

class PlaceConstraint : virtual public Constraint, protected CartesianPoseConstraint {

 public:

  PlaceConstraint(World& world, size_t t_place, const std::string& name_object,
                  const Eigen::Vector3d& x_des, const Eigen::Quaterniond& quat_des,
                  const Eigen::Vector3d& ee_offset = Eigen::Vector3d::Zero(),
                  Layout layout = Layout::VECTOR_SCALAR);

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

  virtual void RegisterSimulationStates(World& world) override;

  virtual void InterpolateSimulation(const World& world, Eigen::Ref<const Eigen::VectorXd> q, double t,
                                     std::map<std::string, World::ObjectState>& object_states) const override;

  Eigen::Vector3d x_des_place;
  Eigen::Quaterniond quat_des_place;
  const std::string name_object;

 protected:

  virtual void ComputePlacePose(Eigen::Ref<const Eigen::MatrixXd> Q);

  World& world_;

};

class PlaceOnConstraint : virtual public Constraint, protected PlaceConstraint {

 public:

  PlaceOnConstraint(World& world, size_t t_place, const std::string& name_object,
                    const std::string& name_place);

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  // virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
  //                      Eigen::Ref<const Eigen::VectorXd> lambda,
  //                      Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

  // virtual void RegisterSimulationStates(World& world) override;

  // virtual void InterpolateSimulation(const World& world, Eigen::Ref<const Eigen::VectorXd> q, double t,
  //                                    std::map<std::string, World::ObjectState>& object_states) const override;

  const std::string name_place;

 protected:

  virtual void ComputeError(Eigen::Ref<const Eigen::MatrixXd> Q) override;

  Eigen::Vector4d xy_err_;
  double z_err_;
  Eigen::Vector3d quat_err_;

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_CONSTRAINTS_H_

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
             const std::string& name = "")
      : num_constraints(num_constraints), len_jacobian(len_jacobian),
        t_start(t_start), num_timesteps(num_timesteps), name(name) {}

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

  virtual void InterpolateSimulation(const World& world, Eigen::Ref<const Eigen::VectorXd> q,
                                     std::map<std::string, World::ObjectState>& object_states) const {}

  virtual void RegisterSimulationStates(World& world) {}

  virtual Type constraint_type(size_t idx_constraint) const;

  const size_t num_constraints;
  const size_t len_jacobian;
  const size_t t_start;
  const size_t num_timesteps;

  const std::string name;
  std::ofstream log;

};

typedef std::vector<std::unique_ptr<Constraint>> Constraints;

class MultiConstraint : virtual public Constraint {

 public:

  MultiConstraint() : Constraint(0, 0, 0) {}

  virtual ~MultiConstraint() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

  virtual void HessianStructure(Eigen::SparseMatrix<bool>& Hessian, size_t T) override;

  virtual void Simulate(World& world, Eigen::Ref<const Eigen::MatrixXd> Q) override;

  virtual void RegisterSimulationStates(World& world) override;

  virtual Type constraint_type(size_t idx_constraint) const override;

 protected:

  std::vector<std::unique_ptr<Constraint>> constraints_;

};

class JointPositionConstraint : virtual public Constraint {

 public:

  JointPositionConstraint(const SpatialDyn::ArticulatedBody& ab, size_t t_goal,
                          Eigen::Ref<const Eigen::VectorXd> q_des)
      : Constraint(ab.dof(), ab.dof(), t_goal, 1,
                   "constraint_joint_pos_t" + std::to_string(t_goal)), q_des(q_des) {}

  virtual ~JointPositionConstraint() {}

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
                   "constraint_cart_pos_t" + std::to_string(t_goal)),
        layout(layout), x_des(x_des), quat_des(quat_des), ee_offset(ee_offset), ab_(ab) {}

  virtual ~CartesianPoseConstraint() {}

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

  virtual ~PickConstraint() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

  virtual void Simulate(World& world, Eigen::Ref<const Eigen::MatrixXd> Q) override;

  virtual void RegisterSimulationStates(World& world) override;

  virtual void InterpolateSimulation(const World& world, Eigen::Ref<const Eigen::VectorXd> q,
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

  virtual ~PlaceConstraint() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

  virtual void Simulate(World& world, Eigen::Ref<const Eigen::MatrixXd> Q) override;

  virtual void RegisterSimulationStates(World& world) override;

  virtual void InterpolateSimulation(const World& world, Eigen::Ref<const Eigen::VectorXd> q,
                                     std::map<std::string, World::ObjectState>& object_states) const override;

  Eigen::Vector3d x_des_place;
  Eigen::Quaterniond quat_des_place;
  const std::string name_object;

 protected:

  virtual void ComputePlacePose(Eigen::Ref<const Eigen::MatrixXd> Q);

  World& world_;
  Eigen::Isometry3d T_ee_to_object_;

};

class SurfaceContactConstraint : virtual public Constraint, protected PlaceConstraint {

 public:
  enum class Direction { POS_X, POS_Y, POS_Z, NEG_X, NEG_Y, NEG_Z };

  SurfaceContactConstraint(World& world, size_t t_contact, const std::string& name_object,
                           const std::string& name_surface, Direction direction_surface);

  virtual ~SurfaceContactConstraint() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual Type constraint_type(size_t idx_constraint) const override;

  const std::string name_surface;

 protected:

  virtual void ComputePlacePose(Eigen::Ref<const Eigen::MatrixXd> Q) override;

  virtual void ComputeError(Eigen::Ref<const Eigen::MatrixXd> Q) override;

  static int Axis(Direction direction);
  static int SignAxis(Direction direction);
  static std::array<int, 2> OrthogonalAxes(Direction direction);

  const int kNormal;
  const int kSignNormal;
  const std::array<int, 2> kSurfaceAxes;

  Eigen::Vector4d surface_des_;
  Eigen::Vector4d surface_err_;

  friend class PushConstraint;

};

class PlaceOnConstraint : virtual public Constraint, protected SurfaceContactConstraint {

 public:

  PlaceOnConstraint(World& world, size_t t_place, const std::string& name_object,
                    const std::string& name_surface);

  virtual ~PlaceOnConstraint() {}

};

class SlideOnConstraint : virtual public Constraint, protected MultiConstraint {

 public:

  SlideOnConstraint(World& world, size_t t_start, size_t num_timesteps,
                    const std::string& name_object, const std::string& name_surface);

  virtual ~SlideOnConstraint() {}

  virtual void RegisterSimulationStates(World& world) override;

  virtual Type constraint_type(size_t idx_constraint) const override;

};

class PushConstraint : virtual public Constraint, protected MultiConstraint {

 public:

  enum class Direction { POS_X, POS_Y, NEG_X, NEG_Y };

  PushConstraint(World& world, size_t t_start, size_t num_timesteps,
                 const std::string& name_pusher, const std::string& name_pushee,
                 Direction direction_push);

  virtual ~PushConstraint() {}

  virtual void RegisterSimulationStates(World& world) override;

 protected:

  class PushSurfaceContactConstraint : virtual public Constraint, public SurfaceContactConstraint {

   public:

    PushSurfaceContactConstraint(World& world, size_t t_contact, const std::string& name_object,
                                 const std::string& name_surface, Direction direction_surface);

    virtual ~PushSurfaceContactConstraint() {}

    virtual void Simulate(World& world, Eigen::Ref<const Eigen::MatrixXd> Q) override;

    virtual void RegisterSimulationStates(World& world) override;

    virtual void InterpolateSimulation(const World& world, Eigen::Ref<const Eigen::VectorXd> q,
                                       std::map<std::string, World::ObjectState>& object_states) const override;

   protected:

    virtual void ComputePlacePose(Eigen::Ref<const Eigen::MatrixXd> Q) override;

  };

  class PushActionConstraint : virtual public Constraint, public PushSurfaceContactConstraint {

   public:

    PushActionConstraint(World& world, size_t t_contact, const std::string& name_object,
                         const std::string& name_surface, Direction direction_surface);

    virtual ~PushActionConstraint() {}

    virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                          Eigen::Ref<Eigen::VectorXd> constraints) override;

    virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                          Eigen::Ref<Eigen::VectorXd> Jacobian) override;

    virtual Type constraint_type(size_t idx_constraint) const override;

  };

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_CONSTRAINTS_H_

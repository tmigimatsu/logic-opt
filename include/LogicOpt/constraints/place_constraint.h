/**
 * place_constraint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_PLACE_CONSTRAINT_H_
#define LOGIC_OPT_PLACE_CONSTRAINT_H_

#include "LogicOpt/constraints/cartesian_pose_constraint.h"

namespace LogicOpt {

class PlaceConstraint : virtual public Constraint, protected CartesianPoseConstraint {

 public:

  PlaceConstraint(World& world, size_t t_place, const std::string& name_object,
                  const Eigen::Vector3d& x_des, const Eigen::Quaterniond& quat_des,
                  const Eigen::Vector3d& ee_offset = Eigen::Vector3d::Zero(),
                  Layout layout = Layout::VECTOR_SCALAR);

  virtual ~PlaceConstraint() {}

  // Optimization methods
  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

  // Simulation methods
  virtual void Simulate(World& world, Eigen::Ref<const Eigen::MatrixXd> Q) override;

  virtual void RegisterSimulationStates(World& world) override;

  virtual void InterpolateSimulation(const World& world, Eigen::Ref<const Eigen::VectorXd> q,
                                     std::map<std::string, World::ObjectState>& object_states) const override;

 protected:

  virtual void ComputePlacePose(Eigen::Ref<const Eigen::MatrixXd> Q);

  World& world_;

  Eigen::Vector3d x_des_place_;
  Eigen::Quaterniond quat_des_place_;

  const std::string name_object_;

  Eigen::Isometry3d T_ee_to_object_;

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_PLACE_CONSTRAINT_H_

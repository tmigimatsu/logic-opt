/**
 * constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_CONSTRAINT_H_
#define TRAJ_OPT_CONSTRAINT_H_

#include "TrajOpt/world.h"

#include <SpatialDyn/SpatialDyn.h>

#include <fstream>  // std::ofstream
#include <memory>   // std::unique_ptr
#include <vector>   // std::vector

namespace TrajOpt {

class Constraint {

 public:

  enum class Type { EQUALITY, INEQUALITY };

  Constraint(size_t num_constraints, size_t len_jacobian, size_t t_start, size_t num_timesteps = 0,
             const std::string& name = "")
      : num_constraints_(num_constraints), len_jacobian_(len_jacobian),
        t_start_(t_start), num_timesteps_(num_timesteps), name(name) {}

  virtual ~Constraint() {}

  // Optimization methods
  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) {
    if (!log.is_open()) return;
    log << constraints.transpose() << std::endl;
  }

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) = 0;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                               Eigen::Ref<Eigen::ArrayXi> idx_j) = 0;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {}

  virtual void HessianStructure(Eigen::SparseMatrix<bool>& Hessian) {}

  // Simulation methods
  virtual void Simulate(World& world, Eigen::Ref<const Eigen::MatrixXd> Q) {}

  virtual void InterpolateSimulation(const World& world, Eigen::Ref<const Eigen::VectorXd> q,
                                     std::map<std::string, World::ObjectState>& object_states) const {}

  virtual void RegisterSimulationStates(World& world) {}

  // Constraint properties
  virtual Type constraint_type(size_t idx_constraint) const { return Type::EQUALITY; }

  virtual const size_t& num_constraints() const { return num_constraints_; }
  virtual const size_t& len_jacobian() const { return len_jacobian_; }

  virtual const size_t& t_start() const { return t_start_; }
  virtual const size_t& num_timesteps() const { return num_timesteps_; }

  // Debug properties
  std::string name;   // Debug name of constraint
  std::ofstream log;  // Debug log (written to by Evaluate())

 protected:

  const size_t num_constraints_;  // Dimension of constraint vector
  const size_t len_jacobian_;     // Number of nonzeros in Jacobian

  const size_t t_start_;          // Start timestep
  const size_t num_timesteps_;    // Duration of constraint

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_CONSTRAINT_H_

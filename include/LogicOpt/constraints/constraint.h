/**
 * constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_CONSTRAINT_H_
#define LOGIC_OPT_CONSTRAINT_H_

#include "LogicOpt/world.h"

#include <spatial_dyn/spatial_dyn.h>

#include <fstream>  // std::ofstream
#include <memory>   // std::unique_ptr
#include <string>   // std::string
#include <vector>   // std::vector

namespace LogicOpt {

class Constraint;
using Constraints = std::vector<std::unique_ptr<Constraint>>;

class Constraint {

 public:

  enum class Type { EQUALITY, INEQUALITY };

  Constraint(size_t num_constraints, size_t len_jacobian, size_t t_start, size_t num_timesteps,
             const std::string& name_constraint)
      : num_constraints_(num_constraints), len_jacobian_(len_jacobian),
        t_start_(t_start), num_timesteps_(num_timesteps), name(name_constraint) {}

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

class FrameConstraint : public Constraint {

 public:

  FrameConstraint(size_t num_constraints, size_t len_jacobian, size_t t_start, size_t num_timesteps,
                  const std::string& control_frame, const std::string& target_frame,
                  const std::string& name_constraint)
      : Constraint(num_constraints, len_jacobian, t_start, num_timesteps, name_constraint),
        control_frame_(control_frame), target_frame_(target_frame) {}

  const std::string& control_frame() const { return control_frame_; }
  const std::string& target_frame() const { return target_frame_; }

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) override {
    idx_i += Eigen::VectorXi::LinSpaced(num_constraints_, 0, num_constraints_ - 1).array();
    idx_j = Eigen::VectorXi::LinSpaced(num_constraints_, 6 * t_start_, 6 * t_start_ + num_constraints_ - 1).array();
  }

 protected:

  std::string control_frame_;
  std::string target_frame_;

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_CONSTRAINT_H_

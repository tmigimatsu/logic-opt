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

#include "logic_opt/world.h"

#include <spatial_dyn/spatial_dyn.h>

#include <fstream>  // std::ofstream
#include <memory>   // std::unique_ptr
#include <string>   // std::string
#include <vector>   // std::vector

namespace logic_opt {

class Constraint;
using Constraints = std::vector<std::unique_ptr<Constraint>>;

class Constraint {

 public:

  enum class Type { kEquality, kInequality };

  Constraint(size_t num_constraints, size_t len_jacobian, size_t t_start, size_t num_timesteps,
             const std::string& name_constraint)
      : num_constraints_(num_constraints), len_jacobian_(len_jacobian),
        t_start_(t_start), num_timesteps_(num_timesteps), name(name_constraint) {}

  virtual ~Constraint() = default;

  // Optimization methods
  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) {
    if (!log_constraint_.is_open()) return;
    log_constraint_ << constraints.transpose() << std::endl;
  }

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) {
    if (!log_jacobian_.is_open()) return;
    log_jacobian_ << Jacobian.transpose() << std::endl;
  }

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                               Eigen::Ref<Eigen::ArrayXi> idx_j) = 0;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {}

  virtual void HessianStructure(Eigen::SparseMatrix<bool>& Hessian) {}

  // Constraint properties
  virtual Type constraint_type(size_t idx_constraint) const { return Type::kEquality; }

  virtual size_t num_constraints() const { return num_constraints_; }
  virtual size_t len_jacobian() const { return len_jacobian_; }

  virtual size_t t_start() const { return t_start_; }
  virtual size_t num_timesteps() const { return num_timesteps_; }

  // Debug properties
  std::string name;   // Debug name of constraint

  virtual void OpenConstraintLog(const std::string& filepath) { log_constraint_.open(filepath + name + "_constraint.log"); }
  virtual void OpenJacobianLog(const std::string& filepath) { log_jacobian_.open(filepath + name + "_jacobian.log"); }

  virtual void CloseConstraintLog() { log_constraint_.close(); }
  virtual void CloseJacobianLog() { log_jacobian_.close(); }

 protected:

  const size_t num_constraints_;  // Dimension of constraint vector
  const size_t len_jacobian_;     // Number of nonzeros in Jacobian

  const size_t t_start_;          // Start timestep
  const size_t num_timesteps_;    // Duration of constraint

  std::ofstream log_constraint_;  // Debug log (written to by Evaluate())
  std::ofstream log_jacobian_;  // Debug log (written to by Evaluate())

};

class FrameConstraint : public Constraint {

 public:

  FrameConstraint(size_t num_constraints, size_t len_jacobian, size_t t_start, size_t num_timesteps,
                  const std::string& control_frame, const std::string& target_frame,
                  const std::string& name_constraint)
      : Constraint(num_constraints, len_jacobian, t_start, num_timesteps, name_constraint),
        control_frame_(control_frame), target_frame_(target_frame) {}

  virtual ~FrameConstraint() = default;

  static const size_t kDof = 6;

  const std::string& control_frame() const { return control_frame_; }
  const std::string& target_frame() const { return target_frame_; }

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i, Eigen::Ref<Eigen::ArrayXi> idx_j) override {
    idx_i += Eigen::VectorXi::LinSpaced(num_constraints_, 0, num_constraints_ - 1).array();
    idx_j = Eigen::VectorXi::LinSpaced(num_constraints_, kDof * t_start_, kDof * t_start_ + num_constraints_ - 1).array();
  }

 protected:

  std::string control_frame_;
  std::string target_frame_;

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_CONSTRAINT_H_

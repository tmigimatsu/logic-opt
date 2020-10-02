/**
 * frame_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_CONSTRAINTS_FRAME_CONSTRAINT_H_
#define LOGIC_OPT_CONSTRAINTS_FRAME_CONSTRAINT_H_

#include <spatial_opt/constraint.h>

namespace logic_opt {

/**
 * Constraint base class for relative frames.
 */
class FrameConstraint : public spatial_opt::Constraint {
 public:
  /**
   * Constraint constructor.
   *
   * @param num_constraints Number of constraints.
   * @param len_jacobian Number of nonzero elements in sparse Jacobian.
   * @param t_start Start time of constraint.
   * @param num_timesteps Duration of constraint.
   * @param control_frame Name of control frame.
   * @param target_frame Name of target frame.
   * @param name_constraint Name of constraint (for debugging).
   */
  FrameConstraint(size_t num_constraints, size_t len_jacobian, size_t t_start,
                  size_t num_timesteps, const std::string& control_frame,
                  const std::string& target_frame,
                  const std::string& name_constraint)
      : spatial_opt::Constraint(num_constraints, len_jacobian, t_start,
                                num_timesteps, name_constraint),
        control_frame_(control_frame),
        target_frame_(target_frame) {}

  virtual ~FrameConstraint() = default;

  static const size_t kDof = 7;

  /**
   * Control frame.
   */
  const std::string& control_frame() const { return control_frame_; }

  /**
   * Target frame.
   */
  const std::string& target_frame() const { return target_frame_; }

  /**
   * Configure indices for sparse Jacobian matrix.
   *
   * @param idx_i Output i indices (array of size len_jacobian).
   * @param idx_j Output j indices (array of size len_jacobian).
   */
  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                               Eigen::Ref<Eigen::ArrayXi> idx_j) override {
    idx_i +=
        Eigen::VectorXi::LinSpaced(num_constraints_, 0, num_constraints_ - 1)
            .array();
    idx_j = Eigen::VectorXi::LinSpaced(num_constraints_, kDof * t_start_,
                                       kDof * t_start_ + num_constraints_ - 1)
                .array();
  }

 protected:
  std::string control_frame_;
  std::string target_frame_;
};

}  // namespace logic_opt

#endif  // LOGIC_OPT_CONSTRAINTS_FRAME_CONSTRAINT_H_

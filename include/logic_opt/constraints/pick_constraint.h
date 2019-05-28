/**
 * pick_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: December 13, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_PICK_CONSTRAINT_H_
#define LOGIC_OPT_PICK_CONSTRAINT_H_

#include "logic_opt/constraints/constraint.h"

namespace logic_opt {

class PickConstraint : virtual public FrameConstraint {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  static constexpr size_t kDof = FrameVariables<3>::kDof;  // TODO: Use FrameConstraint kDof
  static constexpr size_t kNumConstraints = 1;
  static constexpr size_t kLenJacobian = 3;
  static constexpr size_t kNumTimesteps = 1;

  PickConstraint(World3& world, size_t t_pick, const std::string& name_ee,
                 const std::string& name_object);

  virtual ~PickConstraint() = default;

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                               Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  virtual Type constraint_type(size_t idx_constraint) const { return Type::kInequality; }

 protected:

  double x_err_ = 0;

  Eigen::Vector3d x_ee_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dx_err_ = Eigen::Vector3d::Zero();

  const World3& world_;

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_PICK_CONSTRAINT_H_

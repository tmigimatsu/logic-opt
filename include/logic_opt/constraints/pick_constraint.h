/**
 * pick_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: December 13, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_CONSTRAINTS_PICK_CONSTRAINT_H_
#define LOGIC_OPT_CONSTRAINTS_PICK_CONSTRAINT_H_

#include <spatial_opt/constraints/multi_constraint.h>

#include "logic_opt/constraints/frame_constraint.h"
#include "logic_opt/world.h"

namespace logic_opt {

class PickConstraint : public spatial_opt::MultiConstraint {
 public:
  PickConstraint(World& world, size_t t_pick, const std::string& name_ee,
                 const std::string& name_object);

  virtual ~PickConstraint() = default;

  class SignedDistanceConstraint : virtual public FrameConstraint {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static const size_t kNumConstraints = 1;
    static const size_t kLenJacobian = 3;
    static const size_t kNumTimesteps = 1;

    SignedDistanceConstraint(World& world, size_t t_pick,
                             const std::string& name_ee,
                             const std::string& name_object);

    virtual ~SignedDistanceConstraint() = default;

    virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> constraints) override;

    virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> Jacobian) override;

    virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                 Eigen::Ref<Eigen::ArrayXi> idx_j) override;

    virtual Type constraint_type(size_t idx_constraint) const override {
      return Type::kInequality;
    }

   protected:
    double x_err_ = 0.;

    Eigen::Vector3d x_ee_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d dx_err_ = Eigen::Vector3d::Zero();

    const World& world_;
  };
};

}  // namespace logic_opt

#endif  // LOGIC_OPT_CONSTRAINTS_PICK_CONSTRAINT_H_

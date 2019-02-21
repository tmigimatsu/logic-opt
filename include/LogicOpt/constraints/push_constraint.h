/**
 * push_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_PUSH_CONSTRAINT_H_
#define LOGIC_OPT_PUSH_CONSTRAINT_H_

#include "LogicOpt/constraints/constraint.h"

#include "LogicOpt/constraints/multi_constraint.h"

namespace LogicOpt {

class PushConstraint : public MultiConstraint {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PushConstraint(World& world, size_t t_push, const std::string& name_pusher,
                 const std::string& name_pushee);

  virtual ~PushConstraint() = default;

  class SupportAreaConstraint : virtual public FrameConstraint {

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    SupportAreaConstraint(World& world, size_t t_contact, const std::string& name_control,
                          const std::string& name_target, PushConstraint& push_constraint);

    virtual ~SupportAreaConstraint() = default;

    virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> constraints) override;

    virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> Jacobian) override;

    virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                 Eigen::Ref<Eigen::ArrayXi> idx_j) override;

    virtual Type constraint_type(size_t idx_constraint) const override;

   protected:

    virtual void ComputeError(Eigen::Ref<const Eigen::MatrixXd> X);

    double z_err_max_ = 0.;
    double z_err_min_ = 0.;
    double z_max_ = 0.;
    double z_min_ = 0.;

    const World& world_;

    PushConstraint& push_constraint_;

  };

  class DestinationConstraint : virtual public FrameConstraint {

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    DestinationConstraint(World& world, size_t t_contact, const std::string& name_control,
                          const std::string& name_target, PushConstraint& push_constraint);

    virtual ~DestinationConstraint() = default;

    virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> constraints) override;

    virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> Jacobian) override;

    virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                 Eigen::Ref<Eigen::ArrayXi> idx_j) override;

    virtual Type constraint_type(size_t idx_constraint) const override;

   protected:

    virtual void ComputeError(Eigen::Ref<const Eigen::MatrixXd> X);

    Eigen::Vector2d normal_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d xy_ = Eigen::Vector2d::Zero();
    double z_err_ = 0.;

    const World& world_;

    PushConstraint& push_constraint_;

  };

  protected:

   Eigen::Vector3d normal_ = Eigen::Vector3d::Zero();

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_PUSH_CONSTRAINT_H_

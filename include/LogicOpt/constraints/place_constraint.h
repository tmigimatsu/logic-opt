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

#include "LogicOpt/constraints/constraint.h"
#include "LogicOpt/constraints/multi_constraint.h"
#include "LogicOpt/constraints/touch_constraint.h"

namespace LogicOpt {

class PlaceConstraint : virtual public MultiConstraint {

 public:

  PlaceConstraint(World& world, size_t t_place, const std::string& name_object,
                  const std::string& name_target);

  virtual ~PlaceConstraint() = default;

  class SupportAreaConstraint : virtual public FrameConstraint {

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    SupportAreaConstraint(World& world, size_t t_contact, const std::string& name_control,
                          const std::string& name_target);

    virtual ~SupportAreaConstraint() = default;

    virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> constraints) override;

    virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> Jacobian) override;

    virtual Type constraint_type(size_t idx_constraint) const override;

   protected:

    virtual void ComputeError(Eigen::Ref<const Eigen::MatrixXd> X);

    Eigen::Vector2d xy_err_ = Eigen::Vector2d::Zero();
    double z_err_ = 0.;

    const World& world_;

  };

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_PLACE_CONSTRAINT_H_

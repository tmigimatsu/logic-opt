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

#include "logic_opt/constraints/constraint.h"
#include "logic_opt/constraints/multi_constraint.h"
#include "logic_opt/constraints/touch_constraint.h"

#define PLACE_SUPPORT_CONSTRAINT_NUMERICAL_JACOBIAN

namespace logic_opt {

class PlaceConstraint : public MultiConstraint {

 public:

  static constexpr size_t kNumTimesteps = 1;

  PlaceConstraint(World3& world, size_t t_place, const std::string& name_object,
                  const std::string& name_target);

  virtual ~PlaceConstraint() = default;

  class NormalConstraint : virtual public FrameConstraint {

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    NormalConstraint(size_t t_contact, const std::string& name_control,
                     const std::string& name_target);

    virtual ~NormalConstraint() = default;

    virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> constraints) override;

    virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> Jacobian) override;

    virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                 Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  };

  class SupportAreaConstraint : virtual public FrameConstraint {

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    SupportAreaConstraint(World3& world, size_t t_contact, const std::string& name_control,
                          const std::string& name_target);

    virtual ~SupportAreaConstraint() = default;

    virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> constraints) override;

    virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> Jacobian) override;

    virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                 Eigen::Ref<Eigen::ArrayXi> idx_j) override;

    virtual Type constraint_type(size_t idx_constraint) const override {
      return idx_constraint == 0 ? Type::kEquality : Type::kInequality;
    };

   protected:

    virtual Eigen::Vector3d ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                         double* z_err = nullptr) const;

    Eigen::Vector3d x_err_ = Eigen::Vector3d::Zero();
    double z_err_ = 0.;

    std::array<Eigen::Vector3d, 2> xy_support_;
    double z_surface_ = 0.;
    std::unique_ptr<const ncollide2d::shape::Shape> target_2d_;

    const World3& world_;

  };

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_PLACE_CONSTRAINT_H_

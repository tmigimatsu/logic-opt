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

#include <array>  // std::array

#include "logic_opt/constraints/constraint.h"
#include "logic_opt/constraints/multi_constraint.h"

namespace logic_opt {

class PushConstraint : public MultiConstraint {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PushConstraint(World3& world, size_t t_push, const std::string& name_pusher,
                 const std::string& name_pushee, const std::string& name_target);

  virtual ~PushConstraint() = default;

  class ContactAreaConstraint : virtual public FrameConstraint {

   public:

    static constexpr size_t kNumConstraints = 1;
    static constexpr size_t kLenJacobian = 6;
    static constexpr size_t kNumTimesteps = 1;

    ContactAreaConstraint(World3& world, size_t t_contact, const std::string& name_control,
                          const std::string& name_target, PushConstraint& push_constraint);

    virtual ~ContactAreaConstraint() = default;

    virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> constraints) override;

    virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> Jacobian) override;

    virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                 Eigen::Ref<Eigen::ArrayXi> idx_j) override;

    // virtual Type constraint_type(size_t idx_constraint) const { return Type::kInequality; }

   protected:

    virtual Eigen::Vector3d ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                         Eigen::Vector3d* contact_pusher = nullptr) const;

    Eigen::Vector3d x_err_ = Eigen::Vector3d::Zero();

    const World3& world_;

    PushConstraint& push_constraint_;

  };

  class AlignmentConstraint : virtual public FrameConstraint {

   public:

    static constexpr size_t kNumConstraints = 3;
    static constexpr size_t kLenJacobian = 3;
    static constexpr size_t kNumTimesteps = 1;

    AlignmentConstraint(World3& world, size_t t_contact);

    virtual ~AlignmentConstraint() = default;

    virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> constraints) override;

    virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> Jacobian) override;

    virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                 Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  };

  class DestinationConstraint : virtual public FrameConstraint {

   public:

    static constexpr size_t kNumConstraints = 2;
    static constexpr size_t kLenJacobian = 3;
    static constexpr size_t kNumTimesteps = 1;

    DestinationConstraint(World3& world, size_t t_contact, const std::string& name_control,
                          const std::string& name_target, PushConstraint& push_constraint);

    virtual ~DestinationConstraint() = default;

    virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> constraints) override;

    virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> Jacobian) override;

    virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                 Eigen::Ref<Eigen::ArrayXi> idx_j) override;

    virtual Type constraint_type(size_t idx_constraint) const { return idx_constraint > 0 ? Type::kInequality : Type::kEquality; }

   protected:

    const double kWorkspaceRadius = 0.4;

    virtual double ComputeError(Eigen::Ref<const Eigen::MatrixXd> X) const;

    double z_err_ = 0.;

    const World3& world_;

  };

 protected:

  const std::string name_pusher_;
  const std::string name_pushee_;

  const World3& world_;

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_PUSH_CONSTRAINT_H_

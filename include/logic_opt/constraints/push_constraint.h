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

#define PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE

namespace logic_opt {

class PushConstraint : public MultiConstraint {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PushConstraint(World3& world, size_t t_push, const std::string& name_pusher,
                 const std::string& name_pushee);

  virtual ~PushConstraint() = default;

  class ContactAreaConstraint : virtual public FrameConstraint {

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    ContactAreaConstraint(World3& world, size_t t_contact, const std::string& name_control,
                          const std::string& name_target, PushConstraint& push_constraint);

    virtual ~ContactAreaConstraint() = default;

    virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> constraints) override;

    virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> Jacobian) override;

    virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                 Eigen::Ref<Eigen::ArrayXi> idx_j) override;

    virtual Type constraint_type(size_t idx_constraint) const override { return Type::kInequality; }

   protected:

    virtual Eigen::Vector2d ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                         const ncollide3d::query::Contact& contact) const;

    double z_max_ = 0.;
    double z_min_ = 0.;
    Eigen::Vector2d z_xy_contact_ = Eigen::Vector2d::Zero();

    Eigen::Vector2d x_err_ = Eigen::Vector2d::Zero();

    const World3& world_;

    PushConstraint& push_constraint_;

  };

  class AlignmentConstraint : virtual public FrameConstraint {

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    DestinationConstraint(World3& world, size_t t_contact, const std::string& name_control,
                          const std::string& name_target, PushConstraint& push_constraint);

    virtual ~DestinationConstraint() = default;

    virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> constraints) override;

    virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                          Eigen::Ref<Eigen::VectorXd> Jacobian) override;

    virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                                 Eigen::Ref<Eigen::ArrayXi> idx_j) override;

   protected:

    virtual double ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                const ncollide3d::query::Contact& contact,
                                double* z_err = nullptr) const;

    // Eigen::Vector2d normal_ = Eigen::Vector2d::Zero();
    // Eigen::Vector2d xy_ = Eigen::Vector2d::Zero();
    double xy_dot_normal_ = 0.;
    double z_err_ = 0.;

    const World3& world_;

    PushConstraint& push_constraint_;

  };

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

 protected:

  ncollide3d::query::Contact contact_;

std::array<ncollide3d::query::Contact, 6> contact_hp_;
#ifdef PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
  std::array<ncollide3d::query::Contact, 6> contact_hn_;
#endif  // PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE

  const std::string name_pusher_;
  const std::string name_pushee_;

  Eigen::Vector2d dir_push_object_ = Eigen::Vector2d::Zero();

  const World3& world_;

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_PUSH_CONSTRAINT_H_

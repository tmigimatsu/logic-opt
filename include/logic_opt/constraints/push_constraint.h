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

    virtual Type constraint_type(size_t idx_constraint) const override { return Type::kInequality; }

   protected:

    virtual double ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                const ncollide3d::query::PointProjection& projection) const;

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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static constexpr size_t kNumConstraints = 2;
    static constexpr size_t kLenJacobian = 7;
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

    virtual Type constraint_type(size_t idx_constraint) const { return idx_constraint <= 2 ? Type::kInequality : Type::kEquality; }

   protected:

    // virtual double ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
    //                             const ncollide3d::query::Contact& contact,
    //                             double* z_err = nullptr) const;

    virtual double ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                                const ncollide3d::query::PointProjection& projection,
                                double* z_err = nullptr,
                                Eigen::Vector2d* normal = nullptr,
                                Eigen::Vector2d* xy = nullptr) const;

    Eigen::Vector2d normal_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d xy_ = Eigen::Vector2d::Zero();
    double xy_dot_normal_ = 0.;
    double z_err_ = 0.;

    double z_max_ = 0.;
    double z_min_ = 0.;

    const World3& world_;

    PushConstraint& push_constraint_;

  };

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

 protected:

  std::optional<ncollide3d::query::Contact> contact_;
  ncollide3d::query::PointProjection projection_;
  Eigen::Vector3d projection_origin_ = Eigen::Vector3d::Zero();

  // std::array<std::optional<ncollide3d::query::Contact>, 6> contact_hp_;
  std::array<ncollide3d::query::PointProjection, 6> projection_hp_;
#ifdef PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE
  // std::array<std::optional<ncollide3d::query::Contact>, 6> contact_hn_;
  std::array<ncollide3d::query::PointProjection, 6> projection_hn_;
#endif  // PUSH_CONSTRAINT_SYMMETRIC_DIFFERENCE

  const std::string name_pusher_;
  const std::string name_pushee_;

  // Eigen::Vector2d dir_push_object_ = Eigen::Vector2d::Zero();

  const World3& world_;

  std::unique_ptr<ncollide3d::shape::Compound> pushee_cylinder_;


};

}  // namespace logic_opt

#endif  // LOGIC_OPT_PUSH_CONSTRAINT_H_

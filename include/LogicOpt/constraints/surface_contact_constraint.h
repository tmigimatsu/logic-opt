/**
 * surface_contact_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_SURFACE_CONTACT_CONSTRAINT_H_
#define LOGIC_OPT_SURFACE_CONTACT_CONSTRAINT_H_

#include "LogicOpt/constraints/place_constraint.h"

namespace LogicOpt {

class SurfaceContactConstraint : virtual public Constraint, protected PlaceConstraint {

 public:
  enum class Direction { POS_X, POS_Y, POS_Z, NEG_X, NEG_Y, NEG_Z };

  SurfaceContactConstraint(World& world, size_t t_contact, const std::string& name_object,
                           const std::string& name_surface, Direction direction_surface);

  virtual ~SurfaceContactConstraint() {}

  // Optimization methods
  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  // Optimization properties
  virtual Type constraint_type(size_t idx_constraint) const override;

 protected:

  // Static methods
  static int Axis(Direction direction);
  static int SignAxis(Direction direction);
  static std::array<int, 2> OrthogonalAxes(Direction direction);

  // Internal methods
  virtual void ComputePlacePose(Eigen::Ref<const Eigen::MatrixXd> Q) override;

  virtual void ComputeError(Eigen::Ref<const Eigen::MatrixXd> Q) override;

  // Internal properties
  const std::string name_surface_;

  const int axis_normal_;
  const int sign_normal_;
  const std::array<int, 2> axes_surface_;

  Eigen::Vector4d surface_des_;
  Eigen::Vector4d surface_err_;

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_SURFACE_CONTACT_CONSTRAINT_H_

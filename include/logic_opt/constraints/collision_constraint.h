/**
 * collision_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: December 13, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_COLLISION_CONSTRAINT_H_
#define LOGIC_OPT_COLLISION_CONSTRAINT_H_

#include <set>     // std::set
#include <vector>  // std::vector

#include "logic_opt/constraints/constraint.h"

namespace logic_opt {

class CollisionConstraint : virtual public FrameConstraint {

 public:

  static constexpr size_t kNumConstraints = 1;
  static constexpr size_t kLenJacobian = logic_opt::FrameConstraint::kDof;
  static constexpr size_t kNumTimesteps = 1;

  CollisionConstraint(World3& world, size_t t_collision, bool ignore_control_target = true,
                      const std::set<std::string>& ignore_obstacles = {});

  virtual ~CollisionConstraint() = default;

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                               Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  virtual Type constraint_type(size_t idx_constraint) const { return Type::kInequality; }

 protected:

  virtual std::optional<ncollide3d::query::Contact>
  ComputeError(Eigen::Ref<const Eigen::MatrixXd> X, std::string* ee_closest = nullptr,
               std::string* object_closest = nullptr) const;

  virtual double ComputeDistance(Eigen::Ref<const Eigen::MatrixXd> X,
                                 const std::string& ee_frame,
                                 const std::string& object_frame) const;

  double x_err_ = 0.;
  std::optional<ncollide3d::query::Contact> contact_;

  std::string ee_closest_;
  std::string object_closest_;

  std::vector<std::string> ee_frames_;
  std::vector<std::string> objects_;

  const bool ignore_control_target_;

  const World3& world_;

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_COLLISION_CONSTRAINT_H_

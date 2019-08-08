/**
 * trajectory_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: December 13, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_TRAJECTORY_CONSTRAINT_H_
#define LOGIC_OPT_TRAJECTORY_CONSTRAINT_H_

// #define LOGIC_OPT_TRAJECTORY_CONVEX_HULL

#include "logic_opt/constraints/constraint.h"
#include "logic_opt/constraints/multi_constraint.h"

namespace logic_opt {

class TrajectoryConstraint : virtual public FrameConstraint {

 public:

#ifdef LOGIC_OPT_TRAJECTORY_CONVEX_HULL
  using ConvexHull = ncollide3d::shape::ConvexHull;
#else  // LOGIC_OPT_TRAJECTORY_CONVEX_HULL
  using ConvexHull = ncollide3d::shape::TriMesh;
#endif  // LOGIC_OPT_TRAJECTORY_CONVEX_HULL

  static constexpr size_t kNumConstraints = 1;
  static constexpr size_t kLenJacobian = 2 * logic_opt::FrameConstraint::kDof;
  static constexpr size_t kNumTimesteps = 1;

  TrajectoryConstraint(World3& world, size_t t_trajectory);

  virtual ~TrajectoryConstraint() = default;

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                               Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  virtual Type constraint_type(size_t idx_constraint) const { return Type::kInequality; }

 protected:

  virtual double ComputeError(Eigen::Ref<const Eigen::MatrixXd> X,
                              std::optional<ncollide3d::query::Contact>* out_contact = nullptr,
                              std::string* out_object_closest = nullptr);

  virtual ConvexHull ComputeConvexHull(Eigen::Ref<const Eigen::MatrixXd> X,
                                       const std::string& ee_frame);

  virtual double ComputeDistance(Eigen::Ref<const Eigen::MatrixXd> X,
                                 const std::string& ee_frame,
                                 const std::string& object_frame,
                                 const ConvexHull& ee_convex_hull,
                                 double max_dist,
                                 std::optional<ncollide3d::query::Contact>* out_contact = nullptr) const;

  virtual double ComputeJacobianError(Eigen::Ref<const Eigen::MatrixXd> X,
                                      const std::string& ee_frame,
                                      const std::string& object_frame,
                                      double max_dist);

  double x_err_ = 0.;
  std::optional<ncollide3d::query::Contact> contact_;

  std::vector<std::string> ee_frames_;
  std::vector<std::string> object_frames_;

  std::string object_closest_;

  std::vector<std::vector<std::array<double, 3>>> augmented_ee_points_;

  const World3& world_;

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_TRAJECTORY_CONSTRAINT_H_

/**
 * linear_distance_objective.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 26, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_OBJECTIVES_LINEAR_DISTANCE_OBJECTIVE_H_
#define LOGIC_OPT_OBJECTIVES_LINEAR_DISTANCE_OBJECTIVE_H_

#include <spatial_opt/objective.h>

#include "logic_opt/world.h"

namespace logic_opt {

class LinearDistanceObjective : virtual public spatial_opt::Objective {
 public:
  LinearDistanceObjective(const World& world, const std::string& name_ee,
                          double coeff = 1.)
      : spatial_opt::Objective(coeff, "objective_lin_vel"),
        world_(world),
        name_ee_(name_ee) {}

  virtual ~LinearDistanceObjective() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::MatrixXd> Gradient) override;

  static Eigen::Vector3d qxqinv(const Eigen::Quaterniond& q,
                                const Eigen::Vector3d& x);

  static Eigen::Matrix3d dqxqinv_dx(const Eigen::Quaterniond& q);

  static Eigen::Matrix<double, 3, 4> dqxqinv_dq(const Eigen::Quaterniond& q,
                                                const Eigen::Vector3d& x);

  /**
   * Computes Jacobian of normalized quaternion wrt the quaternion itself.
   */
  static Eigen::Matrix4d dqhat_dq(const Eigen::Quaterniond& q);

  /**
   * Computes the flattened Jacobian for all frame variables at the given
   * timestep.
   */
  static Eigen::Matrix3Xd PositionJacobian(const World& world,
                                           const std::string& name_frame,
                                           Eigen::Ref<const Eigen::MatrixXd> X,
                                           size_t t);

 protected:
  const World& world_;
  const std::string name_ee_;
};

}  // namespace logic_opt

#endif  // LOGIC_OPT_OBJECTIVES_LINEAR_DISTANCE_OBJECTIVE_H_

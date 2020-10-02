/**
 * angular_distance_objective.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 26, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_OBJECTIVES_ANGULAR_DISTANCE_OBJECTIVE_H_
#define LOGIC_OPT_OBJECTIVES_ANGULAR_DISTANCE_OBJECTIVE_H_

#include <spatial_opt/objective.h>

#include <array>     // std::array
#include <optional>  // std::optional

#include "logic_opt/world.h"

namespace logic_opt {

class AngularDistanceObjective : virtual public spatial_opt::Objective {
 public:
  AngularDistanceObjective(const World& world, const std::string& name_ee,
                           double coeff = 1.)
      : spatial_opt::Objective(coeff, "objective_ang_vel"),
        world_(world),
        name_ee_(name_ee) {}

  virtual ~AngularDistanceObjective() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::MatrixXd> Gradient) override;

  static Eigen::Quaterniond aqinvb(const Eigen::Quaterniond& q,
                                   const Eigen::Quaterniond& a,
                                   const Eigen::Quaterniond& b);

  static Eigen::Quaterniond bqc(const Eigen::Quaterniond& q,
                                const Eigen::Quaterniond& b,
                                const Eigen::Quaterniond& c);

  static Eigen::Quaterniond aqinvbqc(const Eigen::Quaterniond& q,
                                     const Eigen::Quaterniond& a,
                                     const Eigen::Quaterniond& b,
                                     const Eigen::Quaterniond& c);

  static Eigen::Matrix<double, 1, 4> daqinvbw_dq(const Eigen::Quaterniond& q,
                                                 const Eigen::Quaterniond& a,
                                                 const Eigen::Quaterniond& b);

  static Eigen::Matrix<double, 3, 4> daqinvbv_dq(const Eigen::Quaterniond& q,
                                                 const Eigen::Quaterniond& a,
                                                 const Eigen::Quaterniond& b);

  static Eigen::Matrix<double, 1, 4> dbqcw_dq(const Eigen::Quaterniond& q,
                                              const Eigen::Quaterniond& b,
                                              const Eigen::Quaterniond& c);

  static Eigen::Matrix<double, 3, 4> dbqcv_dq(const Eigen::Quaterniond& q,
                                              const Eigen::Quaterniond& b,
                                              const Eigen::Quaterniond& c);

  static Eigen::Matrix<double, 1, 4> daqinvbqcw_dq(const Eigen::Quaterniond& q,
                                                   const Eigen::Quaterniond& a,
                                                   const Eigen::Quaterniond& b,
                                                   const Eigen::Quaterniond& c);

  static Eigen::Matrix<double, 3, 4> daqinvbqcv_dq(const Eigen::Quaterniond& q,
                                                   const Eigen::Quaterniond& a,
                                                   const Eigen::Quaterniond& b,
                                                   const Eigen::Quaterniond& c);

  /**
   * Computes Jacobian of normalized quaternion wrt the quaternion itself.
   */
  static Eigen::Matrix4d dqhat_dq(const Eigen::Quaterniond& q);

  /**
   * Calls the appropriate quaternion Jacobian functions based on `Qs`.
   */
  static Eigen::Matrix4d QuaternionJacobian(
      const std::array<std::optional<Eigen::Quaterniond>, 3>& Qs,
      const Eigen::Quaterniond& q);

  /**
   * Computes the flattened Jacobian for all frame variables at the given
   * timestep.
   */
  static Eigen::Matrix4Xd OrientationJacobian(
      const World& world, const std::string& name_frame,
      Eigen::Ref<const Eigen::MatrixXd> X, size_t t);

 protected:
  const World& world_;
  const std::string name_ee_;
};

}  // namespace logic_opt

#endif  // LOGIC_OPT_OBJECTIVES_ANGULAR_DISTANCE_OBJECTIVE_H_

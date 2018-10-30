/**
 * objectives.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 26, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_OBJECTIVES_H_
#define TRAJ_OPT_OBJECTIVES_H_

#include <SpatialDyn/SpatialDyn.h>

#include <memory>      // std::unique_ptr
#include <vector>      // std::vector

namespace TrajOpt {

class Objective {

 public:
  Objective(double coeff = 1.) : coeff(coeff) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) = 0;
  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::MatrixXd> Gradient) = 0;

  const double coeff;

};

typedef std::vector<std::unique_ptr<Objective>> Objectives;

class JointPositionObjective : public Objective {

 public:
  JointPositionObjective(Eigen::Ref<const Eigen::VectorXd> q_des, double coeff = 1.)
      : Objective(coeff), q_des(q_des) {}

  void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) override;
  void Gradient(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::MatrixXd> Gradient) override;

  const Eigen::VectorXd q_des;

};

class JointVelocityObjective : public Objective {

 public:
  JointVelocityObjective(double coeff = 1.) : Objective(coeff) {}

  void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) override;
  void Gradient(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::MatrixXd> Gradient) override;

  const Eigen::VectorXd q_des;

};

class JointAccelerationObjective : public Objective {

 public:
  JointAccelerationObjective(double coeff = 1.) : Objective(coeff) {}

  void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) override;
  void Gradient(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::MatrixXd> Gradient) override;

  const Eigen::VectorXd q_des;

};

class LinearVelocityObjective : public Objective {

 public:
  LinearVelocityObjective(const SpatialDyn::ArticulatedBody& ab, double coeff = 1.)
      : Objective(coeff), ab_(ab) {}

  void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) override;
  void Gradient(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::MatrixXd> Grad) override;

  const Eigen::VectorXd q_des;

 private:
  const SpatialDyn::ArticulatedBody& ab_;

};

// TODO: Derive correct jacobian (doesn't work)
class AngularVelocityObjective : public Objective {

 public:
  AngularVelocityObjective(const SpatialDyn::ArticulatedBody& ab, double coeff = 1.)
      : Objective(coeff), ab_(ab) {}

  void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) override;
  void Gradient(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::MatrixXd> Grad) override;

  const Eigen::VectorXd q_des;

 private:
  const SpatialDyn::ArticulatedBody& ab_;

};

}  // namespace TrajOpt

#endif  // TRAJ_OPT_OBJECTIVES_H_

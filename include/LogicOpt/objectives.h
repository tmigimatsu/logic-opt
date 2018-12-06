/**
 * objectives.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 26, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_OBJECTIVES_H_
#define LOGIC_OPT_OBJECTIVES_H_

#include <SpatialDyn/SpatialDyn.h>

#include <fstream>  // std::ofstream
#include <memory>   // std::unique_ptr
#include <vector>   // std::vector

namespace LogicOpt {

class Objective {

 public:
  Objective(double coeff = 1., const std::string& name = "")
      : coeff(coeff), name(name) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective);

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::MatrixXd> Gradient) = 0;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q, double sigma,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {};

  virtual void HessianStructure(Eigen::SparseMatrix<bool>& Hessian, size_t T) {};

  const double coeff;
  const std::string name;
  std::ofstream log;

};

typedef std::vector<std::unique_ptr<Objective>> Objectives;

class JointPositionObjective : public Objective {

 public:
  JointPositionObjective(Eigen::Ref<const Eigen::VectorXd> q_des, double coeff = 1.)
      : Objective(coeff, "objective_joint_pos"), q_des(q_des) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::MatrixXd> Gradient) override;

  const Eigen::VectorXd q_des;

};

class JointVelocityObjective : public Objective {

 public:
  JointVelocityObjective(double coeff = 1.)
      : Objective(coeff, "objective_joint_vel") {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::MatrixXd> Gradient) override;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q, double sigma,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

  virtual void HessianStructure(Eigen::SparseMatrix<bool>& Hessian, size_t T) override;

};

class JointAccelerationObjective : public Objective {

 public:
  JointAccelerationObjective(double coeff = 1.)
      : Objective(coeff, "objective_joint_acc") {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::MatrixXd> Gradient) override;

};

class LinearVelocityObjective : public Objective {

 public:
  LinearVelocityObjective(const SpatialDyn::ArticulatedBody& ab, double coeff = 1.)
      : Objective(coeff, "objective_lin_vel"), ab_(ab) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::MatrixXd> Grad) override;

 private:
  const SpatialDyn::ArticulatedBody& ab_;

};

// TODO: Derive correct jacobian (doesn't work)
class AngularVelocityObjective : public Objective {

 public:
  AngularVelocityObjective(const SpatialDyn::ArticulatedBody& ab, double coeff = 1.)
      : Objective(coeff), ab_(ab) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::MatrixXd> Grad) override;

 private:
  const SpatialDyn::ArticulatedBody& ab_;

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_OBJECTIVES_H_

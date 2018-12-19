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

#include "LogicOpt/world.h"

namespace LogicOpt {

class Objective;
using Objectives = std::vector<std::unique_ptr<Objective>>;

class Objective {

 public:

  Objective(double coeff, const std::string& name)
      : coeff_(coeff), name(name) {}

  virtual ~Objective() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective);

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> X, Eigen::Ref<Eigen::MatrixXd> Gradient) = 0;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> X, double sigma,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {}

  virtual void HessianStructure(Eigen::SparseMatrix<bool>& Hessian, size_t T) {}

  virtual double coeff() const { return coeff_; }

  // Debug properties
  std::string name;
  std::ofstream log;

 protected:

  const double coeff_;

};

class MinNormObjective : virtual public Objective {

 public:

  MinNormObjective(double coeff = 1.)
      : Objective(coeff, "objective_min_norm") {}

  virtual ~MinNormObjective() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> X, Eigen::Ref<Eigen::MatrixXd> Gradient) override;

};

class LinearVelocityObjective : virtual public Objective {

 public:

  LinearVelocityObjective(const World& world, const std::string& name_ee, double coeff = 1.)
      : Objective(coeff, "objective_lin_vel"), world_(world), name_ee_(name_ee) {}

  virtual ~LinearVelocityObjective() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> X, Eigen::Ref<Eigen::MatrixXd> Gradient) override;

 protected:

  const World& world_;
  const std::string name_ee_;

};

#if 0
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

class AngularVelocityObjective : public Objective {

 public:
  AngularVelocityObjective(const SpatialDyn::ArticulatedBody& ab, double coeff = 1.)
      : Objective(coeff), ab_(ab) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q, double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> Q, Eigen::Ref<Eigen::MatrixXd> Grad) override;

 private:
  const SpatialDyn::ArticulatedBody& ab_;

};

class LinearVelocityCartesianObjective : public Objective {

 public:
  LinearVelocityCartesianObjective(const FrameVariables& skeleton, double coeff = 1.)
      : Objective(coeff, "objective_lin_vel_cart"), skeleton_(skeleton) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> X, Eigen::Ref<Eigen::MatrixXd> Grad) override;

 private:
  const FrameVariables& skeleton_;

};

class AngularVelocityCartesianObjective : public Objective {

 public:
  AngularVelocityCartesianObjective(const FrameVariables& skeleton, double coeff = 1.)
      : Objective(coeff, "objective_ang_vel_cart"), skeleton_(skeleton) {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> X, Eigen::Ref<Eigen::MatrixXd> Grad) override;

 private:
  const FrameVariables& skeleton_;

};
#endif

}  // namespace LogicOpt

#endif  // LOGIC_OPT_OBJECTIVES_H_

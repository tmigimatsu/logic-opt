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

#include <spatial_dyn/spatial_dyn.h>

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

  LinearVelocityObjective(const World3& world, const std::string& name_ee, double coeff = 1.)
      : Objective(coeff, "objective_lin_vel"), world_(world), name_ee_(name_ee) {}

  virtual ~LinearVelocityObjective() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> X, Eigen::Ref<Eigen::MatrixXd> Gradient) override;

 protected:

  const World3& world_;
  const std::string name_ee_;

};

class AngularVelocityObjective : virtual public Objective {

 public:

  AngularVelocityObjective(const World3& world, const std::string& name_ee, double coeff = 1.)
      : Objective(coeff, "objective_ang_vel"), world_(world), name_ee_(name_ee) {}

  virtual ~AngularVelocityObjective() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> X, Eigen::Ref<Eigen::MatrixXd> Gradient) override;

 protected:

  const World3& world_;
  const std::string name_ee_;

};

}  // namespace LogicOpt

#endif  // LOGIC_OPT_OBJECTIVES_H_

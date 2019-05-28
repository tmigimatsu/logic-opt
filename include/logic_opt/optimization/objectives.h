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

#include "logic_opt/world.h"

namespace logic_opt {

class Objective;
using Objectives = std::vector<std::unique_ptr<Objective>>;

class Objective {

 public:

  Objective(double coeff, const std::string& name)
      : coeff_(coeff), name(name) {}

  virtual ~Objective() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective);

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> X, Eigen::Ref<Eigen::MatrixXd> Gradient);

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> X, double sigma,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) {}

  virtual void HessianStructure(Eigen::SparseMatrix<bool>& Hessian, size_t T) {}

  virtual double coeff() const { return coeff_; }

  // Debug properties
  std::string name;

  virtual void OpenObjectiveLog(const std::string& filepath);
  virtual void OpenGradientLog(const std::string& filepath);

  virtual void CloseObjectiveLog() { log_objective_.close(); }
  virtual void CloseGradientLog() { log_gradient_.close(); }

 protected:

  const double coeff_;

  std::ofstream log_objective_;
  std::ofstream log_gradient_;

};

class MinL2NormObjective : virtual public Objective {

 public:

  MinL2NormObjective(size_t row_start, size_t num_rows, double coeff = 1.)
      : Objective(coeff, "objective_min_norm"), row_start_(row_start), num_rows_(num_rows) {}

  virtual ~MinL2NormObjective() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> X, Eigen::Ref<Eigen::MatrixXd> Gradient) override;

 private:

  const size_t row_start_;
  const size_t num_rows_;

};

class MinL1NormObjective : virtual public Objective {

 public:

  MinL1NormObjective(size_t row_start, size_t num_rows, double coeff = 1.)
      : Objective(coeff, "objective_min_norm"), row_start_(row_start), num_rows_(num_rows),
        X_0(Eigen::VectorXd::Zero(num_rows)) {}

  virtual ~MinL1NormObjective() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> X, Eigen::Ref<Eigen::MatrixXd> Gradient) override;

  Eigen::MatrixXd X_0;

 private:

  const size_t row_start_;
  const size_t num_rows_;

};

template<int Dim>
class LinearVelocityObjective : virtual public Objective {

 public:

  LinearVelocityObjective(const World<Dim>& world, const std::string& name_ee, double coeff = 1.)
      : Objective(coeff, "objective_lin_vel"), world_(world), name_ee_(name_ee) {}

  virtual ~LinearVelocityObjective() {}

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X, double& objective) override;

  virtual void Gradient(Eigen::Ref<const Eigen::MatrixXd> X, Eigen::Ref<Eigen::MatrixXd> Gradient) override;

 protected:

  const World<Dim>& world_;
  const std::string name_ee_;

};

typedef LinearVelocityObjective<3> LinearVelocityObjective3;
typedef LinearVelocityObjective<2> LinearVelocityObjective2;

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

}  // namespace logic_opt

#endif  // LOGIC_OPT_OBJECTIVES_H_

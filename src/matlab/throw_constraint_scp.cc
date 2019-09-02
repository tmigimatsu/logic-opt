/**
 * throw_constraint_scp.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 4, 2019
 * Authors: Toki Migimatsu
 */

#include "logic_opt/control/throw_constraint_scp.h"

#include <cmath>  // std::lround

#include "MatlabEngine.hpp"
#include "MatlabDataArray.hpp"

namespace {

static const double tf = 1.;
static const double dt = 0.01;
static const size_t T = std::lround(tf / dt);

}  // namespace

namespace logic_opt {

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ThrowConstraintScp(const spatial_dyn::ArticulatedBody& ab,
                                                               Eigen::Ref<const Eigen::VectorXd> q_start,
                                                               Eigen::Ref<const Eigen::Vector3d> x_target,
                                                               const Eigen::Vector3d& ee_offset) {

  // Start MATLAB engine synchronously
  std::shared_ptr<matlab::engine::MATLABEngine> matlab = matlab::engine::startMATLAB();

  // Factory to create MATALB data arrays
  matlab::data::ArrayFactory factory;

  // cd to function directory
  matlab->feval(u"cd", { factory.createCharArray(u"/scr2/takatoki/Documents/School/aa203") });

  // spatial_dyn::ArticulatedBody* ptr_ab = new spatial_dyn::ArticulatedBody(ab);
  matlab::data::TypedArray<double> q_arr = factory.createArray<double>({ ab.dof(), 1 });
  Eigen::Map<Eigen::VectorXd> q(&*q_arr.begin(), ab.dof());
  q = q_start;

  std::vector<matlab::data::Array> args({
      // factory.createScalar<uintptr_t>(reinterpret_cast<uintptr_t>(ptr_ab)),  // ab
      factory.createCharArray(u"resources/kuka_iiwa/kuka_iiwa.urdf"),  // ab
      q_arr,  // q_0
      factory.createArray<double>({ 3, 1 }, { x_target(0), x_target(1), x_target(2) }),  // x_g
      factory.createScalar<double>(tf),  // t_f
      factory.createScalar<double>(dt),
      factory.createArray<double>({ 3, 1 }, { ee_offset(0), ee_offset(1), ee_offset(2) })  // dt
  });

  const std::vector<matlab::data::Array> output = matlab->feval(u"ThrowConstraint", 2, args);
  const matlab::data::TypedArray<double> Q_output = output[0];
  const matlab::data::TypedArray<double> X_output = output[1];
  const matlab::data::ArrayDimensions dim_X = X_output.getDimensions();
  // const matlab::data::TypedArray<double> output =
  Eigen::Map<const Eigen::MatrixXd> Q(&*Q_output.begin(), ab.dof(), T);
  Eigen::Map<const Eigen::MatrixXd> X(&*X_output.begin(), dim_X[0], dim_X[1]);

  return { Q, X };
}

}  // namespace logic_opt

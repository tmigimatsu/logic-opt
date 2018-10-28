/**
 * ipopt.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 25, 2018
 * Authors: Toki Migimatsu
 */

#ifndef TRAJ_OPT_IPOPT_H_
#define TRAJ_OPT_IPOPT_H_

#include "constraints.h"
#include "objectives.h"

#include <SpatialDyn/SpatialDyn.h>
#include <IpTNLP.hpp>

#include <functional>  // std::function
#include <map>         // std::map
#include <memory>      // std::unique_ptr
#include <mutex>       // std::mutex
#include <string>      // std::string
#include <vector>      // std::vector

namespace TrajOpt {
namespace Ipopt {

std::vector<Eigen::VectorXd> Trajectory(const SpatialDyn::ArticulatedBody& ab,
                                        const std::map<std::string,
                                                       SpatialDyn::RigidBody>& world_objects,
                                        const Eigen::VectorXd& q_des,
                                        size_t T);

std::vector<Eigen::VectorXd> Trajectory(const SpatialDyn::ArticulatedBody& ab,
                                        const std::map<std::string,
                                                       SpatialDyn::RigidBody>& world_objects,
                                        const Eigen::Vector3d& x_des,
                                        const Eigen::Quaterniond& quat_des,
                                        size_t T);

class NonlinearProgram : public ::Ipopt::TNLP {

 public:

  NonlinearProgram(SpatialDyn::ArticulatedBody ab, const Eigen::VectorXd& q_des, size_t T)
      : ab_(ab), q_des_(q_des), T_(T) {
    q_0_ = ab.q();
    objectives_.emplace_back(new JointVelocityObjective());
    constraints_.emplace_back(new JointPositionConstraint(ab, 0, ab.q()));
    constraints_.emplace_back(new JointPositionConstraint(ab, T - 1, q_des));
  }

  NonlinearProgram(SpatialDyn::ArticulatedBody ab, const Eigen::Vector3d& x_des,
        const Eigen::Quaterniond& quat_des, size_t T)
      : ab_(ab), x_des_(x_des), quat_des_(quat_des), T_(T) {
    q_0_ = ab.q();
    // objectives_.emplace_back(new JointPositionObjective(ab.q(), 0.1));
    objectives_.emplace_back(new JointVelocityObjective());
    // objectives_.emplace_back(new JointAccelerationObjective());
    // objectives_.emplace_back(new LinearVelocityObjective(ab));
    constraints_.emplace_back(new JointPositionConstraint(ab, 0, ab.q()));
    constraints_.emplace_back(new CartesianPoseConstraint(ab, T - 1, x_des, quat_des));
  }

  SpatialDyn::ArticulatedBody ab_;
  std::mutex ab_lock_;
  size_t T_;
  Eigen::VectorXd q_0_;
  Eigen::VectorXd q_des_;
  Eigen::Vector3d x_des_;
  Eigen::Quaterniond quat_des_;
  std::vector<Eigen::VectorXd> trajectory_;
  std::string status_;

  std::vector<std::unique_ptr<Objective>> objectives_;
  std::vector<std::unique_ptr<Constraint>> constraints_;

  bool get_nlp_info(int& n, int& m, int& nnz_jac_g,
                    int& nnz_h_lag, IndexStyleEnum& index_style) override;

  bool get_bounds_info(int n, double* x_l, double* x_u,
                       int m, double* g_l, double* g_u) override;

  bool get_starting_point(int n, bool init_x, double* x,
                          bool init_z, double* z_L, double* z_U,
                          int m, bool init_lambda, double* lambda) override;

  bool eval_f(int n, const double* x, bool new_x, double& obj_value) override;

  bool eval_grad_f(int n, const double* x, bool new_x, double* grad_f) override;

  bool eval_g(int n, const double* x, bool new_x, int m, double* g) override;

  bool eval_jac_g(int n, const double* x, bool new_x,
                  int m, int nele_jac, int* iRow, int *jCol, double* values) override;

  void finalize_solution(::Ipopt::SolverReturn status,
                         int n, const double* x, const double* z_L, const double* z_U,
                         int m, const double* g, const double* lambda,
                         double obj_value, const ::Ipopt::IpoptData* ip_data,
                         ::Ipopt::IpoptCalculatedQuantities* ip_cq) override;

};

}  // namespace Ipopt
}  // namespace TrajOpt

#endif  // TRAJ_OPT_IPOPT_H_

/**
 * main.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 19, 2018
 * Authors: Toki Migimatsu
 */

#include <SpatialDyn/SpatialDyn.h>

// #include "gurobi.h"
#include "ipopt.h"
#include "nlopt.h"
#include "objectives.h"
#include "constraints.h"

#include <cmath>     // M_PI
#include <csignal>   // std::signal
#include <iostream>  // std::cout
#include <map>       // std::map
#include <string>    // std::string

static volatile std::sig_atomic_t g_runloop = true;
void stop(int) { g_runloop = false; }

int main(int argc, char *argv[]) {

  // Load robot
  SpatialDyn::ArticulatedBody ab = SpatialDyn::Urdf::LoadModel("../resources/kuka_iiwa/kuka_iiwa.urdf");

  // Create world objects
  std::map<std::string, SpatialDyn::RigidBody> world_objects;
  // {
  //   SpatialDyn::RigidBody table("table");
  //   table.graphics.geometry.type = SpatialDyn::Geometry::Type::BOX;
  //   table.graphics.geometry.scale = Eigen::Vector3d(0.6, 0.6, 0.02);
  //   table.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.2, -0.3, 1.0));
  //   // table.graphics.geometry.scale = Eigen::Vector3d(1., 1., 0.02);
  //   // table.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0., 0., 0.8));
  //   world_objects[table.name] = table;
  // }
  {
    SpatialDyn::RigidBody box("box");
    box.graphics.geometry.type = SpatialDyn::Geometry::Type::BOX;
    box.graphics.geometry.scale = Eigen::Vector3d(0.05, 0.05, 0.05);
    box.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0., -0.5, 0.3));
    world_objects[box.name] = box;
  }
  {
    SpatialDyn::RigidBody box("box_start");
    box.graphics.geometry.type = SpatialDyn::Geometry::Type::BOX;
    box.graphics.geometry.scale = Eigen::Vector3d(0.05, 0.05, 0.05);
    box.graphics.material.rgba(3) = 0.25;
    box.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0., -0.5, 0.3));
    world_objects[box.name] = box;
  }
  {
    SpatialDyn::RigidBody box("box_end");
    box.graphics.geometry.type = SpatialDyn::Geometry::Type::BOX;
    box.graphics.geometry.scale = Eigen::Vector3d(0.05, 0.05, 0.05);
    box.graphics.material.rgba(3) = 0.25;
    box.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.3, -0.5, 0.3));
    world_objects[box.name] = box;
  }

  // Set up timer and Redis
  SpatialDyn::Timer timer(1000);
  SpatialDyn::RedisClient redis_client;
  redis_client.connect();
  redis_client.set("spatialdyn::kuka_iiwa", SpatialDyn::Json::Serialize(ab).dump());
  for (const std::pair<std::string, SpatialDyn::RigidBody>& key_val : world_objects) {
    const SpatialDyn::RigidBody& object = key_val.second;
    redis_client.set("spatialdyn::objects::" + object.name, SpatialDyn::Json::Serialize(object).dump());
  }

  // Set up signal handlers
  std::signal(SIGINT, stop);

  // ab.set_q(Eigen::VectorXd::Zero(ab.dof()));
  Eigen::VectorXd q_des(ab.dof());
  q_des << 90., -30., 0., 60., 0., -90., -60.;
  q_des *= M_PI / 180.;

  Eigen::Vector3d x_des(0., -0.6, 0.6);
  Eigen::Quaterniond quat_des(0., 1., 0., 0.);
  Eigen::Vector3d ee_offset(0., 0., 0.06);

  const size_t T = 30;
  const size_t t_pick = 10;
  const size_t t_place = 20;
  std::vector<Eigen::VectorXd> q_des_traj;

  // Eigen::MatrixXd Q_0(ab.dof(), T);
  // for (size_t t = 0; t < T; t++) {
  //   Q_0.col(t).fill(static_cast<double>(t) / T);
  // }
  // TrajOpt::JointVariables variables(ab, T, Q_0);
  TrajOpt::JointVariables variables(ab, T, q_des);

  TrajOpt::Objectives objectives;
  objectives.emplace_back(new TrajOpt::JointVelocityObjective(ab.dof(), T));

  // ab.set_q(Eigen::VectorXd::Ones(ab.dof()));
  // Eigen::Tensor<double, 3> H = SpatialDyn::Hessian(ab);
  // Eigen::Matrix6Xd J = SpatialDyn::Jacobian(ab);
  // std::cout << J << std::endl << std::endl;

  // for (size_t i = 0; i < ab.dof(); i++) {
  //   std::cout << H.chip(i, 0) << std::endl << std::endl;
  // }
  // return 0;
  // Eigen::MatrixXd H(ab.dof() * T, ab.dof() * T);
  // H.setZero();
  // const size_t len_hessian = (3 * T - 2) * ab.dof();
  // Eigen::VectorXd h(len_hessian);
  // h.setZero();
  // Eigen::ArrayXi idx_i(len_hessian);
  // Eigen::ArrayXi idx_j(len_hessian);

  // objectives.back()->Hessian(q_des, 1, h);
  // objectives.back()->HessianIndices(idx_i, idx_j);
  // for (size_t i = 0; i < len_hessian; i++) {
  //   H(idx_i(i), idx_j(i)) = h(i);
  // }
  // std::cout << H << std::endl;
  // return 0;

  // objectives.emplace_back(new TrajOpt::JointVelocityObjective(0.01));
  // objectives.emplace_back(new TrajOpt::LinearVelocityObjective(ab));

  TrajOpt::Constraints constraints;
  constraints.emplace_back(new TrajOpt::JointPositionConstraint(ab, 0, ab.q()));
  constraints.emplace_back(new TrajOpt::PickConstraint(ab, world_objects["box"], t_pick, ee_offset));
  constraints.emplace_back(new TrajOpt::PlaceConstraint(ab, world_objects["box"], world_objects["box_end"].T_to_parent().translation(), Eigen::Quaterniond::Identity(), t_pick, t_place));
  // constraints.emplace_back(new TrajOpt::JointPositionConstraint(ab, T - 1, q_des));
  constraints.emplace_back(new TrajOpt::CartesianPoseConstraint(ab, T - 1, x_des, quat_des));
  // constraints.emplace_back(new TrajOpt::AboveTableConstraint(ab, world_objects["table"], 0, T));

  TrajOpt::Nlopt::OptimizationData data;
  q_des_traj = TrajOpt::Nlopt::Trajectory(variables, objectives, constraints, &data);

  // TrajOpt::Ipopt::OptimizationData data;
  // q_des_traj = TrajOpt::Ipopt::Trajectory(variables, objectives, constraints, &data);

  for (const Eigen::VectorXd& q : q_des_traj) {
    std::cout << q.transpose() << std::endl;
  }
  std::cout << std::endl;

  // Eigen::Map<const Eigen::MatrixXd> Q(&data.x[0], ab.dof(), T);
  // Eigen::VectorXd c(T);
  // constraints.back()->Evaluate(Q, c);
  // std::cout << "CONSTRAINTS: " << c.transpose() << std::endl;

  // std::vector<Eigen::VectorXd> q_des_traj = TaskSpaceTrajectory(ab, world_objects, x_des, quat_des, T);
  // std::vector<Eigen::VectorXd> q_des_traj = TaskSpaceTrajectory(ab, q_des);

  size_t idx_trajectory = 0;
  while (g_runloop) {
    timer.Sleep();
    Eigen::VectorXd q_err = ab.q() - q_des_traj[idx_trajectory];
    Eigen::VectorXd dq_err = ab.dq();
    Eigen::VectorXd ddq = -10 * q_err - 6 * dq_err;
    Eigen::VectorXd tau = SpatialDyn::InverseDynamics(ab, ddq, {}, true, true);

    Eigen::Isometry3d T_object_to_ee;
    if (idx_trajectory > t_pick && idx_trajectory <= t_place) {
      const Eigen::Isometry3d& T_ee_to_world = ab.T_to_world(-1);
      const Eigen::Isometry3d& T_object_to_world = world_objects["box"].T_to_parent();
      T_object_to_ee = T_ee_to_world.inverse() * T_object_to_world;
    }

    SpatialDyn::Integrate(ab, tau, timer.dt());

    if (idx_trajectory > t_pick && idx_trajectory <= t_place) {
      world_objects["box"].set_T_to_parent(ab.T_to_world(-1) * T_object_to_ee);
      redis_client.set("spatialdyn::objects::box", SpatialDyn::Json::Serialize(world_objects["box"]).dump());
    }

    redis_client.set("spatialdyn::kuka_iiwa::sensor::q", ab.q().toMatlab());
    redis_client.set("spatialdyn::kuka_iiwa::trajectory::pos", SpatialDyn::Position(ab).toMatlab());
    redis_client.commit();

    if (q_err.norm() < 0.01) {
      if (idx_trajectory < q_des_traj.size() - 1) {
        idx_trajectory++;
      } else if (dq_err.norm() < 0.0001) {
        break;
      }
    }
  }

  std::cout << "Simulated " << timer.time_sim() << "s in " << timer.time_elapsed() << "s." << std::endl;

  return 0;
}

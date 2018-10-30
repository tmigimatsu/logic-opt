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
#include <string>    // std::string

static volatile std::sig_atomic_t g_runloop = true;
void stop(int) { g_runloop = false; }

int main(int argc, char *argv[]) {

  // Load robot
  SpatialDyn::ArticulatedBody ab = SpatialDyn::Urdf::LoadModel("../resources/kuka_iiwa/kuka_iiwa.urdf");

  // Create world objects
  std::map<std::string, SpatialDyn::RigidBody> world_objects;
  SpatialDyn::RigidBody table("table");
  table.graphics.geometry.type = SpatialDyn::GeometryType::BOX;
  // table.graphics.geometry.scale = Eigen::Vector3d(0.2, 0.2, 0.02);
  // table.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.2, -0.3, 1.0));
  table.graphics.geometry.scale = Eigen::Vector3d(1., 1., 0.02);
  table.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0., 0., 0.8));
  world_objects[table.name] = table;

  // Set up timer and Redis
  SpatialDyn::Timer timer(1000);
  SpatialDyn::RedisClient redis_client;
  redis_client.connect();
  redis_client.set("spatialdyn::kuka_iiwa", SpatialDyn::Json::Serialize(ab).dump());
  for (const std::pair<std::string, SpatialDyn::RigidBody>& key_val : world_objects) {
    const SpatialDyn::RigidBody& object = key_val.second;
    redis_client.set("spatialdyn::" + object.name, SpatialDyn::Json::Serialize(object).dump());
  }

  // Set up signal handlers
  std::signal(SIGINT, stop);

  // ab.set_q(Eigen::VectorXd::Zero(ab.dof()));
  Eigen::VectorXd q_des(ab.dof());
  q_des << 90., -30., 0., 60., 0., -90., -60.;
  q_des *= M_PI / 180.;

  Eigen::Vector3d x_des(0., -0.6, 0.6);
  Eigen::Quaterniond quat_des(0., 1., 0., 0.);

  const size_t T = 10;
  std::vector<Eigen::VectorXd> q_des_traj;

  ab.set_q(Eigen::VectorXd::Ones(ab.dof()));
  TrajOpt::JointVariables variables(ab, T);

  TrajOpt::Objectives objectives;
  objectives.emplace_back(new TrajOpt::JointVelocityObjective());

  TrajOpt::Constraints constraints;
  constraints.emplace_back(new TrajOpt::JointPositionConstraint(ab, 0, ab.q()));
  constraints.emplace_back(new TrajOpt::JointPositionConstraint(ab, T - 1, q_des));

  // TrajOpt::Ipopt::OptimizationData data;
  // q_des_traj = TrajOpt::Ipopt::Trajectory(variables, objectives, constraints, &data);

  TrajOpt::Nlopt::OptimizationData data;
  q_des_traj = TrajOpt::Nlopt::Trajectory(variables, objectives, constraints, &data);

  for (const Eigen::VectorXd& q : q_des_traj) {
    std::cout << q.transpose() << std::endl;
  }
  std::cout << std::endl;
  // std::vector<Eigen::VectorXd> q_des_traj = TaskSpaceTrajectory(ab, world_objects, x_des, quat_des, T);
  // std::vector<Eigen::VectorXd> q_des_traj = TaskSpaceTrajectory(ab, q_des);

  size_t idx_trajectory = 0;
  while (g_runloop) {
    timer.Sleep();
    Eigen::VectorXd q_err = ab.q() - q_des_traj[idx_trajectory];
    Eigen::VectorXd dq_err = ab.dq();
    Eigen::VectorXd ddq = -10 * q_err - 6 * dq_err;
    Eigen::VectorXd tau = SpatialDyn::InverseDynamics(ab, ddq, {}, true, true);

    SpatialDyn::Integrate(ab, tau, timer.dt());
    redis_client.set("spatialdyn::kuka_iiwa::sensor::q", ab.q().toMatlab());
    redis_client.set("spatialdyn::kuka_iiwa::trajectory::pos", SpatialDyn::Position(ab).toMatlab());
    redis_client.commit();

    if (q_err.norm() < 0.01) {
      if (idx_trajectory < q_des_traj.size() - 1) {
        idx_trajectory++;
      } else if (dq_err.norm() < 0.001) {
        break;
      }
    }
  }

  std::cout << "Simulated " << timer.time_sim() << "s in " << timer.time_elapsed() << "s." << std::endl;

  return 0;
}

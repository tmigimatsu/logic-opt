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

#include <chrono>    // std::chrono
#include <cmath>     // M_PI
#include <csignal>   // std::signal
#include <fstream>   // std::ofstream
#include <iostream>  // std::cout
#include <map>       // std::map
#include <string>    // std::string
#include <time.h>    // ::gmtime_r, std::strftime
#include <sys/stat.h>  // mkdir

std::string DateTimeString(std::chrono::system_clock::time_point t) {
  time_t as_time_t = std::chrono::system_clock::to_time_t(t);
  struct tm tm;
  char buf[40];
  if (!::localtime_r(&as_time_t, &tm) || !std::strftime(buf, sizeof(buf), "%m%d%y-%H%M%S", &tm)) {
    throw std::runtime_error("Failed to get current date as string");
  }
  return buf;
}

static volatile std::sig_atomic_t g_runloop = true;
void stop(int) { g_runloop = false; }

struct Args {
  enum class Optimizer { NLOPT, IPOPT };
  enum class Task { CARTESIAN_POSE, PICK_PLACE };

  Optimizer optimizer = Optimizer::IPOPT;
  Task task = Task::CARTESIAN_POSE;
  bool with_scalar_constraints = false;
  bool with_hessian = false;
  std::string logdir;
};

static Args ParseArgs(int argc, char *argv[]) {
  Args parsed_args;
  int i;
  std::string arg;
  for (i = 1; i < argc; i++) {
    arg = argv[i];
    if (arg == "--optimizer") {
      i++;
      if (i >= argc) continue;
      std::string optimizer(argv[i]);
      if (optimizer == "nlopt") {
        parsed_args.optimizer = Args::Optimizer::NLOPT;
      } else if (optimizer == "ipopt") {
        parsed_args.optimizer = Args::Optimizer::IPOPT;
      } else {
        break;
      }
    } else if (arg == "--task") {
      i++;
      if (i >= argc) continue;
      std::string task(argv[i]);
      if (task == "cartesian-pose") {
        parsed_args.task = Args::Task::CARTESIAN_POSE;
      } else if (task == "pick-place") {
        parsed_args.task = Args::Task::PICK_PLACE;
      } else {
        break;
      }
    } else if (arg == "--logdir") {
      i++;
      if (i >= argc) continue;
      parsed_args.logdir = argv[i];
    } else if (arg == "--with-scalar-constraints") {
      parsed_args.with_scalar_constraints = true;
    } else if (arg == "--with-hessian") {
      parsed_args.with_hessian = true;
    } else {
      break;
    }
  }

  if (i != argc) throw std::invalid_argument("ParseArgs(): Invalid '" + arg + "' argument.");
  return parsed_args;
}

int main(int argc, char *argv[]) {
  Args args = ParseArgs(argc, argv);

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

  TrajOpt::JointVariables variables(ab, T, q_des);
  // TrajOpt::JointVariables variables(ab, T, Eigen::VectorXd::Zero(ab.dof()));

  TrajOpt::Objectives objectives;
  objectives.emplace_back(new TrajOpt::JointVelocityObjective());
  // objectives.emplace_back(new TrajOpt::LinearVelocityObjective(ab));

  TrajOpt::Constraints constraints;
  constraints.emplace_back(new TrajOpt::JointPositionConstraint(ab, 0, ab.q()));
  if (args.task == Args::Task::PICK_PLACE) {
    constraints.emplace_back(new TrajOpt::PickConstraint(ab, t_pick, world_objects["box"], ee_offset));
    constraints.emplace_back(new TrajOpt::PlaceConstraint(ab, t_pick, t_place, world_objects["box"],
                                                          world_objects["box_end"].T_to_parent().translation(),
                                                          Eigen::Quaterniond::Identity()));
  }
  // constraints.emplace_back(new TrajOpt::JointPositionConstraint(ab, T - 1, q_des));
  TrajOpt::CartesianPoseConstraint::Layout layout;
  if (args.with_scalar_constraints) {
    layout = TrajOpt::CartesianPoseConstraint::Layout::SCALAR_SCALAR;
  } else {
    layout = TrajOpt::CartesianPoseConstraint::Layout::VECTOR_SCALAR;
  }
  constraints.emplace_back(new TrajOpt::CartesianPoseConstraint(ab, T - 1, x_des, quat_des,
                                                                Eigen::Vector3d::Zero(), layout));
  // constraints.emplace_back(new TrajOpt::AboveTableConstraint(ab, world_objects["table"], 0, T));

  std::string logdir;
  if (!args.logdir.empty()) {
    logdir = args.logdir + "/";
    logdir += (args.optimizer == Args::Optimizer::NLOPT) ? "nlopt" : "ipopt";
    logdir += (args.task == Args::Task::PICK_PLACE) ? "_pickplace" : "";
    logdir += args.with_scalar_constraints ? "_scalar" : "";
    logdir += args.with_hessian ? "_hessian" : "";
    logdir += "/";
    mkdir(logdir.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
  }

  Eigen::MatrixXd Q_optimal;
  auto t_start = std::chrono::high_resolution_clock::now();
  if (args.optimizer == Args::Optimizer::NLOPT) {
    TrajOpt::Nlopt::OptimizationData data;
    Q_optimal = TrajOpt::Nlopt::Trajectory(variables, objectives, constraints, &data, logdir);
  } else {
    TrajOpt::Ipopt::OptimizationData data;
    Q_optimal = TrajOpt::Ipopt::Trajectory(variables, objectives, constraints, &data, logdir, args.with_hessian);
  }
  auto t_end = std::chrono::high_resolution_clock::now();

  std::cout << Q_optimal.transpose() << std::endl << std::endl;

  std::ofstream log(logdir + "results.log");
  log << "Q*:" << std::endl << Q_optimal.transpose() << std::endl << std::endl;
  double obj_value = 0;
  for (const std::unique_ptr<TrajOpt::Objective>& o : objectives) {
    double obj_o = 0;
    o->Evaluate(Q_optimal, obj_o);
    obj_value += obj_o;
    log << o->name << ":" << std::endl << obj_value << std::endl << std::endl;
  }
  log << "objective:" << std::endl << obj_value << std::endl << std::endl;
  for (const std::unique_ptr<TrajOpt::Constraint>& c : constraints) {
    Eigen::VectorXd g = Eigen::VectorXd::Zero(c->num_constraints);
    c->Evaluate(Q_optimal, g);
    log << c->name << ":" << std::endl << g.transpose() << std::endl << std::endl;
  }
  log << "time:" << std::endl << std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count() << std::endl << std::endl;
  log.close();

  // Eigen::Map<const Eigen::MatrixXd> Q(&data.x[0], ab.dof(), T);
  // Eigen::VectorXd c(T);
  // constraints.back()->Evaluate(Q, c);
  // std::cout << "CONSTRAINTS: " << c.transpose() << std::endl;

  // std::vector<Eigen::VectorXd> Q_optimal = TaskSpaceTrajectory(ab, world_objects, x_des, quat_des, T);
  // std::vector<Eigen::VectorXd> Q_optimal = TaskSpaceTrajectory(ab, q_des);

  size_t idx_trajectory = 0;
  while (g_runloop) {
    timer.Sleep();
    Eigen::VectorXd q_err = ab.q() - Q_optimal.col(idx_trajectory);
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
      if (idx_trajectory < Q_optimal.cols() - 1) {
        idx_trajectory++;
      } else if (dq_err.norm() < 0.0001) {
        break;
      }
    }
  }

  std::cout << "Simulated " << timer.time_sim() << "s in " << timer.time_elapsed() << "s." << std::endl;

  return 0;
}

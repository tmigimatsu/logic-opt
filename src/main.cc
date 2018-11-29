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
#include "TrajOpt/ipopt.h"
#include "TrajOpt/nlopt.h"
#include "TrajOpt/objectives.h"
#include "TrajOpt/constraints.h"
#include "TrajOpt/world.h"

#include <algorithm>  // std::max
#include <chrono>    // std::chrono
#include <cmath>     // M_PI
#include <csignal>   // std::signal
#include <fstream>   // std::ofstream
#include <iostream>  // std::cout
#include <map>       // std::map
#include <string>    // std::string
#include <time.h>    // ::gmtime_r, std::strftime
#include <sys/stat.h>  // mkdir

namespace {

std::string DateTimeString(std::chrono::system_clock::time_point t) {
  time_t as_time_t = std::chrono::system_clock::to_time_t(t);
  struct tm tm;
  char buf[40];
  if (!::localtime_r(&as_time_t, &tm) || !std::strftime(buf, sizeof(buf), "%m%d%y-%H%M%S", &tm)) {
    throw std::runtime_error("Failed to get current date as string");
  }
  return buf;
}

volatile std::sig_atomic_t g_runloop = true;
void stop(int) { g_runloop = false; }

struct Args {
  enum class Optimizer { NLOPT, IPOPT };
  enum class Task { JOINT_POSITION, CARTESIAN_POSE, PICK_PLACE, SLIDE, PUSH };

  Optimizer optimizer = Optimizer::IPOPT;
  Task task = Task::CARTESIAN_POSE;
  bool with_scalar_constraints = false;
  bool with_hessian = false;
  std::string logdir;
};

Args ParseArgs(int argc, char *argv[]) {
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
      } else if (task == "slide") {
        parsed_args.task = Args::Task::SLIDE;
      } else if (task == "push") {
        parsed_args.task = Args::Task::PUSH;
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

}  // namespace

int main(int argc, char *argv[]) {
  Args args = ParseArgs(argc, argv);

  // Load robot
  SpatialDyn::ArticulatedBody ab = SpatialDyn::Urdf::LoadModel("../resources/kuka_iiwa/kuka_iiwa.urdf");

  // Create world objects
  std::map<std::string, SpatialDyn::RigidBody> world_objects;
  {
    SpatialDyn::RigidBody table("table");
    table.graphics.geometry.type = SpatialDyn::Geometry::Type::BOX;
    table.graphics.geometry.scale = Eigen::Vector3d(1., 0.8, 0.02);
    table.graphics.material.rgba(3) = 0.5;
    table.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0., -0.5, 0.3));
    world_objects[table.name] = table;
  }
  {
    SpatialDyn::RigidBody shelf("shelf");
    shelf.graphics.geometry.type = SpatialDyn::Geometry::Type::BOX;
    shelf.graphics.geometry.scale = Eigen::Vector3d(0.2, 0.2, 0.02);
    shelf.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.3, -0.5, 0.4));
    world_objects[shelf.name] = shelf;
  }
  {
    SpatialDyn::RigidBody box("box");
    box.graphics.geometry.type = SpatialDyn::Geometry::Type::BOX;
    box.graphics.geometry.scale = Eigen::Vector3d(0.05, 0.05, 0.05);
    box.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0., -0.5, 0.4));
    world_objects[box.name] = box;
  }
  {
    SpatialDyn::RigidBody box("box2");
    box.graphics.geometry.type = SpatialDyn::Geometry::Type::BOX;
    box.graphics.geometry.scale = Eigen::Vector3d(0.05, 0.05, 0.05);
    box.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.1, -0.5, 0.4));
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

  const size_t T = 50;

  TrajOpt::JointVariables variables(ab, T, q_des);
  ab.set_q(q_des);
  // TrajOpt::JointVariables variables(ab, T, Eigen::VectorXd::Zero(ab.dof()));

  TrajOpt::Objectives objectives;
  objectives.emplace_back(new TrajOpt::JointVelocityObjective());
  // objectives.emplace_back(new TrajOpt::LinearVelocityObjective(ab));

  TrajOpt::World world(ab, world_objects, T);

  auto layout = TrajOpt::CartesianPoseConstraint::Layout::VECTOR_SCALAR;
  auto layout_pos = TrajOpt::CartesianPoseConstraint::Layout::POS_VECTOR;
  if (args.with_scalar_constraints) {
    layout = TrajOpt::CartesianPoseConstraint::Layout::SCALAR_SCALAR;
    layout_pos = TrajOpt::CartesianPoseConstraint::Layout::POS_SCALAR;
  }

  // Set up task constraints
  TrajOpt::Constraints constraints;
  constraints.emplace_back(new TrajOpt::JointPositionConstraint(ab, 0, ab.q()));
  switch (args.task) {
    case Args::Task::JOINT_POSITION:
      constraints.emplace_back(new TrajOpt::JointPositionConstraint(ab, T - 1, q_des));
      break;
    case Args::Task::CARTESIAN_POSE:
      constraints.emplace_back(new TrajOpt::CartesianPoseConstraint(world, T - 1, x_des, quat_des,
                                                                    Eigen::Vector3d::Zero(), layout));
      break;
    case Args::Task::PICK_PLACE:
      constraints.emplace_back(new TrajOpt::PickConstraint(world, 10, "box", ee_offset, layout_pos));
      // constraints.emplace_back(new TrajOpt::PlaceConstraint(world, 20, "box",
      //                                                       world_objects["shelf"].T_to_parent().translation(),
      //                                                       Eigen::Quaterniond::Identity(),
      //                                                       Eigen::Vector3d::Zero(), layout));
      constraints.emplace_back(new TrajOpt::PlaceOnConstraint(world, 20, "box", "shelf"));
      constraints.emplace_back(new TrajOpt::PickConstraint(world, 30, "box2", ee_offset, layout_pos));
      constraints.emplace_back(new TrajOpt::PlaceOnConstraint(world, 40, "box2", "box"));
      constraints.emplace_back(new TrajOpt::CartesianPoseConstraint(world, T - 1, x_des, quat_des,
                                                                    Eigen::Vector3d::Zero(), layout));
      break;
    case Args::Task::SLIDE:
      constraints.emplace_back(new TrajOpt::PickConstraint(world, 10, "box2", ee_offset, layout_pos));
      // constraints.emplace_back(new TrajOpt::PlaceOnConstraint(world, 20, "box2", "table"));
      constraints.emplace_back(new TrajOpt::SlideOnConstraint(world, 20, 10, "box2", "table"));
      // constraints.emplace_back(new TrajOpt::PickConstraint(world, 30, "box2", ee_offset, layout_pos));
      constraints.emplace_back(new TrajOpt::CartesianPoseConstraint(world, T - 1, x_des, quat_des,
                                                                    Eigen::Vector3d::Zero(), layout));
      break;
    case Args::Task::PUSH:
      constraints.emplace_back(new TrajOpt::PickConstraint(world, 10, "box2", ee_offset, layout_pos));
      // constraints.emplace_back(new TrajOpt::PlaceOnConstraint(world, 20, "box2", "table"));
      constraints.emplace_back(new TrajOpt::PushConstraint(world, 20, 10, "box2", "box",
                                                           TrajOpt::PushConstraint::Direction::NEG_X));
      // constraints.emplace_back(new TrajOpt::PickConstraint(world, 30, "box2", ee_offset, layout_pos));
      constraints.emplace_back(new TrajOpt::CartesianPoseConstraint(world, T - 1, x_des, quat_des,
                                                                    Eigen::Vector3d::Zero(), layout));
      break;
  }

  world.InitializeConstraintSchedule(constraints);

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
  world.Simulate(Q_optimal);

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
    Eigen::VectorXd g = Eigen::VectorXd::Zero(c->num_constraints());
    c->Evaluate(Q_optimal, g);
    log << c->name << ":" << std::endl << g.transpose() << std::endl << std::endl;
  }
  log << "time:" << std::endl << std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count() << std::endl << std::endl;
  log.close();

  std::map<std::string, SpatialDyn::RigidBody> simulation_objects = world_objects;

  size_t idx_trajectory = 0;
  while (g_runloop) {
    timer.Sleep();
    Eigen::VectorXd q_err = ab.q() - Q_optimal.col(idx_trajectory);
    Eigen::VectorXd dq_err = ab.dq();
    Eigen::VectorXd ddq = -10. * q_err - 6. * dq_err;
    Eigen::VectorXd tau = SpatialDyn::InverseDynamics(ab, ddq, {}, true, true);

    SpatialDyn::Integrate(ab, tau, timer.dt());

    std::map<std::string, TrajOpt::World::ObjectState> world_state_t =
        world.InterpolateSimulation(ab.q(), std::max(static_cast<int>(idx_trajectory) - 1, 0));

    for (auto& key_val : world_state_t) {
      const std::string& name_object = key_val.first;
      const TrajOpt::World::ObjectState object_state_t = key_val.second;
      SpatialDyn::RigidBody& rb = simulation_objects[name_object];
      rb.set_T_to_parent(object_state_t.quat, object_state_t.pos);

      redis_client.set("spatialdyn::objects::" + name_object, SpatialDyn::Json::Serialize(rb).dump());
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

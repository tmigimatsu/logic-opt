/**
 * main.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 19, 2018
 * Authors: Toki Migimatsu
 */

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

#include <spatial_dyn/spatial_dyn.h>
#include <ctrl_utils/eigen_string.h>
#include <ctrl_utils/filesystem.h>
#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/timer.h>

// #include "gurobi.h"
#include "LogicOpt/optimization/ipopt.h"
#include "LogicOpt/optimization/nlopt.h"
#include "LogicOpt/constraints.h"
#include "LogicOpt/world.h"

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
void stop(int) { g_runloop = false; LogicOpt::Ipopt::Terminate(); }

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
      if (task == "joint-position") {
        parsed_args.task = Args::Task::JOINT_POSITION;
      } else if (task == "cartesian-pose") {
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

const std::string kNameApp   = "simulator";
const std::string kNameRobot = "kuka_iiwa";

const std::string KEY_PREFIX         = "logic_opt::";
const std::string KEY_OBJECTS_PREFIX = KEY_PREFIX + "objects::";
const std::string KEY_MODEL          = KEY_PREFIX + kNameRobot + "::model";
const std::string KEY_WEB_RESOURCES  = "webapp::resources";
const std::string KEY_WEB_ARGS       = "webapp::" + kNameApp + "::args";
const std::string KEY_SENSOR_Q       = KEY_PREFIX + kNameRobot + "::sensor::q";
const std::string KEY_TRAJ_POS       = KEY_PREFIX + kNameRobot + "::trajectory::pos";

const std::string kDirUrdf = "../resources/" + kNameRobot;
const std::string kPathUrdf = kDirUrdf + "/" + kNameRobot + ".urdf";

}  // namespace

int main(int argc, char *argv[]) {
  Args args = ParseArgs(argc, argv);

  // Load robot
  spatial_dyn::ArticulatedBody ab = spatial_dyn::urdf::LoadModel(kPathUrdf);

  Eigen::VectorXd q_des(ab.dof());
  q_des << 90., -30., 0., 60., 0., -90., 0.;
  q_des *= M_PI / 180.;
  ab.set_q(q_des);

  // Create world objects
  const std::string kEeFrame = "ee";
  auto world_objects = std::make_shared<std::map<std::string, spatial_dyn::RigidBody>>();
  {
    spatial_dyn::RigidBody table("table");
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::BOX;
    graphics.geometry.scale = Eigen::Vector3d(1., 0.8, 0.02);
    graphics.material.rgba(3) = 0.5;
    table.graphics.push_back(std::move(graphics));
    table.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0., -0.5, 0.3));
    world_objects->emplace(table.name, table);
  }
  {
    spatial_dyn::RigidBody shelf("shelf");
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::BOX;
    graphics.geometry.scale = Eigen::Vector3d(0.2, 0.2, 0.02);
    shelf.graphics.push_back(std::move(graphics));
    shelf.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.3, -0.5, 0.4));
    world_objects->emplace(shelf.name, shelf);
  }
  {
    spatial_dyn::RigidBody box("box");
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::BOX;
    graphics.geometry.scale = Eigen::Vector3d(0.05, 0.05, 0.05);
    box.graphics.push_back(std::move(graphics));
    box.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0., -0.5, 0.4));
    world_objects->emplace(box.name, box);
  }
  {
    spatial_dyn::RigidBody box("box2");
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::BOX;
    graphics.geometry.scale = Eigen::Vector3d(0.05, 0.05, 0.05);
    box.graphics.push_back(std::move(graphics));
    box.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.1, -0.5, 0.4));
    world_objects->emplace(box.name, box);
  }
  {
    spatial_dyn::RigidBody ee(kEeFrame);
    ee.set_T_to_parent(spatial_dyn::Orientation(ab).inverse(), Eigen::Vector3d(0., 0., 0.06));
    world_objects->emplace(ee.name, ee);
  }

  // Set up timer and Redis
  ctrl_utils::Timer timer(1000);
  ctrl_utils::RedisClient redis_client;
  redis_client.connect();
  redis_client.set(KEY_MODEL, spatial_dyn::json::Serialize(ab));
  for (const std::pair<std::string, spatial_dyn::RigidBody>& key_val : *world_objects) {
    if (key_val.first == kEeFrame) continue;
    const spatial_dyn::RigidBody& object = key_val.second;
    redis_client.set(KEY_OBJECTS_PREFIX + object.name, spatial_dyn::json::Serialize(object));
  }
  std::string path_urdf = ctrl_utils::AbsolutePath(ctrl_utils::CurrentPath() + "/" + kDirUrdf);
  redis_client.hset(KEY_WEB_RESOURCES, kNameApp, path_urdf);
  nlohmann::json web_keys;
  web_keys["key_model"]          = KEY_MODEL;
  web_keys["key_objects_prefix"] = KEY_OBJECTS_PREFIX;
  web_keys["key_q"]              = KEY_SENSOR_Q;
  web_keys["key_traj"]           = KEY_TRAJ_POS;
  redis_client.set(KEY_WEB_ARGS, web_keys);
  redis_client.sync_commit();

  // Set up signal handlers
  std::signal(SIGINT, stop);

  // ab.set_q(Eigen::VectorXd::Zero(ab.dof()));

  // End-effector parameters
  const Eigen::Isometry3d& T_ee = world_objects->at(kEeFrame).T_to_parent();
  Eigen::Quaterniond quat_ee(T_ee.linear());
  Eigen::Ref<const Eigen::Vector3d> ee_offset = T_ee.translation();

  // Eigen::Vector3d x_des = Eigen::Vector3d(0., -0.6, 0.6) - T_ee.translation();
  // Eigen::Quaterniond quat_des(Eigen::Quaterniond(0., 1., 0., 0.) * T_ee.linear());

  const size_t T = 4;

  LogicOpt::World world(world_objects, T);

  // LogicOpt::JointVariables variables(ab, T, q_des);
  // LogicOpt::JointVariables variables(ab, T, Eigen::VectorXd::Zero(ab.dof()));
  LogicOpt::FrameVariables variables(T);

  LogicOpt::Objectives objectives;
  // objectives.emplace_back(new LogicOpt::JointVelocityObjective());
  objectives.emplace_back(new LogicOpt::LinearVelocityObjective(world, kEeFrame));
  // objectives.emplace_back(new LogicOpt::MinNormObjective());
  objectives.emplace_back(new LogicOpt::AngularVelocityObjective(world, kEeFrame));

  // Set up task constraints
  LogicOpt::Constraints constraints;

  constraints.emplace_back(new LogicOpt::CartesianPoseConstraint(
      world, 0, kEeFrame, world.kWorldFrame, spatial_dyn::Position(ab) - ee_offset,
      spatial_dyn::Orientation(ab) * quat_ee));

  constraints.emplace_back(new LogicOpt::PickConstraint(world, 1, kEeFrame, "box"));
  constraints.emplace_back(new LogicOpt::PlaceConstraint(world, 2, "box", "shelf"));

  constraints.emplace_back(new LogicOpt::CartesianPoseConstraint(
      world, 3, kEeFrame, world.kWorldFrame, spatial_dyn::Position(ab) - ee_offset,
      spatial_dyn::Orientation(ab) * quat_ee));

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

  Eigen::MatrixXd X_optimal;
  auto t_start = std::chrono::high_resolution_clock::now();
  if (args.optimizer == Args::Optimizer::NLOPT) {
    LogicOpt::Nlopt::Options options = { logdir };
    LogicOpt::Nlopt nlopt(options);
    LogicOpt::Nlopt::OptimizationData data;
    X_optimal = nlopt.Trajectory(variables, objectives, constraints, &data);
  } else {
    LogicOpt::Ipopt::Options options;
    options.derivative_test = true;
    options.use_hessian = args.with_hessian;
    LogicOpt::Ipopt ipopt(options);
    LogicOpt::Ipopt::OptimizationData data;
    X_optimal = ipopt.Trajectory(variables, objectives, constraints, &data);
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  // X_optimal = Eigen::MatrixXd::Zero(6, T);
  // X_optimal.col(0).head<3>() = x_des;
  // X_optimal.col(T-1).head<3>() = x_des;

  std::cout << X_optimal << std::endl << std::endl;
  std::cout << world << std::endl << std::endl;

  // std::ofstream log(logdir + "results.log");
  // log << "X*:" << std::endl << X_optimal.transpose() << std::endl << std::endl;
  // double obj_value = 0;
  // for (const std::unique_ptr<LogicOpt::Objective>& o : objectives) {
  //   double obj_o = 0;
  //   o->Evaluate(X_optimal, obj_o);
  //   obj_value += obj_o;
  //   log << o->name << ":" << std::endl << obj_value << std::endl << std::endl;
  // }
  // log << "objective:" << std::endl << obj_value << std::endl << std::endl;
  // for (const std::unique_ptr<LogicOpt::Constraint>& c : constraints) {
  //   Eigen::VectorXd g = Eigen::VectorXd::Zero(c->num_constraints());
  //   c->Evaluate(X_optimal, g);
  //   log << c->name << ":" << std::endl << g.transpose() << std::endl << std::endl;
  // }
  // log << "time:" << std::endl << std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count() << std::endl << std::endl;
  // log.close();

  size_t idx_trajectory = 0;
  std::map<std::string, spatial_dyn::RigidBody> sim_objects_abs = *world_objects;
  while (g_runloop) {
    timer.Sleep();

    // Controller frames
    const std::pair<std::string, std::string>& controller_frames = world.controller_frames(idx_trajectory);
    const std::string& control_frame = controller_frames.first;
    const std::string& target_frame = controller_frames.second;

    // TODO: from perception
    Eigen::Isometry3d T_control_to_ee = quat_ee * world.T_to_frame(control_frame, kEeFrame, X_optimal, idx_trajectory);
    Eigen::Isometry3d T_target_to_world = world.T_to_world(target_frame, X_optimal, idx_trajectory);
    Eigen::Isometry3d T_control_to_target = world.T_control_to_target(X_optimal, idx_trajectory);

    // Prepare position-orientation task
    Eigen::Isometry3d T_des_to_world = T_target_to_world * T_control_to_target * T_control_to_ee.inverse();
    const Eigen::Matrix6Xd& J = spatial_dyn::Jacobian(ab, -1, ee_offset);

    // Position
    Eigen::Vector3d x = spatial_dyn::Position(ab, -1, ee_offset);
    Eigen::Vector3d x_des = T_des_to_world.translation();
    Eigen::Vector3d dx = J.topRows<3>() * ab.dq();

    const double Kp = 10.;
    const double Kv = 6.;
    const double kMaxVel = 0.1;
    Eigen::Vector3d ddx;
    Eigen::Vector3d dx_des = -(Kp / Kv) * (x - x_des);
    if (dx_des.norm() > 0.01) {
      double v = kMaxVel / dx_des.norm();
      if (v > 1.) v = 1.;
      ddx = -Kv * (dx - v * dx_des);
    } else {
      ddx = -Kp * (x - x_des) - Kv * dx;
    }

    // Orientation
    Eigen::Quaterniond quat = spatial_dyn::Orientation(ab);
    Eigen::Quaterniond quat_des = spatial_dyn::opspace::NearQuaternion(T_des_to_world.linear(), quat);
    Eigen::Vector3d quat_err = spatial_dyn::opspace::OrientationError(quat, quat_des);
    Eigen::Vector3d w = J.bottomRows<3>() * ab.dq();
    Eigen::Vector3d dw = -40 * quat_err - 40 * w;

    // Opspace
    Eigen::Vector6d ddx_dw;
    ddx_dw << ddx, dw;
    Eigen::MatrixXd N;
    Eigen::VectorXd tau = spatial_dyn::opspace::InverseDynamics(ab, J, ddx_dw, &N);

    // Nullspace
    static const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(ab.dof(), ab.dof());
    Eigen::VectorXd ddq = -16 * (ab.q() - q_des) - 8 * ab.dq();
    tau += spatial_dyn::opspace::InverseDynamics(ab, I, ddq, &N);

    // Gravity
    tau += spatial_dyn::Gravity(ab);

    // Simulate
    spatial_dyn::Integrate(ab, tau, timer.dt());

    // Update object states
    const Eigen::Isometry3d& T_ee_to_world = ab.T_to_world(-1) * T_ee;
    const LogicOpt::Tree<std::string, LogicOpt::Frame> frame_tree = world.frames(idx_trajectory);
    if (frame_tree.is_ancestor(kEeFrame, control_frame)) {
      for (const auto& key_val : frame_tree.ancestors(control_frame)) {
        // Only check frames between control frame and ee
        const std::string& frame = key_val.first;
        if (frame == kEeFrame) break;
        Eigen::Isometry3d T_to_ee = world.T_to_frame(frame, kEeFrame, X_optimal, idx_trajectory);
        spatial_dyn::RigidBody& rb = sim_objects_abs.at(frame);
        rb.set_T_to_parent(T_to_ee * T_ee_to_world);

        redis_client.set(KEY_OBJECTS_PREFIX + frame, spatial_dyn::json::Serialize(rb).dump());
      }
    } else if (frame_tree.is_ancestor(control_frame, kEeFrame)) {
      for (const auto& key_val : frame_tree.ancestors(*frame_tree.parent(kEeFrame))) {
        // Only check frames between control frame and ee
        const std::string& frame = key_val.first;
        Eigen::Isometry3d T_to_ee = world.T_to_frame(frame, kEeFrame, X_optimal, idx_trajectory);
        spatial_dyn::RigidBody& rb = sim_objects_abs.at(frame);
        rb.set_T_to_parent(T_to_ee * T_ee_to_world);

        redis_client.set(KEY_OBJECTS_PREFIX + frame, spatial_dyn::json::Serialize(rb).dump());
        if (frame == control_frame) break;
      }
    }

    redis_client.set(KEY_SENSOR_Q, ab.q());
    redis_client.set(KEY_TRAJ_POS, spatial_dyn::Position(ab));
    redis_client.commit();

    if (ddx_dw.norm() < 0.01) {
      if (idx_trajectory < X_optimal.cols() - 1) {
        idx_trajectory++;
      } else {
        break;
      }
    }
  }

  std::cout << "Simulated " << timer.time_sim() << "s in " << timer.time_elapsed() << "s." << std::endl;

  return 0;
}

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
#include <ctrl_utils/euclidian.h>
#include <ctrl_utils/control.h>
#include <ctrl_utils/filesystem.h>
#include <ctrl_utils/json.h>
#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/timer.h>
#include <ncollide_cpp/ncollide3d.h>

// #include "gurobi.h"
#include "logic_opt/optimization/constraints.h"
#include "logic_opt/optimization/ipopt.h"
#include "logic_opt/optimization/nlopt.h"
#include "logic_opt/world.h"

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
void stop(int) { g_runloop = false; logic_opt::Ipopt::Terminate(); }

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

// Robot
const std::string kNameRobot = "kuka_iiwa";
const std::string kPathUrdf = "../resources/" + kNameRobot + "/" + kNameRobot + ".urdf";

const std::string KEY_PREFIX         = "logic_opt::";
const std::string KEY_MODELS_PREFIX  = KEY_PREFIX + "model::";
const std::string KEY_OBJECTS_PREFIX = KEY_PREFIX + "object::";
const std::string KEY_OBJECT_MODELS_PREFIX = KEY_OBJECTS_PREFIX + "model::";
const std::string KEY_TRAJ_PREFIX    = KEY_PREFIX + "trajectory::";

// SET keys
const std::string KEY_SENSOR_Q       = KEY_PREFIX + kNameRobot + "::sensor::q";
const std::string KEY_CONTROL_POS    = KEY_PREFIX + kNameRobot + "::control::pos";
const std::string KEY_TRAJ_POS       = KEY_TRAJ_PREFIX + kNameRobot + "::pos";

// Webapp keys
const std::string kNameApp   = "simulator";
const std::string KEY_WEB_RESOURCES   = "webapp::resources";
const std::string KEY_WEB_ARGS        = "webapp::" + kNameApp + "::args";
const std::string KEY_WEB_INTERACTION = "webapp::" + kNameApp + "::interaction";

// Controller gains
const std::string KEY_KP_KV_POS   = KEY_PREFIX + "control::kp_kv_pos";
const std::string KEY_KP_KV_ORI   = KEY_PREFIX + "control::kp_kv_ori";
const std::string KEY_KP_KV_JOINT = KEY_PREFIX + "control::kp_kv_joint";

// Controller parameters
const Eigen::VectorXd kQHome     = (M_PI / 180.) * (Eigen::Matrix<double,7,1>() <<
                                   90., -30., 0., 60., 0., -90., 0.).finished();
const Eigen::Vector2d kKpKvPos   = Eigen::Vector2d(10., 6.);
const Eigen::Vector2d kKpKvOri   = Eigen::Vector2d(40., 40.);
const Eigen::Vector2d kKpKvJoint = Eigen::Vector2d(16., 8.);
const double kMaxVel             = 0.1;
const double kTimerFreq          = 1000.;
const double kGainClickDrag      = 100.;

const std::string kEeFrame = "ee";

void InitializeWebApp(ctrl_utils::RedisClient& redis_client, const spatial_dyn::ArticulatedBody& ab,
                      const std::map<std::string, logic_opt::Object3>& objects, size_t T);

std::map<size_t, spatial_dyn::SpatialForced> ComputeExternalForces(const spatial_dyn::ArticulatedBody& ab,
                                                                   const nlohmann::json& interaction);

}  // namespace

int main(int argc, char *argv[]) {
  Args args = ParseArgs(argc, argv);

  // Set up timer and Redis client
  ctrl_utils::Timer timer(kTimerFreq);
  ctrl_utils::RedisClient redis_client;
  redis_client.connect();

  // Initialize robot
  spatial_dyn::ArticulatedBody ab = spatial_dyn::urdf::LoadModel(kPathUrdf);
  ab.set_q(kQHome);

  // Initialize controller parameters
  Eigen::VectorXd q_des = kQHome;

  // Create world objects
  auto world_objects = std::make_shared<std::map<std::string, logic_opt::Object3>>();
  {
    spatial_dyn::RigidBody table("table");
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    graphics.geometry.scale = Eigen::Vector3d(1., 0.8, 0.05);
    graphics.material.rgba(3) = 0.5;
    // table.collision = std::make_unique<ncollide3d::shape::Cuboid>(graphics.geometry.scale / 2);
    table.graphics.push_back(std::move(graphics));
    table.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0., -0.5, 0.3));
    world_objects->emplace(std::string(table.name), std::move(table));
  }
  {
    spatial_dyn::RigidBody shelf("shelf");
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    graphics.geometry.scale = Eigen::Vector3d(0.2, 0.2, 0.02);
    // shelf.collision = std::make_unique<ncollide3d::shape::Cuboid>(graphics.geometry.scale / 2);
    shelf.graphics.push_back(std::move(graphics));
    shelf.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.3, -0.5, 0.4));
    world_objects->emplace(std::string(shelf.name), std::move(shelf));
  }
  {
    spatial_dyn::RigidBody box("box");
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    // graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kSphere;
    graphics.geometry.radius = 0.025;
    graphics.geometry.scale = Eigen::Vector3d(0.05, 0.05, 0.05);
    // box.collision = std::make_unique<ncollide3d::shape::Cuboid>(graphics.geometry.scale / 2);
    box.graphics.push_back(std::move(graphics));
    // box.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0., -1.0, 0.4));
    box.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0., -0.5, 0.4));
    world_objects->emplace(std::string(box.name), std::move(box));
  }
  {
    spatial_dyn::RigidBody box("hook");
    spatial_dyn::Graphics graphics;
    // graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    // graphics.geometry.scale = Eigen::Vector3d(0.04, 0.2, 0.04);
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kCapsule;
    graphics.geometry.radius = 0.01;
    graphics.geometry.length = 0.2;
    graphics.T_to_parent = Eigen::Translation3d(0.0317, -0.0183, 0.) *
                           Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitZ());
    // box.collision = std::make_unique<ncollide3d::shape::Cuboid>(graphics.geometry.scale / 2);
    box.graphics.push_back(std::move(graphics));

    // graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    // graphics.geometry.scale = Eigen::Vector3d(0.04, 0.1, 0.04);
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kCapsule;
    graphics.geometry.radius = 0.01;
    graphics.geometry.length = 0.1;
    graphics.T_to_parent = Eigen::Translation3d(-0.0683, 0.0317, 0.);
    box.graphics.push_back(std::move(graphics));

    box.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.1, -0.5, 0.4));
    world_objects->emplace(std::string(box.name), std::move(box));
  }
  {
    logic_opt::Object3 ee(kEeFrame);
    ee.set_T_to_parent(spatial_dyn::Orientation(ab).inverse(), Eigen::Vector3d(0., 0., 0.03));
    world_objects->emplace(std::string(ee.name), std::move(ee));
  }

  // Create signal handler
  std::signal(SIGINT, stop);

  // End-effector parameters
  const Eigen::Isometry3d& T_ee = world_objects->at(kEeFrame).T_to_parent();
  Eigen::Quaterniond quat_ee(T_ee.linear());
  Eigen::Ref<const Eigen::Vector3d> ee_offset = T_ee.translation();

  logic_opt::World3 world(world_objects);

  logic_opt::Objectives objectives;
  objectives.emplace_back(new logic_opt::LinearVelocityObjective(world, kEeFrame));
  objectives.emplace_back(new logic_opt::AngularVelocityObjective(world, kEeFrame));

  // Set up task constraints
  logic_opt::Constraints constraints;

  size_t t = 0;
  constraints.emplace_back(new logic_opt::CartesianPoseConstraint(
      world, t, kEeFrame, world.kWorldFrame, spatial_dyn::Position(ab) - ee_offset,
      spatial_dyn::Orientation(ab) * quat_ee));
  t += constraints.back()->num_timesteps();

  constraints.emplace_back(new logic_opt::PickConstraint(world, t, kEeFrame, "hook"));
  t += constraints.back()->num_timesteps();

  constraints.emplace_back(new logic_opt::PushConstraint(world, t, "hook", "box"));
  t += constraints.back()->num_timesteps();

  constraints.emplace_back(new logic_opt::PlaceConstraint(world, t, "hook", "shelf"));
  t += constraints.back()->num_timesteps();

  constraints.emplace_back(new logic_opt::PickConstraint(world, t, kEeFrame, "box"));
  t += constraints.back()->num_timesteps();

  constraints.emplace_back(new logic_opt::PlaceConstraint(world, t, "box", "shelf"));
  t += constraints.back()->num_timesteps();

  constraints.emplace_back(new logic_opt::CartesianPoseConstraint(
      world, t, kEeFrame, world.kWorldFrame, spatial_dyn::Position(ab) - ee_offset,
      spatial_dyn::Orientation(ab) * quat_ee));
  t += constraints.back()->num_timesteps();

  const size_t T = world.num_timesteps();
  if (t != T) throw std::runtime_error("Constraint timesteps must equal T.");
  std::cout << "T: " << T << std::endl;

  logic_opt::FrameVariables<3> variables(T);
  variables.X_0 = Eigen::MatrixXd::Zero(world.kDof, world.num_timesteps());
  variables.X_0.block<3,1>(0, 3) = world_objects->at("box").T_to_parent().translation();
  auto obj_norm = dynamic_cast<logic_opt::MinL1NormObjective *>(objectives[0].get());
  obj_norm->X_0 = variables.X_0;

  // Initialize Redis keys
  InitializeWebApp(redis_client, ab, *world_objects, T);
  redis_client.set(KEY_SENSOR_Q, ab.q());
  redis_client.set(KEY_KP_KV_POS, kKpKvPos);
  redis_client.set(KEY_KP_KV_ORI, kKpKvOri);
  redis_client.set(KEY_KP_KV_JOINT, kKpKvJoint);
  redis_client.sync_commit();

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
    logic_opt::Nlopt::Options options = { logdir };
    logic_opt::Nlopt nlopt(options);
    logic_opt::Nlopt::OptimizationData data;
    X_optimal = nlopt.Trajectory(variables, objectives, constraints, &data);
  } else {
    logic_opt::Ipopt::Options options;
    options.derivative_test = true;
    options.use_hessian = args.with_hessian;
    // options.max_iter = 1e6;
    // options.max_cpu_time = 1e3;
    logic_opt::Ipopt ipopt(options);
    logic_opt::Ipopt::OptimizationData data;
    X_optimal = ipopt.Trajectory(variables, objectives, constraints, &data,
                                 [&world, &redis_client](int i, const Eigen::MatrixXd& X) {
      // for (size_t t = 0; t < world.num_timesteps(); t++) {
      //   const std::string& control_frame = world.control_frame(t);
      //   Eigen::Isometry3d T_control_to_world = world.T_to_world(control_frame, X, t);
      //   redis_client.set<Eigen::Vector3d>(KEY_TRAJ_PREFIX + "frame::" + std::to_string(t) + "::pos",
      //                                     T_control_to_world.translation());
      // }
      // redis_client.commit();
    });
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  // X_optimal = Eigen::MatrixXd::Zero(6, T);
  // X_optimal.col(0).head<3>() = x_des;
  // X_optimal.col(T-1).head<3>() = x_des;

  std::cout << X_optimal << std::endl << std::endl;
  for (const std::unique_ptr<logic_opt::Constraint>& c : constraints) {
    Eigen::VectorXd f(c->num_constraints());
    c->Evaluate(X_optimal, f);
    std::cout << c->name << ":" << std::endl;
    for (size_t i = 0; i < c->num_constraints(); i++) {
      std::cout << "  "
                << (c->constraint_type(i) == logic_opt::Constraint::Type::kInequality ?
                    "<" : "=")
                << " : " << f(i) << std::endl;
    }
  }
  std::cout << world << std::endl << std::endl;

  // std::ofstream log(logdir + "results.log");
  // log << "X*:" << std::endl << X_optimal.transpose() << std::endl << std::endl;
  // double obj_value = 0;
  // for (const std::unique_ptr<logic_opt::Objective>& o : objectives) {
  //   double obj_o = 0;
  //   o->Evaluate(X_optimal, obj_o);
  //   obj_value += obj_o;
  //   log << o->name << ":" << std::endl << obj_value << std::endl << std::endl;
  // }
  // log << "objective:" << std::endl << obj_value << std::endl << std::endl;
  // for (const std::unique_ptr<logic_opt::Constraint>& c : constraints) {
  //   Eigen::VectorXd g = Eigen::VectorXd::Zero(c->num_constraints());
  //   c->Evaluate(X_optimal, g);
  //   log << c->name << ":" << std::endl << g.transpose() << std::endl << std::endl;
  // }
  // log << "time:" << std::endl << std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count() << std::endl << std::endl;
  // log.close();

  size_t idx_trajectory = 0;
  std::map<std::string, logic_opt::Object3> sim_objects_abs = *world_objects;
  while (g_runloop) {
    timer.Sleep();

    // Get Redis values
    std::future<Eigen::Vector2d> fut_kp_kv_pos   = redis_client.get<Eigen::Vector2d>(KEY_KP_KV_POS);
    std::future<Eigen::Vector2d> fut_kp_kv_ori   = redis_client.get<Eigen::Vector2d>(KEY_KP_KV_ORI);
    std::future<Eigen::Vector2d> fut_kp_kv_joint = redis_client.get<Eigen::Vector2d>(KEY_KP_KV_JOINT);
    std::future<nlohmann::json> fut_interaction  = redis_client.get<nlohmann::json>(KEY_WEB_INTERACTION);
    redis_client.commit();

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

    // Compute position PD control
    Eigen::Vector3d x_des = T_des_to_world.translation();
    Eigen::Vector3d x = spatial_dyn::Position(ab, -1, ee_offset);
    Eigen::Vector3d dx = J.topRows<3>() * ab.dq();
    Eigen::Vector3d ddx = ctrl_utils::PdControl(x, x_des, dx, fut_kp_kv_pos.get(), kMaxVel);

    // Compute orientation PD control
    Eigen::Quaterniond quat = spatial_dyn::Orientation(ab);
    Eigen::Quaterniond quat_des = ctrl_utils::NearQuaternion(T_des_to_world.linear(), quat);
    Eigen::Vector3d w = J.bottomRows<3>() * ab.dq();
    Eigen::Vector3d dw = ctrl_utils::PdControl(quat, quat_des, w, fut_kp_kv_ori.get());

    // Compute opspace torques
    Eigen::Vector6d ddx_dw;
    ddx_dw << ddx, dw;
    Eigen::MatrixXd N;
    Eigen::VectorXd tau = spatial_dyn::opspace::InverseDynamics(ab, J, ddx_dw, &N);

    // Add joint task in nullspace
    static const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(ab.dof(), ab.dof());
    Eigen::VectorXd ddq = ctrl_utils::PdControl(ab.q(), q_des, ab.dq(), fut_kp_kv_joint.get());
    tau += spatial_dyn::opspace::InverseDynamics(ab, I, ddq, &N);

    // Add gravity compensation
    tau += spatial_dyn::Gravity(ab);

    // Parse interaction from web app
    nlohmann::json interaction = fut_interaction.get();
    std::map<size_t, spatial_dyn::SpatialForced> f_ext = ComputeExternalForces(ab, interaction);

    // Integrate
    spatial_dyn::Integrate(ab, tau, timer.dt(), f_ext);

    // Update object states
    const Eigen::Isometry3d& T_ee_to_world = ab.T_to_world(-1) * T_ee;
    const ctrl_utils::Tree<std::string, logic_opt::Frame> frame_tree = world.frames(idx_trajectory);
    if (frame_tree.is_ancestor(kEeFrame, control_frame)) {
      for (const auto& key_val : frame_tree.ancestors(control_frame)) {
        // Only check frames between control frame and ee
        const std::string& frame = key_val.first;
        if (frame == kEeFrame) break;
        Eigen::Isometry3d T_to_ee = world.T_to_frame(frame, kEeFrame, X_optimal, idx_trajectory);
        spatial_dyn::RigidBody& rb = sim_objects_abs.at(frame);
        rb.set_T_to_parent(T_ee_to_world * T_to_ee);

        redis_client.set(KEY_OBJECTS_PREFIX + frame + "::pos", rb.T_to_parent().translation());
        redis_client.set(KEY_OBJECTS_PREFIX + frame + "::ori", Eigen::Quaterniond(rb.T_to_parent().linear()).coeffs());
      }
    } else if (frame_tree.is_ancestor(control_frame, kEeFrame)) {
      for (const auto& key_val : frame_tree.ancestors(*frame_tree.parent(kEeFrame))) {
        // Only check frames between control frame and ee
        const std::string& frame = key_val.first;
        Eigen::Isometry3d T_to_ee = world.T_to_frame(frame, kEeFrame, X_optimal, idx_trajectory);
        spatial_dyn::RigidBody& rb = sim_objects_abs.at(frame);
        rb.set_T_to_parent(T_ee_to_world * T_to_ee);

        redis_client.set(KEY_OBJECTS_PREFIX + frame + "::pos", rb.T_to_parent().translation());
        redis_client.set(KEY_OBJECTS_PREFIX + frame + "::ori", Eigen::Quaterniond(rb.T_to_parent().linear()).coeffs());
        if (frame == control_frame) break;
      }
    }

    redis_client.set(KEY_SENSOR_Q, ab.q());
    redis_client.set(KEY_CONTROL_POS, x_des);
    redis_client.set(KEY_TRAJ_POS, spatial_dyn::Position(ab));
    redis_client.commit();

    if (ddx_dw.norm() < 0.001) {
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

namespace {

void InitializeWebApp(ctrl_utils::RedisClient& redis_client, const spatial_dyn::ArticulatedBody& ab,
                      const std::map<std::string, logic_opt::Object3>& objects, size_t T) {
  
  // Register the urdf path so the server knows it's safe to fulfill requests for files in that directory
  std::string path_urdf = ctrl_utils::AbsolutePath(ctrl_utils::CurrentPath() + "/" + kPathUrdf);
  redis_client.hset(KEY_WEB_RESOURCES, kNameApp, ctrl_utils::ParentPath(path_urdf));

  // Register key prefixes so the web app knows which models and objects to render.
  nlohmann::json web_keys;
  web_keys["key_models_prefix"]       = KEY_MODELS_PREFIX;
  web_keys["key_objects_prefix"]      = KEY_OBJECT_MODELS_PREFIX;
  web_keys["key_trajectories_prefix"] = KEY_TRAJ_PREFIX;
  redis_client.set(KEY_WEB_ARGS, web_keys);

  // Register the robot
  nlohmann::json web_model;
  web_model["model"] = ab;
  web_model["key_q"] = KEY_SENSOR_Q;
  redis_client.set(KEY_MODELS_PREFIX + ab.name, web_model);

  // Create objects
  for (const std::pair<std::string, spatial_dyn::RigidBody>& key_val : objects) {
    const std::string& frame = key_val.first;
    if (frame == kEeFrame) continue;
    const spatial_dyn::RigidBody& object = key_val.second;
    
    nlohmann::json web_object;
    web_object["graphics"] = object.graphics;
    web_object["key_pos"] = KEY_OBJECTS_PREFIX + frame + "::pos";
    web_object["key_ori"] = KEY_OBJECTS_PREFIX + frame + "::ori";
    redis_client.set(KEY_OBJECT_MODELS_PREFIX + frame, web_object);
    redis_client.set(KEY_OBJECTS_PREFIX + frame + "::pos", object.T_to_parent().translation());
    redis_client.set(KEY_OBJECTS_PREFIX + frame + "::ori", Eigen::Quaterniond(object.T_to_parent().linear()).coeffs());
  }

  // Create a sphere marker for x_des
  nlohmann::json web_object;
  spatial_dyn::Graphics x_des_marker("x_des_marker");
  x_des_marker.geometry.type = spatial_dyn::Graphics::Geometry::Type::kSphere;
  x_des_marker.geometry.radius = 0.01;
  web_object["graphics"] = std::vector<spatial_dyn::Graphics>{ x_des_marker };
  web_object["key_pos"] = KEY_CONTROL_POS;
  redis_client.set(KEY_OBJECT_MODELS_PREFIX + x_des_marker.name, web_object);

  // Create a sphere marker for each frame
  // for (size_t t = 0; t < T; t++) {
  //   nlohmann::json web_object;
  //   spatial_dyn::Graphics frame_marker("frame_marker::" + std::to_string(t));
  //   frame_marker.geometry.type = spatial_dyn::Graphics::Geometry::Type::kSphere;
  //   frame_marker.geometry.radius = 0.01;
  //   web_object["graphics"] = std::vector<spatial_dyn::Graphics>{ frame_marker };
  //   web_object["key_pos"] = KEY_TRAJ_PREFIX + "frame::" + std::to_string(t) + "::pos";
  //   redis_client.set(KEY_OBJECT_MODELS_PREFIX + frame_marker.name, web_object);
  // }
}

std::map<size_t, spatial_dyn::SpatialForced> ComputeExternalForces(const spatial_dyn::ArticulatedBody& ab,
    const nlohmann::json& interaction) {
  std::map<size_t, spatial_dyn::SpatialForced> f_ext;

  // Check if the clicked object is the robot
  std::string key_object = interaction["key_object"].get<std::string>();
  if (key_object != KEY_MODELS_PREFIX + ab.name) return f_ext;

  // Extract the json fields
  size_t idx_link = interaction["idx_link"].get<size_t>();
  Eigen::Vector3d pos_mouse = interaction["pos_mouse_in_world"].get<Eigen::Vector3d>();
  Eigen::Vector3d pos_click = interaction["pos_click_in_link"].get<Eigen::Vector3d>();

  // Get the click position in world coordinates
  Eigen::Vector3d pos_click_in_world = spatial_dyn::Position(ab, idx_link, pos_click);

  // Set the click force
  Eigen::Vector3d f = kGainClickDrag * (pos_mouse - pos_click_in_world);
  spatial_dyn::SpatialForced f_click(f, Eigen::Vector3d::Zero());

  // Translate the spatial force to the world frame
  f_ext[idx_link] = Eigen::Translation3d(pos_click_in_world) * f_click;

  return f_ext;
}

}  // namespace

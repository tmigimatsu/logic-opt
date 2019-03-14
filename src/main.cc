/**
 * main.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 19, 2018
 * Authors: Toki Migimatsu
 */

#include <algorithm>  // std::max, std::transform
#include <atomic>     // std::atomic
#include <cctype>     // std::tolower
#include <chrono>     // std::chrono
#include <cmath>      // M_PI
#include <csignal>    // std::signal, std::sig_atomic_t
#include <fstream>    // std::ofstream
#include <future>     // std::future
#include <iostream>   // std::cout
#include <map>        // std::map
#include <numeric>    // std::accumulate
#include <string>     // std::string
#include <thread>     // std::thread
#include <time.h>     // ::gmtime_r, std::strftime
#include <sys/stat.h>  // mkdir

#include <spatial_dyn/spatial_dyn.h>
#include <ctrl_utils/filesystem.h>
#include <ctrl_utils/json.h>
#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/timer.h>
#include <ctrl_utils/yaml.h>

#include "LogicOpt/optimization/ipopt.h"
#include "LogicOpt/optimization/nlopt.h"
#include "LogicOpt/optimization/objectives.h"
#include "LogicOpt/constraints.h"
#include "LogicOpt/world.h"

#include "LogicOpt/planning/a_star.h"
#include "LogicOpt/planning/breadth_first_search.h"
#include "LogicOpt/planning/depth_first_search.h"
#include "LogicOpt/planning/pddl.h"
#include "LogicOpt/planning/planner.h"

#include "LogicOpt/utils/atomic_queue.h"

namespace {

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

AtomicQueue<std::tuple<Eigen::MatrixXd, LogicOpt::World, std::vector<LogicOpt::Planner::Node>>> g_redis_queue;
std::atomic<int> g_num_optimizations = { 0 };
std::condition_variable g_cv_optimizations_clear;
std::atomic<bool> g_is_redis_thread_running = { false };

volatile std::sig_atomic_t g_runloop = true;
void stop(int) {
  g_runloop = false;
  LogicOpt::Ipopt::Terminate();
  g_redis_queue.Terminate();
}

struct Args {
  enum class Optimizer { NLOPT, IPOPT };
  enum class Task { JOINT_POSITION, CARTESIAN_POSE, PICK_PLACE, SLIDE, PUSH };

  Optimizer optimizer = Optimizer::IPOPT;
  Task task = Task::CARTESIAN_POSE;
  bool with_scalar_constraints = false;
  bool with_hessian = false;
  std::string logdir;
  std::string yaml;
};

Args ParseArgs(int argc, char *argv[]) {
  Args parsed_args;
  if (argc < 2) throw std::invalid_argument("ParseArgs(): Missing config.yaml argument.");
  int i = 1;
  parsed_args.yaml = argv[i];

  std::string arg;
  for (i++; i < argc; i++) {
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
    } else if (arg == "--logdir") {
      i++;
      if (i >= argc) continue;
      parsed_args.logdir = argv[i];
    } else if (arg == "--with-scalar-constraints") {
      parsed_args.with_scalar_constraints = true;
    } else if (arg == "--with-hessian") {
      parsed_args.with_hessian = false;
    } else {
      break;
    }
  }

  if (i != argc) throw std::invalid_argument("ParseArgs(): Invalid '" + arg + "' argument.");
  return parsed_args;
}

void CheckRequired(const YAML::Node& node, std::vector<std::string>::const_iterator it,
                   std::vector<std::string>::const_iterator it_end,
                   const std::string& name) {
  if (it == it_end) return;
  if (*it == "//array") {
    if (!node.IsSequence()) {
      throw std::invalid_argument("ValidateYaml(): " + name + " must be an array.");
    }
    for (size_t i = 0; i < node.size(); i++) {
      CheckRequired(node[i], it + 1, it_end, name + "[" + std::to_string(i) + "]");
    }
    return;
  }
  if (!node[*it]) {
    throw std::invalid_argument("ValidateYaml(): " + name + " is missing required field: " + *it);
  }
  CheckRequired(node[*it], it + 1, it_end, name + "." + *it);
}

void CheckRequired(const YAML::Node& node, const std::vector<std::string>& fields) {
  CheckRequired(node, fields.begin(), fields.end(), "yaml");
}

void ValidateYaml(const YAML::Node& yaml) {
  CheckRequired(yaml, {"planner", "domain"});
  CheckRequired(yaml, {"planner", "problem"});
  CheckRequired(yaml, {"optimizer", "engine"});
  CheckRequired(yaml, {"robot", "urdf"});
  // CheckRequired(yaml, {"robot", "q_home"});
  // CheckRequired(yaml, {"robot", "ee_offset"});
  CheckRequired(yaml, {"world", "objects"});
  CheckRequired(yaml, {"world", "objects", "//array", "name"});
  CheckRequired(yaml, {"world", "objects", "//array", "T_to_parent"});
  CheckRequired(yaml, {"world", "objects", "//array", "T_to_parent", "pos"});
  CheckRequired(yaml, {"world", "objects", "//array", "T_to_parent", "ori"});
  CheckRequired(yaml, {"world", "objects", "//array", "T_to_parent", "ori", "w"});
  CheckRequired(yaml, {"world", "objects", "//array", "T_to_parent", "ori", "x"});
  CheckRequired(yaml, {"world", "objects", "//array", "T_to_parent", "ori", "y"});
  CheckRequired(yaml, {"world", "objects", "//array", "T_to_parent", "ori", "z"});
  CheckRequired(yaml, {"world", "objects", "//array", "graphics"});
  CheckRequired(yaml, {"world", "objects", "//array", "graphics", "//array", "geometry"});
  CheckRequired(yaml, {"world", "objects", "//array", "graphics", "//array", "geometry", "type"});
}

void ValidateWorldObjects(const std::shared_ptr<const std::map<std::string, LogicOpt::Object>>& world_objects,
                          const LogicOpt::Planner& planner) {
  for (const auto& key_val : planner.objects()) {
    const std::vector<const VAL::parameter_symbol*> objects = key_val.second;
    for (const VAL::parameter_symbol* object : objects) {
      std::string name = object->getName();
      if (world_objects->find(name) == world_objects->end()) {
        throw std::runtime_error("ValidateWorldObjects(): '" + name + "' does not exist in world object store.");
      }
    }
  }
}

void InitializeWebApp(ctrl_utils::RedisClient& redis_client, const spatial_dyn::ArticulatedBody& ab,
                      const std::map<std::string, LogicOpt::Object>& objects, size_t T);

std::map<size_t, spatial_dyn::SpatialForced> ComputeExternalForces(const spatial_dyn::ArticulatedBody& ab,
                                                                   const nlohmann::json& interaction);

void RedisPublishTrajectories(spatial_dyn::ArticulatedBody ab,
                              const std::shared_ptr<const std::map<std::string, LogicOpt::Object>> world_objects) {
  g_is_redis_thread_running = true;

  // Set up timer and Redis
  ctrl_utils::RedisClient redis_client;
  redis_client.connect();

  // End-effector parameters
  const Eigen::Isometry3d& T_ee = world_objects->at(kEeFrame).T_to_parent();
  Eigen::Quaterniond quat_ee(T_ee.linear());
  Eigen::Ref<const Eigen::Vector3d> ee_offset = T_ee.translation();

  // Initialize controller parameters
  Eigen::VectorXd q_des = kQHome;

  try {
    while (g_runloop) {
      if (g_num_optimizations <= 0) g_cv_optimizations_clear.notify_all();
      const auto optimization_result = g_redis_queue.Pop();
      --g_num_optimizations;

      const Eigen::MatrixXd& X_optimal = std::get<0>(optimization_result);
      const LogicOpt::World& world = std::get<1>(optimization_result);
      const std::vector<LogicOpt::Planner::Node>& plan = std::get<2>(optimization_result);

      for (const LogicOpt::Planner::Node& node : plan) {
        std::cout << node << std::endl;
      }
      std::cout << std::endl;
      // std::cout << X_optimal << std::endl << std::endl;

      ab.set_q(kQHome);
      ab.set_dq(Eigen::VectorXd::Zero(ab.dof()));
      InitializeWebApp(redis_client, ab, *world_objects, world.num_timesteps());
      redis_client.set(KEY_SENSOR_Q, ab.q());
      redis_client.set(KEY_KP_KV_POS, kKpKvPos);
      redis_client.set(KEY_KP_KV_ORI, kKpKvOri);
      redis_client.set(KEY_KP_KV_JOINT, kKpKvJoint);
      redis_client.sync_commit();

      ctrl_utils::Timer timer(1000);

      size_t idx_trajectory = 0;
      std::map<std::string, LogicOpt::Object> sim_objects_abs = *world_objects;
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
        Eigen::Vector3d ddx = spatial_dyn::PdControl(x, x_des, dx, fut_kp_kv_pos.get(), kMaxVel);

        // Compute orientation PD control
        Eigen::Quaterniond quat = spatial_dyn::Orientation(ab);
        Eigen::Quaterniond quat_des = spatial_dyn::opspace::NearQuaternion(T_des_to_world.linear(), quat);
        Eigen::Vector3d w = J.bottomRows<3>() * ab.dq();
        Eigen::Vector3d dw = spatial_dyn::PdControl(quat, quat_des, w, fut_kp_kv_ori.get());

        // Compute opspace torques
        Eigen::Vector6d ddx_dw;
        ddx_dw << ddx, dw;
        Eigen::MatrixXd N;
        Eigen::VectorXd tau = spatial_dyn::opspace::InverseDynamics(ab, J, ddx_dw, &N);

        // Add joint task in nullspace
        static const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(ab.dof(), ab.dof());
        Eigen::VectorXd ddq = spatial_dyn::PdControl(ab.q(), q_des, ab.dq(), fut_kp_kv_joint.get());
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
        const LogicOpt::Tree<std::string, LogicOpt::Frame> frame_tree = world.frames(idx_trajectory);
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

        if ((ab.q().array() != ab.q().array()).any()) break;
      }
      std::cout << "Simulated " << timer.time_sim() << "s in " << timer.time_elapsed() << "s." << std::endl;
      std::cout << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "RedisPublishTrajectories(): " << e.what() << std::endl;
    g_runloop = false;
  }

  g_is_redis_thread_running = false;
  g_cv_optimizations_clear.notify_all();
  std::cout << "Exiting Redis thread." << std::endl;
}

using ConstraintConstructor = std::function<LogicOpt::Constraint*(const LogicOpt::Proposition&,
                                                                  LogicOpt::World& world,
                                                                  spatial_dyn::ArticulatedBody&, size_t)>;
std::map<std::string, ConstraintConstructor> CreateConstraintFactory() {
  std::map<std::string, ConstraintConstructor> actions;
  actions[""] = [](const LogicOpt::Proposition& action, LogicOpt::World& world,
                             spatial_dyn::ArticulatedBody& ab, size_t t) {
    const Eigen::Isometry3d& T_ee = world.objects()->at(kEeFrame).T_to_parent();
    const auto& pos = spatial_dyn::Position(ab) - T_ee.translation();
    const Eigen::Quaterniond quat = spatial_dyn::Orientation(ab) * Eigen::Quaterniond(T_ee.linear());
    // std::cout << "t = " << t << ": cartesian_pose(" << kEeFrame << ", pos("
    //           << pos(0) << ", " << pos(1) << ", " << pos(2) << "), quat("
    //           << quat.x() << ", " << quat.y() << ", " << quat.z() << "; " << quat.w()
    //           << "))" << std::endl;
    return new LogicOpt::CartesianPoseConstraint(world, t, kEeFrame, world.kWorldFrame, pos, quat);
  };
  actions["pick"] = [](const LogicOpt::Proposition& action, LogicOpt::World& world,
                                 spatial_dyn::ArticulatedBody& ab, size_t t) {
    std::string object = action.variables()[0]->getName();
    // std::cout << "t = " << t << ": pick(" << object << ")" << std::endl;
    return new LogicOpt::PickConstraint(world, t, kEeFrame, object);
  };
  actions["place"] = [](const LogicOpt::Proposition& action, LogicOpt::World& world,
                        spatial_dyn::ArticulatedBody& ab, size_t t) {
    std::string object = action.variables()[0]->getName();
    std::string target = action.variables()[1]->getName();
    // std::cout << "t = " << t << ": place(" << object << ", " << target << ")" << std::endl;
    return new LogicOpt::PlaceConstraint(world, t, object, target);
  };
  actions["push"] = [](const LogicOpt::Proposition& action, LogicOpt::World& world,
                       spatial_dyn::ArticulatedBody& ab, size_t t) {
    std::string pusher = action.variables()[0]->getName();
    std::string object = action.variables()[1]->getName();
    std::string surface = action.variables()[2]->getName();
    // std::cout << "t = " << t << ": push(" << pusher << ", " << object << ", " << surface << ")" << std::endl;
    return new LogicOpt::PushConstraint(world, t, pusher, object);
  };

  return actions;
};

std::future<Eigen::MatrixXd> AsyncOptimize(const std::vector<LogicOpt::Planner::Node>& plan,
                                           const std::shared_ptr<const std::map<std::string, LogicOpt::Object>>& world_objects,
                                           const std::unique_ptr<LogicOpt::Optimizer>& optimizer,
                                           const std::map<std::string, ConstraintConstructor>& constraint_factory,
                                           const spatial_dyn::ArticulatedBody& const_ab) {

  std::function<Eigen::MatrixXd()> optimize = [plan, world_objects, &optimizer,
                                               &constraint_factory, &const_ab]() -> Eigen::MatrixXd {
    try {
      spatial_dyn::ArticulatedBody ab = const_ab;
      const Eigen::Isometry3d& T_ee = world_objects->at(kEeFrame).T_to_parent();
      const Eigen::Quaterniond quat_ee(T_ee.linear());
      const Eigen::Ref<const Eigen::Vector3d> ee_offset = T_ee.translation();

      LogicOpt::World world(world_objects);

      // Create objectives
      LogicOpt::Objectives objectives;
      objectives.emplace_back(new LogicOpt::LinearVelocityObjective(world, kEeFrame));
      objectives.emplace_back(new LogicOpt::AngularVelocityObjective(world, kEeFrame));

      // Create task constraints
      LogicOpt::Constraints constraints;
      size_t t = 0;
      for (const LogicOpt::Planner::Node& node : plan) {
        const LogicOpt::Proposition& action = node.action();
        constraints.emplace_back(constraint_factory.at(action.predicate())(action, world, ab, t));
        t += constraints.back()->num_timesteps();
      }

      // Return to home at end of trajectory
      const LogicOpt::Proposition& home_action = plan.front().action();
      constraints.emplace_back(constraint_factory.at(home_action.predicate())(home_action, world, ab, t));
      t += constraints.back()->num_timesteps();

      // Check num timesteps
      const size_t T = world.num_timesteps();
      if (t != T) throw std::runtime_error("Constraint timesteps must equal T.");

      // Create variables
      LogicOpt::FrameVariables variables(T);

      // Optimize
      auto t_start = std::chrono::high_resolution_clock::now();
      Eigen::MatrixXd X_optimal = optimizer->Trajectory(variables, objectives, constraints);
      auto t_end = std::chrono::high_resolution_clock::now();

      std::cout << "Optimization time: " << std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count() << std::endl << std::endl;
      // std::cout << X_optimal << std::endl << std::endl;
      // for (const std::unique_ptr<LogicOpt::Constraint>& c : constraints) {
      //   Eigen::VectorXd f(c->num_constraints());
      //   c->Evaluate(X_optimal, f);
      //   std::cout << c->name << ": " << f.transpose() << std::endl;
      // }
      std::cout << std::endl;

      // Push results
      g_redis_queue.Emplace(X_optimal, world, plan);

      return X_optimal;
    } catch (const std::exception& e) {
      std::cerr << "Exception: " << e.what() << std::endl;
      throw e;
    }
  };

  return std::async(std::launch::async, std::move(optimize));
}

}  // namespace

int main(int argc, char *argv[]) {
  // Set up signal handlers
  std::signal(SIGINT, stop);

  Args args = ParseArgs(argc, argv);

  // Load yaml config
  YAML::Node yaml = YAML::LoadFile(args.yaml);
  ValidateYaml(yaml);

  // Load robot
  spatial_dyn::ArticulatedBody ab = spatial_dyn::urdf::LoadModel(yaml["robot"]["urdf"].as<std::string>());
  Eigen::VectorXd q_home = yaml["robot"]["q_home"] ? yaml["robot"]["q_home"].as<Eigen::VectorXd>()
                                                   : Eigen::VectorXd::Zero(ab.dof());
  Eigen::Vector3d ee_offset = yaml["robot"]["ee_offset"] ? yaml["robot"]["ee_offset"].as<Eigen::VectorXd>()
                                                         : Eigen::Vector3d::Zero();
  ab.set_q(q_home);

  // Create world objects
  auto world_objects = std::make_shared<std::map<std::string, LogicOpt::Object>>();
  for (const YAML::Node& node : yaml["world"]["objects"]) {
    world_objects->emplace(node["name"].as<std::string>(), node.as<spatial_dyn::RigidBody>());
  }
  {
    LogicOpt::Object ee(kEeFrame);
    ee.collision = std::make_unique<ncollide3d::shape::Ball>(0.01);
    ee.set_T_to_parent(spatial_dyn::Orientation(ab).inverse(), ee_offset);
    world_objects->emplace(std::string(ee.name), std::move(ee));
  }

  // Initialize planner
  std::string domain = yaml["planner"]["domain"].as<std::string>();
  std::string problem = yaml["planner"]["problem"].as<std::string>();
  std::unique_ptr<VAL::analysis> analysis = LogicOpt::ParsePddl(domain, problem);
  LogicOpt::Planner planner(analysis->the_domain, analysis->the_problem);

  // Validate planner and yaml consistency
  ValidateWorldObjects(world_objects, planner);

  // Initialize optimizer
  std::string name_optimizer = yaml["optimizer"]["engine"].as<std::string>();
  std::unique_ptr<LogicOpt::Optimizer> optimizer;
  if (name_optimizer == "nlopt") {
    optimizer = std::make_unique<LogicOpt::Nlopt>();
  } else if (name_optimizer == "ipopt") {
    optimizer = std::make_unique<LogicOpt::Ipopt>(yaml["optimizer"]["ipopt"]);
  }

  // Create constraints
  std::map<std::string, ConstraintConstructor> constraint_factory = CreateConstraintFactory();

  // Create redis listener
  std::thread redis_thread(RedisPublishTrajectories, ab, world_objects);

  // Perform search
  std::list<std::future<Eigen::MatrixXd>> optimization_results;
  auto t_start = std::chrono::high_resolution_clock::now();
  LogicOpt::BreadthFirstSearch<LogicOpt::Planner::Node> bfs(planner.root(), 5);
  for (const std::vector<LogicOpt::Planner::Node>& plan : bfs) {
    std::future<Eigen::MatrixXd> future_result = AsyncOptimize(plan, world_objects, optimizer,
                                                               constraint_factory, ab);
    optimization_results.push_back(std::move(future_result));
    ++g_num_optimizations;
    std::cout << "Optimize " << g_num_optimizations << std::endl;
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  std::cout << "Planning time: " << std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count() << std::endl << std::endl;

  // Join threads
  std::mutex m;
  std::unique_lock<std::mutex> lk(m);
  while (g_is_redis_thread_running && g_num_optimizations != 0) {
    g_cv_optimizations_clear.wait(lk);
  }
  g_redis_queue.Finish();
  redis_thread.join();

  return 0;
}

namespace {

void InitializeWebApp(ctrl_utils::RedisClient& redis_client, const spatial_dyn::ArticulatedBody& ab,
                      const std::map<std::string, LogicOpt::Object>& objects, size_t T) {
  
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

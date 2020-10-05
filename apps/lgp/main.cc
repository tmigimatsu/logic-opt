/**
 * main.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 19, 2018
 * Authors: Toki Migimatsu
 */

#include <ctrl_utils/atomic_queue.h>
#include <ctrl_utils/filesystem.h>
#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/yaml.h>
#include <spatial_dyn/spatial_dyn.h>
#include <spatial_opt/optimizers/ipopt.h>
#include <spatial_opt/optimizers/nlopt.h>
#include <spatial_opt/variables/frame_variables.h>
#include <symbolic/pddl.h>
#include <symbolic/planning/a_star.h>
#include <symbolic/planning/breadth_first_search.h>
#include <symbolic/planning/depth_first_search.h>
#include <symbolic/planning/planner.h>
#include <sys/stat.h>  // mkdir
#include <time.h>      // ::gmtime_r, std::strftime

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

#include "logic_opt/constraints.h"
#include "logic_opt/control/opspace_controller.h"
#include "logic_opt/objectives.h"
#include "logic_opt/world.h"

namespace Eigen {

using Vector7d = Eigen::Matrix<double, 7, 1>;

}  // namespace Eigen

namespace {

using ::logic_opt::Object;
using ::logic_opt::World;
using ::spatial_opt::Constraint;
using ::symbolic::Planner;
using ::symbolic::Proposition;

const std::string kEeFrame = "ee";

AtomicQueue<std::tuple<Eigen::MatrixXd, World, std::vector<Planner::Node>>>
    g_redis_queue;
std::atomic<int> g_num_optimizations = {0};
std::condition_variable g_cv_optimizations_clear;
std::atomic<bool> g_is_redis_thread_running = {false};

volatile std::sig_atomic_t g_runloop = true;
void stop(int) {
  g_runloop = false;
  g_redis_queue.Terminate();
}

struct Args {
  enum class Optimizer { NLOPT, IPOPT };

  Optimizer optimizer = Optimizer::IPOPT;
  bool with_scalar_constraints = false;
  bool with_hessian = false;
  std::string logdir;
  std::string yaml;
};

Args ParseArgs(int argc, char* argv[]) {
  Args parsed_args;
  if (argc < 2)
    throw std::invalid_argument("ParseArgs(): Missing config.yaml argument.");
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

  if (i != argc)
    throw std::invalid_argument("ParseArgs(): Invalid '" + arg + "' argument.");
  return parsed_args;
}

void CheckRequired(const YAML::Node& node,
                   std::vector<std::string>::const_iterator it,
                   std::vector<std::string>::const_iterator it_end,
                   const std::string& name) {
  if (it == it_end) return;
  if (*it == "//array") {
    if (!node.IsSequence()) {
      throw std::invalid_argument("ValidateYaml(): " + name +
                                  " must be an array.");
    }
    for (size_t i = 0; i < node.size(); i++) {
      CheckRequired(node[i], it + 1, it_end,
                    name + "[" + std::to_string(i) + "]");
    }
    return;
  }
  if (!node[*it]) {
    throw std::invalid_argument("ValidateYaml(): " + name +
                                " is missing required field: " + *it);
  }
  CheckRequired(node[*it], it + 1, it_end, name + "." + *it);
}

void CheckRequired(const YAML::Node& node,
                   const std::vector<std::string>& fields) {
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
  CheckRequired(yaml,
                {"world", "objects", "//array", "T_to_parent", "ori", "w"});
  CheckRequired(yaml,
                {"world", "objects", "//array", "T_to_parent", "ori", "x"});
  CheckRequired(yaml,
                {"world", "objects", "//array", "T_to_parent", "ori", "y"});
  CheckRequired(yaml,
                {"world", "objects", "//array", "T_to_parent", "ori", "z"});
  CheckRequired(yaml, {"world", "objects", "//array", "graphics"});
  CheckRequired(
      yaml, {"world", "objects", "//array", "graphics", "//array", "geometry"});
  CheckRequired(yaml, {"world", "objects", "//array", "graphics", "//array",
                       "geometry", "type"});
}

void ValidateWorldObjects(
    const std::shared_ptr<const std::map<std::string, Object>>& world_objects,
    const symbolic::Pddl& pddl) {
  for (const symbolic::Object& object : pddl.objects()) {
    if (world_objects->find(object.name()) == world_objects->end()) {
      throw std::runtime_error("ValidateWorldObjects(): '" + object.name() +
                               "' does not exist in world object store.");
    }
  }
}

void RedisPublishTrajectories(
    spatial_dyn::ArticulatedBody ab,
    const std::shared_ptr<const std::map<std::string, Object>> world_objects) {
  g_is_redis_thread_running = true;

  const Eigen::VectorXd q_home = ab.q();

  try {
    while (g_runloop) {
      if (g_num_optimizations <= 0) g_cv_optimizations_clear.notify_all();
      const auto optimization_result = g_redis_queue.Pop();
      --g_num_optimizations;

      const Eigen::MatrixXd& X_optimal = std::get<0>(optimization_result);
      const World& world = std::get<1>(optimization_result);
      const std::vector<Planner::Node>& plan = std::get<2>(optimization_result);

      for (const Planner::Node& node : plan) {
        std::cout << node << std::endl;
      }
      std::cout << std::endl;
      // std::cout << X_optimal << std::endl << std::endl;

      // Initialize robot.
      ab.set_q(q_home);
      ab.set_dq(Eigen::VectorXd::Zero(ab.dof()));

      // Create initial world.
      World world_0(world_objects);

      // Create shared memory with controller.
      const auto shared_memory =
          std::make_shared<logic_opt::PlannerControllerInterface>(g_runloop);
      // shared_memory->SetExecutionUpdate(*world_0.objects(),
      // world_0.frames(0), 0);

      // Start controller thread.
      std::thread thread_controller = std::thread(logic_opt::OpspaceController,
                                                  std::cref(*world_0.objects()),
                                                  std::cref(ab), shared_memory);

      // Publish initial result.
      shared_memory->SetOptimizationResult(Eigen::MatrixXd(X_optimal),
                                           World(world), 0);

      if (thread_controller.joinable()) {
        thread_controller.join();
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "RedisPublishTrajectories(): " << e.what() << std::endl;
    g_runloop = false;
  }

  g_is_redis_thread_running = false;
  g_cv_optimizations_clear.notify_all();
  std::cout << "Exiting Redis thread." << std::endl;
}

using ConstraintConstructor = std::function<Constraint*(
    const std::string&, World& world, spatial_dyn::ArticulatedBody&, size_t)>;
std::map<std::string, ConstraintConstructor> CreateConstraintFactory() {
  std::map<std::string, ConstraintConstructor> actions;
  actions[""] = [](const std::string& action, World& world,
                   spatial_dyn::ArticulatedBody& ab, size_t t) {
    // TODO: Get from yaml
    const Eigen::Vector3d kEeOffset =
        Eigen::Vector3d(0., 0., 0.107);  // Without gripper
    const Eigen::Vector3d kRobotiqGripperOffset =
        Eigen::Vector3d(0., 0., 0.144);  // Ranges from 0.130 to 0.144
    const Eigen::Vector3d ee_offset = kEeOffset + kRobotiqGripperOffset;
    const Eigen::Vector3d pos = spatial_dyn::Position(ab, -1, ee_offset);
    // const Eigen::Isometry3d& T_ee =
    // world.objects()->at(kEeFrame).T_to_parent(); const Eigen::Vector3d pos =
    // spatial_dyn::Position(ab, -1, T_ee.translation());
    const Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    // std::cout << "t = " << t << ": cartesian_pose(" << kEeFrame << ", pos("
    //           << pos(0) << ", " << pos(1) << ", " << pos(2) << "), quat("
    //           << quat.x() << ", " << quat.y() << ", " << quat.z() << "; " <<
    //           quat.w()
    //           << "))" << std::endl;
    return new logic_opt::CartesianPoseConstraint(world, t, kEeFrame,
                                                  world.kWorldFrame, pos, quat);
  };
  actions["pick"] = [](const std::string& action, World& world,
                       spatial_dyn::ArticulatedBody& ab, size_t t) {
    const std::vector<symbolic::Object> arguments =
        symbolic::Object::ParseArguments(action);
    const std::string& object = arguments[0].name();
    // std::cout << "t = " << t << ": pick(" << object << ")" << std::endl;
    return new logic_opt::PickConstraint(world, t, kEeFrame, object);
  };
  actions["place"] = [](const std::string& action, World& world,
                        spatial_dyn::ArticulatedBody& ab, size_t t) {
    const std::vector<symbolic::Object> arguments =
        symbolic::Object::ParseArguments(action);
    const std::string& object = arguments[0].name();
    const std::string& target = arguments[1].name();
    // std::cout << "t = " << t << ": place(" << object << ", " << target << ")"
    // << std::endl;
    return new logic_opt::PlaceConstraint(world, t, object, target);
  };
  actions["push"] = [](const std::string& action, World& world,
                       spatial_dyn::ArticulatedBody& ab, size_t t) {
    const std::vector<symbolic::Object> arguments =
        symbolic::Object::ParseArguments(action);
    const std::string& pusher = arguments[0].name();
    const std::string& object = arguments[1].name();
    const std::string& surface = arguments[2].name();
    // std::cout << "t = " << t << ": push(" << pusher << ", " << object << ", "
    // << surface << ")" << std::endl;
    return new logic_opt::PushConstraint(world, t, pusher, object, surface);
  };
  actions["throw"] = [](const std::string& action, World& world,
                        spatial_dyn::ArticulatedBody& ab, size_t t) {
    const std::vector<symbolic::Object> arguments =
        symbolic::Object::ParseArguments(action);
    const std::string& object = arguments[0].name();
    const std::string& target = arguments[1].name();
    // std::cout << "t = " << t << ": place(" << object << ", " << target << ")"
    // << std::endl;
    return new logic_opt::ThrowConstraint(world, t, object, target);
  };

  return actions;
};

std::future<Eigen::MatrixXd> AsyncOptimize(
    const std::vector<Planner::Node>& plan,
    const std::shared_ptr<const std::map<std::string, Object>>& world_objects,
    const std::unique_ptr<spatial_opt::Optimizer>& optimizer,
    const std::map<std::string, ConstraintConstructor>& constraint_factory,
    const spatial_dyn::ArticulatedBody& const_ab) {
  std::function<Eigen::MatrixXd()> optimize = [plan, world_objects, &optimizer,
                                               &constraint_factory,
                                               &const_ab]() -> Eigen::MatrixXd {
    try {
      spatial_dyn::ArticulatedBody ab = const_ab;

      World world(world_objects);

      // Initialize kinematic tree
      for (const symbolic::Proposition& P : plan.begin()->state()) {
        if (P.name() != "on") continue;
        assert(P.arguments().size() == 2);
        const std::string& control_frame = P.arguments()[0].name();
        const std::string& target_frame = P.arguments()[1].name();
        world.AttachFrame(control_frame, target_frame, 0, true);
        // TODO: Find better way to do this (also in controller)
      }

      // Create objectives
      spatial_opt::Objectives objectives;
      objectives.emplace_back(
          new logic_opt::LinearDistanceObjective(world, kEeFrame));
      objectives.emplace_back(
          new logic_opt::AngularDistanceObjective(world, kEeFrame, 3.));

      // Create task constraints
      spatial_opt::Constraints constraints;
      size_t t = 0;
      for (const Planner::Node& node : plan) {
        const std::string& action = node.action();
        constraints.emplace_back(constraint_factory.at(
            Proposition::ParseHead(action))(action, world, ab, t));
        t += constraints.back()->num_timesteps();
      }

      // Return to home at end of trajectory
      const std::string& home_action = plan.front().action();
      constraints.emplace_back(constraint_factory.at(
          Proposition::ParseHead(home_action))(home_action, world, ab, t));
      t += constraints.back()->num_timesteps();

      // Check num timesteps
      const size_t T = world.num_timesteps();
      if (t != T)
        throw std::runtime_error("Constraint timesteps must equal T.");

      // Create variables
      spatial_opt::FrameVariables variables(T);

      // Optimize
      auto t_start = std::chrono::high_resolution_clock::now();
      Eigen::MatrixXd X_optimal =
          optimizer->Trajectory(variables, objectives, constraints);
      auto t_end = std::chrono::high_resolution_clock::now();

      std::cout << "Optimization time: "
                << std::chrono::duration_cast<std::chrono::duration<double>>(
                       t_end - t_start)
                       .count()
                << std::endl
                << std::endl;
      std::cout << X_optimal << std::endl << std::endl;
      for (const std::unique_ptr<Constraint>& c : constraints) {
        Eigen::VectorXd f(c->num_constraints());
        c->Evaluate(X_optimal, f);
        std::cout << c->name << ":" << std::endl;
        for (size_t i = 0; i < c->num_constraints(); i++) {
          std::cout << "  "
                    << (c->constraint_type(i) == Constraint::Type::kInequality
                            ? "<"
                            : "=")
                    << " : " << f(i) << std::endl;
        }
      }
      std::cout << world << std::endl << std::endl;

      // Push results
      g_redis_queue.Emplace(X_optimal, world, plan);

      return X_optimal;
    } catch (const std::exception& e) {
      std::cerr << "AsyncOptimize(): Exception " << e.what() << std::endl;
      throw e;
    }
  };

  return std::async(std::launch::async, std::move(optimize));
}

}  // namespace

int main(int argc, char* argv[]) {
  // Set up signal handlers
  std::signal(SIGINT, stop);

  Args args = ParseArgs(argc, argv);

  // Load yaml config
  YAML::Node yaml = YAML::LoadFile(args.yaml);
  ValidateYaml(yaml);

  // Load robot
  spatial_dyn::ArticulatedBody ab =
      spatial_dyn::urdf::LoadModel(yaml["robot"]["urdf"].as<std::string>());
  Eigen::VectorXd q_home = yaml["robot"]["q_home"]
                               ? yaml["robot"]["q_home"].as<Eigen::VectorXd>()
                               : Eigen::VectorXd::Zero(ab.dof());
  Eigen::Vector3d ee_offset =
      yaml["robot"]["ee_offset"]
          ? yaml["robot"]["ee_offset"].as<Eigen::VectorXd>()
          : Eigen::Vector3d::Zero();
  ab.set_q(q_home);

  // Create world objects
  auto world_objects = std::make_shared<std::map<std::string, Object>>();
  for (const YAML::Node& node : yaml["world"]["objects"]) {
    world_objects->emplace(node["name"].as<std::string>(),
                           node.as<spatial_dyn::RigidBody>());
    std::cout << node["name"].as<std::string>() << ": " << std::endl
              << world_objects->at(node["name"].as<std::string>())
                     .T_to_parent()
                     .translation()
                     .transpose()
              << std::endl;
  }
  {
    if (world_objects->find(kEeFrame) == world_objects->end()) {
      world_objects->emplace(kEeFrame, kEeFrame);
    }
    Object& ee = world_objects->at(kEeFrame);
    // ee.set_T_to_parent(spatial_dyn::Orientation(ab).inverse(), ee_offset);
    ee.set_T_to_parent(Eigen::Quaterniond::Identity(),
                       spatial_dyn::Position(ab, -1, ee_offset));
  }

  // Initialize planner
  const std::filesystem::path path_resources =
      std::filesystem::path(args.yaml).parent_path();
  const std::string domain =
      (path_resources / yaml["planner"]["domain"].as<std::string>()).string();
  const std::string problem =
      (path_resources / yaml["planner"]["problem"].as<std::string>()).string();
  symbolic::Pddl pddl(domain, problem);
  Planner planner(pddl);

  // Validate planner and yaml consistency
  ValidateWorldObjects(world_objects, pddl);

  // Initialize optimizer
  std::string name_optimizer = yaml["optimizer"]["engine"].as<std::string>();
  std::unique_ptr<spatial_opt::Optimizer> optimizer;
  if (name_optimizer == "nlopt") {
    optimizer = std::make_unique<spatial_opt::Nlopt>();
  } else if (name_optimizer == "ipopt") {
    optimizer = std::make_unique<spatial_opt::Ipopt>(yaml["optimizer"]["ipopt"],
                                                     &g_runloop);
  }

  // Create constraints
  std::map<std::string, ConstraintConstructor> constraint_factory =
      CreateConstraintFactory();

  // Create redis listener
  std::thread redis_thread(RedisPublishTrajectories, ab, world_objects);

  // Perform search
  std::list<std::future<Eigen::MatrixXd>> optimization_results;
  auto t_start = std::chrono::high_resolution_clock::now();
  symbolic::BreadthFirstSearch<Planner::Node> bfs(
      planner.root(), yaml["planner"]["depth"].as<size_t>());
  for (const std::vector<Planner::Node>& plan : bfs) {
    for (const Planner::Node& node : plan) {
      std::cout << node << std::endl;
    }
    std::future<Eigen::MatrixXd> future_result =
        AsyncOptimize(plan, world_objects, optimizer, constraint_factory, ab);
    optimization_results.push_back(std::move(future_result));
    ++g_num_optimizations;
    std::cout << "Optimize " << g_num_optimizations << std::endl;
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  std::cout << "Planning time: "
            << std::chrono::duration_cast<std::chrono::duration<double>>(
                   t_end - t_start)
                   .count()
            << std::endl
            << std::endl;

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

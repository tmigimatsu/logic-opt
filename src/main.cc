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
#include <ctrl_utils/atomic_queue.h>
#include <ctrl_utils/filesystem.h>
#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/yaml.h>

#include "logic_opt/control/opspace_controller.h"
#include "logic_opt/optimization/constraints.h"
#include "logic_opt/optimization/ipopt.h"
#include "logic_opt/optimization/nlopt.h"
#include "logic_opt/optimization/objectives.h"
#include "logic_opt/world.h"

#include "logic_opt/planning/a_star.h"
#include "logic_opt/planning/breadth_first_search.h"
#include "logic_opt/planning/depth_first_search.h"
#include "logic_opt/planning/pddl.h"
#include "logic_opt/planning/planner.h"

namespace Eigen {

using Vector7d = Eigen::Matrix<double,7,1>;

}  // namespace Eigen

namespace {

const std::string kEeFrame = "ee";

AtomicQueue<std::tuple<Eigen::MatrixXd, logic_opt::World3, std::vector<logic_opt::Planner::Node>>> g_redis_queue;
std::atomic<int> g_num_optimizations = { 0 };
std::condition_variable g_cv_optimizations_clear;
std::atomic<bool> g_is_redis_thread_running = { false };

volatile std::sig_atomic_t g_runloop = true;
void stop(int) {
  g_runloop = false;
  logic_opt::Ipopt::Terminate();
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

void ValidateWorldObjects(const std::shared_ptr<const std::map<std::string, logic_opt::Object3>>& world_objects,
                          const logic_opt::Planner& planner) {
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

void RedisPublishTrajectories(spatial_dyn::ArticulatedBody ab,
                              const std::shared_ptr<const std::map<std::string, logic_opt::Object3>> world_objects) {
  g_is_redis_thread_running = true;

  const Eigen::VectorXd q_home = ab.q();

  try {
    while (g_runloop) {
      if (g_num_optimizations <= 0) g_cv_optimizations_clear.notify_all();
      const auto optimization_result = g_redis_queue.Pop();
      --g_num_optimizations;

      const Eigen::MatrixXd& X_optimal = std::get<0>(optimization_result);
      const logic_opt::World3& world = std::get<1>(optimization_result);
      const std::vector<logic_opt::Planner::Node>& plan = std::get<2>(optimization_result);

      for (const logic_opt::Planner::Node& node : plan) {
        std::cout << node << std::endl;
      }
      std::cout << std::endl;
      // std::cout << X_optimal << std::endl << std::endl;

      ab.set_q(q_home);
      ab.set_dq(Eigen::VectorXd::Zero(ab.dof()));

      logic_opt::ExecuteOpspaceController(ab, world, X_optimal, g_runloop);
    }
  } catch (const std::exception& e) {
    std::cerr << "RedisPublishTrajectories(): " << e.what() << std::endl;
    g_runloop = false;
  }

  g_is_redis_thread_running = false;
  g_cv_optimizations_clear.notify_all();
  std::cout << "Exiting Redis thread." << std::endl;
}

using ConstraintConstructor = std::function<logic_opt::Constraint*(const logic_opt::Proposition&,
                                                                  logic_opt::World3& world,
                                                                  spatial_dyn::ArticulatedBody&, size_t)>;
std::map<std::string, ConstraintConstructor> CreateConstraintFactory() {
  std::map<std::string, ConstraintConstructor> actions;
  actions[""] = [](const logic_opt::Proposition& action, logic_opt::World3& world,
                   spatial_dyn::ArticulatedBody& ab, size_t t) {
    const Eigen::Isometry3d& T_ee = world.objects()->at(kEeFrame).T_to_parent();
    const Eigen::Vector3d pos = spatial_dyn::Position(ab, -1, T_ee.translation());
    const Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    // std::cout << "t = " << t << ": cartesian_pose(" << kEeFrame << ", pos("
    //           << pos(0) << ", " << pos(1) << ", " << pos(2) << "), quat("
    //           << quat.x() << ", " << quat.y() << ", " << quat.z() << "; " << quat.w()
    //           << "))" << std::endl;
    return new logic_opt::CartesianPoseConstraint<3>(world, t, kEeFrame, world.kWorldFrame, pos, quat);
  };
  actions["pick"] = [](const logic_opt::Proposition& action, logic_opt::World3& world,
                       spatial_dyn::ArticulatedBody& ab, size_t t) {
    std::string object = action.variables()[0]->getName();
    // std::cout << "t = " << t << ": pick(" << object << ")" << std::endl;
    return new logic_opt::PickConstraint(world, t, kEeFrame, object);
  };
  actions["place"] = [](const logic_opt::Proposition& action, logic_opt::World3& world,
                        spatial_dyn::ArticulatedBody& ab, size_t t) {
    std::string object = action.variables()[0]->getName();
    std::string target = action.variables()[1]->getName();
    // std::cout << "t = " << t << ": place(" << object << ", " << target << ")" << std::endl;
    return new logic_opt::PlaceConstraint(world, t, object, target);
  };
  actions["push"] = [](const logic_opt::Proposition& action, logic_opt::World3& world,
                       spatial_dyn::ArticulatedBody& ab, size_t t) {
    std::string pusher = action.variables()[0]->getName();
    std::string object = action.variables()[1]->getName();
    std::string surface = action.variables()[2]->getName();
    // std::cout << "t = " << t << ": push(" << pusher << ", " << object << ", " << surface << ")" << std::endl;
    return new logic_opt::PushConstraint(world, t, pusher, object, surface);
  };
  actions["throw"] = [](const logic_opt::Proposition& action, logic_opt::World3& world,
                        spatial_dyn::ArticulatedBody& ab, size_t t) {
    std::string object = action.variables()[0]->getName();
    std::string target = action.variables()[1]->getName();
    // std::cout << "t = " << t << ": place(" << object << ", " << target << ")" << std::endl;
    return new logic_opt::ThrowConstraint(world, t, object, target);
  };

  return actions;
};

std::future<Eigen::MatrixXd> AsyncOptimize(const std::vector<logic_opt::Planner::Node>& plan,
                                           const std::shared_ptr<const std::map<std::string, logic_opt::Object3>>& world_objects,
                                           const std::unique_ptr<logic_opt::Optimizer>& optimizer,
                                           const std::map<std::string, ConstraintConstructor>& constraint_factory,
                                           const spatial_dyn::ArticulatedBody& const_ab) {

  std::function<Eigen::MatrixXd()> optimize = [plan, world_objects, &optimizer,
                                               &constraint_factory, &const_ab]() -> Eigen::MatrixXd {
    try {
      spatial_dyn::ArticulatedBody ab = const_ab;
      const Eigen::Isometry3d& T_ee = world_objects->at(kEeFrame).T_to_parent();
      const Eigen::Quaterniond quat_ee(T_ee.linear());
      const Eigen::Ref<const Eigen::Vector3d> ee_offset = T_ee.translation();

      logic_opt::World3 world(world_objects);

      // Create objectives
      logic_opt::Objectives objectives;
      objectives.emplace_back(new logic_opt::LinearVelocityObjective3(world, kEeFrame));
      objectives.emplace_back(new logic_opt::AngularVelocityObjective(world, kEeFrame, 2.));

      // Create task constraints
      logic_opt::Constraints constraints;
      size_t t = 0;
      for (const logic_opt::Planner::Node& node : plan) {
        const logic_opt::Proposition& action = node.action();
        constraints.emplace_back(constraint_factory.at(action.predicate())(action, world, ab, t));
        t += constraints.back()->num_timesteps();
      }

      // Return to home at end of trajectory
      const logic_opt::Proposition& home_action = plan.front().action();
      constraints.emplace_back(constraint_factory.at(home_action.predicate())(home_action, world, ab, t));
      t += constraints.back()->num_timesteps();

      // Check num timesteps
      const size_t T = world.num_timesteps();
      if (t != T) throw std::runtime_error("Constraint timesteps must equal T.");

      // Create variables
      logic_opt::FrameVariables<3> variables(T);

      // Optimize
      auto t_start = std::chrono::high_resolution_clock::now();
      Eigen::MatrixXd X_optimal = optimizer->Trajectory(variables, objectives, constraints);
      auto t_end = std::chrono::high_resolution_clock::now();

      std::cout << "Optimization time: " << std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count() << std::endl << std::endl;
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
  auto world_objects = std::make_shared<std::map<std::string, logic_opt::Object3>>();
  for (const YAML::Node& node : yaml["world"]["objects"]) {
    world_objects->emplace(node["name"].as<std::string>(), node.as<spatial_dyn::RigidBody>());
  }
  {
    if (world_objects->find(kEeFrame) == world_objects->end()) {
      world_objects->emplace(kEeFrame, kEeFrame);
    }
    logic_opt::Object3& ee = world_objects->at(kEeFrame);
    ee.set_T_to_parent(spatial_dyn::Orientation(ab).inverse(), ee_offset);
  }

  // Initialize planner
  const std::filesystem::path path_resources = std::filesystem::path(args.yaml).parent_path();
  const std::string domain = (path_resources / yaml["planner"]["domain"].as<std::string>()).string();
  const std::string problem = (path_resources / yaml["planner"]["problem"].as<std::string>()).string();
  const std::unique_ptr<VAL::analysis> analysis = logic_opt::ParsePddl(domain, problem);
  logic_opt::Planner planner(analysis->the_domain, analysis->the_problem);

  // Validate planner and yaml consistency
  ValidateWorldObjects(world_objects, planner);

  // Initialize optimizer
  std::string name_optimizer = yaml["optimizer"]["engine"].as<std::string>();
  std::unique_ptr<logic_opt::Optimizer> optimizer;
  if (name_optimizer == "nlopt") {
    optimizer = std::make_unique<logic_opt::Nlopt>();
  } else if (name_optimizer == "ipopt") {
    optimizer = std::make_unique<logic_opt::Ipopt>(yaml["optimizer"]["ipopt"]);
  }

  // Create constraints
  std::map<std::string, ConstraintConstructor> constraint_factory = CreateConstraintFactory();

  // Create redis listener
  std::thread redis_thread(RedisPublishTrajectories, ab, world_objects);

  // Perform search
  std::list<std::future<Eigen::MatrixXd>> optimization_results;
  auto t_start = std::chrono::high_resolution_clock::now();
  logic_opt::BreadthFirstSearch<logic_opt::Planner::Node> bfs(planner.root(), 5);
  for (const std::vector<logic_opt::Planner::Node>& plan : bfs) {
    for (const logic_opt::Planner::Node& node : plan) {
      std::cout << node << std::endl;
    }
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

/**
 * main.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 19, 2018
 * Authors: Toki Migimatsu
 */

#include <algorithm>  // std::max, std::transform
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
#include <yaml-cpp/yaml.h>

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

namespace YAML {

template<>
struct convert<Eigen::VectorXd> {

  static Node encode(const Eigen::VectorXd& rhs) {
    Node node;
    for (size_t i = 0; i < rhs.size(); i++) {
      node.push_back(rhs(i));
    }
    return node;
  }

  static bool decode(const Node& node, Eigen::VectorXd& rhs) {
    if (!node.IsSequence()) return false;
    rhs.resize(node.size());
    for (size_t i = 0; i < node.size(); i++) {
      rhs(i) = node[i].as<double>();
    }
    return true;
  }

};

template<>
struct convert<Eigen::Quaterniond> {

  static Node encode(const Eigen::Quaterniond& rhs) {
    Node node;
    node["w"] = rhs.w();
    node["x"] = rhs.x();
    node["y"] = rhs.y();
    node["z"] = rhs.z();
    return node;
  }

  static bool decode(const Node& node, Eigen::Quaterniond& rhs) {
    if (!node.IsMap() || !node["w"] || !node["x"] || !node["y"] || !node["z"]) return false;
    rhs.w() = node["w"].as<double>();
    rhs.x() = node["x"].as<double>();
    rhs.y() = node["y"].as<double>();
    rhs.z() = node["z"].as<double>();
    return true;
  }

};

}  // namespace YAML

namespace {

AtomicQueue<std::pair<Eigen::MatrixXd, LogicOpt::World>> g_redis_queue;
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
      parsed_args.with_hessian = true;
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
  CheckRequired(yaml, {"world", "objects", "//array", "type"});
  CheckRequired(yaml, {"world", "objects", "//array", "scale"});
  CheckRequired(yaml, {"world", "objects", "//array", "pos"});
  CheckRequired(yaml, {"world", "objects", "//array", "ori"});
  CheckRequired(yaml, {"world", "objects", "//array", "ori", "w"});
  CheckRequired(yaml, {"world", "objects", "//array", "ori", "x"});
  CheckRequired(yaml, {"world", "objects", "//array", "ori", "y"});
  CheckRequired(yaml, {"world", "objects", "//array", "ori", "z"});
}

std::map<std::string, spatial_dyn::RigidBody> ParseYamlWorldObjects(const YAML::Node& yaml_objects) {
  std::map<std::string, spatial_dyn::RigidBody> world_objects;
  for (const YAML::Node& yaml_object : yaml_objects) {
    spatial_dyn::RigidBody obj(yaml_object["name"].as<std::string>());

    std::string type = yaml_object["type"].as<std::string>();
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);
    if (type == "box") {
      obj.graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::BOX;
    } else {
      throw std::invalid_argument("world.objects[i].type = " + type + " not supported yet.");
    }

    obj.graphics.geometry.scale = yaml_object["scale"].as<Eigen::VectorXd>();

    Eigen::Vector3d pos = yaml_object["pos"].as<Eigen::VectorXd>();
    Eigen::Quaterniond ori = yaml_object["ori"].as<Eigen::Quaterniond>();

    obj.set_T_to_parent(ori, pos);

    world_objects[obj.name] = obj;
  }
  return world_objects;
}

void ValidateWorldObjects(const std::map<std::string, spatial_dyn::RigidBody>& world_objects,
                          const LogicOpt::Planner& planner) {
  for (const auto& key_val : planner.objects()) {
    const std::vector<const VAL::parameter_symbol*> objects = key_val.second;
    for (const VAL::parameter_symbol* object : objects) {
      std::string name = object->getName();
      if (world_objects.find(name) == world_objects.end()) {
        throw std::runtime_error("ValidateWorldObjects(): '" + name + "' does not exist in world object store.");
      }
    }
  }
}

void RedisPublishTrajectories(spatial_dyn::ArticulatedBody ab,
                              const std::map<std::string, spatial_dyn::RigidBody>& world_objects) {
  // Set up timer and Redis
  spatial_dyn::RedisClient redis_client;
  redis_client.connect();
  redis_client.set("spatialdyn::kuka_iiwa", spatial_dyn::Json::Serialize(ab).dump());
  for (const std::pair<std::string, spatial_dyn::RigidBody>& key_val : world_objects) {
    const spatial_dyn::RigidBody& object = key_val.second;
    redis_client.set("spatialdyn::objects::" + object.name, spatial_dyn::Json::Serialize(object).dump());
  }

  try {
    while (g_runloop) {
      std::pair<Eigen::MatrixXd, LogicOpt::World> optimization_result = g_redis_queue.Pop();

      const Eigen::MatrixXd& Q_optimal = optimization_result.first;
      LogicOpt::World& world = optimization_result.second;

      std::cout << Q_optimal.transpose() << std::endl << std::endl;
      world.Simulate(Q_optimal);

      spatial_dyn::Timer timer(1000);

      std::map<std::string, spatial_dyn::RigidBody> simulation_objects = world_objects;

      size_t idx_trajectory = 0;
      while (g_runloop) {
        timer.Sleep();
        Eigen::VectorXd q_err = ab.q() - Q_optimal.col(idx_trajectory);
        Eigen::VectorXd dq_err = ab.dq();
        Eigen::VectorXd ddq = -10. * q_err - 6. * dq_err;
        Eigen::VectorXd tau = spatial_dyn::InverseDynamics(ab, ddq, {}, true, true);

        spatial_dyn::Integrate(ab, tau, timer.dt());

        std::map<std::string, LogicOpt::World::ObjectState> world_state_t =
            world.InterpolateSimulation(ab.q(), std::max(static_cast<int>(idx_trajectory) - 1, 0));

        for (auto& key_val : world_state_t) {
          const std::string& name_object = key_val.first;
          const LogicOpt::World::ObjectState object_state_t = key_val.second;
          spatial_dyn::RigidBody& rb = simulation_objects[name_object];
          rb.set_T_to_parent(object_state_t.quat, object_state_t.pos);

          redis_client.set("spatialdyn::objects::" + name_object, spatial_dyn::Json::Serialize(rb).dump());
        }

        redis_client.set("spatialdyn::kuka_iiwa::sensor::q", ab.q().toMatlab());
        redis_client.set("spatialdyn::kuka_iiwa::trajectory::pos", spatial_dyn::Position(ab).toMatlab());
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
    }
  } catch (...) {}
}

using ConstraintConstructor = std::function<LogicOpt::Constraint*(const LogicOpt::Proposition&,
                                                                  LogicOpt::World& world,
                                                                  spatial_dyn::ArticulatedBody&, size_t)>;
std::map<std::string, ConstraintConstructor> CreateConstraintFactory(const Eigen::Vector3d& ee_offset) {
  std::map<std::string, ConstraintConstructor> actions;
  actions[""] = [](const LogicOpt::Proposition& action, LogicOpt::World& world,
                   spatial_dyn::ArticulatedBody& ab, size_t t) {
    std::cout << "t = " << t << ": joint(" << ab.q().transpose() << ")" << std::endl;
    return new LogicOpt::JointPositionConstraint(ab, t, ab.q());
  };
  actions["pick"] = [&ee_offset](const LogicOpt::Proposition& action, LogicOpt::World& world,
                                 spatial_dyn::ArticulatedBody& ab, size_t t) {
    std::string object = action.variables()[0]->getName();
    std::cout << "t = " << t << ": pick(" << object << ")" << std::endl;
    return new LogicOpt::JointPickConstraint(world, t, object, ee_offset);
  };
  actions["place"] = [](const LogicOpt::Proposition& action, LogicOpt::World& world,
                        spatial_dyn::ArticulatedBody& ab, size_t t) {
    std::string object = action.variables()[0]->getName();
    std::string target = action.variables()[1]->getName();
    std::cout << "t = " << t << ": place(" << object << ", " << target << ")" << std::endl;
    return new LogicOpt::PlaceOnConstraint(world, t, object, target);
  };
  actions["push"] = [](const LogicOpt::Proposition& action, LogicOpt::World& world,
                       spatial_dyn::ArticulatedBody& ab, size_t t) {
    std::string pusher = action.variables()[0]->getName();
    std::string object = action.variables()[1]->getName();
    std::string surface = action.variables()[2]->getName();
    std::cout << "t = " << t << ": push(" << pusher << ", " << object << ", " << surface << ")" << std::endl;
    return new LogicOpt::PushConstraint(world, t, 10, pusher, object, LogicOpt::PushConstraint::Direction::NEG_X);
  };

  return actions;
};

std::future<Eigen::MatrixXd> AsyncOptimize(const std::vector<LogicOpt::Planner::Node>& plan,
                                           const std::map<std::string, spatial_dyn::RigidBody>& world_objects,
                                           const std::unique_ptr<LogicOpt::Optimizer>& optimizer,
                                           const LogicOpt::Objectives& objectives,
                                           const std::map<std::string, ConstraintConstructor>& constraint_factory,
                                           const spatial_dyn::ArticulatedBody& const_ab) {

  std::function<Eigen::MatrixXd()> optimize = [plan, &world_objects, &optimizer, &objectives,
                                               &constraint_factory, &const_ab]() -> Eigen::MatrixXd {
    spatial_dyn::ArticulatedBody ab = const_ab;

    const size_t T = 10 * plan.size() + 1;
    LogicOpt::World world(ab, world_objects, T);

    LogicOpt::Constraints constraints;
    size_t t = 0;
    for (const LogicOpt::Planner::Node& node : plan) {
      const LogicOpt::Proposition& action = node.action();

      constraints.emplace_back(constraint_factory.at(action.predicate())(action, world, ab, t));
      t += 10;
    }

    // Return to home at end of trajectory
    const LogicOpt::Proposition& home_action = plan.front().action();
    constraints.emplace_back(constraint_factory.at(home_action.predicate())(home_action, world, ab, t));

    world.InitializeConstraintSchedule(constraints);

    LogicOpt::JointVariables variables(ab, T, ab.q());
    Eigen::MatrixXd Q_optimal = optimizer->Trajectory(variables, objectives, constraints);

    g_redis_queue.Emplace(Q_optimal, world);

    return Q_optimal;
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
  spatial_dyn::ArticulatedBody ab = spatial_dyn::Urdf::LoadModel(yaml["robot"]["urdf"].as<std::string>());
  Eigen::VectorXd q_home = yaml["robot"]["q_home"] ? yaml["robot"]["q_home"].as<Eigen::VectorXd>()
                                                   : Eigen::VectorXd::Zero(ab.dof());
  Eigen::Vector3d ee_offset = yaml["robot"]["ee_offset"] ? yaml["robot"]["ee_offset"].as<Eigen::VectorXd>()
                                                         : Eigen::Vector3d::Zero();
  ab.set_q(q_home);

  // Create world objects
  std::map<std::string, spatial_dyn::RigidBody> world_objects = ParseYamlWorldObjects(yaml["world"]["objects"]);

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

  // Create objectives
  LogicOpt::Objectives objectives;
  objectives.emplace_back(new LogicOpt::JointVelocityObjective());

  // Create constraints
  std::map<std::string, ConstraintConstructor> constraint_factory = CreateConstraintFactory(ee_offset);

  // Create redis listener
  std::thread redis_thread(RedisPublishTrajectories, ab, world_objects);

  // Perform search
  std::list<std::future<Eigen::MatrixXd>> optimization_results;
  LogicOpt::BreadthFirstSearch<LogicOpt::Planner::Node> bfs(planner.root(), 5);
  for (const std::vector<LogicOpt::Planner::Node>& plan : bfs) {
    std::future<Eigen::MatrixXd> future_result = AsyncOptimize(plan, world_objects, optimizer,
                                                               objectives, constraint_factory, ab);
    optimization_results.push_back(std::move(future_result));
  }

  // Join threads
  g_redis_queue.Finish();
  redis_thread.join();

  return 0;
}

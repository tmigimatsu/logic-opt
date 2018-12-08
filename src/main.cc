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
#include <iostream>   // std::cout
#include <map>        // std::map
#include <numeric>    // std::accumulate
#include <string>     // std::string
#include <time.h>     // ::gmtime_r, std::strftime
#include <sys/stat.h>  // mkdir

#include <SpatialDyn/SpatialDyn.h>
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

namespace {

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

}  // namespace

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

template<>
struct convert<LogicOpt::Ipopt::Options> {

  static Node encode(const LogicOpt::Ipopt::Options& rhs) {
    Node node;
    node["derivative_test"] = rhs.derivative_test;
    node["use_hessian"] = rhs.use_hessian;
    node["max_cpu_time"] = rhs.max_cpu_time;
    node["max_iter"] = rhs.max_iter;
    node["acceptable_tol"] = rhs.acceptable_tol;
    node["acceptable_iter"] = rhs.acceptable_iter;
    return node;
  }

  static bool decode(const Node& node, LogicOpt::Ipopt::Options& rhs) {
    if (!node.IsMap()) return false;
    if (node["derivative_test"]) rhs.derivative_test = node["derivative_test"].as<bool>();
    if (node["use_hessian"]) rhs.use_hessian = node["use_hessian"].as<bool>();
    if (node["max_cpu_time"]) rhs.max_cpu_time = node["max_cpu_time"].as<double>();
    if (node["max_iter"]) rhs.max_iter = node["max_iter"].as<size_t>();
    if (node["acceptable_tol"]) rhs.acceptable_tol = node["acceptable_tol"].as<double>();
    if (node["acceptable_iter"]) rhs.acceptable_iter = node["acceptable_iter"].as<size_t>();
    return true;
  }

};

}  // namespace YAML

int main(int argc, char *argv[]) {
  // Set up signal handlers
  std::signal(SIGINT, stop);

  Args args = ParseArgs(argc, argv);

  YAML::Node yaml = YAML::LoadFile(args.yaml);
  ValidateYaml(yaml);

  // Load robot
  SpatialDyn::ArticulatedBody ab = SpatialDyn::Urdf::LoadModel(yaml["robot"]["urdf"].as<std::string>());
  Eigen::VectorXd q_home = yaml["robot"]["q_home"] ? yaml["robot"]["q_home"].as<Eigen::VectorXd>()
                                                   : Eigen::VectorXd::Zero(ab.dof());
  Eigen::Vector3d ee_offset = yaml["robot"]["ee_offset"] ? yaml["robot"]["ee_offset"].as<Eigen::VectorXd>()
                                                         : Eigen::Vector3d::Zero();

  // Create world objects
  std::map<std::string, SpatialDyn::RigidBody> world_objects;
  for (const YAML::Node& yaml_object : yaml["world"]["objects"]) {
    SpatialDyn::RigidBody obj(yaml_object["name"].as<std::string>());

    std::string type = yaml_object["type"].as<std::string>();
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);
    if (type == "box") {
      obj.graphics.geometry.type = SpatialDyn::Graphics::Geometry::Type::BOX;
    } else {
      throw std::invalid_argument(args.yaml + ".world.objects[i].type = " + type + " not supported yet.");
    }

    obj.graphics.geometry.scale = yaml_object["scale"].as<Eigen::VectorXd>();

    Eigen::Vector3d pos = yaml_object["pos"].as<Eigen::VectorXd>();
    Eigen::Quaterniond ori = yaml_object["ori"].as<Eigen::Quaterniond>();

    obj.set_T_to_parent(ori, pos);

    world_objects[obj.name] = obj;
  }

  // Set up timer and Redis
  SpatialDyn::RedisClient redis_client;
  redis_client.connect();
  redis_client.set("spatialdyn::kuka_iiwa", SpatialDyn::Json::Serialize(ab).dump());
  for (const std::pair<std::string, SpatialDyn::RigidBody>& key_val : world_objects) {
    const SpatialDyn::RigidBody& object = key_val.second;
    redis_client.set("spatialdyn::objects::" + object.name, SpatialDyn::Json::Serialize(object).dump());
  }

  std::string domain = yaml["planner"]["domain"].as<std::string>();
  std::string problem = yaml["planner"]["problem"].as<std::string>();
  std::unique_ptr<VAL::analysis> analysis = LogicOpt::ParsePddl(domain, problem);

  LogicOpt::Planner planner(analysis->the_domain, analysis->the_problem);
  for (const auto& key_val : planner.objects()) {
    const std::vector<const VAL::parameter_symbol*> objects = key_val.second;
    for (const VAL::parameter_symbol* object : objects) {
      std::string name = object->getName();
      if (world_objects.find(name) == world_objects.end()) {
        throw std::runtime_error("main(): '" + name + "' does not exist in world object store.");
      }
    }
  }

  std::string optimizer = yaml["optimizer"]["engine"].as<std::string>();

  LogicOpt::Objectives objectives;
  objectives.emplace_back(new LogicOpt::JointVelocityObjective());

  LogicOpt::BreadthFirstSearch<LogicOpt::Planner::Node> bfs(planner.root(), 5);
  for (const std::vector<LogicOpt::Planner::Node>& plan : bfs) {
    ab.set_q(q_home);

    const size_t T = 10 * plan.size() + 1;
    LogicOpt::JointVariables variables(ab, T, q_home);
    LogicOpt::World world(ab, world_objects, T);
    std::cout << "T = " << T << std::endl;

    LogicOpt::Constraints constraints;

    size_t t = 0;
    for (const LogicOpt::Planner::Node& node : plan) {
      const LogicOpt::Proposition& action = node.action();

      if (action.predicate() == "") {

        constraints.emplace_back(new LogicOpt::JointPositionConstraint(ab, t, ab.q()));
        std::cout << "t = " << t << ": joint(" << ab.q().transpose() << ")" << std::endl;

        t += 10;

      } else if (action.predicate() == "pick") {

        std::string object = action.variables()[0]->getName();
        constraints.emplace_back(new LogicOpt::PickConstraint(world, t, object, ee_offset));
        std::cout << "t = " << t << ": pick(" << object << ")" << std::endl;

        t += 10;

      } else if (action.predicate() == "place") {

        std::string object = action.variables()[0]->getName();
        std::string target = action.variables()[1]->getName();
        constraints.emplace_back(new LogicOpt::PlaceOnConstraint(world, t, object, target));
        std::cout << "t = " << t << ": place(" << object << ", " << target << ")" << std::endl;

        t += 10;

      } else if (action.predicate() == "push") {

        std::string pusher = action.variables()[0]->getName();
        std::string object = action.variables()[1]->getName();
        std::string surface = action.variables()[2]->getName();
        constraints.emplace_back(new LogicOpt::PushConstraint(world, t, 10, pusher, object, LogicOpt::PushConstraint::Direction::NEG_X));
        std::cout << "t = " << t << ": push(" << pusher << ", " << object << ", " << surface << ")" << std::endl;

        t += 10;

      }
    }

    constraints.emplace_back(new LogicOpt::JointPositionConstraint(ab, t, ab.q()));
    std::cout << "t = " << t << ": joint(" << ab.q().transpose() << ")" << std::endl;

    world.InitializeConstraintSchedule(constraints);

    Eigen::MatrixXd Q_optimal;
    if (optimizer == "nlopt") {
      LogicOpt::Nlopt::OptimizationData data;
      Q_optimal = LogicOpt::Nlopt::Trajectory(variables, objectives, constraints, &data);
    } else if (optimizer == "ipopt") {
      LogicOpt::Ipopt::OptimizationData data;
      LogicOpt::Ipopt::Options options = yaml["optimizer"]["ipopt"].as<LogicOpt::Ipopt::Options>();
      Q_optimal = LogicOpt::Ipopt::Trajectory(variables, objectives, constraints, &data);
    }

    std::cout << Q_optimal.transpose() << std::endl << std::endl;
    world.Simulate(Q_optimal);

    SpatialDyn::Timer timer(1000);

    std::map<std::string, SpatialDyn::RigidBody> simulation_objects = world_objects;

    size_t idx_trajectory = 0;
    while (g_runloop) {
      timer.Sleep();
      Eigen::VectorXd q_err = ab.q() - Q_optimal.col(idx_trajectory);
      Eigen::VectorXd dq_err = ab.dq();
      Eigen::VectorXd ddq = -10. * q_err - 6. * dq_err;
      Eigen::VectorXd tau = SpatialDyn::InverseDynamics(ab, ddq, {}, true, true);

      SpatialDyn::Integrate(ab, tau, timer.dt());

      std::map<std::string, LogicOpt::World::ObjectState> world_state_t =
          world.InterpolateSimulation(ab.q(), std::max(static_cast<int>(idx_trajectory) - 1, 0));

      for (auto& key_val : world_state_t) {
        const std::string& name_object = key_val.first;
        const LogicOpt::World::ObjectState object_state_t = key_val.second;
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
  }

  return 0;
}

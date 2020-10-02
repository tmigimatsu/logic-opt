/**
 * main.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 19, 2018
 * Authors: Toki Migimatsu
 */

#include <ctrl_utils/eigen.h>
#include <ncollide_cpp/ncollide3d.h>
#include <spatial_dyn/spatial_dyn.h>
#include <spatial_opt/optimizers/ipopt.h>
#include <spatial_opt/optimizers/nlopt.h>
#include <spatial_opt/spatial_opt.h>
#include <spatial_opt/variables/frame_variables.h>
#include <sys/stat.h>  // mkdir
#include <time.h>      // ::gmtime_r, std::strftime

#include <algorithm>  // std::max
#include <chrono>     // std::chrono
#include <cmath>      // M_PI
#include <csignal>    // std::signal
#include <fstream>    // std::ofstream
#include <iostream>   // std::cout
#include <map>        // std::map
#include <string>     // std::string
#include <thread>     // std::thread

#include "logic_opt/constraints/cartesian_pose_constraint.h"
#include "logic_opt/constraints/pick_constraint.h"
#include "logic_opt/constraints/place_constraint.h"
#include "logic_opt/constraints/push_constraint.h"
#include "logic_opt/control/opspace_controller.h"
#include "logic_opt/objectives/angular_distance_objective.h"
#include "logic_opt/objectives/linear_distance_objective.h"
#include "logic_opt/world.h"

namespace Eigen {

using Vector7d = Eigen::Matrix<double, 7, 1>;

}  // namespace Eigen

namespace {

using Isometry = ::spatial_opt::Isometry;
using Frame = ::logic_opt::Frame;
using Object = ::logic_opt::Object;
using World = ::logic_opt::World;

volatile std::sig_atomic_t g_runloop = true;
void stop(int) { g_runloop = false; }

struct Args {
  enum class Optimizer { NLOPT, IPOPT };
  enum class Task { JOINT_POSITION, CARTESIAN_POSE };

  Optimizer optimizer = Optimizer::IPOPT;
  Task task = Task::CARTESIAN_POSE;
  bool with_scalar_constraints = false;
  bool with_hessian = false;
  bool derivative_test = false;
  bool valgrind = false;
  std::string logdir;

  static Args Parse(int argc, char* argv[]);
};

// Robot
const std::string kNameRobot = "franka_panda";
const std::string kPathUrdf =
    "../resources/" + kNameRobot + "/" + kNameRobot + ".urdf";

// Controller parameters
const Eigen::Vector7d kQHome = (Eigen::Vector7d() << 0., -M_PI / 6., 0.,
                                -5. * M_PI / 6., 0., 2. * M_PI / 3., 0.)
                                   .finished();
const Eigen::Vector3d kEeOffset =
    Eigen::Vector3d(0., 0., 0.107);  // Without gripper
const Eigen::Vector3d kFrankaGripperOffset = Eigen::Vector3d(0., 0., 0.1034);
const Eigen::Vector3d kRobotiqGripperOffset =
    Eigen::Vector3d(0., 0., 0.140);  // Ranges from 0.130 to 0.144

const Eigen::Vector3d ee_offset = kEeOffset + kRobotiqGripperOffset;

const std::string kEeFrame = "ee";

}  // namespace

std::string CreateLogDirectory(const Args& args) {
  std::string logdir;
  if (!args.logdir.empty()) {
    logdir = args.logdir + "/";
    // logdir += (args.optimizer == Args::Optimizer::NLOPT) ? "nlopt" : "ipopt";
    // logdir += (args.task == Args::Task::PICK_PLACE) ? "_pickplace" : "";
    // logdir += args.with_scalar_constraints ? "_scalar" : "";
    // logdir += args.with_hessian ? "_hessian" : "";
    // logdir += "/";
    mkdir(logdir.c_str(),
          S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
    std::cout << "Log output: " << logdir << std::endl;
  }
  return logdir;
}

std::shared_ptr<std::map<std::string, Object>> CreateObjects(
    const spatial_dyn::ArticulatedBody& ab) {
  // Create world objects
  auto world_objects = std::make_shared<std::map<std::string, Object>>();
  // // {
  // //   spatial_dyn::RigidBody base("base");
  // //   spatial_dyn::Graphics graphics;
  // //   graphics.geometry.type =
  // spatial_dyn::Graphics::Geometry::Type::kCapsule;
  // //   graphics.geometry.radius = 0.2;
  // //   graphics.geometry.length = 0.3;
  // //   graphics.material.rgba(3) = 0.2;
  // //   graphics.T_to_parent = Eigen::Translation3d(0., 0., 0.15) *
  // //                          Eigen::AngleAxisd(M_PI / 2.,
  // //                          Eigen::Vector3d::UnitX());
  // //   base.graphics.push_back(std::move(graphics));

  //   world_objects->emplace(std::string(base.name), std::move(base));
  // }
  {
    spatial_dyn::RigidBody wall("wall");
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    graphics.geometry.scale = Eigen::Vector3d(2., 1., 1.);
    graphics.material.rgba(3) = 0.;
    graphics.T_to_parent = Eigen::Translation3d(0., -1., 0.);
    wall.graphics.push_back(std::move(graphics));

    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    graphics.geometry.scale = Eigen::Vector3d(2., 1., 1.);
    graphics.material.rgba(3) = 0.;
    graphics.T_to_parent = Eigen::Translation3d(0., 1., 0.);
    wall.graphics.push_back(std::move(graphics));

    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    graphics.geometry.scale = Eigen::Vector3d(2., 1., 1.);
    graphics.material.rgba(3) = 0.;
    graphics.T_to_parent = Eigen::Translation3d(-1.3, 0., 0.);
    wall.graphics.push_back(std::move(graphics));

    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    graphics.geometry.scale = Eigen::Vector3d(1., 1., 1.);
    graphics.material.rgba(3) = 0.;
    graphics.T_to_parent = Eigen::Translation3d(1., 0., 0.);
    wall.graphics.push_back(std::move(graphics));

    wall.set_T_to_parent(Eigen::Quaterniond::Identity(),
                         Eigen::Vector3d(0.5, 0., 0.));
    world_objects->emplace(std::string(wall.name), std::move(wall));
  }
  {
    spatial_dyn::RigidBody table("table");
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    graphics.geometry.scale = Eigen::Vector3d(2., 1., 0.5);
    graphics.material.rgba(3) = 0.5;
    // graphics.T_to_parent = Eigen::Translation3d(0., 0., -0.25);

    // table.collision =
    // std::make_unique<ncollide3d::shape::Cuboid>(graphics.geometry.scale / 2);
    table.graphics.push_back(std::move(graphics));
    table.set_T_to_parent(Eigen::Quaterniond::Identity(),
                          Eigen::Vector3d(0.5, 0., -0.25));
    world_objects->emplace(std::string(table.name), std::move(table));
  }
  {
    spatial_dyn::RigidBody shelf("shelf");
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    graphics.geometry.scale = Eigen::Vector3d(0.40, 0.32, 0.09);
    // graphics.T_to_parent = Eigen::Translation3d(0., 0., -0.05);
    // shelf.collision =
    // std::make_unique<ncollide3d::shape::Cuboid>(graphics.geometry.scale / 2);
    shelf.graphics.push_back(std::move(graphics));
    shelf.set_T_to_parent(Eigen::Quaterniond::Identity(),
                          Eigen::Vector3d(0.35, -0.35, 0.05));
    world_objects->emplace(std::string(shelf.name), std::move(shelf));
  }
  {
    spatial_dyn::RigidBody box("box_2");
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    graphics.geometry.scale = Eigen::Vector3d(0.06, 0.06, 0.06);
    // graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kSphere;
    // graphics.geometry.radius = 0.025;
    // box.collision =
    // std::make_unique<ncollide3d::shape::Cuboid>(graphics.geometry.scale / 2);
    box.graphics.push_back(std::move(graphics));
    // box.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.,
    // -1.0, 0.4));
    box.set_T_to_parent(Eigen::Quaterniond::Identity(),
                        Eigen::Vector3d(0.9, 0., 0.025));
    world_objects->emplace(std::string(box.name), std::move(box));
  }
  {
    spatial_dyn::RigidBody hook("hook");
    spatial_dyn::Graphics graphics;
    // graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    // graphics.geometry.scale = Eigen::Vector3d(0.04, 0.2, 0.04);
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kCapsule;
    const double kLengthVertical = 0.3;     // 0.25;
    const double kLengthHorizontal = 0.15;  // 0.1;
    const double kBoxWidth = 0.05;
    const double kRadius = 0.02;  // 0.01;
    graphics.geometry.radius = kRadius;
    graphics.geometry.length = kLengthVertical;
    graphics.T_to_parent =
        Eigen::Translation3d(-kLengthVertical / 2. + kRadius + kBoxWidth / 2.,
                             kLengthHorizontal / 2., 0.) *
        Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitZ());
    // box.collision =
    // std::make_unique<ncollide3d::shape::Cuboid>(graphics.geometry.scale / 2);
    hook.graphics.push_back(std::move(graphics));

    // graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    // graphics.geometry.scale = Eigen::Vector3d(0.04, 0.1, 0.04);
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kCapsule;
    graphics.geometry.radius = kRadius;
    graphics.geometry.length = kLengthHorizontal;
    graphics.T_to_parent =
        Eigen::Translation3d(kRadius + kBoxWidth / 2., 0., 0.);
    hook.graphics.push_back(std::move(graphics));
    Eigen::Quaterniond quat(0.88, 0, 0, 0.48);
    // hook.set_T_to_parent(quat.normalized(),
    //                      Eigen::Vector3d(0.4, 0.3, 0.005));
    hook.set_T_to_parent(Eigen::Quaterniond::Identity(),
                         Eigen::Vector3d(0.9, 0.3, 0.005));
    world_objects->emplace(std::string(hook.name), std::move(hook));
  }
  {
    spatial_dyn::RigidBody ee(kEeFrame);
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kCapsule;
    graphics.geometry.radius = 0.05;
    graphics.geometry.length = 0.1;
    graphics.material.rgba(3) = 0.2;
    graphics.T_to_parent =
        Eigen::Translation3d(0., 0., 0.14) *
        Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX());
    ee.graphics.push_back(std::move(graphics));

    // Fingers
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kCapsule;
    graphics.geometry.radius = 0.02;
    graphics.geometry.length = 0.08;
    graphics.T_to_parent =
        Eigen::Translation3d(0., 0.06, 0.05) *
        Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX());
    ee.graphics.push_back(graphics);
    graphics.T_to_parent =
        Eigen::Translation3d(0., -0.06, 0.05) *
        Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX());
    ee.graphics.push_back(graphics);
    // graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kSphere;
    // graphics.geometry.radius = 0.01;
    // graphics.T_to_parent = Eigen::Translation3d(0., 0., 0.1);
    // ee.graphics.push_back(graphics);

    // Set identity orientation to home position
    ee.set_T_to_parent(Eigen::Quaterniond::Identity(),
                       spatial_dyn::Position(ab, -1, ee_offset));
    world_objects->emplace(std::string(ee.name), std::move(ee));
  }
  return world_objects;
}

spatial_opt::Objectives CreateObjectives(World& world) {
  spatial_opt::Objectives objectives;
  // objectives.emplace_back(new spatial_opt::MinL2NormObjective(3, 3));

  objectives.emplace_back(
      new logic_opt::LinearDistanceObjective(world, kEeFrame));

  objectives.emplace_back(
      new logic_opt::AngularDistanceObjective(world, kEeFrame, 3.));

  // objectives.emplace_back(new spatial_opt::WorkspaceObjective(world,
  // kEeFrame));

  return objectives;
}

using ConstraintConstructor = std::function<spatial_opt::Constraint*(
    World& world, size_t t, size_t t_start)>;

std::vector<ConstraintConstructor> DefineConstraints(
    const spatial_dyn::ArticulatedBody& ab) {
  return {
      [](World& world, size_t t, size_t t_start) -> spatial_opt::Constraint* {
        return new logic_opt::PickConstraint(world, t, kEeFrame, "hook");
      },
      [](World& world, size_t t, size_t t_start) -> spatial_opt::Constraint* {
        return new logic_opt::PushConstraint(world, t, "hook", "box_2", "table",
                                             t_start);
      },
      [](World& world, size_t t, size_t t_start) -> spatial_opt::Constraint* {
        return new logic_opt::PlaceConstraint(world, t, "hook", "shelf");
      },
      [](World& world, size_t t, size_t t_start) -> spatial_opt::Constraint* {
        return new logic_opt::PickConstraint(world, t, kEeFrame, "box_2");
      },
      [](World& world, size_t t, size_t t_start) -> spatial_opt::Constraint* {
        return new logic_opt::PlaceConstraint(world, t, "box_2", "shelf");
      },
      [pos = spatial_dyn::Position(ab, -1, ee_offset)](
          World& world, size_t t, size_t t_start) -> spatial_opt::Constraint* {
        return new logic_opt::CartesianPoseConstraint(world, t, kEeFrame,
                                                      world.kWorldFrame, pos);
      }};
}

spatial_opt::Constraints CreateConstraints(
    World& world, const spatial_dyn::ArticulatedBody& ab,
    const std::vector<ConstraintConstructor>& constraint_functions,
    std::vector<size_t>* t_constraints) {
  size_t t = 0;

  // Vector of constraint timesteps, starting at 0 and ending at T.
  t_constraints->clear();
  t_constraints->reserve(constraint_functions.size() + 1);
  // t_constraints->push_back(t);

  spatial_opt::Constraints constraints;

  // Add home position of end-effector as first constraint
  constraints.emplace_back(new logic_opt::CartesianPoseConstraint(
      world, t, kEeFrame, world.kWorldFrame,
      spatial_dyn::Position(ab, -1, ee_offset),
      Eigen::Quaterniond::Identity()));
  t += constraints.back()->num_timesteps();
  t_constraints->push_back(t);

  // Add remaining constraints
  for (const auto& Constraint : constraint_functions) {
    constraints.emplace_back(Constraint(world, t, 0));
    t += constraints.back()->num_timesteps();
    t_constraints->push_back(t);
  }

  return constraints;
}

spatial_opt::Constraints CreateConstraints(
    World& world, const spatial_dyn::ArticulatedBody& ab,
    const std::vector<ConstraintConstructor>& constraint_functions,
    const std::vector<size_t> t_constraints, const World& world_0,
    size_t t_action) {
  // Factory function to add constraints starting from t_action. The given world
  // should already be initialized with the proper kinematic tree and updated
  // object poses.

  // Initialize kinematic tree to tree in world_0 at t_action
  // world.ReserveTimesteps(1);
  // world.InitializeTree(world_0.frames(t_action));
  // world.set_T_to_world()...

  // Find constraint at t_action
  size_t idx_constraint;  // Index of constraint at t_action
  size_t t_start;         // Timestep to start constraint at t_action
  for (idx_constraint = 0; idx_constraint < constraint_functions.size();
       idx_constraint++) {
    const size_t t_constraint_start = t_constraints[idx_constraint];
    const size_t t_constraint_end = t_constraints[idx_constraint + 1];
    if (t_constraint_end > t_action) {
      // t_constraints starts at t=1, since the initial pose constraint is not
      // counted.
      t_start =
          (t_constraint_start < t_action) ? t_action - t_constraint_start : 0;
      break;
    }
  }

  size_t t = 0;
  spatial_opt::Constraints constraints;

  // Add current pose as first constraint
  {
    const std::string& control = world_0.control_frame(t_action);
    const std::string& target = world_0.target_frame(t_action);
    const Isometry& T_control_to_target =
        world.objects()->at(control).T_to_parent();
    constraints.emplace_back(new logic_opt::CartesianPoseConstraint(
        world, t, control, target, T_control_to_target));
    t += constraints.back()->num_timesteps();
  }

  // Add remaining constraints
  for (; idx_constraint < constraint_functions.size(); idx_constraint++) {
    const auto& Constraint = constraint_functions[idx_constraint];
    constraints.emplace_back(Constraint(world, t, t_start));
    t += constraints.back()->num_timesteps();
    t_start = 0;
  }

  const size_t T = world.num_timesteps();
  if (t != T) throw std::runtime_error("Constraint timesteps must equal T.");

  return constraints;
}

Eigen::MatrixXd Optimize(const spatial_opt::Variables& variables,
                         const spatial_opt::Objectives& objectives,
                         const spatial_opt::Constraints& constraints,
                         const Args& args, std::string* status = nullptr) {
  if (args.optimizer == Args::Optimizer::NLOPT) {
    spatial_opt::Nlopt::Options options = {CreateLogDirectory(args)};
    spatial_opt::Nlopt nlopt(options);
    spatial_opt::Nlopt::OptimizationData data;
    return nlopt.Trajectory(variables, objectives, constraints, &data);
  } else {
    spatial_opt::Ipopt::Options options;
    options.derivative_test = args.derivative_test;
    options.use_hessian = args.with_hessian;
    options.logdir = CreateLogDirectory(args);
    options.max_cpu_time = 10;
    options.print_level = 0;
    // options.max_iter = 1e6;
    // options.max_cpu_time = 1e3;
    spatial_opt::Ipopt ipopt(options, &g_runloop);
    spatial_opt::Ipopt::OptimizationData data;
    if (args.valgrind) {
      return Eigen::MatrixXd(variables.dof, variables.T);
    } else {
      Eigen::MatrixXd X_optimal =
          ipopt.Trajectory(variables, objectives, constraints, &data);
      if (status != nullptr) {
        *status = ipopt.status();
      }
      return std::move(X_optimal);
    }
  }
}

int main(int argc, char* argv[]) {
  Args args = Args::Parse(argc, argv);

  std::signal(SIGINT, stop);

  // Initialize robot.
  spatial_dyn::ArticulatedBody ab =
      spatial_dyn::urdf::LoadModel(kPathUrdf, kNameRobot);
  ab.set_q(kQHome);

  // Create initial world.
  World world_0(CreateObjects(ab));

  // Create constraints and set up kinematic tree in initial world.
  const std::vector<ConstraintConstructor> constraint_functions =
      DefineConstraints(ab);
  std::vector<size_t> t_constraints;
  CreateConstraints(world_0, ab, constraint_functions, &t_constraints);

  // Create shared memory with controller.
  const auto shared_memory =
      std::make_shared<logic_opt::PlannerControllerInterface>(g_runloop);
  shared_memory->SetExecutionUpdate(*world_0.objects(), world_0.frames(0), 0);

  // Start controller thread.
  std::thread thread_controller =
      std::thread(logic_opt::OpspaceController, std::cref(*world_0.objects()),
                  std::cref(ab), shared_memory);

  // Initialize variables.
  Eigen::MatrixXd X_0(World::kDof, world_0.num_timesteps());
  X_0.row(World::kDof - 1).setOnes();

  // Run trajectory optimization loop.
  while (shared_memory->g_runloop) {
    // Get updated world object poses and controller timestep.
    logic_opt::PlannerControllerInterface::ExecutionUpdate exec_update =
        shared_memory->GetExecutionUpdate();

    // Clear variable indices.
    for (auto& key_val : exec_update.tree.nodes()) {
      Frame& frame = exec_update.tree.at(key_val.first);
      frame.set_idx_var();
    }

#ifdef VERBOSE
    std::cout << "t_action: " << exec_update.t_action << std::endl;
    std::cout << "world_poses:" << std::endl;
    exec_update.tree.printf(
        std::cout,
        [&world_poses = exec_update.world_poses](
            const std::string& key, const Frame& frame) -> std::string {
          std::stringstream ss;
          if (frame.is_variable()) {
            ss << "idx_var: " << frame.idx_var() << "\t";
          }
          if (world_poses.find(key) != world_poses.end()) {
            ss << world_poses.at(key).T_to_parent();
          }
          return ss.str();
        });
#endif  // VERBOSE

    // Convert world poses to relative poses.
    auto world_objects = std::make_shared<std::map<std::string, Object>>(
        exec_update.world_poses);
    World::set_T_to_world(exec_update.world_poses, exec_update.tree,
                          world_objects.get(),
                          /*freeze_children_in_world=*/false);

    // Create new trajectory optimization.
    World world(world_objects);
    world.InitializeTree(exec_update.tree);
    const spatial_opt::Objectives objectives = CreateObjectives(world);
    const spatial_opt::Constraints constraints =
        CreateConstraints(world, ab, constraint_functions, t_constraints,
                          world_0, exec_update.t_action);

    const size_t T = world.num_timesteps();
    spatial_opt::FrameVariables variables(T);

    variables.X_0 = X_0.rightCols(world.num_timesteps());
    const Isometry& T_ee_to_world = world_objects->at(kEeFrame).T_to_parent();
    variables.X_0.topLeftCorner<3, 1>() = T_ee_to_world.translation();
    variables.X_0.bottomLeftCorner<4, 1>() = T_ee_to_world.rotation().coeffs();

    // Optimize.
    std::cout << "X_0:" << std::endl << variables.X_0 << std::endl;
    const auto t_start = std::chrono::high_resolution_clock::now();
    std::string status;
    Eigen::MatrixXd X_optimal =
        Optimize(variables, objectives, constraints, args, &status);
    const auto t_end = std::chrono::high_resolution_clock::now();
    X_0 = X_optimal;
    const std::chrono::duration<double> dt = t_end - t_start;

    // Print stats.
    std::cout << status << std::endl;
    std::cout << "===================" << std::endl;
    std::cout << "Solved in " << dt.count() << " seconds." << std::endl;
    std::cout << "X_optimal: " << std::endl
              << X_optimal << std::endl
              << std::endl;

    // Publish updated result
    if (status == "success") {
      shared_memory->SetOptimizationResult(
          std::move(X_optimal), std::move(world), exec_update.t_action);
    }
  }

  if (thread_controller.joinable()) {
    thread_controller.join();
  }
}

Args Args::Parse(int argc, char* argv[]) {
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
    } else if (arg == "--derivative-test") {
      parsed_args.derivative_test = true;
    } else if (arg == "--valgrind") {
      parsed_args.valgrind = true;
    } else {
      break;
    }
  }

  if (i != argc)
    throw std::invalid_argument("ParseArgs(): Invalid '" + arg + "' argument.");
  return parsed_args;
}

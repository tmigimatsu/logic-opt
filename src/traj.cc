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
#include <ncollide_cpp/ncollide3d.h>

// #include "gurobi.h"
#include "logic_opt/control/opspace_controller.h"
#include "logic_opt/optimization/constraints.h"
#include "logic_opt/optimization/ipopt.h"
#include "logic_opt/optimization/nlopt.h"
#include "logic_opt/world.h"

namespace Eigen {

using Vector7d = Eigen::Matrix<double,7,1>;

}  // namespace Eigen

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
  bool derivative_test = false;
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
    } else if (arg == "--derivative-test") {
      parsed_args.derivative_test = true;
    } else {
      break;
    }
  }

  if (i != argc) throw std::invalid_argument("ParseArgs(): Invalid '" + arg + "' argument.");
  return parsed_args;
}

// Robot
const std::string kNameRobot = "franka_panda";
const std::string kPathUrdf = "../resources/" + kNameRobot + "/" + kNameRobot + ".urdf";

// Controller parameters
const Eigen::Vector7d kQHome     = (Eigen::Vector7d() <<
                                    0., -M_PI/6., 0., -5.*M_PI/6., 0., 2.*M_PI/3., 0.).finished();
const Eigen::Vector3d kEeOffset  = Eigen::Vector3d(0., 0., 0.107);  // Without gripper
const Eigen::Vector3d kFrankaGripperOffset  = Eigen::Vector3d(0., 0., 0.1034);
const Eigen::Vector3d kRobotiqGripperOffset = Eigen::Vector3d(0., 0., 0.140);  // Ranges from 0.130 to 0.144

const std::string kEeFrame = "ee";

}  // namespace

int main(int argc, char *argv[]) {
  // {
  //   const ncollide3d::shape::Capsule g1(0.1, 0.025);
  //   const Eigen::ArrayXd X = Eigen::ArrayXd::LinSpaced(10, -0.05, 0.05);
  //   const Eigen::ArrayXd Y = Eigen::ArrayXd::LinSpaced(10, -0.2, 0.2);
  //   const Eigen::ArrayXd Z = Eigen::ArrayXd::LinSpaced(10, -0.05, 0.05);
  //   for (size_t i = 0; i < X.size(); i++) {
  //     for (size_t j = 0; j < Y.size(); j++) {
  //       for (size_t k = 0; k < Z.size(); k++) {
  //         const Eigen::Vector3d dir(X(i), Y(j), Z(k));
  //         const ncollide3d::query::Ray r(Eigen::Vector3d::Zero(), dir.normalized());
  //         const double toi = g1.toi_with_ray(Eigen::Isometry3d::Identity(), r, false);
  //       }
  //     }
  //   }

  // }
  // {
  //   ncollide3d::shape::Cuboid g1(0.025, 0.025, 0.025);
  //   std::map<std::string, ncollide3d::shape::TriMesh> collisions;
  //   collisions.emplace("a", g1.to_trimesh());
  //   const ncollide3d::shape::TriMesh& trimesh = collisions.at("a");
  //   // ncollide3d::shape::TriMesh trimesh = g1.to_trimesh();
  //   ncollide3d::shape::ShapeVector hook;
  //   std::vector<std::array<double, 3>> points;
  //   for (size_t i = 0; i < trimesh.num_points(); i++) {
  //     const auto point = trimesh.point(i);
  //     points.push_back({point(0), point(1), point(2)});
  //     points.push_back({point(0) + 0.1, point(1) + 0.1, point(2) + 0.1});
  //   }
  //   ncollide3d::shape::TriMesh convex_hull = ncollide3d::transformation::convex_hull(points);
  //   for (size_t i = 0; i < convex_hull.num_points(); i++) {
  //     const auto point = convex_hull.point(i);
  //     std::cout << point.transpose() << std::endl;
  //   }
  //   // hook.emplace_back(Eigen::Isometry3d(Eigen::Translation3d(0.0317, -0.0183, 0.) *
  //   //                   Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitZ())),
  //   //                   std::make_unique<ncollide3d::shape::Capsule>(0.1, 0.01));
  //   // hook.emplace_back(Eigen::Isometry3d(Eigen::Translation3d(-0.0633, 0.0367, 0.)),
  //   //                   std::make_unique<ncollide3d::shape::Cuboid>(0.005, 0.05, 0.005));
  //   // ncollide3d::shape::Compound g2(std::move(hook));

  //   // Eigen::Isometry3d m2;
  //   // m2.matrix() << 0.7774206427904, -0.541897597154303, 0.319318239945283, -0.00371361443282269,
  //   //                0.319318239944298, 0.7774206427904, 0.541897597154884, 0.0405069911519239,
  //   //                -0.541897597154884, -0.319318239945285, 0.777420642789995, 0.0251000099226302,
  //   //                0, 0, 0, 1;
  //   // ncollide3d::query::contact(Eigen::Isometry3d::Identity(), g1, m2, g2, 100.);
  //   return 0;
  // }

  // {
  //   ncollide3d::shape::Cuboid g1(1, 1, 1);
  //   ncollide3d::shape::ShapeVector shapes;
  //   // shapes.emplace_back()
  //   ncollide3d::shape::Capsule g2(0.2, 0.05);
  //   std::cout << g1.half_extents().transpose() << std::endl;
  //   std::cout << g2.half_height() << " " << g2.radius() << std::endl;

  //   Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
  //   Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity() * Eigen::Translation3d(2.5, 0, 0);
  //   std::cout << "A" << std::endl;
  //   std::cout << ncollide3d::query::distance(T1, g1, T2, g2) << std::endl;

  //   auto contact = ncollide3d::query::contact(T1, g1, T2, g2, 0.5);
  //   if (contact) {
  //     std::cout << contact->depth << std::endl;
  //   }

  //   std::cout << "B" << std::endl;
  //   // ncollide3d::shape::Compound c({{T1, g1}, {T2, g2}});
  //   std::cout << "C" << std::endl;
  //   auto points = ncollide3d::query::closest_points(T1, g1, T2, g2, 2.0);
  //   std::cout << (points.status == ncollide3d::query::ClosestPoints::Status::Disjoint ? "disjoint" : (points.status == ncollide3d::query::ClosestPoints::Status::Intersecting ? "intersecting" : "disjoint")) << std::endl;
  //   std::cout << points.point1.transpose() << "; " << points.point2.transpose() << std::endl;

  //   ncollide3d::query::Ray ray(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
  //   std::optional<double> toi = g1.toi_with_ray(Eigen::Isometry3d::Identity(), ray, true);
  //   if (toi) {
  //     std::cout << *toi << std::endl;
  //   } else {
  //     std::cout << "nothing" << std::endl;
  //   }
  // }
  // return 0;
  Args args = ParseArgs(argc, argv);

  // Initialize robot
  spatial_dyn::ArticulatedBody ab = spatial_dyn::urdf::LoadModel(kPathUrdf, kNameRobot);
  ab.set_q(kQHome);

  // Create world objects
  auto world_objects = std::make_shared<std::map<std::string, logic_opt::Object3>>();
  // {
  //   spatial_dyn::RigidBody base("base");
  //   spatial_dyn::Graphics graphics;
  //   graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kCapsule;
  //   graphics.geometry.radius = 0.2;
  //   graphics.geometry.length = 0.3;
  //   graphics.material.rgba(3) = 0.2;
  //   graphics.T_to_parent = Eigen::Translation3d(0., 0., 0.15) *
  //                          Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX());
  //   base.graphics.push_back(std::move(graphics));

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

    wall.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.5, 0., 0.));
    world_objects->emplace(std::string(wall.name), std::move(wall));
  }
  {
    spatial_dyn::RigidBody table("table");
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    graphics.geometry.scale = Eigen::Vector3d(2., 1., 0.5);
    graphics.material.rgba(3) = 0.5;
    // graphics.T_to_parent = Eigen::Translation3d(0., 0., -0.25);

    // table.collision = std::make_unique<ncollide3d::shape::Cuboid>(graphics.geometry.scale / 2);
    table.graphics.push_back(std::move(graphics));
    table.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.5, 0., -0.25));
    world_objects->emplace(std::string(table.name), std::move(table));
  }
  {
    spatial_dyn::RigidBody shelf("shelf");
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    graphics.geometry.scale = Eigen::Vector3d(0.34, 0.26, 0.08);
    // graphics.T_to_parent = Eigen::Translation3d(0., 0., -0.05);
    // shelf.collision = std::make_unique<ncollide3d::shape::Cuboid>(graphics.geometry.scale / 2);
    shelf.graphics.push_back(std::move(graphics));
    shelf.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.35, -0.35, 0.05));
    world_objects->emplace(std::string(shelf.name), std::move(shelf));
  }
  {
    spatial_dyn::RigidBody box("box_3");
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    graphics.geometry.scale = Eigen::Vector3d(0.05, 0.05, 0.05);
    // graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kSphere;
    // graphics.geometry.radius = 0.025;
    // box.collision = std::make_unique<ncollide3d::shape::Cuboid>(graphics.geometry.scale / 2);
    box.graphics.push_back(std::move(graphics));
    // box.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0., -1.0, 0.4));
    box.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.9, 0., 0.025));
    world_objects->emplace(std::string(box.name), std::move(box));
  }
  {
    spatial_dyn::RigidBody hook("hook");
    spatial_dyn::Graphics graphics;
    // graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    // graphics.geometry.scale = Eigen::Vector3d(0.04, 0.2, 0.04);
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kCapsule;
    const double kLengthVertical = 0.3; // 0.25;
    const double kLengthHorizontal = 0.15; // 0.1;
    const double kBoxWidth = 0.05;
    const double kRadius = 0.02; // 0.01;
    graphics.geometry.radius = kRadius;
    graphics.geometry.length = kLengthVertical;
    graphics.T_to_parent = Eigen::Translation3d(-kLengthVertical / 2. + kRadius + kBoxWidth / 2., kLengthHorizontal / 2., 0.) *
                           Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitZ());
    // box.collision = std::make_unique<ncollide3d::shape::Cuboid>(graphics.geometry.scale / 2);
    hook.graphics.push_back(std::move(graphics));

    // graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kBox;
    // graphics.geometry.scale = Eigen::Vector3d(0.04, 0.1, 0.04);
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kCapsule;
    graphics.geometry.radius = kRadius;
    graphics.geometry.length = kLengthHorizontal;
    graphics.T_to_parent = Eigen::Translation3d(kRadius + kBoxWidth / 2., 0., 0.);
    hook.graphics.push_back(std::move(graphics));

    hook.set_T_to_parent(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.9, 0.3, 0.005));
    world_objects->emplace(std::string(hook.name), std::move(hook));
  }
  {
    spatial_dyn::RigidBody ee(kEeFrame);
    spatial_dyn::Graphics graphics;
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kCapsule;
    graphics.geometry.radius = 0.05;
    graphics.geometry.length = 0.1;
    graphics.material.rgba(3) = 0.2;
    graphics.T_to_parent = Eigen::Translation3d(0., 0., 0.14) *
                           Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX());
    ee.graphics.push_back(std::move(graphics));

    // Fingers
    graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kCapsule;
    graphics.geometry.radius = 0.02;
    graphics.geometry.length = 0.08;
    graphics.T_to_parent = Eigen::Translation3d(0., 0.06, 0.05) *
                           Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX());
    ee.graphics.push_back(graphics);
    graphics.T_to_parent = Eigen::Translation3d(0., -0.06, 0.05) *
                           Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX());
    ee.graphics.push_back(graphics);
    // graphics.geometry.type = spatial_dyn::Graphics::Geometry::Type::kSphere;
    // graphics.geometry.radius = 0.01;
    // graphics.T_to_parent = Eigen::Translation3d(0., 0., 0.1);
    // ee.graphics.push_back(graphics);

    // Set identity orientation to home position
    // ee.set_T_to_parent(spatial_dyn::Orientation(ab).inverse(), kEeOffset + kRobotiqGripperOffset);
    world_objects->emplace(std::string(ee.name), std::move(ee));
  }

  // Create signal handler
  std::signal(SIGINT, stop);

  // End-effector parameters
  // const Eigen::Isometry3d& T_ee = world_objects->at(kEeFrame).T_to_parent();
  // Eigen::Ref<const Eigen::Vector3d> ee_offset = T_ee.translation();
  const Eigen::Vector3d ee_offset = kEeOffset + kRobotiqGripperOffset;

  logic_opt::World3 world(world_objects);
  // {
  //   world.ReserveTimesteps(1);
  //   std::string control_frame = world.control_frame(0);
  //   std::string target_frame = world.target_frame(0);
  //   world.AttachFrame("box", "table", 0);
  //   world.AttachFrame("hook", "table", 0);
  //   world.set_controller_frames(control_frame, target_frame, 0);
  // }

  logic_opt::Objectives objectives;
  // objectives.emplace_back(new logic_opt::MinL2NormObjective(3, 3));
  objectives.emplace_back(new logic_opt::LinearVelocityObjective3(world, kEeFrame));
  objectives.emplace_back(new logic_opt::AngularVelocityObjective(world, kEeFrame, 2.));
  // objectives.emplace_back(new logic_opt::WorkspaceObjective(world, kEeFrame));

  // Set up task constraints
  logic_opt::Constraints constraints;

  size_t t = 0;

  // constraints.emplace_back(new logic_opt::CartesianPoseConstraint<3>(
  //     // world, t, "box", world.kWorldFrame, Eigen::Vector3d(0.25, -0.5, 0.35),
  //     world, t, "box", "shelf", Eigen::Vector3d(0., 0., -0.05),
  //     spatial_dyn::Orientation(ab) * quat_ee));
  // t += constraints.back()->num_timesteps();

  // for (size_t t_end = t + 11; t < t_end; t += constraints.back()->num_timesteps()) {
  //   constraints.emplace_back(new logic_opt::TrajectoryConstraint(world, t));
  // }

  // constraints.emplace_back(new logic_opt::CartesianPoseConstraint<3>(
  //     // world, t, "box", world.kWorldFrame, Eigen::Vector3d(0.25, -0.5, 0.45),
  //     world, t, "box", "shelf", Eigen::Vector3d(0., 0., 0.05),
  //     spatial_dyn::Orientation(ab) * quat_ee));
  // t += constraints.back()->num_timesteps();


  // std::cout << world << std::endl;

  // size_t t = 0;
  constraints.emplace_back(new logic_opt::CartesianPoseConstraint<3>(
      world, t, kEeFrame, world.kWorldFrame, spatial_dyn::Position(ab, -1, ee_offset),
      Eigen::Quaterniond::Identity()));
  t += constraints.back()->num_timesteps();

  constraints.emplace_back(new logic_opt::PickConstraint(world, t, kEeFrame, "hook"));
  t += constraints.back()->num_timesteps();

  constraints.emplace_back(new logic_opt::PushConstraint(world, t, "hook", "box_3", "table"));
  // constraints.emplace_back(new logic_opt::WorkspaceConstraint(world, t, kEeFrame));
  t += constraints.back()->num_timesteps();

  // t += 2;

  constraints.emplace_back(new logic_opt::PlaceConstraint(world, t, "hook", "table"));
  t += constraints.back()->num_timesteps();

  // for (int t_last = t - constraints.back()->num_timesteps(), dt = -2; dt <= 0; dt++) {
  //   world.set_controller_frames(world.control_frame(t_last), world.target_frame(t_last), t_last + dt);
  //   constraints.emplace_back(new logic_opt::TrajectoryConstraint(world, t_last + dt));
  // }

  constraints.emplace_back(new logic_opt::PickConstraint(world, t, kEeFrame, "box_3"));
  t += constraints.back()->num_timesteps();

  // // t += 2;

  constraints.emplace_back(new logic_opt::PlaceConstraint(world, t, "box_3", "shelf"));
  t += constraints.back()->num_timesteps();

  // for (int t_last = t - constraints.back()->num_timesteps(), dt = -2; dt <= 0; dt++) {
  //   world.set_controller_frames(world.control_frame(t_last), world.target_frame(t_last), t_last + dt);
  //   constraints.emplace_back(new logic_opt::TrajectoryConstraint(world, t_last + dt));
  // }

  constraints.emplace_back(new logic_opt::CartesianPoseConstraint<3>(
      world, t, kEeFrame, world.kWorldFrame, spatial_dyn::Position(ab, -1, ee_offset),
      Eigen::Quaterniond::Identity()));
  t += constraints.back()->num_timesteps();

  const size_t T = world.num_timesteps();
  if (t != T) throw std::runtime_error("Constraint timesteps must equal T.");

  logic_opt::FrameVariables<3> variables(T);
  variables.X_0 = Eigen::MatrixXd::Zero(world.kDof, world.num_timesteps());
  // variables.X_0.block<3,1>(0, 1) = Eigen::Vector3d(-0.3, 0., 0.);
  // variables.X_0.block<3,1>(0, 3) = world_objects->at("box").T_to_parent().translation();
  // auto obj_norm = dynamic_cast<logic_opt::MinL1NormObjective *>(objectives[0].get());
  // obj_norm->X_0 = variables.X_0;
  // for (size_t t = 0; t <= 10; t++) {
  //   variables.X_0.block<3,1>(0, t + 2) = (1 - 0.1 * t) * Eigen::Vector3d(0.25, -0.5, 0.35) +
  //                                        0.1 * t * Eigen::Vector3d(0.25, -0.5, 0.45);
  // }
  // for (size_t t = 0; t < 3; t++) {
  //   variables.X_0.block<3,1>(0, t + 2) = (1. - t / 3.) * Eigen::Vector3d(0.25, -0.5, 0.35) +
  //                                        t / 3. * Eigen::Vector3d(0.19, -0.5, 0.35);
  // }
  // for (size_t t = 0; t < 5; t++) {
  //   variables.X_0.block<3,1>(0, t + 2 + 3) = (1. - t / 5.) * Eigen::Vector3d(0.19, -0.5, 0.35) +
  //                                        t / 5. * Eigen::Vector3d(0.19, -0.5, 0.45);
  // }
  // for (size_t t = 0; t <= 3; t++) {
  //   variables.X_0.block<3,1>(0, t + 2 + 3 + 5) = (1. - t / 3.) * Eigen::Vector3d(0.19, -0.5, 0.45) +
  //                                        t / 3. * Eigen::Vector3d(0.25, -0.5, 0.45);
  // }
  std::cout << variables.X_0 << std::endl;

  std::string logdir;
  if (!args.logdir.empty()) {
    logdir = args.logdir + "/";
    // logdir += (args.optimizer == Args::Optimizer::NLOPT) ? "nlopt" : "ipopt";
    // logdir += (args.task == Args::Task::PICK_PLACE) ? "_pickplace" : "";
    // logdir += args.with_scalar_constraints ? "_scalar" : "";
    // logdir += args.with_hessian ? "_hessian" : "";
    // logdir += "/";
    mkdir(logdir.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
    std::cout << "Log output: " << logdir << std::endl;
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
    options.derivative_test = args.derivative_test;
    options.use_hessian = args.with_hessian;
    options.logdir = logdir;
    // options.print_level = 12;
    // options.max_iter = 1e6;
    // options.max_cpu_time = 1e3;
    logic_opt::Ipopt ipopt(options);
    logic_opt::Ipopt::OptimizationData data;
    X_optimal = ipopt.Trajectory(variables, objectives, constraints, &data,
                                 [&world](int i, const Eigen::MatrixXd& X) {
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

  ExecuteOpspaceController(ab, world, X_optimal, g_runloop);
}

/**
 * opspace_controller.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 4, 2019
 * Authors: Toki Migimatsu
 */

#include "logic_opt/control/opspace_controller.h"

#include <mutex>  // std::mutex

#include <ctrl_utils/control.h>
#include <ctrl_utils/euclidian.h>
#include <ctrl_utils/filesystem.h>
#include <ctrl_utils/json.h>
#include <ctrl_utils/math.h>
#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/timer.h>
#include <ncollide_cpp/ncollide3d.h>
#include <ncollide_cpp/ncollide2d.h>
#include <spatial_dyn/algorithms/inverse_kinematics.h>
#include <redis_gl/redis_gl.h>

#include "logic_opt/control/throw_constraint_scp.h"
#include "logic_opt/optimization/constraints.h"
#include "logic_opt/optimization/ipopt.h"
#include "logic_opt/optimization/objectives.h"

// #define REAL_WORLD

namespace Eigen {

using Vector7d = Eigen::Matrix<double,7,1>;
using Matrix32d = Matrix<double,3,2>;

}  // namespace Eigen

namespace {

// Robot
const std::string kNameRobot = "franka_panda";
const std::string kPathResources = "../resources";
const std::string kPathUrdf = kPathResources + "/" + kNameRobot + "/" + kNameRobot + ".urdf";

const std::string KEY_MODELS_PREFIX  = "dbot::model::";
const std::string KEY_OBJECTS_PREFIX = "dbot::object::";
const std::string KEY_OBJECT_MODELS_PREFIX = KEY_OBJECTS_PREFIX + "model::";

// SET keys
const std::string KEY_SENSOR_Q        = kNameRobot + "::sensor::q";
const std::string KEY_SENSOR_DQ       = kNameRobot + "::sensor::dq";
const std::string KEY_CONTROL_DX        = kNameRobot + "::control::dx";
const std::string KEY_CONTROL_POS     = kNameRobot + "::control::pos";
const std::string KEY_CONTROL_ORI     = kNameRobot + "::control::ori";
const std::string KEY_CONTROL_POS_ERR = kNameRobot + "::control::pos_err";
const std::string KEY_CONTROL_ORI_ERR = kNameRobot + "::control::ori_err";
const std::string KEY_CONTROL_POS_DES = kNameRobot + "::control::pos_des";
const std::string KEY_CONTROL_ORI_DES = kNameRobot + "::control::ori_des";
const std::string KEY_COLLISION_POS    = kNameRobot + "::collision::pos";
const std::string KEY_COLLISION_ACTIVE = kNameRobot + "::collision::active";
const std::string KEY_COLLISION_KP_KV  = kNameRobot + "::collision::kp_kv";

// Gripper keys
const std::string kNameGripper = "robotiq_gripper";
const std::string KEY_GRIPPER_COMMAND = kNameGripper + "::control::pub::command";
const std::string KEY_GRIPPER_STATUS  = kNameGripper + "::control::pub::status";

// Controller gains
const std::string KEY_CONTROL_POS_TOL = kNameRobot + "::control::pos_tol";
const std::string KEY_CONTROL_ORI_TOL = kNameRobot + "::control::ori_tol";
const std::string KEY_CONTROL_POS_VEL_TOL = kNameRobot + "::control::pos_vel_tol";
const std::string KEY_CONTROL_ORI_VEL_TOL = kNameRobot + "::control::ori_vel_tol";
const std::string KEY_CONTROL_POS_ERR_MAX = kNameRobot + "::control::pos_err_max";
const std::string KEY_CONTROL_ORI_ERR_MAX = kNameRobot + "::control::ori_err_max";

const double kTimerFreq          = 1000.;

const Eigen::Vector7d kQHome     = (Eigen::Vector7d() <<
                                    0., -M_PI/6., 0., -5.*M_PI/6., 0., 2.*M_PI/3., 0.).finished();
const Eigen::Vector3d kEeOffset  = Eigen::Vector3d(0., 0., 0.107);  // Without gripper
const Eigen::Vector3d kFrankaGripperOffset  = Eigen::Vector3d(0., 0., 0.1034);
const Eigen::Vector3d kRobotiqGripperOffset = Eigen::Vector3d(0., 0., 0.144);  // Ranges from 0.130 to 0.144
const Eigen::Vector3d kEpsilonPos = Eigen::Vector3d(0.02, 0.02, 0.01);
const double kEpsilonOri    = 0.1;
const double kEpsilonVelPos = 0.02;
const double kEpsilonVelOri = 0.05;
const double kMaxErrorPos   = 80 * 0.04;
const double kMaxErrorOri   = 80 * M_PI / 20;

const std::string kEeFrame = "ee";

#ifdef REAL_WORLD
const std::set<std::string> tracked_objects = { "hook", "box_1", "box_2", "box_3", "shelf", "platform_left", "platform_middle", "platform_right" };
#else  // REAL_WORLD
const std::set<std::string> tracked_objects = {};
#endif  // REAL_WORLD

std::vector<int> gripper_widths;

std::unique_ptr<ncollide3d::shape::Shape>
MakeCollision(const spatial_dyn::Graphics::Geometry& geometry) {
  switch (geometry.type) {
    case spatial_dyn::Graphics::Geometry::Type::kBox: {
      return std::make_unique<ncollide3d::shape::Cuboid>(geometry.scale.array() / 2.);
    case spatial_dyn::Graphics::Geometry::Type::kCapsule:
      return std::make_unique<ncollide3d::shape::Capsule>(geometry.length / 2., geometry.radius);
    case spatial_dyn::Graphics::Geometry::Type::kSphere:
      return std::make_unique<ncollide3d::shape::Ball>(geometry.radius);
    case spatial_dyn::Graphics::Geometry::Type::kMesh:
      return std::make_unique<ncollide3d::shape::TriMesh>(geometry.mesh);
    default:
      throw std::runtime_error("MakeCollision(): Geometry type " +
                               ctrl_utils::ToString(geometry.type) + " not implemented yet.");
      break;
    }
  }
}

struct WorldState {

  WorldState(const logic_opt::World3& world) {
    const std::map<std::string, logic_opt::Object3>& objects_in = *world.objects();
    objects = std::make_shared<std::map<std::string, logic_opt::Object3>>();
    for (const auto& key_val : objects_in) {
      const std::string& name_object = key_val.first;
      if (name_object == "wall") continue;
      const logic_opt::Object3& rb = key_val.second;
      objects->emplace(name_object, rb);

      logic_opt::Object3& object = objects->at(name_object);
      const Eigen::Isometry3d T_to_world = world.T_to_world(name_object, Eigen::MatrixXd::Zero(6, world.num_timesteps()), 0);
      object.set_T_to_parent(T_to_world);

      if (rb.graphics.empty()) continue;

      std::shared_ptr<ncollide3d::shape::Shape> collision;
      if (rb.graphics.size() == 1) {
        collision = MakeCollision(rb.graphics[0].geometry);
        continue;
      }

      ncollide3d::shape::ShapeVector shapes;
      shapes.reserve(rb.graphics.size());
      for (const spatial_dyn::Graphics& graphics : rb.graphics) {
        shapes.emplace_back(logic_opt::ConvertIsometry<3>(graphics.T_to_parent),
                            MakeCollision(graphics.geometry));
      }
      collision = std::make_unique<ncollide3d::shape::Compound>(std::move(shapes));
      object.collision = std::move(collision);
    }
  }

  std::shared_ptr<std::map<std::string, logic_opt::Object3>> objects;
  size_t t;
  std::mutex mtx_objects;

  Eigen::MatrixXd X_plan;
  std::mutex mtx_plan;

};

bool HasPoseConverged(const spatial_dyn::ArticulatedBody& ab, const Eigen::Vector3d& x_des,
                      const Eigen::Quaterniond& quat_des, const Eigen::Vector3d& ee_offset,
                      const Eigen::Vector3d& eps_pos, double eps_ori) {
  const Eigen::Vector3d x = spatial_dyn::Position(ab, -1, ee_offset);
  const Eigen::Quaterniond quat = spatial_dyn::Orientation(ab, -1);
  return ((x - x_des).array().abs() < eps_pos.array()).all() &&
         ctrl_utils::OrientationError(quat, quat_des).norm() < eps_ori;
}

bool HasVelocityConverged(const spatial_dyn::ArticulatedBody& ab, const Eigen::Vector3d& ee_offset,
                          double eps_vel_pos, double eps_vel_ori) {
  const Eigen::Matrix6Xd& J = spatial_dyn::Jacobian(ab, -1, ee_offset);
  const Eigen::Vector3d dx = J.topRows<3>() * ab.dq();
  const Eigen::Vector3d w = J.bottomRows<3>() * ab.dq();
  return dx.norm() < eps_vel_pos && w.norm() < eps_vel_ori;
}

bool HasConverged(const spatial_dyn::ArticulatedBody& ab, const Eigen::Vector3d& x_des,
                  const Eigen::Quaterniond& quat_des, const Eigen::Vector3d& ee_offset,
                  const Eigen::Vector3d& eps_pos, double eps_vel_pos, double eps_ori, double eps_vel_ori) {
  return HasPoseConverged(ab, x_des, quat_des, ee_offset, eps_pos, eps_ori) &&
         HasVelocityConverged(ab, ee_offset, eps_vel_pos, eps_vel_ori);
}

void UpdateObjectStates(ctrl_utils::RedisClient& redis, const logic_opt::World3& world,
                        WorldState& sim, size_t idx_trajectory, const Eigen::MatrixXd& X_optimal,
                        const Eigen::Isometry3d& T_ee_to_world,
                        const redis_gl::simulator::Interaction& interaction);

Eigen::MatrixXd PlanGrasps(const logic_opt::World3& world, const Eigen::MatrixXd& X_optimal);

std::map<size_t, std::pair<Eigen::MatrixXd, Eigen::MatrixXd>>
ComputeThrowTrajectories(spatial_dyn::ArticulatedBody& ab, const logic_opt::World3& world,
                         const Eigen::MatrixXd& X_optimal);

void InitializeRedisKeys(ctrl_utils::RedisClient& redis, const logic_opt::World3& world) {
  // Initialize controller parameters
  redis.set(KEY_CONTROL_POS_TOL, 3 * kEpsilonPos);
  redis.set(KEY_CONTROL_POS_VEL_TOL, kEpsilonVelPos);
  redis.set(KEY_CONTROL_ORI_TOL, kEpsilonOri);
  redis.set(KEY_CONTROL_ORI_VEL_TOL, kEpsilonVelOri);
  redis.set(KEY_CONTROL_POS_ERR_MAX, kMaxErrorPos);
  redis.set(KEY_CONTROL_ORI_ERR_MAX, kMaxErrorOri);

  const redis_gl::simulator::ModelKeys kModelKeys("dbot");
  redis_gl::simulator::RegisterModelKeys(redis, kModelKeys);
  redis_gl::simulator::RegisterResourcePath(redis, (std::filesystem::current_path() / kPathResources).string());
  for (const std::pair<std::string, spatial_dyn::RigidBody>& key_val : *world.objects()) {
    const std::string& frame = key_val.first;
    if (tracked_objects.find(frame) != tracked_objects.end()) continue;
    // if (frame == kEeFrame) continue;
    const spatial_dyn::RigidBody& object = key_val.second;
    redis_gl::simulator::ObjectModel object_model;
    std::cout << "Register: " << frame;// << std::endl;
    object_model.name = frame;
    object_model.graphics = object.graphics;
    object_model.key_pos = KEY_OBJECTS_PREFIX + object_model.name + "::pos";
    object_model.key_ori = KEY_OBJECTS_PREFIX + object_model.name + "::ori";
    redis_gl::simulator::RegisterObject(redis, kModelKeys, object_model);
    const Eigen::Isometry3d T_to_world = world.T_to_world(frame, Eigen::MatrixXd::Zero(6, world.num_timesteps()), 0);
    std::cout << " at [" << T_to_world.translation().transpose() << "]" << std::endl;
    redis.set(object_model.key_pos, T_to_world.translation());
    redis.set(object_model.key_ori, Eigen::Quaterniond(T_to_world.linear()).coeffs());
  }

  redis.set(KEY_OBJECTS_PREFIX + world.kWorldFrame + "::pos", Eigen::Vector3d::Zero());
  redis.set(KEY_OBJECTS_PREFIX + world.kWorldFrame + "::ori", Eigen::Quaterniond::Identity().coeffs());

  redis.sync_commit();
}

void TrajectoryOptimizationThread(const logic_opt::World3* world, const Eigen::MatrixXd* X,
                                  WorldState* sim, std::atomic_bool* m_runloop) {
  logic_opt::Ipopt::Options options;
  options.print_level = 0;
  logic_opt::Ipopt ipopt(options);
  logic_opt::Ipopt::OptimizationData data;

  std::cout << "TrajectoryOptimizationThread" << std::endl;

  ctrl_utils::Timer timer(10);

  while (*m_runloop) {
    timer.Sleep();

    sim->mtx_objects.lock();
    const double t_action = sim->t;
    const auto sim_objects = std::make_shared<std::map<std::string, logic_opt::Object3>>(*sim->objects);
    sim->mtx_objects.unlock();

    logic_opt::World3 sim_world(sim_objects);
    const std::string& control = world->control_frame(t_action);
    const std::string& target = world->target_frame(t_action);
    const ctrl_utils::Tree<std::string, logic_opt::Frame>& tree = world->frames(t_action);

    // Copy kinematic tree from world
    auto sim_objects_abs = std::make_shared<std::map<std::string, logic_opt::Object3>>(*sim_objects);
    for (const std::pair<std::string, logic_opt::Frame>& node : tree.values()) {
      const std::string& frame = node.first;
      const std::optional<std::string> parent = tree.parent(frame);
      if (!parent || *parent == sim_world.kWorldFrame) continue;

      // Fix pose relative to parent
      sim_world.AttachFrame(frame, *parent, 0);
      sim_world.set_controller_frames("", "", 0);
      const Eigen::Isometry3d& T_to_world = sim_objects_abs->at(frame).T_to_parent();
      const Eigen::Isometry3d& T_parent_to_world = sim_objects_abs->at(*parent).T_to_parent();
      sim_objects->at(frame).set_T_to_parent(T_parent_to_world.inverse() * T_to_world);
    }
    sim_objects_abs.reset();

    const Eigen::Isometry3d& T_control_to_target = sim_objects->at(control).T_to_parent();
    const Eigen::Isometry3d& T_control_to_target_des = world->T_control_to_target(*X, t_action);

    logic_opt::Objectives objectives;
    objectives.emplace_back(new logic_opt::LinearVelocityObjective3(sim_world, kEeFrame));
    objectives.emplace_back(new logic_opt::AngularVelocityObjective(sim_world, kEeFrame, 2.));

    logic_opt::Constraints constraints;

    size_t t = 0;
    constraints.emplace_back(new logic_opt::CartesianPoseConstraint<3>(
        sim_world, t, control, target, T_control_to_target));
    t += constraints.back()->num_timesteps();

    t += 2;

    constraints.emplace_back(new logic_opt::CartesianPoseConstraint<3>(
        sim_world, t, control, target, T_control_to_target_des));
    t += constraints.back()->num_timesteps();

    for (int t_last = t - constraints.back()->num_timesteps(), dt = -2; dt < -1; dt++) {
      constraints.emplace_back(new logic_opt::TrajectoryConstraint(sim_world, t_last + dt));
    }

    const size_t T = sim_world.num_timesteps();
    if (t != T) throw std::runtime_error("Constraint timesteps must equal T.");

    logic_opt::FrameVariables<3> variables(T);
    variables.X_0 = Eigen::MatrixXd::Zero(sim_world.kDof, sim_world.num_timesteps());
    const Eigen::VectorXd x_0 = ctrl_utils::Log(T_control_to_target);
    const Eigen::VectorXd x_T = ctrl_utils::Log(T_control_to_target_des);
    for (size_t i = 0; i < logic_opt::FrameVariables<3>::kDof; i++) {
      variables.X_0.row(i).setLinSpaced(variables.X_0.cols(), x_0(i), x_T(i));
    }

    // std::cout << variables.X_0 << std::endl << std::endl;

    continue;
    try {
      const Eigen::MatrixXd X_plan = ipopt.Trajectory(variables, objectives, constraints, &data);
      if (ipopt.status() == "success") {
        sim->mtx_plan.lock();
        sim->X_plan = X_plan;
        sim->mtx_plan.unlock();
        // std::cout << X_plan << std::endl << std::endl;
      } else {
        sim->mtx_plan.lock();
        sim->X_plan = variables.X_0;
        sim->X_plan.col(1) = sim->X_plan.col(sim->X_plan.cols() - 1);
        sim->mtx_plan.unlock();
      }
    } catch (...) {}
    // for (const std::unique_ptr<logic_opt::Constraint>& c : constraints) {
    //   Eigen::VectorXd f(c->num_constraints());
    //   c->Evaluate(X_plan, f);
    //   std::cout << c->name << ":" << std::endl;
    //   for (size_t i = 0; i < c->num_constraints(); i++) {
    //     std::cout << "  "
    //               << (c->constraint_type(i) == logic_opt::Constraint::Type::kInequality ?
    //                   "<" : "=")
    //               << " : " << f(i) << std::endl;
    //   }
    // }
    // std::cout << sim_world << std::endl << std::endl;
  }
}

std::pair<std::set<std::string>, std::set<std::string>>
ComputeCollisionPairs(const logic_opt::World3& world, size_t t_collision) {
  const ctrl_utils::Tree<std::string, logic_opt::Frame>& frames = world.frames(t_collision);
  std::pair<std::set<std::string>, std::set<std::string>> ee_obstacles;
  for (const std::pair<std::string, logic_opt::Frame>& key_val : frames.values()) {
    const std::string& frame = key_val.first;
    if (frame == logic_opt::World3::kWorldFrame || !world.objects()->at(frame).collision) continue;
    if (frame == "wall") continue;

    // Descendants of the control frame are fixed to the ee and can't collide
    if (frames.is_descendant(frame, world.control_frame(t_collision))) {
      ee_obstacles.first.insert(frame);
    } else {
      ee_obstacles.second.insert(frame);
    }
  }
  return ee_obstacles;
}

}  // namespace

namespace logic_opt {

void ExecuteOpspaceController(spatial_dyn::ArticulatedBody& ab, const World3& world,
                              const Eigen::MatrixXd& X_optimal, volatile std::sig_atomic_t& g_runloop) {

  // auto throw_trajectories = ComputeThrowTrajectories(ab, world, X_optimal);

  // Modify trajectory for grasping
  Eigen::MatrixXd X_final = PlanGrasps(world, X_optimal);
  std::cout << X_final << std::endl << std::endl;

  // Set up timer and Redis
  ctrl_utils::RedisClient redis;
  redis.connect();
  InitializeRedisKeys(redis, world);
  ctrl_utils::RedisClient redis_robot;
#ifdef REAL_WORLD
  redis_robot.connect("172.24.69.103");
#else  // REAL_WORLD
  redis_robot.connect();
#endif  // REAL_WORLD

  ctrl_utils::Timer timer(kTimerFreq);

  // End-effector parameters
  ab.set_q(kQHome);
  Eigen::Quaterniond quat_grasp_to_ee(spatial_dyn::Orientation(ab).inverse());
  const Eigen::Vector3d ee_offset = kEeOffset + kRobotiqGripperOffset;
  const Eigen::Isometry3d T_grasp_to_ee = Eigen::Translation3d(ee_offset) * quat_grasp_to_ee;
  // const Eigen::Isometry3d& T_ee = world.objects()->at(kEeFrame).T_to_parent();
  // Eigen::Quaterniond quat_ee(T_ee.linear());
  // Eigen::Ref<const Eigen::Vector3d> ee_offset = T_ee.translation();

  // Create sim world
  WorldState sim(world);
  // std::thread thread_trajectory;

  size_t idx_trajectory = 0;
  sim.t = idx_trajectory;

  auto ee_objects = ComputeCollisionPairs(world, idx_trajectory);
  std::atomic_bool m_runloop = { true };
  while (g_runloop) {
    timer.Sleep();

    // Controller frames
    const std::pair<std::string, std::string>& controller_frames = world.controller_frames(idx_trajectory);
    const std::string& control_frame = controller_frames.first;
    const std::string& target_frame = controller_frames.second;

    // Get Redis values
    auto fut_interaction = redis.get<redis_gl::simulator::Interaction>(redis_gl::simulator::KEY_INTERACTION);
    std::future<Eigen::Vector7d> fut_q = redis_robot.get<Eigen::Vector7d>(KEY_SENSOR_Q);
    std::future<Eigen::Vector7d> fut_dq = redis_robot.get<Eigen::Vector7d>(KEY_SENSOR_DQ);
    std::future<std::string> fut_sensor_pos = redis_robot.get<std::string>("franka_panda::sensor::pos");
    std::future<std::string> fut_sensor_ori = redis_robot.get<std::string>("franka_panda::sensor::ori");
#ifdef REAL_WORLD
    std::future<Eigen::Vector3d> fut_eps_pos = redis.get<Eigen::Vector3d>(KEY_CONTROL_POS_TOL);
    std::future<double> fut_eps_ori = redis.get<double>(KEY_CONTROL_ORI_TOL);
    std::future<double> fut_eps_vel_pos = redis.get<double>(KEY_CONTROL_POS_VEL_TOL);
    std::future<double> fut_eps_vel_ori = redis.get<double>(KEY_CONTROL_ORI_VEL_TOL);
#endif  // REAL_WORLD
    // std::future<Eigen::Vector3d> fut_pos_control = redis.get<Eigen::Vector3d>(KEY_OBJECTS_PREFIX + control_frame + "::pos");
    // std::future<Eigen::Vector4d> fut_quat_control = redis.get<Eigen::Vector4d>(KEY_OBJECTS_PREFIX + control_frame + "::ori");
    std::future<Eigen::Vector3d> fut_pos_target = redis.get<Eigen::Vector3d>(KEY_OBJECTS_PREFIX + target_frame + "::pos");
    std::future<Eigen::Vector4d> fut_quat_target = redis.get<Eigen::Vector4d>(KEY_OBJECTS_PREFIX + target_frame + "::ori");

    std::map<std::string, std::pair<std::future<Eigen::Vector3d>, std::future<Eigen::Vector4d>>> fut_poses;
    for (const std::string& name_object : tracked_objects) {
      if (name_object == control_frame || name_object == target_frame) continue;
      fut_poses.emplace(name_object,
                        std::make_pair<std::future<Eigen::Vector3d>, std::future<Eigen::Vector4d>>(
                            redis.get<Eigen::Vector3d>(KEY_OBJECTS_PREFIX + name_object + "::pos"),
                            redis.get<Eigen::Vector4d>(KEY_OBJECTS_PREFIX + name_object + "::ori")));
    }
    redis.commit();
    redis_robot.commit();

    // Compute forward kinematics
    ab.set_q(fut_q.get());
    ab.set_dq(fut_dq.get());
#ifdef REAL_WORLD
    redis.set(KEY_SENSOR_Q, ab.q());
    redis.set(KEY_SENSOR_DQ, ab.dq());
    redis.set("franka_panda::sensor::pos", fut_sensor_pos.get());
    redis.set("franka_panda::sensor::ori", fut_sensor_ori.get());
#endif
    redis.commit();

    // TODO: from perception
    const Eigen::Quaterniond quat_control_to_world =
        world.controller(idx_trajectory) == "push_2"
            ? Eigen::Quaterniond(world.Orientation(control_frame, World3::kWorldFrame, X_final, idx_trajectory))
            // : Eigen::Quaterniond(fut_quat_control.get());
            : Eigen::Quaterniond(sim.objects->at(control_frame).T_to_parent().linear());
    const Eigen::Quaterniond quat_target_to_world =
        world.controller(idx_trajectory) == "push_1"
            ? Eigen::Quaterniond(world.Orientation(target_frame, World3::kWorldFrame, X_final, idx_trajectory))
            : Eigen::Quaterniond(fut_quat_target.get());

    // const Eigen::Isometry3d T_control_to_world = Eigen::Translation3d(fut_pos_control.get()) *
    const Eigen::Isometry3d T_control_to_world = Eigen::Translation3d(sim.objects->at(control_frame).T_to_parent().translation()) *
                                                 quat_control_to_world;
    const Eigen::Isometry3d T_target_to_world = Eigen::Translation3d(fut_pos_target.get()) *
                                                quat_target_to_world;
    const Eigen::Isometry3d& T_grasp_to_world = ab.T_to_world(-1) * T_grasp_to_ee;
    const Eigen::Isometry3d T_control_to_ee = Eigen::Translation3d(-ee_offset) * ab.T_to_world(-1).inverse() * T_control_to_world;

    // const Eigen::Isometry3d T_control_to_ee = quat_ee * world.T_to_frame(control_frame, kEeFrame, X_final, idx_trajectory);
    // const Eigen::Isometry3d T_target_to_world = target_frame != World3::kWorldFrame
    //                                               ? sim.objects->at(target_frame).T_to_parent()
    //                                               : Eigen::Isometry3d::Identity();world.T_to_world(target_frame, X_final, idx_trajectory);
    const Eigen::Isometry3d T_control_to_target_des = world.T_control_to_target(X_final, idx_trajectory);

    // Update object states
    sim.mtx_objects.lock();
#ifdef REAL_WORLD
    if (target_frame != World3::kWorldFrame) {
      sim.objects->at(target_frame).set_T_to_parent(T_target_to_world);
    }
    for (auto& key_val : fut_poses) {
      const std::string& name_object = key_val.first;
      auto& pos_quat = key_val.second;
      if (sim.objects->find(name_object) == sim.objects->end()) continue;
      sim.objects->at(name_object).set_T_to_parent(Eigen::Quaterniond(pos_quat.second.get()),
                                                   pos_quat.first.get());
    }
#endif  // REAL_WORLD
    UpdateObjectStates(redis, world, sim, idx_trajectory, X_final, T_grasp_to_world,
                       fut_interaction.get());
    sim.objects->at(kEeFrame).set_T_to_parent(T_grasp_to_world);
    sim.t = idx_trajectory;
    sim.mtx_objects.unlock();
    // if (!thread_trajectory.joinable()) {
    //   thread_trajectory = std::thread(TrajectoryOptimizationThread, &world, &X_final, &sim, &m_runloop);
    // }
    // TrajectoryOptimizationThread(&world, &X_final, &sim, &g_runloop);

    // Compute desired pose
    const Eigen::Isometry3d T_des_to_world = T_target_to_world * T_control_to_target_des * T_control_to_ee.inverse();
    Eigen::Vector3d x_des = T_des_to_world.translation();
    if (x_des(2) <= 0.03) x_des(2) = 0.03;
    const Eigen::Quaterniond quat_des = Eigen::Quaterniond(T_des_to_world.linear());

    // Check for convergence
#ifdef REAL_WORLD
    const double eps_vel_pos = fut_eps_vel_pos.get();
    const double eps_vel_ori = fut_eps_vel_ori.get();
    if (HasConverged(ab, x_des, quat_des, ee_offset, fut_eps_pos.get(),
                     eps_vel_pos, fut_eps_ori.get(), eps_vel_ori)) {
#else  // REAL_WORLD
    const double eps_vel_pos = kEpsilonVelPos;
    const double eps_vel_ori = kEpsilonVelOri;
    if (HasConverged(ab, x_des, quat_des, ee_offset, kEpsilonPos,
                     eps_vel_pos, kEpsilonOri, eps_vel_ori)) {
#endif  // REAL_WORLD
      static size_t t_pick;
      static Eigen::Vector6d X_saved;
      const std::string& controller = world.controller(idx_trajectory);
      std::string gripper_status = "done";
      if (controller == "place") {
#ifdef REAL_WORLD
        if (t_pick == 0) {
          t_pick++;
          std::cout << "next 1" << std::endl;
          X_final.col(idx_trajectory) = X_saved;
          X_final(2, idx_trajectory) += 0.03;
          redis.set(KEY_CONTROL_POS_TOL, kEpsilonPos);
          continue;
        } else if (t_pick == 1) {
          t_pick++;
          std::cout << "next 2" << std::endl;
          X_final(2, idx_trajectory) -= 0.03;
          redis.set(KEY_CONTROL_POS_TOL, kEpsilonPos + Eigen::Vector3d(0.01, 0.01, 0.));
          continue;
        }
#endif  // REAL_WORLD
        std::cout << "Opening gripper... " << std::flush;
        gripper_status = redis.sync_request<std::string>(KEY_GRIPPER_COMMAND, "o", KEY_GRIPPER_STATUS);
        std::cout << "Done." << std::endl;
      } else if (controller == "pick") {
#ifdef REAL_WORLD
        if (t_pick == 0) {
          t_pick++;
          std::cout << "next" << std::endl;
          X_final(2, idx_trajectory) -= 0.05;
          redis.set(KEY_CONTROL_POS_TOL, kEpsilonPos + Eigen::Vector3d(0.01, 0.01, 0.));
          continue;
        }
#endif  // REAL_WORLD
        std::cout << "Closing gripper to " << gripper_widths[idx_trajectory] << "... " << std::endl;
        gripper_status = redis.sync_request<std::string>(KEY_GRIPPER_COMMAND, gripper_widths[idx_trajectory], KEY_GRIPPER_STATUS);
        std::cout << "Done." << std::endl;
          // t_pick++;
          // X_final(2, idx_trajectory) += 0.03;
          // redis.set(KEY_CONTROL_POS_TOL, kEpsilonPos + Eigen::Vector3d(0.03, 0.03, 0.));
          // continue;
      } else if (controller == "push_1") {
#ifdef REAL_WORLD
        if (t_pick == 0) {
          t_pick++;
          std::cout << "next" << std::endl;
          X_final.block<2,1>(0, idx_trajectory) -= Eigen::Vector2d(0.03, 0.1);
          redis.set(KEY_CONTROL_POS_TOL, kEpsilonPos + Eigen::Vector3d(0.1, 0., 0.));
          redis.set(KEY_CONTROL_POS_VEL_TOL, 3 * kEpsilonVelPos);
          continue;
        }
#endif  // REAL_WORLD
      }
      if (gripper_status != "done") {
        throw std::runtime_error("Gripper command failed: " + gripper_status + ".");
      }

      idx_trajectory++;
      if (idx_trajectory >= X_final.cols()) {
        break;
      }
#ifdef REAL_WORLD
      if (world.controller(idx_trajectory) == "pick") {
        t_pick = 0;
        X_final(2, idx_trajectory) += 0.05;
        redis.set(KEY_CONTROL_POS_TOL, kEpsilonPos + Eigen::Vector3d(0., -0.01, 0.));
      } else if (world.controller(idx_trajectory) == "place") {
        const std::string control_frame = world.control_frame(idx_trajectory);
        const std::string target_frame = world.target_frame(idx_trajectory);
        const Eigen::Isometry3d& T_control_to_world = sim.objects->at(control_frame).T_to_parent();
        const Eigen::Isometry3d& T_target_to_world = sim.objects->at(target_frame).T_to_parent();
        const Eigen::Isometry3d T_control_to_target = T_target_to_world.inverse() * T_control_to_world;
        X_saved = X_final.col(idx_trajectory);
        X_final.col(idx_trajectory) = ctrl_utils::Log(T_control_to_target) + Eigen::Vector6d(0., 0., 0.03, 0., 0., 0.);
        t_pick = 0;
        redis.set(KEY_CONTROL_POS_TOL, kEpsilonPos + Eigen::Vector3d(0.03, 0.03, 0.));
        redis.set(KEY_CONTROL_ORI_TOL, kEpsilonOri);
      } else if (world.controller(idx_trajectory) == "push_1") {
        t_pick = 0;
        X_final.block<2,1>(0, idx_trajectory) += Eigen::Vector2d(0.03, 0.1);
        redis.set(KEY_CONTROL_POS_TOL, 2 * kEpsilonPos);
        redis.set(KEY_CONTROL_ORI_TOL, 3 * kEpsilonOri);
      } else if (world.controller(idx_trajectory) == "push_2") {
        redis.set(KEY_CONTROL_POS_TOL, 3 * kEpsilonPos);
        redis.set(KEY_CONTROL_ORI_TOL, 3 * kEpsilonOri);
      } else if (world.controller(idx_trajectory) == "cart_pos") {
        redis.set(KEY_CONTROL_POS_TOL, 3 * kEpsilonPos);
        redis.set(KEY_CONTROL_ORI_TOL, kEpsilonOri);
      } else {
        redis.set(KEY_CONTROL_POS_TOL, kEpsilonPos);
        redis.set(KEY_CONTROL_ORI_TOL, kEpsilonOri);
      }
      redis.set(KEY_CONTROL_POS_VEL_TOL, kEpsilonVelPos);
      std::cout << idx_trajectory << ": " << world.controller(idx_trajectory) << "("
                << world.control_frame(idx_trajectory) << ", "
                << world.target_frame(idx_trajectory) << "):\t"
                << X_final.col(idx_trajectory).transpose() << std::endl;
#endif  // REAL_WORLD

      ee_objects = ComputeCollisionPairs(world, idx_trajectory);
      const std::string& controller_next = world.controller(idx_trajectory);
      // if (controller_next == "pick") {
      //   redis.set(KEY_CONTROL_POS_TOL, 2 * kEpsilonPos);
      // } else {
      // }
      redis.commit();
      continue;
      // TrajectoryOptimizationThread(&world, &X_final, &sim, &g_runloop);
    }

    bool is_collision_detected = false;
    double dist_proximity = 5e-2;
    Eigen::Vector3d dx_collision;
    Eigen::Vector2d kp_kv(0., 20.);
    for (const std::string& ee : ee_objects.first) {
      const Object3& rb_ee = sim.objects->at(ee);
      for (const std::string& object : ee_objects.second) {
        const Object3& rb = sim.objects->at(object);
        const auto contact = ncollide3d::query::contact(rb_ee.T_to_parent(), *rb_ee.collision,
                                                        rb.T_to_parent(), *rb.collision, dist_proximity);
        if (!contact) continue;
        is_collision_detected = true;

        if (contact->depth <= 0.) {
          // Close proximity
          dist_proximity = -contact->depth;
          dx_collision = contact->world2 - contact->world1;
        } else if (contact->depth > 0.) {
          // Penetrating
          // if (kp_kv(0) == 0.) {
          //   dx_collision.setZero();
          //   dist_proximity = 0.;
          //   kp_kv(0) = -80.;
          // }
          // dx_collision += contact->world1 - contact->world2;
        }
      }
    }
    if (is_collision_detected && world.controller(idx_trajectory) != "push_1" && world.controller(idx_trajectory) != "push_2") {
      const Eigen::Vector3d x = spatial_dyn::Position(ab, -1, ee_offset);
      redis_robot.mset(std::make_pair(KEY_COLLISION_POS, dx_collision + x),
                 std::make_pair(KEY_COLLISION_ACTIVE, is_collision_detected),
                 std::make_pair(KEY_COLLISION_KP_KV, kp_kv));
      if (kp_kv(0) != 0. && HasVelocityConverged(ab, ee_offset, 0.000001, 0.000001) &&
          world.controller(idx_trajectory) != "push") {
        Eigen::Vector3d x_traj = Eigen::Vector3d::Zero();
        sim.mtx_plan.lock();
        if (sim.X_plan.cols() > 1) x_traj = sim.X_plan.block<3,1>(0, 1);
        sim.mtx_plan.unlock();
        if (x_traj.squaredNorm() > 0.) x_traj = T_target_to_world * x_traj;
        // std::cout << ctrl_utils::OrthogonalProjection(x_traj, x_des).transpose() << std::endl;
        x_des = x_traj;
        x_des += ctrl_utils::OrthogonalProjection(x_traj, x_des);
      }
    } else {
      redis_robot.set(KEY_COLLISION_ACTIVE, false);
    }
    // for (const auto& key_val : *sim.objects) {
    //   const Object3& rb = key_val.second;
    //   const auto projection = rb.collision->project_point(rb.T_to_parent(), x, true);
    //   if ((x - projection.point).norm() >= 5e-2) continue;
    //   is_collision_detected = true;
    //   redis.mset(std::make_pair("franka_panda::collision::pos", projection.point),
    //              std::make_pair("franka_panda::collision::active", is_collision_detected),
    //              std::make_pair("franka_panda::collision::kp_kv", Eigen::Vector2d(-10., 100.)));
    //   break;
    // }

    redis_robot.set(KEY_CONTROL_POS_DES, x_des);
    redis_robot.set(KEY_CONTROL_ORI_DES, quat_des.coeffs());
    redis_robot.commit();
#ifdef REAL_WORLD
    redis.set(KEY_CONTROL_POS, spatial_dyn::Position(ab, -1, ee_offset));
    redis.set(KEY_CONTROL_ORI, spatial_dyn::Orientation(ab).coeffs());
#endif  // REAL_WORLD
    redis.set(KEY_CONTROL_POS_DES, x_des);
    redis.set(KEY_CONTROL_ORI_DES, quat_des.coeffs());
    redis.set(KEY_CONTROL_POS_ERR, spatial_dyn::Position(ab, -1, ee_offset) - x_des);
    redis.set(KEY_CONTROL_ORI_ERR, ctrl_utils::OrientationError(spatial_dyn::Orientation(ab), quat_des));
    redis.set(KEY_CONTROL_DX, spatial_dyn::Jacobian(ab, -1, ee_offset) * ab.dq());
    redis.commit();

    if ((ab.q().array() != ab.q().array()).any()) break;
  }
  m_runloop = false;

  // thread_trajectory.join();

  std::cout << "Simulated " << timer.time_sim() << "s in " << timer.time_elapsed() << "s." << std::endl;
  std::cout << std::endl;
}

}  // namespace logic_opt

namespace {

void UpdateObjectStates(ctrl_utils::RedisClient& redis, const logic_opt::World3& world,
                        WorldState& sim, size_t idx_trajectory, const Eigen::MatrixXd& X_optimal,
                        const Eigen::Isometry3d& T_ee_to_world,
                        const redis_gl::simulator::Interaction& interaction) {
  const std::string& control_frame = world.controller_frames(idx_trajectory).first;

  const Eigen::Isometry3d& T_ee_to_world_prev = sim.objects->at(kEeFrame).T_to_parent();
  const Eigen::Isometry3d dT = T_ee_to_world * T_ee_to_world_prev.inverse();

  const ctrl_utils::Tree<std::string, logic_opt::Frame> frame_tree = world.frames(idx_trajectory);
  for (const auto& key_val : frame_tree.descendants(control_frame)) {
    // Only check frames between control frame and ee
    const std::string& frame = key_val.first;
    spatial_dyn::RigidBody& rb = sim.objects->at(frame);
    const Eigen::Isometry3d T_to_world_prev = rb.T_to_parent();
    rb.set_T_to_parent(dT * T_to_world_prev);

#ifdef REAL_WORLD
    if (frame != kEeFrame) continue;
#endif  // REAL_WORLD
    redis.set(KEY_OBJECTS_PREFIX + frame + "::pos", rb.T_to_parent().translation());
    redis.set(KEY_OBJECTS_PREFIX + frame + "::ori",
              Eigen::Quaterniond(rb.T_to_parent().linear()).coeffs());
  }

  // Handle interaction
  if (interaction.key_object.empty()) return;
  const std::string frame_interaction = interaction.key_object.substr(KEY_OBJECT_MODELS_PREFIX.length());
  if (sim.objects->find(frame_interaction) != sim.objects->end() &&
      !(frame_tree.contains(frame_interaction) && frame_tree.is_descendant(frame_interaction, kEeFrame))) {
    // Ignore forces applied to frames attached to the robot

    const spatial_dyn::RigidBody& rb = sim.objects->at(frame_interaction);

    // Compute click force
    Eigen::Vector3d pos = rb.T_to_parent().translation();
    Eigen::Quaterniond quat = Eigen::Quaterniond(rb.T_to_parent().linear()).normalized();
    redis_gl::simulator::ClickAdjustPose(interaction, &pos, &quat);
    const Eigen::Isometry3d T_to_world = Eigen::Translation3d(pos) * quat;
    const Eigen::Isometry3d dT = T_to_world * rb.T_to_parent().inverse();

    // Update position of all descendant frames
    for (const auto& key_val : frame_tree.values()) {
      const std::string& frame_descendant = key_val.first;
      if (!frame_tree.is_descendant(frame_descendant, frame_interaction)) continue;
      if (frame_descendant != frame_interaction &&
          frame_descendant == world.control_frame(idx_trajectory)) continue;
      spatial_dyn::RigidBody& rb = sim.objects->at(frame_descendant);

      const Eigen::Isometry3d T_to_world_new = dT * rb.T_to_parent();
      const Eigen::Quaterniond quat_to_world_new = Eigen::Quaterniond(T_to_world_new.linear()).normalized();
      rb.set_T_to_parent(quat_to_world_new, T_to_world_new.translation());
      redis.set(KEY_OBJECTS_PREFIX + frame_descendant + "::pos", T_to_world_new.translation());
      redis.set(KEY_OBJECTS_PREFIX + frame_descendant + "::ori", quat_to_world_new.coeffs());
    }
  }
}

int GripperWidth(double width) {
  return 224 / 0.085 * (0.085 - width) + 16;
};

Eigen::MatrixXd PlanGrasps(const logic_opt::World3& world, const Eigen::MatrixXd& X_optimal) {
  // 0: 0.085
  // 32: 0.075
  // 64: 0.063
  // 96: 0.050
  // 128: 0.037
  // 160: 0.023
  // 192: 0.011
  // 224: 0.
  gripper_widths = std::vector<int>(X_optimal.cols(), 0.);
  Eigen::MatrixXd X_final = X_optimal;
  for (size_t t = 0; t < X_optimal.cols(); t++) {
    if (world.controller(t) == "pick") {
      const std::string frame_control = world.control_frame(t);
      const std::string frame_target = world.target_frame(t);
      const logic_opt::Object3& object = world.objects()->at(frame_target);

      if (object.graphics.front().geometry.type == spatial_dyn::Graphics::Geometry::Type::kBox) {
        X_final.block<2,1>(0, t).setZero();
        X_final(1, t) = 0.01;
        X_final(5, t) = 0;
        gripper_widths[t] = GripperWidth(object.graphics.front().geometry.scale(1));
        continue;
      }

      const Eigen::Isometry3d T_to_parent = world.T_to_parent(frame_control, X_optimal, t);
      const Eigen::Vector3d x_des = T_to_parent.translation();

      // Compute 2d projection
      const auto shape = world.objects()->at(frame_target).collision;
      const auto shape_2d = shape->project_2d();

      // Shift x_des towards center within workspace limits
      Eigen::Vector2d x_des_2d = x_des.head<2>();
      const Eigen::Isometry3d T_parent_to_world = world.T_to_world(*world.frames(t).parent(frame_control), X_optimal, t);
      const auto x_des_world = T_parent_to_world * x_des;
      const Eigen::Vector3d x_des_limit_world = 0.8 * x_des_world.normalized();
      const Eigen::Vector3d x_des_limit = T_parent_to_world.inverse() * x_des_limit_world;
      const Eigen::Vector3d world_origin = T_parent_to_world.inverse().translation();
      for (size_t i = 0; i < 2; i++) {
        double& x = x_des_2d(i);
        // Use the smaller change between percentage and fixed shortening
        const double ratio = 0.75 * x;
        const double fixed = std::abs(x) < 0.03 ? 0. : x - ctrl_utils::Signum(x) * 0.03;
        x = std::abs(ratio) > std::abs(fixed) ? ratio : fixed;

        // Put x on the correct side of the workspace limit
        if (world_origin(i) > x_des_limit(i)) {
          x = std::max(x, x_des_limit(i));
        } else {
          x = std::min(x, x_des_limit(i));
        }
      }

      // Project x_des onto surface
      const auto proj = shape_2d->project_point(Eigen::Isometry2d::Identity(), x_des_2d, false);

      // Difference between projected point and x_des, pointing towards outside
      const Eigen::Vector2d dir_margin = proj.is_inside ? (proj.point - x_des_2d).normalized()
                                                        : (x_des_2d - proj.point).normalized();

      // Point inside edge
      Eigen::Vector2d point_grasp = proj.is_inside ? x_des_2d
                                                   : (proj.point - 0.001 * dir_margin).eval();

      // Intersect ray from point inside to surface
      const ncollide2d::query::Ray ray_outside(point_grasp, dir_margin);
      const auto intersect_outside = shape_2d->toi_and_normal_with_ray(Eigen::Isometry2d::Identity(),
                                                                       ray_outside, false);

      if (intersect_outside) {
        // Find normal pointing towards outside
        const Eigen::Vector2d normal = (dir_margin.dot(intersect_outside->normal) > 0. ? 1. : -1.) *
                                       intersect_outside->normal;

        // Intersect ray from point inside to opposite surface
        const ncollide2d::query::Ray ray_opposite(point_grasp, -normal);
        const auto maybe_toi_opposite = shape_2d->toi_with_ray(Eigen::Isometry2d::Identity(),
                                                               ray_opposite, false);
        if (*maybe_toi_opposite) {
          // Push point towards inside by maximum of 0.04
          const double margin = std::min(0.08, *maybe_toi_opposite);
          point_grasp = proj.point + 0.5 * (ray_opposite.point_at(margin) - proj.point);
        }
      }
      std::cout << point_grasp.transpose() << std::endl;

      constexpr size_t kNumAngles = 8;
      double min_width = std::numeric_limits<double>::infinity();
      double min_angle;
      for (size_t i = 0; i < kNumAngles; i++) {
        // Angle defined along y-axis of gripper
        const double angle = i * M_PI / kNumAngles;
        const Eigen::Vector2d dir(std::sin(angle), -std::cos(angle));

        // Test for intersection from pads of gripper to grasp point
        const ncollide2d::query::Ray ray_1(point_grasp + dir, -dir);
        const ncollide2d::query::Ray ray_2(point_grasp - dir, dir);
        const auto intersect_1 = shape_2d->toi_and_normal_with_ray(Eigen::Isometry2d::Identity(), ray_1, true);
        const auto intersect_2 = shape_2d->toi_and_normal_with_ray(Eigen::Isometry2d::Identity(), ray_2, true);
        if (!intersect_1 || !intersect_2) continue;

        // Make sure force closure is feasible
        const double force_closure = intersect_1->normal.dot(intersect_2->normal);
        if (force_closure > -M_PI / 8.) continue;

        // Find width of grasp
        const double toi_from_x_des_1 = 1. - intersect_1->toi;
        const double toi_from_x_des_2 = 1. - intersect_2->toi;
        const double width = toi_from_x_des_1 + toi_from_x_des_2;
        if (width < min_width) {
          min_width = width;
          min_angle = angle;
        }
      }
      gripper_widths[t] = GripperWidth(min_width);
      if (min_width == std::numeric_limits<double>::infinity()) {
        throw std::runtime_error("ExecuteOpspaceController(): toi failed in grasp.");
      }

      X_final.block<2,1>(0, t) = point_grasp;
      X_final(5, t) = min_angle;
#ifdef REAL_WORLD
    } else if (world.controller(t) == "place") {
      const std::string frame_control = world.control_frame(t);
      const std::string frame_target = world.target_frame(t);
      const logic_opt::Object3& object = world.objects()->at(frame_target);

      if (object.graphics.front().geometry.type == spatial_dyn::Graphics::Geometry::Type::kBox) {
        Eigen::Array2d pos_new = X_final.block<2,1>(0, t).array() - 0.05;
        X_final.block<2,1>(0, t) = (pos_new < 0.).select(0., pos_new);
        continue;
      }
#endif  // REAL_WORLD
    }
  }
  return X_final;
}

std::map<size_t, std::pair<Eigen::MatrixXd, Eigen::MatrixXd>>
ComputeThrowTrajectories(spatial_dyn::ArticulatedBody& ab, const logic_opt::World3& world,
                         const Eigen::MatrixXd& X_optimal) {
  // End-effector parameters
  const Eigen::Isometry3d& T_ee = world.objects()->at(kEeFrame).T_to_parent();
  Eigen::Quaterniond quat_ee(T_ee.linear());
  Eigen::Ref<const Eigen::Vector3d> ee_offset = T_ee.translation();

  std::map<size_t, std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> throw_trajectories;

  // Optimize trajectory
  for (size_t t = 1; t < X_optimal.cols(); t++) {
    if (world.controller(t) == "throw") {
      // Controller frames
      const std::pair<std::string, std::string>& controller_frames = world.controller_frames(t);
      const std::string& control_frame = controller_frames.first;
      const std::string& target_frame = controller_frames.second;

      const Eigen::Isometry3d T_control_to_ee_0 = quat_ee * world.T_to_frame(control_frame, kEeFrame, X_optimal, t-1);
      const Eigen::Isometry3d T_target_to_world_0 = world.T_to_world(control_frame, X_optimal, t-1);
      const Eigen::Isometry3d T_control_to_target_0 = world.T_control_to_target(X_optimal, t-1);
      const Eigen::Isometry3d T_control_to_world_0 = T_target_to_world_0 * T_control_to_target_0;

      const Eigen::Isometry3d T_obj_to_world = T_target_to_world_0 * T_control_to_target_0;
      const Eigen::Isometry3d T_des_to_world_0 = T_target_to_world_0 * T_control_to_target_0 * T_control_to_ee_0.inverse();
      const Eigen::Quaterniond quat_0(T_des_to_world_0.linear());

      // TODO: from perception
      const Eigen::Isometry3d T_control_to_ee = quat_ee * world.T_to_frame(control_frame, kEeFrame, X_optimal, t);
      const Eigen::Isometry3d T_target_to_world = world.T_to_world(target_frame, X_optimal, t);
      const Eigen::Isometry3d T_control_to_target = world.T_control_to_target(X_optimal, t);
      const Eigen::Isometry3d T_control_to_world = T_target_to_world * T_control_to_target;

      // Prepare position-orientation task
      const Eigen::Isometry3d T_des_to_world = T_target_to_world * T_control_to_target * T_control_to_ee.inverse();

      const Eigen::Vector3d offset = T_control_to_ee.translation() + T_ee.translation();

      const Eigen::VectorXd q_0 = spatial_dyn::InverseKinematics(ab, T_target_to_world_0.translation(), quat_0, -1, offset);

      throw_trajectories[t] = logic_opt::ThrowConstraintScp(ab, q_0, T_control_to_world.translation(), offset);
    }
  }

  return throw_trajectories;
}

// Inside control loop for throw:
    // if (ddx_dw.norm() < 0.001) {
    //   if (idx_trajectory < X_final.cols() - 1) {
    //     idx_trajectory++;
    //     if (world.controller(idx_trajectory) == "throw") {
    //       // Controller frames
    //       const std::pair<std::string, std::string>& controller_frames = world.controller_frames(idx_trajectory);
    //       const std::string& control_frame = controller_frames.first;

    //       Eigen::MatrixXd Q, X;
    //       std::tie(Q, X) = throw_trajectories[idx_trajectory];

    //       redis.set(KEY_CONTROL_POS, T_des_to_world.translation());
    //       redis.commit();

    //       const Eigen::VectorXd q_des = Q.col(0);
    //       while (g_runloop) {
    //         timer.Sleep();

    //         std::future<Eigen::Vector2d> fut_kp_kv_joint = redis.get<Eigen::Vector2d>(KEY_KP_KV_JOINT);
    //         auto fut_interaction  = redis.get<redis_gl::simulator::Interaction>(KEY_WEB_INTERACTION);
    //         redis.commit();

    //         Eigen::VectorXd q_err;
    //         const Eigen::VectorXd ddq = ctrl_utils::PdControl(ab.q(), q_des, ab.dq(), fut_kp_kv_joint.get(), 0, &q_err);
    //         if (q_err.norm() < 1e-3) break;
    //         static spatial_dyn::InverseDynamicsOptions options;
    //         options.centrifugal_coriolis = true;
    //         options.gravity = true;
    //         const Eigen::VectorXd tau = spatial_dyn::InverseDynamics(ab, ddq, {}, options);

    //         const redis_gl::simulator::Interaction interaction = fut_interaction.get();
    //         const auto f_ext = redis_gl::simulator::ComputeExternalForces(kModelKeys, ab, interaction);
    //         spatial_dyn::Integrate(ab, tau, timer.dt(), f_ext);

    //         // Update object states
    //         const Eigen::Isometry3d& T_ee_to_world = ab.T_to_world(-1) * T_ee;
    //         UpdateObjectStates(redis, world, sim_objects_abs, idx_trajectory, X_optimal, T_ee_to_world, interaction);

    //         redis.set(KEY_SENSOR_Q, ab.q());
    //         redis.commit();
    //       }

    //       // ctrl_utils::Timer timer_throw(100);
    //       for (size_t t = 0; t < Q.cols(); t++) {
    //         if (t == Q.cols() - 1) {
    //           timer.Sleep();
    //           if (!g_runloop) break;
    //           ab.set_q(Q.col(t));

    //           const Eigen::Isometry3d& T_ee_to_world = ab.T_to_world(-1) * T_ee;
    //           UpdateObjectStates(redis, world, sim_objects_abs, idx_trajectory, X_optimal, T_ee_to_world, interaction);

    //           redis.set(KEY_SENSOR_Q, ab.q());
    //           redis.set(KEY_TRAJ_POS, spatial_dyn::Position(ab));
    //           redis.commit();
    //           continue;
    //         }
    //         for (double dt = 0.; dt < 1.; dt += 0.1) {
    //           timer.Sleep();
    //           // timer_throw.Sleep();
    //           if (!g_runloop) break;

    //           ab.set_q((1 - dt) * Q.col(t) + dt * Q.col(t+1));

    //           const Eigen::Isometry3d& T_ee_to_world = ab.T_to_world(-1) * T_ee;
    //           UpdateObjectStates(redis, world, sim_objects_abs, idx_trajectory, X_optimal, T_ee_to_world, interaction);

    //           redis.set(KEY_SENSOR_Q, ab.q());
    //           redis.set(KEY_TRAJ_POS, spatial_dyn::Position(ab));
    //           redis.commit();
    //         }
    //       }
    //       for (size_t t = 0; t < X.cols(); t++) {
    //         if (t == X.cols() - 1) {
    //           timer.Sleep();
    //           if (!g_runloop) break;

    //           spatial_dyn::RigidBody& rb = sim_objects_abs.at(control_frame);
    //           Eigen::Isometry3d T_to_world = rb.T_to_parent();
    //           T_to_world.translation() = X.col(t);
    //           rb.set_T_to_parent(T_to_world);

    //           redis.set(KEY_OBJECTS_PREFIX + control_frame + "::pos", rb.T_to_parent().translation());
    //           redis.set(KEY_OBJECTS_PREFIX + control_frame + "::ori", Eigen::Quaterniond(rb.T_to_parent().linear()).coeffs());
    //           redis.commit();
    //           continue;
    //         }
    //         for (double dt = 0.; dt < 1.; dt += 0.1) {
    //           timer.Sleep();
    //           // timer_throw.Sleep();
    //           if (!g_runloop) break;

    //           spatial_dyn::RigidBody& rb = sim_objects_abs.at(control_frame);
    //           Eigen::Isometry3d T_to_world = rb.T_to_parent();
    //           T_to_world.translation() = (1 - dt) * X.col(t) + dt * X.col(t+1);
    //           rb.set_T_to_parent(T_to_world);

    //           redis.set(KEY_OBJECTS_PREFIX + control_frame + "::pos", rb.T_to_parent().translation());
    //           redis.set(KEY_OBJECTS_PREFIX + control_frame + "::ori", Eigen::Quaterniond(rb.T_to_parent().linear()).coeffs());
    //           redis.commit();
    //         }
    //       }
    //       idx_trajectory++;
    //     }
    //   } else {
    //     break;
    //   }
    // }


}  // namespace

/**
 * opspace_controller.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 4, 2019
 * Authors: Toki Migimatsu
 */

#include "logic_opt/control/opspace_controller.h"

#include <ctrl_utils/control.h>
#include <ctrl_utils/euclidian.h>
#include <ctrl_utils/filesystem.h>
#include <ctrl_utils/json.h>
#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/timer.h>
#include <spatial_dyn/algorithms/inverse_kinematics.h>
#include <redis_gl/redis_gl.h>

#include "logic_opt/control/throw_constraint_scp.h"

namespace Eigen {

using Vector7d = Eigen::Matrix<double,7,1>;
using Matrix32d = Matrix<double,3,2>;

}  // namespace Eigen

namespace {

// Robot
const std::string kNameRobot = "franka_panda";
const std::string kPathResources = "../resources";
const std::string kPathUrdf = kPathResources + "/" + kNameRobot + "/" + kNameRobot + ".urdf";

const std::string KEY_PREFIX         = "logic_opt::";
const std::string KEY_MODELS_PREFIX  = "dbot::model::";
const std::string KEY_OBJECTS_PREFIX = "dbot::object::";
const std::string KEY_OBJECT_MODELS_PREFIX = KEY_OBJECTS_PREFIX + "model::";
const std::string KEY_TRAJ_PREFIX    = KEY_PREFIX + "trajectory::";

// SET keys
// const std::string KEY_TRAJ_POS       = KEY_TRAJ_PREFIX + kNameRobot + "::pos";
const std::string KEY_SENSOR_Q        = kNameRobot + "::sensor::q";
const std::string KEY_CONTROL_POS     = kNameRobot + "::control::pos";
const std::string KEY_CONTROL_ORI     = kNameRobot + "::control::ori";
const std::string KEY_CONTROL_POS_ERR = kNameRobot + "::control::pos_err";
const std::string KEY_CONTROL_ORI_ERR = kNameRobot + "::control::ori_err";
const std::string KEY_CONTROL_POS_DES = kNameRobot + "::control::pos_des";
const std::string KEY_CONTROL_ORI_DES = kNameRobot + "::control::ori_des";

// Gripper keys
const std::string kNameGripper = "robotiq_gripper";
const std::string KEY_GRIPPER_COMMAND = kNameGripper + "::control::pub::command";
const std::string KEY_GRIPPER_STATUS  = kNameGripper + "::control::pub::status";

// Controller gains
const std::string KEY_KP_KV_POS   = kNameRobot + "::control::kp_kv_pos";
const std::string KEY_KP_KV_ORI   = kNameRobot + "::control::kp_kv_ori";
const std::string KEY_KP_KV_JOINT = kNameRobot + "::control::kp_kv_joint";

const Eigen::Matrix32d kKpKvPos  = (Eigen::Matrix32d() <<
                                    10., 6.,
                                    10., 6.,
                                    10., 6.).finished();
const Eigen::Vector2d kKpKvOri   = Eigen::Vector2d(40., 40.);
const Eigen::Vector2d kKpKvJoint = Eigen::Vector2d(16., 8.);
const double kMaxPosError        = 0.01;
const double kTimerFreq          = 1000.;
const double kGainClickDrag      = 100.;

const Eigen::Vector7d kQHome     = (Eigen::Vector7d() <<
                                    0., -M_PI/6., 0., -5.*M_PI/6., 0., 2.*M_PI/3., 0.).finished();
const Eigen::Vector3d kEeOffset  = Eigen::Vector3d(0., 0., 0.107);  // Without gripper
const Eigen::Vector3d kFrankaGripperOffset  = Eigen::Vector3d(0., 0., 0.1034);
const Eigen::Vector3d kRobotiqGripperOffset = Eigen::Vector3d(0., 0., 0.135);  // Ranges from 0.130 to 0.144
const double kEpsilonPos         = 0.01;
const double kEpsilonOri         = 0.1;
const double kEpsilonVelPos      = 0.001;
const double kEpsilonVelOri      = 0.001;

const std::string kEeFrame = "ee";

// void InitializeWebApp(ctrl_utils::RedisClient& redis, const spatial_dyn::ArticulatedBody& ab,
//                       const std::map<std::string, logic_opt::Object3>& objects, size_t T);

// std::pair<std::map<size_t, spatial_dyn::SpatialForced>, std::map<std::string, std::pair<Eigen::Vector3d, Eigen::Vector3d>>>
// ComputeExternalForces(const spatial_dyn::ArticulatedBody& ab, const nlohmann::json& interaction);

void UpdateObjectStates(ctrl_utils::RedisClient& redis, const logic_opt::World3& world,
                        std::map<std::string, logic_opt::Object3>& sim_objects_abs,
                        size_t idx_trajectory, const Eigen::MatrixXd& X_optimal,
                        const Eigen::Isometry3d& T_ee_to_world,
                        const redis_gl::simulator::Interaction& interaction);

}  // namespace

namespace logic_opt {

void ExecuteOpspaceController(spatial_dyn::ArticulatedBody& ab, const World3& world,
                              const std::shared_ptr<const std::map<std::string, Object3>>& world_objects,
                              const Eigen::MatrixXd& X_optimal, volatile std::sig_atomic_t& g_runloop) {

  // End-effector parameters
  const Eigen::Isometry3d& T_ee = world_objects->at(kEeFrame).T_to_parent();
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

      Eigen::VectorXd q_0 = spatial_dyn::InverseKinematics(ab, T_target_to_world_0.translation(), quat_0, -1, offset);

      throw_trajectories[t] = ThrowConstraintScp(ab, q_0, T_control_to_world.translation(), offset);
    }
  }

  // Set up timer and Redis
  ctrl_utils::RedisClient redis;
  redis.connect();

  // Initialize controller parameters
  Eigen::VectorXd q_des = kQHome;

  const redis_gl::simulator::ModelKeys kModelKeys("dbot");
  redis_gl::simulator::RegisterModelKeys(redis, kModelKeys);
  redis_gl::simulator::RegisterResourcePath(redis, (std::filesystem::current_path() / kPathResources).string());
  // redis_gl::simulator::RegisterRobot(redis, kModelKeys, ab, KEY_SENSOR_Q);
  for (const std::pair<std::string, spatial_dyn::RigidBody>& key_val : *world_objects) {
    const std::string& frame = key_val.first;
    if (frame == kEeFrame) continue;
    const spatial_dyn::RigidBody& object = key_val.second;
    redis_gl::simulator::ObjectModel object_model;
    object_model.name = frame;
    object_model.graphics = object.graphics;
    object_model.key_pos = KEY_OBJECTS_PREFIX + object_model.name + "::pos";
    object_model.key_ori = KEY_OBJECTS_PREFIX + object_model.name + "::ori";
    redis_gl::simulator::RegisterObject(redis, kModelKeys, object_model);
    redis.set(object_model.key_pos, object.T_to_parent().translation());
    redis.set(object_model.key_ori, Eigen::Quaterniond(object.T_to_parent().linear()).coeffs());
  }

  redis.sync_commit();

  ctrl_utils::Timer timer(1000);

  size_t idx_trajectory = 0;
  std::map<std::string, Object3> sim_objects_abs = *world_objects;
  while (g_runloop) {
    timer.Sleep();

    // Get Redis values
    auto fut_interaction = redis.get<redis_gl::simulator::Interaction>(redis_gl::simulator::KEY_INTERACTION);
    std::future<Eigen::Vector7d> fut_q       = redis.get<Eigen::Vector7d>(KEY_SENSOR_Q);
    redis.commit();

    // Compute forward kinematics
    ab.set_q(fut_q.get());
    const Eigen::Vector3d x = spatial_dyn::Position(ab, -1, ee_offset);
    const Eigen::Quaterniond quat = spatial_dyn::Orientation(ab, -1);
    const Eigen::Matrix6Xd& J = spatial_dyn::Jacobian(ab, -1, ee_offset);
    const Eigen::Vector3d dx = J.topRows<3>() * ab.dq();
    const Eigen::Vector3d w = J.bottomRows<3>() * ab.dq();

    // Controller frames
    const std::pair<std::string, std::string>& controller_frames = world.controller_frames(idx_trajectory);
    const std::string& control_frame = controller_frames.first;
    const std::string& target_frame = controller_frames.second;

    // TODO: from perception
    const Eigen::Isometry3d T_control_to_ee = quat_ee * world.T_to_frame(control_frame, kEeFrame, X_optimal, idx_trajectory);
    const Eigen::Isometry3d T_target_to_world = target_frame != World3::kWorldFrame
                                                  ? sim_objects_abs.at(target_frame).T_to_parent()
                                                  : Eigen::Isometry3d::Identity();//world.T_to_world(target_frame, X_optimal, idx_trajectory);
    const Eigen::Isometry3d T_control_to_target = world.T_control_to_target(X_optimal, idx_trajectory);

    // Compute desired pose
    const Eigen::Isometry3d T_des_to_world = T_target_to_world * T_control_to_target * T_control_to_ee.inverse();
    const Eigen::Vector3d x_des = T_des_to_world.translation();
    const Eigen::Quaterniond quat_des = Eigen::Quaterniond(T_des_to_world.linear());

    // Check for convergence
    if ((x - x_des).norm() < kEpsilonPos &&
        ctrl_utils::OrientationError(quat, quat_des).norm() < kEpsilonOri &&
        dx.norm() < kEpsilonVelPos && w.norm() < kEpsilonVelOri) {

      const std::string& controller = world.controller(idx_trajectory);
      std::cout << controller << std::endl;
      if (controller == "place") {
        const std::string status = redis.sync_request<std::string>(KEY_GRIPPER_COMMAND, "o", KEY_GRIPPER_STATUS);
        std::cout << status << std::endl;
      } else if (controller == "pick") {
        const std::string status = redis.sync_request<std::string>(KEY_GRIPPER_COMMAND, "c", KEY_GRIPPER_STATUS);
        std::cout << status << std::endl;
      }

      idx_trajectory++;
      if (idx_trajectory >= X_optimal.cols()) break;
    }

    redis.set(KEY_CONTROL_POS_DES, x_des);
    redis.set(KEY_CONTROL_ORI_DES, quat_des.coeffs());
    redis.commit();

    // Update object states
    const Eigen::Isometry3d& T_ee_to_world = ab.T_to_world(-1) * T_ee;
    UpdateObjectStates(redis, world, sim_objects_abs, idx_trajectory, X_optimal, T_ee_to_world,
                       fut_interaction.get());

    // if (ddx_dw.norm() < 0.001) {
    //   if (idx_trajectory < X_optimal.cols() - 1) {
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

    if ((ab.q().array() != ab.q().array()).any()) break;
  }
  std::cout << "Simulated " << timer.time_sim() << "s in " << timer.time_elapsed() << "s." << std::endl;
  std::cout << std::endl;
}

}  // namespace logic_opt

namespace {

void UpdateObjectStates(ctrl_utils::RedisClient& redis, const logic_opt::World3& world,
                        std::map<std::string, logic_opt::Object3>& sim_objects_abs,
                        size_t idx_trajectory, const Eigen::MatrixXd& X_optimal,
                        const Eigen::Isometry3d& T_ee_to_world,
                        const redis_gl::simulator::Interaction& interaction) {
  const std::string& control_frame = world.controller_frames(idx_trajectory).first;

  const ctrl_utils::Tree<std::string, logic_opt::Frame> frame_tree = world.frames(idx_trajectory);
  for (const auto& key_val : frame_tree.descendants(control_frame)) {
    // Only check frames between control frame and ee
    const std::string& frame = key_val.first;
    if (frame == kEeFrame) continue;
    const Eigen::Isometry3d T_to_ee = world.T_to_frame(frame, kEeFrame, X_optimal, idx_trajectory);
    spatial_dyn::RigidBody& rb = sim_objects_abs.at(frame);
    rb.set_T_to_parent(T_ee_to_world * T_to_ee);

    redis.set(KEY_OBJECTS_PREFIX + frame + "::pos", rb.T_to_parent().translation());
    redis.set(KEY_OBJECTS_PREFIX + frame + "::ori",
              Eigen::Quaterniond(rb.T_to_parent().linear()).coeffs());
  }

  // Handle interaction
  if (interaction.key_object.empty()) return;
  const std::string frame_interaction = interaction.key_object.substr(KEY_OBJECT_MODELS_PREFIX.length());
  if (sim_objects_abs.find(frame_interaction) != sim_objects_abs.end() &&
      !(frame_tree.contains(frame_interaction) && frame_tree.is_descendant(frame_interaction, kEeFrame))) {
    // Ignore forces applied to frames attached to the robot

    const spatial_dyn::RigidBody& rb = sim_objects_abs.at(frame_interaction);

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
      spatial_dyn::RigidBody& rb = sim_objects_abs.at(frame_descendant);

      const Eigen::Isometry3d T_to_world_new = dT * rb.T_to_parent();
      const Eigen::Quaterniond quat_to_world_new = Eigen::Quaterniond(T_to_world_new.linear()).normalized();
      rb.set_T_to_parent(quat_to_world_new, T_to_world_new.translation());
      redis.set(KEY_OBJECTS_PREFIX + frame_descendant + "::pos", T_to_world_new.translation());
      redis.set(KEY_OBJECTS_PREFIX + frame_descendant + "::ori", quat_to_world_new.coeffs());
    }
  }
}

}  // namespace

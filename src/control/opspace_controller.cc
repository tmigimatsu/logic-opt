/**
 * opspace_controller.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 4, 2019
 * Authors: Toki Migimatsu
 */

#include <ctrl_utils/control.h>
#include <ctrl_utils/euclidian.h>
#include <ctrl_utils/filesystem.h>
#include <ctrl_utils/json.h>
#include <ctrl_utils/math.h>
#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/timer.h>
#include <ncollide_cpp/ncollide2d.h>
#include <ncollide_cpp/ncollide3d.h>
#include <redis_gl/redis_gl.h>
#include <spatial_dyn/algorithms/inverse_kinematics.h>

#include <mutex>       // std::mutex

#include "logic_opt/control/opspace_controller.h"

// #define REAL_WORLD

namespace Eigen {

using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix32d = Matrix<double, 3, 2>;

}  // namespace Eigen

namespace {

using ::logic_opt::Frame;
using ::logic_opt::Object;
using ::logic_opt::PlannerControllerInterface;
using ::logic_opt::World;
using ::spatial_opt::Isometry;

// Robot
const std::string kNameRobot = "franka_panda";
const std::string kPathResources = "../resources";
const std::string kPathUrdf =
    kPathResources + "/" + kNameRobot + "/" + kNameRobot + ".urdf";

const std::string KEY_MODELS_PREFIX = "dbot::model::";
const std::string KEY_OBJECTS_PREFIX = "dbot::object::";
const std::string KEY_OBJECT_MODELS_PREFIX = KEY_OBJECTS_PREFIX + "model::";

// SET keys
const std::string KEY_SENSOR_Q = kNameRobot + "::sensor::q";
const std::string KEY_SENSOR_DQ = kNameRobot + "::sensor::dq";
const std::string KEY_CONTROL_DX = kNameRobot + "::control::dx";
const std::string KEY_CONTROL_POS = kNameRobot + "::control::pos";
const std::string KEY_CONTROL_ORI = kNameRobot + "::control::ori";
const std::string KEY_CONTROL_POS_ERR = kNameRobot + "::control::pos_err";
const std::string KEY_CONTROL_ORI_ERR = kNameRobot + "::control::ori_err";
const std::string KEY_CONTROL_POS_DES = kNameRobot + "::control::pos_des";
const std::string KEY_CONTROL_ORI_DES = kNameRobot + "::control::ori_des";
const std::string KEY_COLLISION_POS = kNameRobot + "::collision::pos";
const std::string KEY_COLLISION_ACTIVE = kNameRobot + "::collision::active";
const std::string KEY_COLLISION_KP_KV = kNameRobot + "::collision::kp_kv";

// Gripper keys
const std::string kNameGripper = "robotiq_gripper";
const std::string KEY_GRIPPER_COMMAND =
    kNameGripper + "::control::pub::command";
const std::string KEY_GRIPPER_STATUS = kNameGripper + "::control::pub::status";

// Controller gains
const std::string KEY_CONTROL_POS_TOL = kNameRobot + "::control::pos_tol";
const std::string KEY_CONTROL_ORI_TOL = kNameRobot + "::control::ori_tol";
const std::string KEY_CONTROL_POS_VEL_TOL =
    kNameRobot + "::control::pos_vel_tol";
const std::string KEY_CONTROL_ORI_VEL_TOL =
    kNameRobot + "::control::ori_vel_tol";
const std::string KEY_CONTROL_POS_ERR_MAX =
    kNameRobot + "::control::pos_err_max";
const std::string KEY_CONTROL_ORI_ERR_MAX =
    kNameRobot + "::control::ori_err_max";

#ifndef REAL_WORLD
const std::string KEY_ROBOTIQ_Q = kNameGripper + "::sensor::q";
const std::string KEY_SENSOR_POS = kNameRobot + "::sensor::pos";
const std::string KEY_SENSOR_ORI = kNameRobot + "::sensor::ori";
#endif  // REAL_WORLD

const double kTimerFreq = 1000.;

const Eigen::Vector7d kQHome = (Eigen::Vector7d() << 0., -M_PI / 6., 0.,
                                -5. * M_PI / 6., 0., 2. * M_PI / 3., 0.)
                                   .finished();
const Eigen::Vector3d kEeOffset =
    Eigen::Vector3d(0., 0., 0.107);  // Without gripper
const Eigen::Vector3d kFrankaGripperOffset = Eigen::Vector3d(0., 0., 0.1034);
const Eigen::Vector3d kRobotiqGripperOffset =
    Eigen::Vector3d(0., 0., 0.140);  // Ranges from 0.130 to 0.144
const Eigen::Vector3d kEpsilonPos = Eigen::Vector3d(0.02, 0.02, 0.01);
const double kEpsilonOri = 0.1;
const double kEpsilonVelPos = 0.02;
const double kEpsilonVelOri = 0.05;
const double kMaxErrorPos = 80 * 0.04;
const double kMaxErrorOri = 80 * M_PI / 20;

const std::string kEeFrame = "ee";

#ifdef REAL_WORLD
const std::unordered_set<std::string> kTrackedObjects = {
    "hook",  "box_1",         "box_2",           "box_3",
    "shelf", "platform_left", "platform_middle", "platform_right"};
#else   // REAL_WORLD
const std::unordered_set<std::string> kTrackedObjects = {};
#endif  // REAL_WORLD

std::vector<int> gripper_widths;

bool HasConverged(const spatial_dyn::ArticulatedBody& ab,
                  const Eigen::Vector3d& x_des,
                  const Eigen::Quaterniond& quat_des,
                  const Eigen::Vector3d& ee_offset,
                  const Eigen::Vector3d& eps_pos, double eps_vel_pos,
                  double eps_ori, double eps_vel_ori);

Eigen::MatrixXd PlanGrasps(const World& world,
                           const Eigen::MatrixXd& X_optimal);

void InitializeRedisKeys(
    ctrl_utils::RedisClient& redis,
    const std::map<std::string, Object>& world_poses,
    const std::unordered_set<std::string>& tracked_objects);

#ifndef REAL_WORLD
void SimulateGripper(ctrl_utils::RedisClient& redis, size_t pos) {
  static const double kQMax = 0.813;
  const double x = pos * kQMax / 255.;
  Eigen::Matrix<double, 10, 1> q;
  q << x, -x, x, x, -x, x, 0., 0., 0., 0.;
  redis.sync_set(KEY_ROBOTIQ_Q, q);
}

void SimulateGripper(ctrl_utils::RedisClient& redis, const std::string& cmd) {
  if (cmd == "c") {
    SimulateGripper(redis, 255);
  } else if (cmd == "o") {
    SimulateGripper(redis, 0);
  }
}
#endif  // REAL_WORLD

/**
 * Struct to keep track of controlled frames (attached to the robot) and
 * environment frames (other objects).
 *
 * Assume collisions can only happen between a controlled frame and an
 * environment frame.
 */
struct CollisionFrames {
  CollisionFrames() = default;
  CollisionFrames(const World& world, size_t t_collision);

  std::unordered_set<std::string> controlled_frames;
  std::unordered_set<std::string> environment_frames;
};

/**
 * Commands the robot to the initial pose.
 *
 * Blocks until the robot pose converges.
 */
void InitializeRobotPose(
    ctrl_utils::RedisClient& redis, spatial_dyn::ArticulatedBody& ab,
    const Eigen::Vector3d& ee_offset,
    const Isometry& T_des_to_world,
    std::shared_ptr<PlannerControllerInterface> shared_memory);

/**
 * Manages Redis variables related to control.
 */
struct RedisControl {
  static RedisControl Get(ctrl_utils::RedisClient& redis);

  void Set(ctrl_utils::RedisClient& redis) const;

  // Values to get.
  std::future<Eigen::Vector7d> q;
  std::future<Eigen::Vector7d> dq;

  // Values to set.
  Eigen::Vector3d x_des;
  Eigen::Quaterniond quat_des;
  Eigen::Vector3d x_err;
  Eigen::Vector3d ori_err;
  Eigen::Vector6d dx;
};

/**
 * Manages Redis variables related to perception.
 */
struct RedisPerception {
  using FuturePose =
      std::pair<std::future<Eigen::Vector3d>, std::future<Eigen::Quaterniond>>;

  static RedisPerception Get(
      ctrl_utils::RedisClient& redis,
      const std::unordered_set<std::string>& tracked_objects);

  /**
   * Updates untracked object poses.
   *
   * @param redis Redis client.
   * @param tracked_objects Objects tracked by perception.
   * @param world_poses Updated world poses.
   */
  static void Set(ctrl_utils::RedisClient& redis,
                  const std::unordered_set<std::string>& tracked_objects,
                  const std::map<std::string, Object>& world_poses);

  std::future<redis_gl::simulator::Interaction> interaction;
  std::map<std::string, FuturePose> object_poses;
};

/**
 * Computes end-effector frames.
 *
 * Frames:
 *   ee: Urdf end-effector.
 *   op: Operational point used in opspace (translate by ee_offset).
 *   grasp: Used in frame optimization (translate by ee_offset, identity
 *          orientation at home configuration).
 *
 * @returns (T_op_to_ee, T_grasp_to_ee) pair.
 */
std::pair<Isometry, Isometry> ComputeEeFrames(
    const spatial_dyn::ArticulatedBody& ab_0);

Isometry get_T_to_world(const std::map<std::string, Object>& world_poses,
                        const std::string& frame) {
  return world_poses.find(frame) == world_poses.end()
             ? Isometry::Identity()
             : world_poses.at(frame).T_to_parent();
}

/**
 * Computes the desired pose of the op frame with respect to the world frame..
 *
 * @returns (T_des_op_to_world, T_op_to_world) pair.
 */
std::pair<Isometry, Isometry> ComputeDesiredOpPose(
    const spatial_dyn::ArticulatedBody& ab, const Isometry& T_op_to_ee,
    const Isometry& T_control_to_world, const Isometry& T_target_to_world,
    const Isometry& T_des_control_to_target);

/**
 * Updates object poses from perception and control.
 */
void UpdateObjectPoses(
    const spatial_dyn::ArticulatedBody& ab, const Isometry& T_grasp_to_ee,
    std::map<std::string, RedisPerception::FuturePose>& updated_poses,
    const ctrl_utils::Tree<std::string, Frame>& tree,
    const std::string& control_frame,
    std::map<std::string, Object>* world_poses);

/**
 * Handles interactions from redis_gl.
 */
void HandleInteraction(const redis_gl::simulator::Interaction& interaction,
                       const ctrl_utils::Tree<std::string, Frame>& tree,
                       const std::string& control_frame,
                       std::map<std::string, Object>* world_poses);

/**
 * Executes the switch controller for the given timestep.
 */
void ExecuteSwitchController(ctrl_utils::RedisClient& redis,
                             const std::string& controller,
                             size_t idx_trajectory);

/**
 * Handles collisions between controlled and environment frames.
 */
void HandleCollisions(const CollisionFrames& collision_frames,
                      const std::map<std::string, Object>& world_objects);

}  // namespace

namespace logic_opt {

void OpspaceController(
    const std::map<std::string, Object>& world_poses_0,
    const spatial_dyn::ArticulatedBody& ab_0,
    std::shared_ptr<PlannerControllerInterface> shared_memory) {
  std::map<std::string, Object> world_poses = world_poses_0;

  // Compute end-effector parameters.
  // Frames:
  //   ee: Urdf end-effector.
  //   op: Operational point used in opspace (translate by ee_offset).
  //   grasp: Used in frame optimization (translate by ee_offset, identity
  //          orientation at home configuration).
  const auto ee_frames = ComputeEeFrames(ab_0);
  const Isometry& T_op_to_ee = ee_frames.first;
  const Isometry& T_grasp_to_ee = ee_frames.second;

  // Initialize robot to home configuration.
  spatial_dyn::ArticulatedBody ab = ab_0;
  ab.set_q(kQHome);
  const Isometry T_home_ee_to_world = ab.T_to_world(-1);

  // Set up redis.
  ctrl_utils::RedisClient redis;
  redis.connect();
#ifdef REAL_WORLD
  ctrl_utils::RedisClient redis_robot;
  redis_robot.connect("172.24.69.103");
#else   // REAL_WORLD
  ctrl_utils::RedisClient& redis_robot = redis;
#endif  // REAL_WORLD

  // Set object frames in Redis.
  InitializeRedisKeys(redis, world_poses, kTrackedObjects);

  // Command robot to home position.
  {
    const Isometry T_home_op_to_world =
        T_home_ee_to_world * T_op_to_ee;
    InitializeRobotPose(redis, ab, T_op_to_ee.translation(), T_home_op_to_world,
                        shared_memory);
  }

  // Wait for first trajectory optimizer plan
  shared_memory->WaitForOptimizationResult();

  // Update ee frame with current robot configuration.
  {
    ab.set_q(redis.sync_get<Eigen::Vector7d>(KEY_SENSOR_Q));
    const Isometry T_ee_to_world = ab.T_to_world(-1);
    const Isometry T_grasp_to_world = T_ee_to_world * T_grasp_to_ee;
    world_poses.at(kEeFrame).set_T_to_parent(T_grasp_to_world.normalized());
  }

  Eigen::MatrixXd X_final;
  World world;
  CollisionFrames collision_frames;

  // Start executing the plan at timestep 1, since the constraint at t=0 simply
  // sets the pose of the end-effector for optimization.
  size_t idx_trajectory = 1;
  size_t t_action = 1;

  // Iterate on trajectory optimizer plans.
  ctrl_utils::Timer timer(kTimerFreq);
  while (shared_memory->g_runloop) {
    timer.Sleep();

    // Get updated optimization result.
    std::optional<PlannerControllerInterface::OptimizationResult> opt_result =
        shared_memory->TryGetOptimizationResult();
    if (opt_result) {
      // Switch optimization plans.
      const World& prev_world =
          world.num_timesteps() == 0 ? opt_result->world : world;
      const int t_remaining = prev_world.num_timesteps() - idx_trajectory;

      world = std::move(opt_result->world);

      X_final = PlanGrasps(world, opt_result->X_optimal);

#ifdef VERBOSE
      std::cout << "UPDATE PLAN" << std::endl
                << "X final:" << std::endl
                << X_final << std::endl
                << "t_action: " << t_action << std::endl
                << "idx_trajectory: " << idx_trajectory << " -> ";
#endif  // VERBOSE

      idx_trajectory = world.num_timesteps() - t_remaining;

#ifdef VERBOSE
      std::cout << idx_trajectory << std::endl
                << "control: " << world.control_frame(idx_trajectory) << ", "
                << "target: " << world.target_frame(idx_trajectory)
                << std::endl;
#endif  // VERBOSE

      collision_frames = CollisionFrames(world, idx_trajectory);
    }

    // Get controller frames.
    const std::pair<std::string, std::string>& controller_frames =
        world.controller_frames(idx_trajectory);
    const std::string& control_frame = controller_frames.first;
    const std::string& target_frame = controller_frames.second;

    // Get Redis values.
    RedisControl redis_control = RedisControl::Get(redis);
    RedisPerception perception = RedisPerception::Get(redis, kTrackedObjects);

    // Update robot state.
    ab.set_q(redis_control.q.get());
    ab.set_dq(redis_control.dq.get());
    if ((ab.q().array() != ab.q().array()).any()) break;

    // Update world poses.
    const ctrl_utils::Tree<std::string, Frame>& tree =
        world.frames(idx_trajectory);
    UpdateObjectPoses(ab, T_grasp_to_ee, perception.object_poses, tree,
                      control_frame, &world_poses);
    HandleInteraction(perception.interaction.get(), tree, control_frame,
                      &world_poses);

    shared_memory->SetExecutionUpdate(world_poses, tree, t_action);
    RedisPerception::Set(redis, kTrackedObjects, world_poses);

    // Compute desired pose
    // TODO: Avoid perception control/target frame during push for stability
    // const Eigen::Quaterniond quat_control_to_world =
    //     world_0.controller(idx_trajectory) == "push_2"
    //         ? world_0.Orientation(control_frame, World::kWorldFrame, X_final,
    //                             idx_trajectory)
    //         : perception.quat_control.get();
    // const Eigen::Quaterniond quat_target_to_world =
    //     world_0.controller(idx_trajectory) == "push_1"
    //         ? world_0.Orientation(target_frame, World::kWorldFrame, X_final,
    //                             idx_trajectory)
    //         : perception.quat_target.get();

    const auto& Ts_op = ComputeDesiredOpPose(
        ab, T_op_to_ee, get_T_to_world(world_poses, control_frame),
        get_T_to_world(world_poses, target_frame),
        world.T_control_to_target(X_final, idx_trajectory));
    const Isometry& T_des_op_to_world = Ts_op.first;
    const Isometry& T_op_to_world = Ts_op.second;

    // Check for convergence
    const auto x_des = T_des_op_to_world.translation();
    const auto quat_des = T_des_op_to_world.rotation();
    if (HasConverged(ab, x_des, quat_des, T_op_to_ee.translation(), kEpsilonPos,
                     kEpsilonVelPos, kEpsilonOri, kEpsilonVelOri)) {
      ExecuteSwitchController(redis, world.controller(idx_trajectory),
                              idx_trajectory);

      idx_trajectory++;
      t_action++;
#ifdef VERBOSE
      std::cout << "INCREMENT TIMESTEP" << std::endl
                << "t_action: " << t_action << std::endl
                << "idx_trajectory: " << idx_trajectory << std::endl
                << "control: " << control_frame << ", target: " << target_frame
                << std::endl;
#endif  // VERBOSE

      if (t_action >= X_final.cols()) {
        // Stop planner when finished.
        // shared_memory->g_runloop = false;
        break;
      }

      collision_frames = CollisionFrames(world, idx_trajectory);
      continue;
    }

    HandleCollisions(collision_frames, world_poses);

    // Set Redis control.
    redis_control.x_des = T_des_op_to_world.translation();
    redis_control.quat_des = T_des_op_to_world.rotation();
    redis_control.x_err = T_op_to_world.translation() - x_des;
    redis_control.ori_err =
        ctrl_utils::OrientationError(T_op_to_world.rotation(), quat_des);
    redis_control.dx =
        spatial_dyn::Jacobian(ab, -1, T_op_to_ee.translation()) * ab.dq();
    redis_control.Set(redis);
  }

  std::cout << "Simulated " << timer.time_sim() << "s in "
            << timer.time_elapsed() << "s." << std::endl;
  std::cout << std::endl;
}

}  // namespace logic_opt

namespace {

int GripperWidth(double width) { return 224 / 0.085 * (0.085 - width) + 16; };

Eigen::MatrixXd PlanGrasps(const World& world,
                           const Eigen::MatrixXd& X_optimal) {
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
      const Object& object = world.objects()->at(frame_target);

      if (object.graphics.front().geometry.type ==
          spatial_dyn::Graphics::Geometry::Type::kBox) {
        X_final.block<2, 1>(0, t).setZero();
        X_final(1, t) = 0.01;
        // Convert quaternion to axis angle representation and back
        Eigen::Quaterniond quat(X_final.block<4, 1>(3, t));
        Eigen::AngleAxisd X_aa_t = Eigen::AngleAxisd(quat);
        Eigen::Vector3d X_aa_t_vec = X_aa_t.axis() * X_aa_t.angle();
        X_aa_t_vec(2) = 0;
        Eigen::AngleAxisd X_aa_t_updated =
            Eigen::AngleAxisd(X_aa_t_vec.norm(), X_aa_t_vec.normalized());
        Eigen::Quaterniond X_quat_t = Eigen::Quaterniond(X_aa_t_updated);
        X_final.block<4, 1>(3, t) = X_quat_t.coeffs();
        // X_final(5, t) = 0;
        gripper_widths[t] =
            GripperWidth(object.graphics.front().geometry.scale(1));
        continue;
      }

      const Isometry T_to_parent =
          world.T_to_parent(frame_control, X_optimal, t);
      const Eigen::Vector3d x_des = T_to_parent.translation();

      // Compute 2d projection
      const auto shape = world.objects()->at(frame_target).collision;
      const auto shape_2d = shape->project_2d();

      // Shift x_des towards center within workspace limits
      Eigen::Vector2d x_des_2d = x_des.head<2>();
      const Isometry T_parent_to_world = world.T_to_world(
          *world.frames(t).parent(frame_control), X_optimal, t);
      const auto x_des_world = T_parent_to_world * x_des;
      const Eigen::Vector3d x_des_limit_world = 0.8 * x_des_world.normalized();
      const Eigen::Vector3d x_des_limit =
          T_parent_to_world.inverse() * x_des_limit_world;
      const Eigen::Vector3d world_origin =
          T_parent_to_world.inverse().translation();
      for (size_t i = 0; i < 2; i++) {
        double& x = x_des_2d(i);
        // Use the smaller change between percentage and fixed shortening
        const double ratio = 0.75 * x;
        const double fixed =
            std::abs(x) < 0.03 ? 0. : x - ctrl_utils::Signum(x) * 0.03;
        x = std::abs(ratio) > std::abs(fixed) ? ratio : fixed;

        // Put x on the correct side of the workspace limit
        if (world_origin(i) > x_des_limit(i)) {
          x = std::max(x, x_des_limit(i));
        } else {
          x = std::min(x, x_des_limit(i));
        }
      }

      // Project x_des onto surface
      const auto proj = shape_2d->project_point(Eigen::Isometry2d::Identity(),
                                                x_des_2d, false);

      // Difference between projected point and x_des, pointing towards outside
      const Eigen::Vector2d dir_margin =
          proj.is_inside ? (proj.point - x_des_2d).normalized()
                         : (x_des_2d - proj.point).normalized();

      // Point inside edge
      Eigen::Vector2d point_grasp =
          proj.is_inside ? x_des_2d : (proj.point - 0.001 * dir_margin).eval();

      // Intersect ray from point inside to surface
      const ncollide2d::query::Ray ray_outside(point_grasp, dir_margin);
      const auto intersect_outside = shape_2d->toi_and_normal_with_ray(
          Eigen::Isometry2d::Identity(), ray_outside, false);

      if (intersect_outside) {
        // Find normal pointing towards outside
        const Eigen::Vector2d normal =
            (dir_margin.dot(intersect_outside->normal) > 0. ? 1. : -1.) *
            intersect_outside->normal;

        // Intersect ray from point inside to opposite surface
        const ncollide2d::query::Ray ray_opposite(point_grasp, -normal);
        const auto maybe_toi_opposite = shape_2d->toi_with_ray(
            Eigen::Isometry2d::Identity(), ray_opposite, false);
        if (*maybe_toi_opposite) {
          // Push point towards inside by maximum of 0.04
          const double margin = std::min(0.08, *maybe_toi_opposite);
          point_grasp =
              proj.point + 0.5 * (ray_opposite.point_at(margin) - proj.point);
        }
      }

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
        const auto intersect_1 = shape_2d->toi_and_normal_with_ray(
            Eigen::Isometry2d::Identity(), ray_1, true);
        const auto intersect_2 = shape_2d->toi_and_normal_with_ray(
            Eigen::Isometry2d::Identity(), ray_2, true);
        if (!intersect_1 || !intersect_2) continue;

        // Make sure force closure is feasible
        const double force_closure =
            intersect_1->normal.dot(intersect_2->normal);
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
        throw std::runtime_error(
            "ExecuteOpspaceController(): toi failed in grasp.");
      }

      X_final.block<2, 1>(0, t) = point_grasp;
      // X_final(5, t) = min_angle;
      // Convert quaternion to axis angle representation and back
      Eigen::Quaterniond quat(X_final.block<4, 1>(3, t));
      Eigen::AngleAxisd X_aa_t = Eigen::AngleAxisd(quat);
      Eigen::Vector3d X_aa_t_vec = X_aa_t.axis() * X_aa_t.angle();
      X_aa_t_vec(2) = min_angle;
      Eigen::AngleAxisd X_aa_t_updated =
          Eigen::AngleAxisd(X_aa_t_vec.norm(), X_aa_t_vec.normalized());
      Eigen::Quaterniond X_quat_t = Eigen::Quaterniond(X_aa_t_updated);
      X_final.block<4, 1>(3, t) = X_quat_t.coeffs();
#ifdef REAL_WORLD
    } else if (world.controller(t) == "place") {
      const std::string frame_control = world.control_frame(t);
      const std::string frame_target = world.target_frame(t);
      const Object& object = world.objects()->at(frame_target);

      if (object.graphics.front().geometry.type ==
          spatial_dyn::Graphics::Geometry::Type::kBox) {
        Eigen::Array2d pos_new = X_final.block<2, 1>(0, t).array() - 0.05;
        X_final.block<2, 1>(0, t) = (pos_new < 0.).select(0., pos_new);
        continue;
      }
#endif  // REAL_WORLD
    }
  }
  return X_final;
}

bool HasPoseConverged(const spatial_dyn::ArticulatedBody& ab,
                      const Eigen::Vector3d& x_des,
                      const Eigen::Quaterniond& quat_des,
                      const Eigen::Vector3d& ee_offset,
                      const Eigen::Vector3d& eps_pos, double eps_ori) {
  const Eigen::Vector3d x = spatial_dyn::Position(ab, -1, ee_offset);
  const Eigen::Quaterniond quat =
      ctrl_utils::NearQuaternion(spatial_dyn::Orientation(ab, -1), quat_des);
  // std::cout << "x_err: " << (x - x_des).transpose() << " vs. "
  //           << eps_pos.transpose() << std::endl;
  // std::cout << "quat_err: ||"
  //           << ctrl_utils::OrientationError(quat, quat_des).transpose()
  //           << "|| = " << ctrl_utils::OrientationError(quat, quat_des).norm()
  //           << " vs. " << eps_ori << std::endl;
  return ((x - x_des).array().abs() < eps_pos.array()).all() &&
         ctrl_utils::OrientationError(quat, quat_des).norm() < eps_ori;
}

bool HasVelocityConverged(const spatial_dyn::ArticulatedBody& ab,
                          const Eigen::Vector3d& ee_offset, double eps_vel_pos,
                          double eps_vel_ori) {
  const Eigen::Matrix6Xd& J = spatial_dyn::Jacobian(ab, -1, ee_offset);
  const Eigen::Vector3d dx = J.topRows<3>() * ab.dq();
  const Eigen::Vector3d w = J.bottomRows<3>() * ab.dq();
  // std::cout << "dx_err: ||" << dx.transpose() << "|| = " << dx.norm() << "
  // vs. "
  //           << eps_vel_pos << std::endl;
  // std::cout << "w_err: ||" << w.transpose() << "|| = " << w.norm() << " vs. "
  //           << eps_vel_ori << std::endl;
  return dx.norm() < eps_vel_pos && w.norm() < eps_vel_ori;
}

bool HasConverged(const spatial_dyn::ArticulatedBody& ab,
                  const Eigen::Vector3d& x_des,
                  const Eigen::Quaterniond& quat_des,
                  const Eigen::Vector3d& ee_offset,
                  const Eigen::Vector3d& eps_pos, double eps_vel_pos,
                  double eps_ori, double eps_vel_ori) {
  return HasPoseConverged(ab, x_des, quat_des, ee_offset, eps_pos, eps_ori) &&
         HasVelocityConverged(ab, ee_offset, eps_vel_pos, eps_vel_ori);
}

void InitializeRedisKeys(
    ctrl_utils::RedisClient& redis,
    const std::map<std::string, Object>& world_poses,
    const std::unordered_set<std::string>& tracked_objects) {
  // Initialize controller parameters
  redis.set(KEY_CONTROL_POS_TOL, 3 * kEpsilonPos);
  redis.set(KEY_CONTROL_POS_VEL_TOL, kEpsilonVelPos);
  redis.set(KEY_CONTROL_ORI_TOL, kEpsilonOri);
  redis.set(KEY_CONTROL_ORI_VEL_TOL, kEpsilonVelOri);
  redis.set(KEY_CONTROL_POS_ERR_MAX, kMaxErrorPos);
  redis.set(KEY_CONTROL_ORI_ERR_MAX, kMaxErrorOri);

  const redis_gl::simulator::ModelKeys kModelKeys("dbot");
  redis_gl::simulator::ClearModelKeys(redis, kModelKeys, true);
  redis_gl::simulator::RegisterModelKeys(redis, kModelKeys);
  redis_gl::simulator::RegisterResourcePath(
      redis, (std::filesystem::current_path() / kPathResources).string());

  // Initialize objects not tracked by DBOT
  for (const auto& key_val : world_poses) {
    const std::string& frame = key_val.first;
    if (tracked_objects.find(frame) != tracked_objects.end()) continue;
    const Object& object = key_val.second;
    redis_gl::simulator::ObjectModel object_model;

    object_model.name = frame;
    object_model.graphics = object.graphics;
    object_model.key_pos = KEY_OBJECTS_PREFIX + object_model.name + "::pos";
    object_model.key_ori = KEY_OBJECTS_PREFIX + object_model.name + "::ori";
    redis_gl::simulator::RegisterObject(redis, kModelKeys, object_model);
    const Isometry& T_obj_to_world = key_val.second.T_to_parent();
    std::cout << "Register: " << frame << " at ["
              << T_obj_to_world.translation().transpose() << "]" << std::endl;
    redis.set(object_model.key_pos, T_obj_to_world.translation());
    redis.set(object_model.key_ori, T_obj_to_world.rotation());
  }

  // Initialize world pose for consistency
  // const std::string KEY_WORLD_PREFIX =
  //     KEY_OBJECTS_PREFIX + World::kWorldFrame + "::";
  // redis.set(KEY_WORLD_PREFIX + "pos", Eigen::Vector3d::Zero());
  // redis.set(KEY_WORLD_PREFIX + "ori", Eigen::Quaterniond::Identity());

#ifndef REAL_WORLD
  {
    const auto kModelKeys = redis_gl::simulator::ModelKeys("robotiq_gripper");
    const std::filesystem::path kPathResources =
        (std::filesystem::current_path() / ".." / "resources").string();
    const std::filesystem::path kPathUrdf =
        kPathResources / "robotiq_2f_85" / "robotiq_2f_85.urdf";
    const spatial_dyn::ArticulatedBody gripper =
        spatial_dyn::urdf::LoadModel(kPathUrdf.string(), "robotiq_2f_85", true);
    redis_gl::simulator::RegisterModelKeys(redis, kModelKeys);
    redis_gl::simulator::RegisterResourcePath(redis, kPathResources.string());
    redis_gl::simulator::RegisterRobot(redis, kModelKeys, gripper,
                                       KEY_ROBOTIQ_Q, KEY_SENSOR_POS,
                                       KEY_SENSOR_ORI);
  }
#endif  // REAL_WORLD

  redis.sync_commit();
}

void InitializeRobotPose(
    ctrl_utils::RedisClient& redis, spatial_dyn::ArticulatedBody& ab,
    const Eigen::Vector3d& ee_offset,
    const Isometry& T_des_to_world,
    std::shared_ptr<PlannerControllerInterface> shared_memory) {
  // Initialize to home position of robot
  std::cout << T_des_to_world << std::endl;
  redis.set(KEY_CONTROL_POS_DES, T_des_to_world.translation());
  redis.set(KEY_CONTROL_ORI_DES, T_des_to_world.rotation());
  redis.commit();

  ctrl_utils::Timer timer(kTimerFreq);
  while (shared_memory->g_runloop) {
    timer.Sleep();

    ab.set_q(redis.sync_get<Eigen::Vector7d>(KEY_SENSOR_Q));
    if (HasConverged(ab, T_des_to_world.translation(),
                     T_des_to_world.rotation(), ee_offset, kEpsilonPos,
                     kEpsilonVelPos, kEpsilonOri, kEpsilonVelOri)) {
      break;
    }
  }
}

CollisionFrames::CollisionFrames(const World& world,
                                 size_t t_collision) {
  const ctrl_utils::Tree<std::string, Frame>& tree = world.frames(t_collision);
  const std::string& control = world.control_frame(t_collision);

  for (const auto& key_val : tree.nodes()) {
    const std::string& frame = key_val.first;

    // if (frame == "wall") continue;
    if (frame == World::kWorldFrame || !world.objects()->at(frame).collision) {
      continue;
    }

    // Descendants of the control frame are fixed to the ee and can't collide
    if (tree.is_descendant(frame, control)) {
      controlled_frames.insert(frame);
    } else {
      environment_frames.insert(frame);
    }
  }
}

RedisControl RedisControl::Get(ctrl_utils::RedisClient& redis) {
  RedisControl rc;
  rc.q = redis.get<Eigen::Vector7d>(KEY_SENSOR_Q);
  rc.dq = redis.get<Eigen::Vector7d>(KEY_SENSOR_DQ);
  return rc;
}

void RedisControl::Set(ctrl_utils::RedisClient& redis) const {
#ifdef REAL_WORLD
  redis_robot.set(KEY_CONTROL_POS_DES, x_des);
  redis_robot.set(KEY_CONTROL_ORI_DES, quat_des.coeffs());
  redis_robot.commit();
  redis.set(KEY_CONTROL_POS, spatial_dyn::Position(ab, -1, ee_offset));
  redis.set(KEY_CONTROL_ORI, spatial_dyn::Orientation(ab).coeffs());
#endif  // REAL_WORLD
  if (ctrl_utils::IsNan(x_des) || ctrl_utils::IsNan(quat_des)) {
    std::stringstream ss;
    ss << "NaN control command" << std::endl
       << "  x_des: " << x_des.transpose() << std::endl
       << "  quat_des: " << quat_des.coeffs().transpose() << std::endl;
    throw std::runtime_error(ss.str());
  }
  redis.set(KEY_CONTROL_POS_DES, x_des);
  redis.set(KEY_CONTROL_ORI_DES, quat_des);
  redis.set(KEY_CONTROL_POS_ERR, x_err);
  redis.set(KEY_CONTROL_ORI_ERR, ori_err);
  redis.set(KEY_CONTROL_DX, dx);
  redis.commit();
}

RedisPerception RedisPerception::Get(
    ctrl_utils::RedisClient& redis,
    const std::unordered_set<std::string>& tracked_objects) {
  RedisPerception rp;
  rp.interaction = redis.get<redis_gl::simulator::Interaction>(
      redis_gl::simulator::KEY_INTERACTION);

  for (const std::string& name_object : tracked_objects) {
    const std::string KEY_OBJ_PREFIX = KEY_OBJECTS_PREFIX + name_object + "::";
    rp.object_poses.emplace(
        name_object,
        std::make_pair(redis.get<Eigen::Vector3d>(KEY_OBJ_PREFIX + "pos"),
                       redis.get<Eigen::Quaterniond>(KEY_OBJ_PREFIX + "ori")));
  }
  redis.commit();
  return rp;

  // TODO: Integrate real world
#ifdef REAL_WORLD
  // auto fut_sensor_pos =
  //     redis_robot.get<std::string>("franka_panda::sensor::pos");
  // auto fut_sensor_ori =
  //     redis_robot.get<std::string>("franka_panda::sensor::ori");
  // auto fut_eps_pos = redis.get<Eigen::Vector3d>(KEY_CONTROL_POS_TOL);
  // auto fut_eps_ori = redis.get<double>(KEY_CONTROL_ORI_TOL);
  // auto fut_eps_vel_pos = redis.get<double>(KEY_CONTROL_POS_VEL_TOL);
  // auto fut_eps_vel_ori = redis.get<double>(KEY_CONTROL_ORI_VEL_TOL);
#endif  // REAL_WORLD
#ifdef REAL_WORLD
  // redis_robot.commit();
#endif  // REAL_WORLD
#ifdef REAL_WORLD
  // redis.set(KEY_SENSOR_Q, ab.q());
  // redis.set(KEY_SENSOR_DQ, ab.dq());
  // redis.set("franka_panda::sensor::pos", fut_sensor_pos.get());
  // redis.set("franka_panda::sensor::ori", fut_sensor_ori.get());
  // redis.commit();
#endif
}

void RedisPerception::Set(
    ctrl_utils::RedisClient& redis,
    const std::unordered_set<std::string>& tracked_objects,
    const std::map<std::string, Object>& world_poses) {
  for (const auto& key_val : world_poses) {
    const std::string& frame = key_val.first;

    // Skip objects tracked by perception.
    if (tracked_objects.find(frame) != tracked_objects.end()) continue;

    const Isometry& T_obj_to_world = world_poses.at(frame).T_to_parent();
    const std::string KEY_OBJ_PREFIX = KEY_OBJECTS_PREFIX + frame + "::";
    redis.set(KEY_OBJ_PREFIX + "pos", T_obj_to_world.translation());
    redis.set(KEY_OBJ_PREFIX + "ori", T_obj_to_world.rotation());
  }
  redis.commit();
}

std::pair<Isometry, Isometry> ComputeEeFrames(
    const spatial_dyn::ArticulatedBody& ab_0) {
  // grasp: Identity orientation at home, ee_offset
  // op: ee_offset
  // ee: urdf ee
  spatial_dyn::ArticulatedBody ab = ab_0;
  ab.set_q(kQHome);
  const Eigen::Vector3d ee_offset = kEeOffset + kRobotiqGripperOffset;
  Isometry T_op_to_ee =
      Isometry::Translation(ee_offset);
  const Isometry T_grasp_to_op =
      Isometry::Rotation(spatial_dyn::Orientation(ab).inverse());
  Isometry T_grasp_to_ee = T_op_to_ee * T_grasp_to_op;

  return {std::move(T_op_to_ee), std::move(T_grasp_to_ee)};
}

std::pair<Isometry, Isometry> ComputeDesiredOpPose(
    const spatial_dyn::ArticulatedBody& ab, const Isometry& T_op_to_ee,
    const Isometry& T_control_to_world, const Isometry& T_target_to_world,
    const Isometry& T_des_control_to_target) {
  const Isometry T_ee_to_world = ab.T_to_world(-1);
  Isometry T_op_to_world = T_ee_to_world * T_op_to_ee;

  // T_op_to_control is constant regardless of control's pose.
  const Isometry T_op_to_control = T_control_to_world.inverse() * T_op_to_world;

  // Transform desired pose at control point to operational point.
  const Isometry T_des_op_to_target = T_des_control_to_target * T_op_to_control;

  // Transform from target frame to world.
  Isometry T_des_op_to_world = T_target_to_world * T_des_op_to_target;

#ifdef VERBOSE
  std::cout << "T_op_to_ee: " << T_op_to_ee << std::endl;
  std::cout << "T_control_to_world: " << T_control_to_world << std::endl;
  std::cout << "T_target_to_world: " << T_target_to_world << std::endl;
  std::cout << "T_des_control_to_target: " << T_des_control_to_target
            << std::endl;
  std::cout << "T_ee_to_world: " << T_ee_to_world << std::endl;
  std::cout << "T_des_op_to_target: " << T_des_op_to_target << std::endl;
  std::cout << "T_des_op_to_world: " << T_des_op_to_world << std::endl;
  std::cout << std::endl;
#endif  // VERBOSE

  return {std::move(T_des_op_to_world), std::move(T_op_to_world)};
}

void UpdateObjectPoses(
    const spatial_dyn::ArticulatedBody& ab, const Isometry& T_grasp_to_ee,
    std::map<std::string, RedisPerception::FuturePose>& updated_poses,
    const ctrl_utils::Tree<std::string, Frame>& tree,
    const std::string& control_frame,
    std::map<std::string, Object>* world_poses) {
  // Update poses from perception.
#ifdef VERBOSE
  std::cout << "Perception updates: " << std::endl;
#endif  // VERBOSE
  for (auto& key_val : updated_poses) {
    const std::string& frame = key_val.first;
    RedisPerception::FuturePose& fut_pos_quat = key_val.second;
    Object& obj = world_poses->at(frame);
#ifdef VERBOSE
    std::cout << "  " << frame << ": " << obj.T_to_parent() << " -> ";
#endif  // VERBOSE
    obj.set_T_to_parent(fut_pos_quat.second.get().normalized(),
                        fut_pos_quat.first.get());
#ifdef VERBOSE
    std::cout << obj.T_to_parent() << std::endl;
#endif  // VERBOSE
  }

  // Update objects in the robot's end-effector (descendants of control frame).
  const Isometry T_ee_to_world = ab.T_to_world(-1);
  const Isometry T_grasp_to_world = T_ee_to_world * T_grasp_to_ee;
  const Isometry& T_prev_grasp_to_world =
      world_poses->at(kEeFrame).T_to_parent();
  const Isometry dT = T_grasp_to_world * T_prev_grasp_to_world.inverse();
#ifdef VERBOSE
  std::cout << "T_grasp_to_world: " << T_grasp_to_world
            << ", T_prev_grasp_to_world: " << T_prev_grasp_to_world
            << ", dT: " << dT << std::endl;

  std::cout << "Manipulation updates: " << std::endl;
#endif  // VERBOSE
  for (const auto& key_val : tree.descendants(control_frame)) {
    const std::string& frame = key_val.first;

    // Skip objects tracked by perception.
    if (updated_poses.find(frame) != updated_poses.end()) continue;

    Object& obj = world_poses->at(frame);
#ifdef VERBOSE
    std::cout << "  " << frame << ": " << obj.T_to_parent() << " -> ";
#endif  // VERBOSE
    obj.set_T_to_parent((dT * obj.T_to_parent()).normalized());
#ifdef VERBOSE
    std::cout << obj.T_to_parent() << std::endl;
#endif  // VERBOSE
  }
}

void HandleInteraction(const redis_gl::simulator::Interaction& interaction,
                       const ctrl_utils::Tree<std::string, Frame>& tree,
                       const std::string& control_frame,
                       std::map<std::string, Object>* world_poses) {
  if (interaction.key_object.empty()) return;

  const std::string frame =
      interaction.key_object.substr(KEY_OBJECT_MODELS_PREFIX.length());

  // Ignore irrelevant objects.
  if (world_poses->find(frame) == world_poses->end() || !tree.contains(frame)) {
    return;
  }

  // Ignore forces applied to frames between robot and ee (should only be ee).
  if (tree.is_descendant(frame, kEeFrame)) return;

  // Compute click force.
  const Object& obj = world_poses->at(frame);
  Eigen::Vector3d pos = obj.T_to_parent().translation();
  Eigen::Quaterniond quat = obj.T_to_parent().rotation();
  redis_gl::simulator::ClickAdjustPose(interaction, &pos, &quat);
  const Isometry T_obj_to_world(quat, pos);
  const Isometry dT = T_obj_to_world * obj.T_to_parent().inverse();

  // Update pose of all descendant frames.
  const bool is_in_ee = tree.is_descendant(frame, control_frame);
  for (const auto& key_val : tree.descendants(frame)) {
    const std::string& frame_descendant = key_val.first;

    // Skip frames attached to control frame.
    if (!is_in_ee && tree.is_descendant(frame_descendant, control_frame)) {
      continue;
    }

    // Skip frames between robot and ee (should only be ee).
    if (tree.is_descendant(frame_descendant, kEeFrame)) continue;

    Object& obj = world_poses->at(frame_descendant);

    obj.set_T_to_parent((dT * obj.T_to_parent()).normalized());
  }
}

void ExecuteSwitchController(ctrl_utils::RedisClient& redis,
                             const std::string& controller,
                             size_t idx_trajectory) {
  static size_t t_pick;
  static Eigen::Vectord<World::kDof> X_saved;
  std::string gripper_status = "done";

  if (controller == "place") {
    std::cout << "Opening gripper... " << std::flush;
#ifdef REAL_WORLD
    gripper_status = redis.sync_request<std::string>(KEY_GRIPPER_COMMAND, "o",
                                                     KEY_GRIPPER_STATUS);
#else   // REAL_WORLD
    std::cout << "Opening gripper... " << std::flush;
    SimulateGripper(redis, "o");
    gripper_status = "done";
#endif  // REAL_WORLD
    std::cout << "Done." << std::endl;
  } else if (controller == "pick") {
    std::cout << "Closing gripper to " << gripper_widths[idx_trajectory]
              << "... " << std::endl;
#ifdef REAL_WORLD
    gripper_status = redis.sync_request<std::string>(
        KEY_GRIPPER_COMMAND, gripper_widths[idx_trajectory],
        KEY_GRIPPER_STATUS);
#else   // REAL_WORLD
    SimulateGripper(redis, gripper_widths[idx_trajectory]);
    gripper_status = "done";
#endif  // REAL_WORLD
    std::cout << "Done." << std::endl;
  }

  if (gripper_status != "done") {
    throw std::runtime_error("Gripper command failed: " + gripper_status + ".");
  }
}

void HandleCollisions(const CollisionFrames& collision_frames,
                      const std::map<std::string, Object>& world_objects) {
  // bool is_collision_detected = false;
  double dist_proximity = 5e-2;
  Eigen::Vector3d dx_collision;
  Eigen::Vector2d kp_kv(0., 20.);
  for (const std::string& ee : collision_frames.controlled_frames) {
    const Object& rb_ee = world_objects.at(ee);
    for (const std::string& object : collision_frames.environment_frames) {
      const Object& rb = world_objects.at(object);
      const auto contact = ncollide3d::query::contact(
          rb_ee.T_to_parent().eigen(), *rb_ee.collision,
          rb.T_to_parent().eigen(), *rb.collision, dist_proximity);
      if (!contact) continue;
      // is_collision_detected = true;

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

  // TODO: Collision detection
  /*
  if (is_collision_detected && world_0.controller(idx_trajectory) != "push_1" &&
      world_0.controller(idx_trajectory) != "push_2") {
    const Eigen::Vector3d x = spatial_dyn::Position(ab, -1, ee_offset);
    redis_robot.mset(
        std::make_pair(KEY_COLLISION_POS, dx_collision + x),
        std::make_pair(KEY_COLLISION_ACTIVE, is_collision_detected),
        std::make_pair(KEY_COLLISION_KP_KV, kp_kv));
    if (kp_kv(0) != 0. &&
        HasVelocityConverged(ab, ee_offset, 0.000001, 0.000001) &&
        world.controller(idx_trajectory) != "push") {
      Eigen::Vector3d x_traj = Eigen::Vector3d::Zero();
      sim.mtx_plan.lock();
      if (sim.X_plan.cols() > 1) x_traj = sim.X_plan.block<3, 1>(0, 1);
      sim.mtx_plan.unlock();
      if (x_traj.squaredNorm() > 0.) x_traj = T_target_to_world * x_traj;
      // std::cout << ctrl_utils::OrthogonalProjection(x_traj,
      // x_des).transpose() << std::endl;
      x_des = x_traj;
      x_des += ctrl_utils::OrthogonalProjection(x_traj, x_des);
    }
  } else {
    redis_robot.set(KEY_COLLISION_ACTIVE, false);
  }
  */
  // for (const auto& key_val : *sim.objects) {
  //   const Object& rb = key_val.second;
  //   const auto projection = rb.collision->project_point(rb.T_to_parent(),
  //   x, true); if ((x - projection.point).norm() >= 5e-2) continue;
  //   is_collision_detected = true;
  //   redis.mset(std::make_pair("franka_panda::collision::pos",
  //   projection.point),
  //              std::make_pair("franka_panda::collision::active",
  //              is_collision_detected),
  //              std::make_pair("franka_panda::collision::kp_kv",
  //              Eigen::Vector2d(-10., 100.)));
  //   break;
  // }
}

}  // namespace

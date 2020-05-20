/**
 * main.cc
 *
 * Copyright 2019. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: January 09, 2019
 * Authors: Toki Migimatsu
 */

#include <atomic>     // std::atomic
#include <cmath>      // std::sin, std::cos
#include <csignal>    // std::signal, std::sig_atomic_t
#include <exception>  // std::exception
#include <future>     // std::future
#include <iostream>   // std::cout
#include <mutex>      // std::mutex
#include <string>     // std::string

#include <spatial_dyn/spatial_dyn.h>
#include <ctrl_utils/atomic.h>
#include <ctrl_utils/control.h>
#include <ctrl_utils/euclidian.h>
#include <ctrl_utils/filesystem.h>
#include <ctrl_utils/json.h>
#include <ctrl_utils/optional.h>
#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/timer.h>

#define USE_WEB_APP

#ifdef USE_WEB_APP
#include <redis_gl/redis_gl.h>
#endif

namespace Eigen {

using Vector7d = Matrix<double,7,1>;
using Matrix32d = Matrix<double,3,2>;

}  // namespace Eigen

namespace {

volatile std::sig_atomic_t g_runloop = true;
void stop(int) {
  g_runloop = false;
}

// Redis keys
const std::string KEY_PREFIX         = "franka_panda::";
const std::string KEY_TRAJ_PREFIX    = KEY_PREFIX + "trajectory::";

// GET keys
const std::string KEY_SENSOR_Q      = KEY_PREFIX + "sensor::q";
const std::string KEY_SENSOR_DQ     = KEY_PREFIX + "sensor::dq";
const std::string KEY_SENSOR_POS    = KEY_PREFIX + "sensor::pos";
const std::string KEY_SENSOR_ORI    = KEY_PREFIX + "sensor::ori";
const std::string KEY_MODEL_EE      = KEY_PREFIX + "model::inertia_ee";
const std::string KEY_DRIVER_STATUS = KEY_PREFIX + "driver::status";
const std::string KEY_COLLISION_ACTIVE = KEY_PREFIX + "collision::active";
const std::string KEY_COLLISION_POS    = KEY_PREFIX + "collision::pos";
const std::string KEY_COLLISION_KP_KV  = KEY_PREFIX + "collision::kp_kv";

// SET keys
const std::string KEY_CONTROL_TAU     = KEY_PREFIX + "control::tau";
const std::string KEY_CONTROL_MODE    = KEY_PREFIX + "control::mode";
const std::string KEY_CONTROL_POS_DES = KEY_PREFIX + "control::pos_des";
const std::string KEY_CONTROL_ORI_DES = KEY_PREFIX + "control::ori_des";
const std::string KEY_CONTROL_POS     = KEY_PREFIX + "control::pos";
const std::string KEY_CONTROL_ORI     = KEY_PREFIX + "control::ori";
const std::string KEY_CONTROL_POS_ERR = KEY_PREFIX + "control::pos_err";
const std::string KEY_CONTROL_ORI_ERR = KEY_PREFIX + "control::ori_err";
const std::string KEY_TRAJ_POS        = KEY_TRAJ_PREFIX + "pos";

const std::string kNameApp            = "simulator";
const std::string KEY_WEB_INTERACTION = "webapp::" + kNameApp + "::interaction";

// SUB keys
const std::string KEY_PUB_COMMAND = KEY_PREFIX + "control::pub::command";

// PUB keys
const std::string KEY_PUB_STATUS = KEY_PREFIX + "control::pub::status";

// Controller gains
const std::string KEY_KP_KV_POS   = KEY_PREFIX + "control::kp_kv_pos";
const std::string KEY_KP_KV_ORI   = KEY_PREFIX + "control::kp_kv_ori";
const std::string KEY_KP_KV_JOINT = KEY_PREFIX + "control::kp_kv_joint";
const std::string KEY_POS_ERR_MAX = KEY_PREFIX + "control::pos_err_max";
const std::string KEY_ORI_ERR_MAX = KEY_PREFIX + "control::ori_err_max";

// Controller parameters
const Eigen::Vector3d kEeOffset  = Eigen::Vector3d(0., 0., 0.107);  // Without gripper
const Eigen::Vector3d kFrankaGripperOffset  = Eigen::Vector3d(0., 0., 0.1034);
const Eigen::Vector3d kRobotiqGripperOffset = Eigen::Vector3d(0., 0., 0.144);  // Ranges from 0.130 to 0.144
const Eigen::VectorXd kQHome     = (Eigen::Vector7d() <<
                                    0., -M_PI/6., 0., -5.*M_PI/6., 0., 2.*M_PI/3., 0.).finished();
const Eigen::Matrix32d kKpKvPos  = (Eigen::Matrix32d() <<
                                    80., 12.,
                                    80., 12.,
                                    80., 12.).finished();
const Eigen::Vector2d kKpKvOri   = Eigen::Vector2d(80., 10.);
const Eigen::Vector2d kKpKvJoint = Eigen::Vector2d(5., 10.);
const double kTimerFreq          = 1000.;
const double kGainKeyPressPos    = 0.1 / kTimerFreq;
const double kGainKeyPressOri    = 0.3 / kTimerFreq;
const double kGainClickDrag      = 100.;
const double kMaxErrorPos        = kKpKvPos(0, 0) * 0.05;
const double kMaxErrorOri        = kKpKvOri(0, 0) * M_PI / 20;
const double kEpsilonPos         = 0.05;
const double kEpsilonOri         = 0.2;
const double kEpsilonVelPos      = 0.005;
const double kEpsilonVelOri      = 0.005;
const double kMaxForce           = 100.;
const Eigen::Array3d kMinPos     = Eigen::Array3d(0.2, -0.4, 0.01);
const Eigen::Array3d kMaxPos     = Eigen::Array3d(0.7, 0.4, 0.5);
const std::chrono::milliseconds kTimePubWait = std::chrono::milliseconds{1000};

const spatial_dyn::opspace::InverseDynamicsOptions kOpspaceOptions = []() {
  spatial_dyn::opspace::InverseDynamicsOptions options;
  options.svd_epsilon = 0.01;
  options.f_acc_max = kMaxForce;
  return options;
}();

struct Args {

  Args(int argc, char* argv[]) {
    size_t idx_required = 0;
    for (int i = 1; i < argc; i++) {
      const std::string arg(argv[i]);
      if (arg == "--no-gripper") {
        gripper = false;
      } else if (arg == "--robotiq") {
        robotiq = true;
        gripper = false;
      } else if (arg == "--friction") {
        friction = true;
      } else if (idx_required == 0) {
        path_urdf = arg;
        idx_required++;
      }
    }
  }

  std::string path_urdf = "../resources/franka_panda/franka_panda.urdf";
  bool gripper = false;
  bool robotiq = true;
  bool friction = false;

};

bool IsOrientationFeasible(const spatial_dyn::ArticulatedBody& ab,
                           const Eigen::Quaterniond& quat, const Eigen::Quaterniond& quat_des) {
  const Eigen::Quaterniond quat_des_near = ctrl_utils::NearQuaternion(quat_des, quat);
  const Eigen::Quaterniond quat_des_to_ee = quat.inverse() * quat_des_near;
  const Eigen::Vector3d aa_des_to_ee = ctrl_utils::OrientationError(Eigen::Quaterniond::Identity(), quat_des_to_ee);
  const double q_min = ab.rigid_bodies(-1).joint().q_min();
  const double q_max = ab.rigid_bodies(-1).joint().q_max();
  const double q_des = ab.q(-1) - aa_des_to_ee(2);
  return q_des > q_min && q_des < q_max;
}

bool IsPublishCommandTimedOut(const std::chrono::steady_clock::time_point& t_pub) {
  const auto t_since_pub = std::chrono::steady_clock::now() - t_pub;
  const auto ms_since_pub = std::chrono::duration_cast<std::chrono::milliseconds>(t_since_pub);
  return ms_since_pub > kTimePubWait;
}

bool IsPoseConverged(const Eigen::Vector3d& x_err, const Eigen::Vector3d& ori_err,
                     double epsilon_pos, double epsilon_ori) {
  return x_err.norm() < epsilon_pos && ori_err.norm() < epsilon_ori;
}

bool IsVelocityConverged(const Eigen::Vector3d& dx, const Eigen::Vector3d& w,
                         double epsilon_vel_pos, double epsilon_vel_ori) {
  return dx.norm() < epsilon_vel_pos && w.norm() < epsilon_vel_ori;
}

struct PublishCommand {

  enum class Type {
    kUndefined,
    kPose,
    kDeltaPose
  };

  std::optional<Eigen::Vector3d> pos;
  std::optional<Eigen::Quaterniond> quat;
  Type type;

  std::optional<double> pos_tolerance;
  std::optional<double> ori_tolerance;

};

void from_json(const nlohmann::json& json, PublishCommand::Type& type) {
  const std::string str_type = json.get<std::string>();
  if (str_type == "pose") {
    type = PublishCommand::Type::kPose;
  } else if (str_type == "delta_pose") {
    type = PublishCommand::Type::kDeltaPose;
  } else {
    type = PublishCommand::Type::kUndefined;
  }
}

void from_json(const nlohmann::json& json, PublishCommand& command) {
  if (json.find("type") != json.end()) {
    command.type = json["type"].get<PublishCommand::Type>();
  } else {
    command.type = PublishCommand::Type::kUndefined;
  }

  if (json.find("pos") != json.end()) {
    command.pos = json["pos"].get<Eigen::Vector3d>();
  } else {
    command.type = PublishCommand::Type::kUndefined;
  }

  if (json.find("quat") != json.end()) {
    command.quat = Eigen::Quaterniond(json["quat"].get<Eigen::Vector4d>());
  } else if (json.find("rot") != json.end()) {
    command.quat = Eigen::Quaterniond(json["rot"].get<Eigen::Matrix3d>());
  }

  if (json.find("pos_tolerance") != json.end()) {
    command.pos_tolerance = json["pos_tolerance"].get<double>();
  }

  if (json.find("ori_tolerance") != json.end()) {
    command.pos_tolerance = json["ori_tolerance"].get<double>();
  }
}

Eigen::Vector3d CollisionAvoidance(const Eigen::Vector3d& x, const Eigen::Vector3d x_collision,
                                   const Eigen::Vector3d& dx, const Eigen::Vector2d& kp_kv_collision) {
  Eigen::Vector3d x_collision_err = x - x_collision;
  const Eigen::Vector3d dir_collision = -x_collision_err.normalized();
  if (x_collision_err.norm() > 0.1) {
    // Normalize collision error for safety
    x_collision_err = 0.1 * x_collision_err.normalized();
  }
  const double dx_dot_dir_collision = dx.dot(dir_collision);
  Eigen::Vector3d ddx_collision = Eigen::Vector3d::Zero();
  if (dx_dot_dir_collision > 0.) {
    // Decelerate if velocity is in the direction of collision
    ddx_collision += -kp_kv_collision(1) * dx_dot_dir_collision * dir_collision;
  }
  // Repulsive field if ee is penetrating
  ddx_collision += -kp_kv_collision(0) * x_collision_err;
  // Normalize acceleration for safety
  if (ddx_collision.norm() > 10.) {
    ddx_collision = ddx_collision.normalized();
  }
  return ddx_collision;
}

}  // namespace

int main(int argc, char* argv[]) {
  std::cout << "Usage:" << std::endl
            << "\t./franka_panda_opspace <franka_panda.urdf> [--friction]" << std::endl
            << std::endl;

  const Args args(argc, argv);

  spatial_dyn::IntegrationOptions integration_options;
  integration_options.friction = args.friction;

  // Create timer and Redis client
  ctrl_utils::Timer timer(kTimerFreq);
  ctrl_utils::RedisClient redis_client;
  redis_client.connect();
  cpp_redis::subscriber redis_sub;
  redis_sub.connect();

  // Load robot
  std::cout << "Loading urdf: " << args.path_urdf << std::endl;
  spatial_dyn::ArticulatedBody ab = spatial_dyn::urdf::LoadModel(args.path_urdf);
  ab.set_q(kQHome);
  ab.set_dq(Eigen::VectorXd::Zero(ab.dof()));

  // Set gripper
  const Eigen::Vector3d gripper_offset = args.gripper ? kFrankaGripperOffset
                                                      : args.robotiq ? kRobotiqGripperOffset
                                                                     : Eigen::Vector3d::Zero();
  const Eigen::Vector3d ee_offset = kEeOffset + gripper_offset;
  if (!args.gripper) {
    std::vector<spatial_dyn::Graphics>& graphics = const_cast<std::vector<spatial_dyn::Graphics>&>(ab.rigid_bodies(-1).graphics);
    for (size_t i = 1, num_graphics = graphics.size(); i < num_graphics; i++) {
      graphics.pop_back();
    }
  }
  // ab.set_inertia_compensation(Eigen::Vector3d(0.2, 0.1, 0.1));
  // ab.set_stiction_coefficients(Eigen::Vector3d(0.8, 0.8, 0.6));
  // ab.set_stiction_activations(Eigen::Vector3d(0.1, 0.1, 0.1));

  // Initialize controller parameters
  Eigen::VectorXd q_des = kQHome;

  // Desired pose needs to be locked when written by main thread or read by
  // Redis subscribe thread
  // std::mutex mtx_des;
  // Eigen::Vector3d x_des       = spatial_dyn::Position(ab, -1, ee_offset);
  // Eigen::Quaterniond quat_des = spatial_dyn::Orientation(ab);
  ctrl_utils::Atomic<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> pose_des =
      std::pair<Eigen::Vector3d, Eigen::Quaterniond>{ spatial_dyn::Position(ab, -1, ee_offset),
                                                      spatial_dyn::Orientation(ab) };

  // Initialize Redis keys
#ifdef USE_WEB_APP
  const std::filesystem::path path_resources = (std::filesystem::current_path() /
                                                std::filesystem::path(args.path_urdf)).parent_path();
  redis_gl::simulator::RegisterResourcePath(redis_client, path_resources.string());

  redis_gl::simulator::ModelKeys model_keys("franka_panda");
  redis_gl::simulator::RegisterModelKeys(redis_client, model_keys);

  redis_gl::simulator::RegisterRobot(redis_client, model_keys, ab, KEY_SENSOR_Q);

  redis_gl::simulator::RegisterTrajectory(redis_client, model_keys, "x_ee", KEY_CONTROL_POS);

  spatial_dyn::Graphics x_des_marker("x_des_marker");
  x_des_marker.geometry.type = spatial_dyn::Graphics::Geometry::Type::kSphere;
  x_des_marker.geometry.radius = 0.01;
  redis_gl::simulator::RegisterObject(redis_client, model_keys, x_des_marker, KEY_CONTROL_POS_DES, KEY_CONTROL_ORI_DES);
  spatial_dyn::Graphics x_control_marker("x_control_marker");
  x_control_marker.geometry.type = spatial_dyn::Graphics::Geometry::Type::kSphere;
  x_control_marker.geometry.radius = 0.01;
  redis_gl::simulator::RegisterObject(redis_client, model_keys, x_control_marker, KEY_CONTROL_POS, KEY_CONTROL_ORI);
  spatial_dyn::Graphics x_collision_marker("x_collision_marker");
  x_collision_marker.geometry.type = spatial_dyn::Graphics::Geometry::Type::kSphere;
  x_collision_marker.geometry.radius = 0.01;
  redis_gl::simulator::RegisterObject(redis_client, model_keys, x_collision_marker, KEY_COLLISION_POS, KEY_CONTROL_ORI_DES);
#endif  // USE_WEB_APP

  redis_client.set(KEY_SENSOR_Q, ab.q());
  redis_client.set(KEY_SENSOR_DQ, ab.dq());
  redis_client.set(KEY_SENSOR_POS, spatial_dyn::Position(ab, -1, kEeOffset));
  redis_client.set(KEY_SENSOR_ORI, spatial_dyn::Orientation(ab).coeffs());
  redis_client.set(KEY_KP_KV_POS, kKpKvPos);
  redis_client.set(KEY_KP_KV_ORI, kKpKvOri);
  redis_client.set(KEY_KP_KV_JOINT, kKpKvJoint);
  redis_client.set(KEY_POS_ERR_MAX, kMaxErrorPos);
  redis_client.set(KEY_ORI_ERR_MAX, kMaxErrorOri);
  redis_client.set(KEY_CONTROL_POS_DES, spatial_dyn::Position(ab, -1, ee_offset));
  redis_client.set(KEY_CONTROL_ORI_DES, spatial_dyn::Orientation(ab).coeffs());
  redis_client.set(KEY_COLLISION_ACTIVE, false);
  redis_client.set(KEY_COLLISION_POS, Eigen::Vector3d(0.37, 0., 0.3));
  redis_client.set(KEY_COLLISION_KP_KV, Eigen::Vector2d(0., 100.));
  redis_client.sync_commit();

  // Initialize subscriber variables
  ctrl_utils::Atomic<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> pose_des_pub = pose_des;
  std::atomic_bool is_pub_available = { false };
  std::atomic<double> epsilon_pos = { kEpsilonPos };
  std::atomic<double> epsilon_ori = { kEpsilonOri };

  redis_sub.subscribe(KEY_PUB_COMMAND,
      [&pose_des, &pose_des_pub, &is_pub_available, &epsilon_pos, &epsilon_ori](
      const std::string& key, const std::string& val) {
    std::cout << "SUB " << key << ": " << val << std::endl;

    // Get current des pose
    Eigen::Vector3d x_des;
    Eigen::Quaterniond quat_des;
    std::tie(x_des, quat_des) = pose_des.load();

    // Parse json command
    const PublishCommand command = nlohmann::json::parse(val).get<PublishCommand>();
    if (command.type == PublishCommand::Type::kPose) {
      if (command.pos) x_des = *command.pos;
      if (command.quat) quat_des = *command.quat;
    } else if (command.type == PublishCommand::Type::kDeltaPose) {
      if (command.pos) x_des += *command.pos;
      if (command.quat) quat_des = quat_des * *command.quat;
    } else {
      std::cerr << "cpp_redis::subscribe_callback(" << key << ", " << val
                << "): Invalid command type." << std::endl;
      return;
    }
    if (command.pos_tolerance) epsilon_pos = *command.pos_tolerance;
    if (command.ori_tolerance) epsilon_ori = *command.ori_tolerance;

    pose_des_pub = std::pair<Eigen::Vector3d, Eigen::Quaterniond>{ x_des, quat_des };
    is_pub_available = true;
  });
  redis_sub.commit();

  // Get end-effector model from driver
  try {
    const nlohmann::json json_ee = redis_client.sync_get<nlohmann::json>(KEY_MODEL_EE);
    ab.ReplaceLoad(json_ee.get<spatial_dyn::SpatialInertiad>());
  } catch (...) {}

  // Create signal handler
  std::signal(SIGINT, &stop);

  bool is_pub_waiting = false;
  auto t_pub = std::chrono::steady_clock::now();
  bool is_pose_des_new = true;

  Eigen::Quaterniond quat_0 = Eigen::Quaterniond::Identity();  // Track previous quat for continuity

  try {
    while (g_runloop) {
      // Wait for next loop
      timer.Sleep();

      // Get Redis values
      std::future<Eigen::Matrix32d> fut_kp_kv_pos  = redis_client.get<Eigen::Matrix32d>(KEY_KP_KV_POS);
      std::future<Eigen::Vector2d> fut_kp_kv_ori   = redis_client.get<Eigen::Vector2d>(KEY_KP_KV_ORI);
      std::future<Eigen::Vector2d> fut_kp_kv_joint = redis_client.get<Eigen::Vector2d>(KEY_KP_KV_JOINT);
      std::future<Eigen::Vector3d> fut_x_des       = redis_client.get<Eigen::Vector3d>(KEY_CONTROL_POS_DES);
      std::future<Eigen::Vector4d> fut_quat_des    = redis_client.get<Eigen::Vector4d>(KEY_CONTROL_ORI_DES);
      std::future<double> fut_max_err_pos = redis_client.get<double>(KEY_POS_ERR_MAX);
      std::future<double> fut_max_err_ori = redis_client.get<double>(KEY_ORI_ERR_MAX);
      std::future<bool> fut_is_collision_active = redis_client.get<bool>(KEY_COLLISION_ACTIVE);
#ifdef USE_WEB_APP
      std::future<redis_gl::simulator::Interaction> fut_interaction =
          redis_client.get<redis_gl::simulator::Interaction>(redis_gl::simulator::KEY_INTERACTION);
#endif  // USE_WEB_APP

      redis_client.commit();

      std::future<Eigen::Vector3d> fut_x_collision;
      std::future<Eigen::Vector2d> fut_kp_kv_collision;
      const bool is_collision_active = fut_is_collision_active.get();
      if (is_collision_active) {
        fut_x_collision = redis_client.get<Eigen::Vector3d>(KEY_COLLISION_POS);
        fut_kp_kv_collision = redis_client.get<Eigen::Vector2d>(KEY_COLLISION_KP_KV);
        redis_client.commit();
      }

      // Update desired pose from Redis
      Eigen::Vector3d x_des = fut_x_des.get();
      x_des = (x_des.array() < kMinPos).select(kMinPos, x_des);
      x_des = (x_des.array() > kMaxPos).select(kMaxPos, x_des);
      Eigen::Quaterniond quat_des = Eigen::Quaterniond(fut_quat_des.get());

      // Check for PUB commands
      if (is_pub_available) {
        // Get desired pose from pub
        std::tie(x_des, quat_des) = pose_des_pub.load();
        is_pub_available = false;

        // Set flag to respond to pub
        is_pub_waiting = true;
        t_pub = std::chrono::steady_clock::now();

        // Set flag to publish new desired pose
        is_pose_des_new = true;
      } else {
        // Set desired pose for pub
        pose_des = std::pair<Eigen::Vector3d, Eigen::Quaterniond>{ x_des, quat_des };
      }

      // Compute Jacobian
      const Eigen::Matrix6Xd& J = spatial_dyn::Jacobian(ab, -1, ee_offset);

      // Compute position PD control
      Eigen::Vector3d x_err;
      const Eigen::Vector3d x = spatial_dyn::Position(ab, -1, ee_offset);
      const Eigen::Vector3d dx = J.topRows<3>() * ab.dq();
      Eigen::Vector3d ddx = ctrl_utils::PdControl(x, x_des, dx, fut_kp_kv_pos.get(),
                                                  fut_max_err_pos.get(), &x_err);
      if (is_collision_active) {
        ddx += CollisionAvoidance(x, fut_x_collision.get(), dx, fut_kp_kv_collision.get());
      }

      // Compute orientation PD control
      Eigen::Vector3d ori_err;
      const Eigen::Quaterniond quat = ctrl_utils::NearQuaternion(spatial_dyn::Orientation(ab), quat_0);
      quat_0 = quat;
      if (IsOrientationFeasible(ab, quat, quat_des)) {
        quat_des = ctrl_utils::NearQuaternion(quat_des, quat);
      } else {
        if (is_pub_waiting) {
          quat_des = quat;

          redis_client.publish(KEY_PUB_STATUS, "infeasible");
          is_pub_waiting = false;
          std::cout << "PUB " << KEY_PUB_STATUS << ": infeasible" << std::endl;
        } else {
          quat_des = ctrl_utils::FarQuaternion(quat_des, quat);
        }
      }
      const Eigen::Vector3d w = J.bottomRows<3>() * ab.dq();
      const Eigen::Vector3d dw = ctrl_utils::PdControl(quat, quat_des, w, fut_kp_kv_ori.get(),
                                                       fut_max_err_ori.get(), &ori_err);

      // Compute opspace torques
      Eigen::MatrixXd N;
      Eigen::VectorXd tau_cmd;
      if (spatial_dyn::opspace::IsSingular(ab, J, kOpspaceOptions.svd_epsilon)) {
        // If robot is at a singularity, control position only
        tau_cmd = spatial_dyn::opspace::InverseDynamics(ab, J.topRows<3>(), ddx, &N, {}, kOpspaceOptions);
      } else {
        // Control position and orientation
        const Eigen::Vector6d ddx_dw = (Eigen::Vector6d() << ddx, dw).finished();;
        tau_cmd = spatial_dyn::opspace::InverseDynamics(ab, J, ddx_dw, &N, {}, kOpspaceOptions);
      }

      // Add joint task in nullspace
      static const Eigen::MatrixXd J_null = Eigen::MatrixXd::Identity(ab.dof() - 1, ab.dof());
      const Eigen::VectorXd ddq = ctrl_utils::PdControl(ab.q(), q_des, ab.dq(), fut_kp_kv_joint.get());
      tau_cmd += spatial_dyn::opspace::InverseDynamics(ab, J_null, ddq.head(ab.dof() - 1), &N);

      // Add friction compensation
      if (args.friction) {
        tau_cmd += spatial_dyn::Friction(ab, tau_cmd);
      }

      // Add gravity compensation
      tau_cmd += spatial_dyn::Gravity(ab);

      // Send control torques
      redis_client.set(KEY_CONTROL_TAU, tau_cmd);
      redis_client.commit();

      // Parse interaction from web app
#ifdef USE_WEB_APP
      std::map<size_t, spatial_dyn::SpatialForced> f_ext;
      try {
        redis_gl::simulator::Interaction interaction = fut_interaction.get();
        if (!interaction.key_down.empty()) {
          const Eigen::Vector3d x_adjust = redis_gl::simulator::KeypressPositionAdjustment(interaction);
          const Eigen::AngleAxisd aa_adjust = redis_gl::simulator::KeypressOrientationAdjustment(interaction);
          if (x_adjust != Eigen::Vector3d::Zero() || aa_adjust.angle() != 0.) {
            x_des += x_adjust;
            quat_des = aa_adjust * quat_des;
            is_pose_des_new = true;
          }
        }

        f_ext = redis_gl::simulator::ComputeExternalForces(model_keys, ab, interaction);
      } catch (...) {}
#endif  // USE_WEB_APP

      // Integrate
#ifdef USE_WEB_APP
      spatial_dyn::Integrate(ab, tau_cmd, timer.dt(), f_ext, integration_options);
#else  // USE_WEB_APP
      spatial_dyn::Integrate(ab, tau_cmd, timer.dt(), {}, integration_options);
#endif  // USE_WEB_APP

      redis_client.set(KEY_SENSOR_Q, ab.q());
      redis_client.set(KEY_SENSOR_DQ, ab.dq());

      // Send PUB command status
      if (is_pub_waiting && IsVelocityConverged(dx, w, kEpsilonVelPos, kEpsilonVelOri) &&
          (IsPoseConverged(x_err, ori_err, epsilon_pos.load(), epsilon_ori.load()) ||
           IsPublishCommandTimedOut(t_pub))) {
        redis_client.publish(KEY_PUB_STATUS, "done");
        is_pub_waiting = false;
        std::cout << "PUB " << KEY_PUB_STATUS << ": done" << std::endl;
      }

      // Send trajectory info to visualizer
      if (is_pose_des_new) {
        redis_client.set(KEY_CONTROL_POS_DES, x_des);
        redis_client.set(KEY_CONTROL_ORI_DES, quat_des.coeffs());
        is_pose_des_new = false;
      }
      redis_client.set(KEY_SENSOR_POS, spatial_dyn::Position(ab, -1, kEeOffset));
      redis_client.set(KEY_SENSOR_ORI, spatial_dyn::Orientation(ab).coeffs());
      redis_client.set(KEY_CONTROL_POS, x);
      redis_client.set(KEY_CONTROL_ORI, quat.coeffs());
      redis_client.set(KEY_CONTROL_POS_ERR, x_err);
      redis_client.set(KEY_CONTROL_ORI_ERR, ori_err);
      redis_client.sync_commit();
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  // Clear torques
  redis_client.sync_set(KEY_CONTROL_TAU, Eigen::VectorXd::Zero(ab.dof()));

#ifdef USE_WEB_APP
  redis_gl::simulator::UnregisterResourcePath(redis_client, path_resources.string());
  redis_gl::simulator::UnregisterModelKeys(redis_client, model_keys);
  redis_client.sync_commit();
#endif  // USE_WEB_APP

  // Print simulation stats
  std::cout << "Simulated " << timer.time_sim() << "s in " << timer.time_elapsed() << "s." << std::endl;

  return 0;
}

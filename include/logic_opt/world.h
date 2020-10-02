/**
 * world.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 12, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_WORLD_H_
#define LOGIC_OPT_WORLD_H_

#include <ctrl_utils/eigen.h>
#include <ctrl_utils/tree.h>
#include <spatial_opt/spatial_opt.h>

#include <algorithm>    // std::max
#include <exception>    // std::out_of_range
#include <map>          // std::map
#include <memory>       // std::unique_ptr, std::shared_ptr
#include <string>       // std::string
#include <vector>       // std::vector

#include "logic_opt/frame.h"
#include "logic_opt/object.h"

namespace logic_opt {

/**
 * World environment.
 */
class World {
 public:
  using Isometry = ::spatial_opt::Isometry;

  static constexpr size_t kDof = 7;

  static const std::string kWorldFrame;  // = "__world";

  /**
   * World constructor.
   *
   * @param objects Map from object name to object.
   * @param T Number of trajectory timesteps.
   */
  World(const std::shared_ptr<const std::map<std::string, Object>>& objects,
        size_t T = 1);

  World() = default;
  virtual ~World() = default;

  /**
   * Number of trajectory timesteps.
   */
  size_t num_timesteps() const { return controller_frames_.size(); }

  /**
   * Objects map from object name to object.
   */
  const std::shared_ptr<const std::map<std::string, Object>>& objects() const {
    return objects_;
  }

  /**
   * Reserve trajectory timesteps.
   *
   * @param T Number of trajectory timesteps.
   */
  void ReserveTimesteps(size_t T);

  /**
   * Attach a frame to the target frame (target frame becomes the parent of the
   * current frame).
   *
   * @param name_frame Current frame.
   * @param name_target Target frame.
   * @param t Time to attach.
   */
  void AttachFrame(const std::string& name_frame,
                   const std::string& name_target, size_t t,
                   bool fixed = false);

  /**
   * Detach a frame from its parent frame.
   *
   * @param name_frame Current frame.
   * @param t Time to detach.
   */
  void DetachFrame(const std::string& name_frame, size_t t);

  /**
   * Initialize tree at timestep t and after.
   *
   * All variable frames will be reset to constant.
   *
   * @param tree Reference tree.
   * @param t Time to initialize.
   */
  void InitializeTree(const ctrl_utils::Tree<std::string, Frame>& tree,
                      size_t t = 0);

  /**
   * Kinematic tree at timestep t.
   */
  const ctrl_utils::Tree<std::string, Frame>& frames(size_t t) const {
    return frames_[t];
  }

  /**
   * Update T_to_parent for the given frame with T_to_world.
   *
   * @param frame Frame to update.
   * @param T_to_world Updated T_to_world to compute new T_to_parent.
   * @param tree Kinematic tree.
   * @param objects Output object map.
   * @param freeze_children_in_world Update T_to_parent for frame's children so
   *                                 they remain frozen in the world frame. If
   *                                 false, they will move with the updated
   * frame.
   */
  static void set_T_to_world(const std::string& frame,
                             const Isometry& T_to_world,
                             const ctrl_utils::Tree<std::string, Frame>& tree,
                             std::map<std::string, Object>* objects,
                             bool freeze_children_in_world = true);

  /**
   * Update T_to_parent for all T_to_world given in the object_poses map.
   *
   * @param object_poses T_to_world for all objects.
   * @param tree Kinematic tree.
   * @param objects Output object map.
   * @param freeze_children_in_world Update T_to_parent for frame's children so
   *                                 they remain frozen in the world frame. If
   *                                 false, they will move with the updated
   * frame.
   */
  static void set_T_to_world(const std::map<std::string, Object>& object_poses,
                             const ctrl_utils::Tree<std::string, Frame>& tree,
                             std::map<std::string, Object>* objects,
                             bool freeze_children_in_world = true);

  /**
   * Set the controller frames at time t.
   *
   * @param control_frame Control frame.
   * @param target_frame Target frame.
   * @param t Timestep.
   */
  void set_controller_frames(const std::string& control_frame,
                             const std::string& target_frame, size_t t);

  /**
   * Controller frames at time t.
   *
   * @return (control, target) pair.
   */
  const std::pair<std::string, std::string>& controller_frames(size_t t) const {
    if (t >= controller_frames_.size()) {
      throw std::out_of_range("World::controller_frames(): t (" +
                              std::to_string(t) + ") must be less than " +
                              std::to_string(controller_frames_.size()));
    }
    return controller_frames_[t];
  };

  /**
   * Control frame at time t.
   */
  const std::string& control_frame(size_t t) const {
    if (t >= controller_frames_.size()) {
      throw std::out_of_range("World::controller_frames(): t (" +
                              std::to_string(t) + ") must be less than " +
                              std::to_string(controller_frames_.size()));
    }
    return controller_frames_[t].first;
  };

  /**
   * Target frame at time t.
   */
  const std::string& target_frame(size_t t) const {
    if (t >= controller_frames_.size()) {
      throw std::out_of_range("World::controller_frames(): t (" +
                              std::to_string(t) + ") must be less than " +
                              std::to_string(controller_frames_.size()));
    }
    return controller_frames_[t].second;
  };

  /**
   * Set the controller at time t.
   */
  void set_controller(const std::string& controller, size_t t) {
    controllers_[t] = controller;
  }

  /**
   * Controller at time t.
   */
  const std::string& controller(size_t t) const { return controllers_[t]; }

  /**
   * Gets a reference to the quaternion variable at time t.
   */
  static Eigen::Map<Eigen::Quaterniond> Quaternion(Eigen::MatrixXd& X,
                                                   size_t t) {
    return Eigen::Map<Eigen::Quaterniond>(X.col(t).tail<4>().data());
  }
  static Eigen::Map<const Eigen::Quaterniond> Quaternion(
      const Eigen::MatrixXd& X, size_t t) {
    return Eigen::Map<const Eigen::Quaterniond>(X.col(t).tail<4>().data());
  }

  /**
   * Gets the normalized quaternion variable at time t.
   */
  static Eigen::Quaterniond NormalizedQuaternion(
      Eigen::Ref<const Eigen::MatrixXd> X, size_t t) {
    const Eigen::Map<const Eigen::Quaterniond> q = Quaternion(X, t);
    if (q.norm() == 0.) return q;
    return q.normalized();
  }

  /**
   * Gets a reference to the position variable at time t.
   */
  static Eigen::Map<Eigen::Vector3d> Position(Eigen::MatrixXd& X, size_t t) {
    return Eigen::Map<Eigen::Vector3d>(X.col(t).data());
  }
  static Eigen::Map<const Eigen::Vector3d> Position(const Eigen::MatrixXd& X,
                                                    size_t t) {
    return Eigen::Map<const Eigen::Vector3d>(X.col(t).data());
  }

  /**
   * Compute the absolute pose of the given frame given the optimization
   * variables.
   *
   * @param name_frame Frame name.
   * @param X Optimization variables.
   * @param t Timestep.
   */
  Isometry T_to_world(const std::string& name_frame,
                      Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const;

  /**
   * Compute the pose of the given frame relative to its parent given the
   * optimization variables.
   *
   * @param name_frame Frame name.
   * @param X Optimization variables.
   * @param t Timestep.
   */
  Isometry T_to_parent(const std::string& name_frame,
                       Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const;

  /**
   * Compute the pose of the 'from' frame relative to the 'to' frame given the
   * optimization variables.
   *
   * @param from_frame Compute the transform from this frame.
   * @param to_frame Compute the transform to this frame.
   * @param X Optimization variables.
   * @param t Timestep.
   */
  Isometry T_to_frame(const std::string& from_frame,
                      const std::string& to_frame,
                      Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const;

  /**
   * Computes the pose of the control frame relative to the target frame given
   * the optimization variables. The quaternion variable is normalized.
   *
   * @param X Optimization variables.
   * @param t Timestep.
   */
  Isometry T_control_to_target(Eigen::Ref<const Eigen::MatrixXd> X,
                               size_t t) const;

  /**
   * Compute the position of one frame in another frame given the
   * optimization variables.
   *
   * @param of_frame Compute the position of this frame.
   * @param in_frame Compute the position relative to this frame.
   * @param X Optimization variables.
   * @param t Timestep.
   */
  Eigen::Vector3d Position(const std::string& of_frame,
                           const std::string& in_frame,
                           Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const;

  /**
   * Compute the orientation of one frame in another frame given the
   * optimization variables.
   *
   * @param of_frame Compute the orientation of this frame.
   * @param in_frame Compute the orientation relative to this frame.
   * @param X Optimization variables.
   * @param t Timestep.
   */
  Eigen::Quaterniond Orientation(const std::string& of_frame,
                                 const std::string& in_frame,
                                 Eigen::Ref<const Eigen::MatrixXd> X,
                                 size_t t) const;

  friend std::ostream& operator<<(std::ostream& os, const World& world);

 protected:
  std::shared_ptr<const std::map<std::string, Object>> objects_;

  std::vector<ctrl_utils::Tree<std::string, Frame>> frames_;

  std::vector<std::pair<std::string, std::string>> controller_frames_;

  std::vector<std::string> controllers_;
};

}  // namespace logic_opt

#endif  // LOGIC_OPT_WORLD_H_

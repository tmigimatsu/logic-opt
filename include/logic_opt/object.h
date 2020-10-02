/**
 * object.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 12, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_OBJECT_H_
#define LOGIC_OPT_OBJECT_H_

#include <ctrl_utils/eigen.h>
#include <ncollide_cpp/ncollide.h>
#include <spatial_opt/spatial_opt.h>
#include <spatial_dyn/spatial_dyn.h>

namespace logic_opt {

/**
 * World object.
 */
class Object : public spatial_dyn::RigidBody {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Isometry = ::spatial_opt::Isometry;

  Object(const std::string& name) : spatial_dyn::RigidBody(name) {}
  Object(const spatial_dyn::RigidBody& rb);

  virtual ~Object() = default;

  /**
   * Mesh for collision checking.
   */
  std::shared_ptr<ncollide3d::shape::Shape> collision;

  /**
   * 3D transform to parent frame.
   */
  const Isometry& T_to_parent() const { return sp_T_to_parent_; }

  void set_T_to_parent(const Eigen::Quaterniond& quat,
                       Eigen::Ref<const Eigen::Vector3d> pos) {
    sp_T_to_parent_ = Isometry(quat, pos);
  }
  void set_T_to_parent(const Eigen::Isometry3d& T_to_parent) {
    sp_T_to_parent_ = T_to_parent;
  }
  void set_T_to_parent(const Isometry& T_to_parent) {
    sp_T_to_parent_ = T_to_parent;
  }
  void set_T_to_parent(Isometry&& T_to_parent) {
    sp_T_to_parent_ = std::move(T_to_parent);
  }

 protected:
  static std::unique_ptr<ncollide3d::shape::Shape> MakeCollision(
      const spatial_dyn::Graphics::Geometry& geometry);

  Isometry sp_T_to_parent_;
  Eigen::Isometry2d T_to_parent_2d_;
};

}  // namespace logic_opt

#endif  // LOGIC_OPT_OBJECT_H_

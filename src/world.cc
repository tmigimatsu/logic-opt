/**
 * world.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 12, 2018
 * Authors: Toki Migimatsu
 */

#include "LogicOpt/world.h"
#include "LogicOpt/constraints.h"

#include <cmath>      // std::fabs

#include <ctrl_utils/string.h>

namespace LogicOpt {

template<>
Eigen::Isometry3d ConvertIsometry<3>(const Eigen::Isometry3d& T) {
  return T;
}

template<>
Eigen::Isometry2d ConvertIsometry<2>(const Eigen::Isometry3d& T) {
  const Eigen::AngleAxisd aa(T.linear());
  const double theta_z = aa.angle() * aa.axis()(2);
  return Eigen::Translation2d(T.translation().head<2>()) * Eigen::Rotation2Dd(theta_z);
}

template<>
std::unique_ptr<ncollide3d::shape::Shape>
Object<3>::MakeCollision(const spatial_dyn::Graphics::Geometry& geometry) {
  switch (geometry.type) {
    case spatial_dyn::Graphics::Geometry::Type::kBox:
      return std::make_unique<ncollide3d::shape::RoundedCuboid>(geometry.scale.array() / 2. - 0.005, 0.005);
      // return std::make_unique<ncollide3d::shape::Cuboid>(geometry.scale.array() / 2.);
    case spatial_dyn::Graphics::Geometry::Type::kCapsule:
      return std::make_unique<ncollide3d::shape::Capsule>(geometry.length / 2., geometry.radius);
    case spatial_dyn::Graphics::Geometry::Type::kSphere:
      return std::make_unique<ncollide3d::shape::Ball>(geometry.radius);
    case spatial_dyn::Graphics::Geometry::Type::kMesh:
      return std::make_unique<ncollide3d::shape::TriMesh>(geometry.mesh);
    default:
      throw std::runtime_error("LogicOpt::Object::MakeCollision(): Geometry type " +
                               ctrl_utils::ToString(geometry.type) + " not implemented yet.");
      break;
  }
}

template<>
std::unique_ptr<ncollide2d::shape::Shape>
Object<2>::MakeCollision(const spatial_dyn::Graphics::Geometry& geometry) {
  switch (geometry.type) {
    case spatial_dyn::Graphics::Geometry::Type::kBox:
      return std::make_unique<ncollide2d::shape::Cuboid>(geometry.scale.head<2>().array() / 2.);
    case spatial_dyn::Graphics::Geometry::Type::kCapsule:
      return std::make_unique<ncollide2d::shape::Capsule>(geometry.length / 2., geometry.radius);
    case spatial_dyn::Graphics::Geometry::Type::kSphere:
      return std::make_unique<ncollide2d::shape::Ball>(geometry.radius);
    default:
      throw std::runtime_error("LogicOpt::Object::MakeCollision(): Geometry type " +
                               ctrl_utils::ToString(geometry.type) + " not implemented yet.");
      break;
  }
}

template<>
Eigen::Rotation2Dd World<2>::ExtractRotation(Eigen::Ref<const Eigen::MatrixXd> X,
                                             size_t t) {
  return Eigen::Rotation2Dd(X(2,t));
}

template<>
Eigen::AngleAxisd World<3>::ExtractRotation(Eigen::Ref<const Eigen::MatrixXd> X,
                                            size_t t) {
  const auto aa = X.block<3, 1>(3, t);
  return Eigen::AngleAxisd(aa.norm(), aa.normalized());
}

// template<int Dim>
// Eigen::Matrix<double, Dim, Dim> ApplyRotation

template<>
Eigen::Vector3d World<3>::Position(const std::string& of_frame, const std::string& in_frame,
                                   Eigen::Ref<const Eigen::MatrixXd> X, size_t t) const {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  bool frame_reached = false;
  for (const std::pair<std::string, Frame>& key_val : frames_[t].ancestors(of_frame)) {
    if (key_val.first == in_frame) {
      frame_reached = true;
      break;
    }

    const Frame& frame = key_val.second;
    if (frame.is_variable()) {
      auto x = X.col(frame.idx_var());
      auto pos = x.head<3>();
      auto aa = x.tail<3>();
      double angle = aa.norm();
      p = Eigen::Translation3d(pos) * Eigen::AngleAxisd(aa.norm(), aa.normalized()) * p;
    } else {
      p = objects_->at(frame.name()).T_to_parent() * p;
    }
  }

  if (!frame_reached) {
    throw std::invalid_argument("World::Position(): frame \"" + of_frame +
                                "\" must be an descendant of \"" + in_frame + "\"");
  }
  return p;
}

template<>
std::ostream& operator<<(std::ostream& os, const World<3>& world) {
  for (size_t t = 0; t < world.num_timesteps(); t++) {
    const auto& frame_pair = world.controller_frames(t);
    os << "Frame: " << t << std::endl
       << "  Control: " << frame_pair.first << std::endl
       << "  Target: " << frame_pair.second << std::endl;

    const auto& frame_tree = world.frames(t);
    os << "  Tree:" << std::endl;
    for (const auto& key_val : frame_tree.values()) {
      const std::string& name = key_val.first;
      const Frame& frame = key_val.second;
      const std::optional<std::string>& id_parent = frame_tree.parent(name);
      os << "    " << name << ":" << std::endl;
      if (id_parent) {
         os << "      parent: " << *id_parent << std::endl;
      }
      if (frame.is_variable()) {
        os << "      idx_var: " << frame.idx_var() << std::endl;
      }
    }
  }
  return os;
}

}  // namespace LogicOpt

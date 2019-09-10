/**
 * world.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 12, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/world.h"
#include "logic_opt/optimization/constraints.h"

#include <cmath>      // std::fabs

#include <ctrl_utils/string.h>

namespace logic_opt {

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
    case spatial_dyn::Graphics::Geometry::Type::kBox: {
        Eigen::Vector3d scale = geometry.scale.array() / 2. - 0.005;
        scale.head<2>().array() = (scale.head<2>().array() <= 0.015).select(scale.head<2>(), scale.head<2>().array() - 0.015);
        return std::make_unique<ncollide3d::shape::RoundedCuboid>(scale, 0.005);
      }
      // return std::make_unique<ncollide3d::shape::Cuboid>(geometry.scale.array() / 2.);
    case spatial_dyn::Graphics::Geometry::Type::kCapsule:
      return std::make_unique<ncollide3d::shape::Capsule>(geometry.length / 2., geometry.radius);
    case spatial_dyn::Graphics::Geometry::Type::kSphere:
      return std::make_unique<ncollide3d::shape::Ball>(geometry.radius);
    case spatial_dyn::Graphics::Geometry::Type::kMesh:
      return std::make_unique<ncollide3d::shape::TriMesh>(geometry.mesh);
    default:
      throw std::runtime_error("logic_opt::Object::MakeCollision(): Geometry type " +
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
      throw std::runtime_error("logic_opt::Object::MakeCollision(): Geometry type " +
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

}  // namespace logic_opt

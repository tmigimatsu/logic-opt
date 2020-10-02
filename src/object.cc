/**
 * object.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 12, 2018
 * Authors: Toki Migimatsu
 */

#include "logic_opt/object.h"

#include <ctrl_utils/string.h>

namespace {

using Isometry = ::spatial_opt::Isometry;

Eigen::Isometry2d ConvertIsometry2D(const Eigen::Isometry3d& T) {
  const Eigen::AngleAxisd aa(T.linear());
  const double theta_z = aa.angle() * aa.axis()(2);
  return Eigen::Translation2d(T.translation().head<2>()) *
         Eigen::Rotation2Dd(theta_z);
}

}  // namespace

namespace logic_opt {

Object::Object(const spatial_dyn::RigidBody& rb) : spatial_dyn::RigidBody(rb) {
  // Convert Isometry
  sp_T_to_parent_ = T_to_parent_;

  // Compute 2d transform
  T_to_parent_2d_ = ConvertIsometry2D(T_to_parent_);

  if (rb.graphics.empty()) return;

  if (rb.graphics.size() == 1) {
    // TODO: Handle graphics.T_to_parent
    collision = MakeCollision(rb.graphics[0].geometry);
    return;
  }

  ncollide3d::shape::ShapeVector shapes;
  shapes.reserve(rb.graphics.size());
  for (const spatial_dyn::Graphics& graphics : rb.graphics) {
    shapes.emplace_back(graphics.T_to_parent, MakeCollision(graphics.geometry));
  }
  collision = std::make_unique<ncollide3d::shape::Compound>(std::move(shapes));
}

std::unique_ptr<ncollide3d::shape::Shape> Object::MakeCollision(
    const spatial_dyn::Graphics::Geometry& geometry) {
  switch (geometry.type) {
    case spatial_dyn::Graphics::Geometry::Type::kBox: {
      Eigen::Vector3d scale = geometry.scale.array() / 2. - 0.005;
      scale.head<2>().array() =
          (scale.head<2>().array() <= 0.015)
              .select(scale.head<2>(), scale.head<2>().array() - 0.015);
      return std::make_unique<ncollide3d::shape::RoundedCuboid>(scale, 0.005);
    }
      // return
      // std::make_unique<ncollide3d::shape::Cuboid>(geometry.scale.array()
      // / 2.);
    case spatial_dyn::Graphics::Geometry::Type::kCapsule:
      return std::make_unique<ncollide3d::shape::Capsule>(geometry.length / 2.,
                                                          geometry.radius);
    case spatial_dyn::Graphics::Geometry::Type::kSphere:
      return std::make_unique<ncollide3d::shape::Ball>(geometry.radius);
    case spatial_dyn::Graphics::Geometry::Type::kMesh:
      return std::make_unique<ncollide3d::shape::TriMesh>(geometry.mesh);
    default:
      throw std::runtime_error(
          "logic_opt::Object::MakeCollision(): Geometry type " +
          ctrl_utils::ToString(geometry.type) + " not implemented yet.");
      break;
  }
}

}  // namespace logic_opt

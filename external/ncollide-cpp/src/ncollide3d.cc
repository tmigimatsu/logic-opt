/**
 * ncollide3d.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 12, 2019
 * Authors: Toki Migimatsu
 */

#include "ncollide/ncollide3d.h"

#include "ncollide/ncollide_ffi.h"

namespace {

ncollide3d_math_isometry_t ConvertIsometry(const Eigen::Isometry3d& T) {
  ncollide3d_math_isometry_t result;
  Eigen::Map<Eigen::Vector3d>(result.translation) = T.translation();
  Eigen::AngleAxisd aa(T.linear());
  Eigen::Map<Eigen::Vector3d>(result.axisangle) = aa.angle() * aa.axis();
  return result;
}

}  // namespace

namespace ncollide3d {
namespace shape {

Shape::Shape(ncollide3d_shape_t* ptr) : ptr_(ptr, ncollide3d_shape_delete) {}

void Shape::set_ptr(ncollide3d_shape_t* ptr) {
  ptr_ = std::shared_ptr<ncollide3d_shape_t>(ptr, ncollide3d_shape_delete);
}

query::point_internal::PointProjection Shape::project_point(const Eigen::Isometry3d& m,
                                                            const Eigen::Vector3d& pt,
                                                            bool solid) const {
  ncollide3d_math_isometry_t c_m = ConvertIsometry(m);
  ncollide3d_query_point_projection_t result;
  ncollide3d_query_project_point(ptr_.get(), &c_m, pt.data(), solid, &result);
  Eigen::Map<const Eigen::Vector3d> point(result.point);
  return { result.is_inside, point };
}

double Shape::distance_to_point(const Eigen::Isometry3d& m, const Eigen::Vector3d& pt,
                                bool solid) const {
  ncollide3d_math_isometry_t c_m = ConvertIsometry(m);
  return ncollide3d_query_distance_to_point(ptr_.get(), &c_m, pt.data(), solid);
}

bool Shape::contains_point(const Eigen::Isometry3d& m, const Eigen::Vector3d& pt) const {
  ncollide3d_math_isometry_t c_m = ConvertIsometry(m);
  return ncollide3d_query_contains_point(ptr_.get(), &c_m, pt.data());
}

Cuboid::Cuboid(const Eigen::Vector3d& half_extents)
    : Shape(ncollide3d_shape_cuboid_new(half_extents(0), half_extents(1), half_extents(2))) {}

Cuboid::Cuboid(double x, double y, double z)
    : Shape(ncollide3d_shape_cuboid_new(x, y, z)) {}

Eigen::Map<const Eigen::Vector3d> Cuboid::half_extents() const {
  return Eigen::Map<const Eigen::Vector3d>(ncollide3d_shape_cuboid_half_extents(ptr()));
}

std::shared_ptr<ncollide2d::shape::Shape> Cuboid::project_2d() const {
  Eigen::Vector3d h = half_extents();
  return std::make_shared<ncollide2d::shape::Cuboid>(h(0), h(1));
}

Ball::Ball(double radius) : Shape(ncollide3d_shape_ball_new(radius)) {}

double Ball::radius() const {
  return ncollide3d_shape_ball_radius(ptr());
}

std::shared_ptr<ncollide2d::shape::Shape> Ball::project_2d() const {
  return std::make_shared<ncollide2d::shape::Ball>(radius());
}

Compound::Compound(const std::vector<std::pair<Eigen::Isometry3d, std::unique_ptr<Shape>>>& shapes) {
  std::vector<ncollide3d_math_isometry_t> transforms;
  std::vector<const ncollide3d_shape_t*> raw_shapes;
  transforms.reserve(shapes.size());
  raw_shapes.reserve(shapes.size());
  for (const std::pair<Eigen::Isometry3d, std::unique_ptr<Shape>>& shape : shapes) {
    transforms.push_back(ConvertIsometry(shape.first));
    raw_shapes.push_back(shape.second->ptr());
  }
  set_ptr(ncollide3d_shape_compound_new(transforms.data(), raw_shapes.data(), shapes.size()));
}

std::shared_ptr<ncollide2d::shape::Shape> Compound::project_2d() const {
  return std::make_shared<ncollide2d::shape::Ball>(0.);
}

TriMesh::TriMesh(const std::string& filename)
    : Shape(ncollide3d_shape_trimesh_file(filename.c_str())) {}

TriMesh::TriMesh(const std::vector<double[3]>& points, const std::vector<size_t[3]>& indices)
    : Shape(ncollide3d_shape_trimesh_new(points.data(), points.size(),
                                         indices.data(), indices.size())) {}

std::shared_ptr<ncollide2d::shape::Shape> TriMesh::project_2d() const {
  return std::make_shared<ncollide2d::shape::Ball>(0.);
}

}  // namespace shape

namespace query {

ClosestPoints closest_points(const Eigen::Isometry3d& m1, const shape::Shape& g1,
                             const Eigen::Isometry3d& m2, const shape::Shape& g2,
                             double max_dist) {
  ncollide3d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide3d_math_isometry_t c_m2 = ConvertIsometry(m2);

  ClosestPoints points;
  auto result = ncollide3d_query_closest_points(&c_m1, g1.ptr(), &c_m2, g2.ptr(), max_dist,
                                                points.point1.data(), points.point2.data());

  switch (result) {
    case ncollide3d_query_closest_points_Intersecting:
      points.status = ClosestPoints::Status::Intersecting;
      break;
    case ncollide3d_query_closest_points_WithinMargin:
      points.status = ClosestPoints::Status::WithinMargin;
      break;
    case ncollide3d_query_closest_points_Disjoint:
      points.status = ClosestPoints::Status::Disjoint;
      break;
  }
  return points;
}

double distance(const Eigen::Isometry3d& m1, const shape::Shape& g1,
                const Eigen::Isometry3d& m2, const shape::Shape& g2) {
  ncollide3d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide3d_math_isometry_t c_m2 = ConvertIsometry(m2);

  return ncollide3d_query_distance(&c_m1, g1.ptr(), &c_m2, g2.ptr());
}

std::optional<Contact> contact(const Eigen::Isometry3d& m1, const shape::Shape& g1,
                               const Eigen::Isometry3d& m2, const shape::Shape& g2,
                               double prediction) {
  ncollide3d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide3d_math_isometry_t c_m2 = ConvertIsometry(m2);

  ncollide3d_query_contact_t out_contact;
  bool result = ncollide3d_query_contact(&c_m1, g1.ptr(), &c_m2, g2.ptr(), prediction, &out_contact);

  std::optional<Contact> contact;
  if (result) {
    contact.emplace(Eigen::Ref<const Eigen::Vector3d>(Eigen::Map<Eigen::Vector3d>(out_contact.world1)),
                    Eigen::Ref<const Eigen::Vector3d>(Eigen::Map<Eigen::Vector3d>(out_contact.world2)),
                    Eigen::Ref<const Eigen::Vector3d>(Eigen::Map<Eigen::Vector3d>(out_contact.normal)),
                    out_contact.depth);
  }
  return contact;
}

Proximity proximity(const Eigen::Isometry3d& m1, const shape::Shape& g1,
                    const Eigen::Isometry3d& m2, const shape::Shape& g2, double margin) {
  ncollide3d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide3d_math_isometry_t c_m2 = ConvertIsometry(m2);

  ncollide3d_query_proximity_t result = ncollide3d_query_proximity(&c_m1, g1.ptr(),
                                                                   &c_m2, g2.ptr(), margin);

  switch (result) {
    case ncollide3d_query_proximity_Intersecting: return Proximity::Intersecting;
    case ncollide3d_query_proximity_WithinMargin: return Proximity::WithinMargin;
    case ncollide3d_query_proximity_Disjoint: return Proximity::Disjoint;
  }
}

std::optional<double> time_of_impact(const Eigen::Isometry3d& m1, const Eigen::Vector3d& v1,
                                     const shape::Shape& g1,
                                     const Eigen::Isometry3d& m2, const Eigen::Vector3d& v2,
                                     const shape::Shape& g2) {
  ncollide3d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide3d_math_isometry_t c_m2 = ConvertIsometry(m2);

  double out_time = 0.;
  bool result = ncollide3d_query_time_of_impact(&c_m1, v1.data(), g1.ptr(),
                                                &c_m2, v2.data(), g2.ptr(), &out_time);

  std::optional<double> time;
  if (result) {
    time = out_time;
  }
  return time;
}

}  // namespace query
}  // namespace ncollide3d

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

#include <limits>  // std::numeric_limits

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
namespace bounding_volume {

AABB::AABB(ncollide3d_bounding_volume_aabb_t* ptr)
    : ptr_(ptr, ncollide3d_bounding_volume_aabb_delete) {}

Eigen::Map<const Eigen::Vector3d> AABB::maxs() const {
  return Eigen::Map<const Eigen::Vector3d>(ncollide3d_bounding_volume_aabb_maxs(ptr()));
}

Eigen::Map<const Eigen::Vector3d> AABB::mins() const {
  return Eigen::Map<const Eigen::Vector3d>(ncollide3d_bounding_volume_aabb_mins(ptr()));
}

AABB aabb(const shape::Shape& g, const Eigen::Isometry3d& m) {
  ncollide3d_math_isometry_t c_m = ConvertIsometry(m);
  return AABB(ncollide3d_bounding_volume_aabb(g.ptr(), &c_m));
}

}  // namespace bounding_volume

namespace query {

Ray::Ray(ncollide3d_query_ray_t* ptr) : ptr_(ptr, ncollide3d_query_ray_delete) {}

Ray::Ray(Eigen::Ref<const Eigen::Vector3d> origin, Eigen::Ref<const Eigen::Vector3d> dir)
    : ptr_(ncollide3d_query_ray_new(origin.data(), dir.data()), ncollide3d_query_ray_delete) {}

void Ray::set_ptr(ncollide3d_query_ray_t* ptr) {
  ptr_ = std::shared_ptr<ncollide3d_query_ray_t>(ptr, ncollide3d_query_ray_delete);
}

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

double distance(const Eigen::Isometry3d& m1, const shape::Shape& g1,
                const Eigen::Isometry3d& m2, const shape::Shape& g2) {
  ncollide3d_math_isometry_t c_m1 = ConvertIsometry(m1);
  ncollide3d_math_isometry_t c_m2 = ConvertIsometry(m2);

  return ncollide3d_query_distance(&c_m1, g1.ptr(), &c_m2, g2.ptr());
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

namespace shape {

Shape::Shape(ncollide3d_shape_t* ptr) : ptr_(ptr, ncollide3d_shape_delete) {}

void Shape::set_ptr(ncollide3d_shape_t* ptr) {
  ptr_ = std::shared_ptr<ncollide3d_shape_t>(ptr, ncollide3d_shape_delete);
}

bounding_volume::AABB Shape::aabb(const Eigen::Isometry3d& m) const {
  return bounding_volume::aabb(*this, m);
}

query::PointProjection Shape::project_point(const Eigen::Isometry3d& m,
                                            const Eigen::Vector3d& pt, bool solid) const {
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

std::optional<double> Shape::toi_with_ray(const Eigen::Isometry3d& m, const query::Ray& ray,
                                          bool solid) const {
  ncollide3d_math_isometry_t c_m = ConvertIsometry(m);
  double out_toi;
  bool result = ncollide3d_query_toi_with_ray(ptr(), &c_m, ray.ptr(), solid, &out_toi);
  std::optional<double> toi;
  if (result) toi = out_toi;
  return toi;
}

Ball::Ball(double radius) : Shape(ncollide3d_shape_ball_new(radius)) {}

double Ball::radius() const {
  return ncollide3d_shape_ball_radius(ptr());
}

std::shared_ptr<ncollide2d::shape::Shape> Ball::project_2d() const {
  return std::make_shared<ncollide2d::shape::Ball>(radius());
}

Eigen::Vector3d Ball::normal(const Eigen::Vector3d& point) const {
  return point.normalized();
}

Capsule::Capsule(double half_height, double radius)
    : Shape(ncollide3d_shape_capsule_new(half_height, radius)) {}

double Capsule::half_height() const {
  return ncollide3d_shape_capsule_half_height(ptr());
}
double Capsule::radius() const {
  return ncollide3d_shape_capsule_radius(ptr());
}

std::shared_ptr<ncollide2d::shape::Shape> Capsule::project_2d() const {
  return std::make_shared<ncollide2d::shape::Capsule>(half_height(), radius());
}

Eigen::Vector3d Capsule::normal(const Eigen::Vector3d& point) const {
  Eigen::Vector3d n = point;
  const double y = point(1);
  const double h = half_height();
  n(1) = y > h ? y - h : (y < h ? y + h : 0.);
  return n.normalized();
}

Compound::Compound(std::vector<std::pair<Eigen::Isometry3d, std::unique_ptr<Shape>>>&& shapes)
    : shapes_(std::move(shapes)) {
  std::vector<ncollide3d_math_isometry_t> transforms;
  std::vector<const ncollide3d_shape_t*> raw_shapes;
  transforms.reserve(shapes_.size());
  raw_shapes.reserve(shapes_.size());
  for (const std::pair<Eigen::Isometry3d, std::unique_ptr<Shape>>& shape : shapes_) {
    transforms.push_back(ConvertIsometry(shape.first));
    raw_shapes.push_back(shape.second->ptr());
  }
  set_ptr(ncollide3d_shape_compound_new(transforms.data(), raw_shapes.data(), shapes_.size()));
}

std::shared_ptr<ncollide2d::shape::Shape> Compound::project_2d() const {
  return std::make_shared<ncollide2d::shape::Ball>(0.);
}

Eigen::Vector3d Compound::normal(const Eigen::Vector3d& point) const {
  Shape* shape_closest = nullptr;
  double dist_min = std::numeric_limits<double>::infinity();
  for (const auto& key_val : shapes_) {
    const Eigen::Isometry3d& m = key_val.first;
    const std::unique_ptr<Shape>& shape = key_val.second;
    const double dist = shape->distance_to_point(m, point, false);
    if (dist < dist_min) {
      dist_min = dist;
      shape_closest = shape.get();
    }
  }
  return shape_closest->normal(point);
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

Eigen::Vector3d Cuboid::normal(const Eigen::Vector3d& point) const {
  Eigen::Vector3d n;
  const Eigen::Vector3d hh = half_extents();
  for (size_t i = 0; i < 3; i++) {
    if (std::abs(point(i)) < hh(i) - 1e-6) {
      n(i) = 0.;
    }
  }
  return n.normalized();
}

RoundedCuboid::RoundedCuboid(const Eigen::Vector3d& half_extents, double radius)
    : Shape(ncollide3d_shape_rounded_cuboid_new(half_extents(0), half_extents(1), half_extents(2),
                                                radius)) {}

RoundedCuboid::RoundedCuboid(double x, double y, double z, double radius)
    : Shape(ncollide3d_shape_rounded_cuboid_new(x, y, z, radius)) {}

Eigen::Map<const Eigen::Vector3d> RoundedCuboid::half_extents() const {
  return Eigen::Map<const Eigen::Vector3d>(ncollide3d_shape_rounded_cuboid_half_extents(ptr()));
}

std::shared_ptr<ncollide2d::shape::Shape> RoundedCuboid::project_2d() const {
  Eigen::Vector3d h = half_extents();
  return std::make_shared<ncollide2d::shape::Cuboid>(h(0), h(1));
}

Eigen::Vector3d RoundedCuboid::normal(const Eigen::Vector3d& point) const {
  Eigen::Vector3d n;
  const Eigen::Vector3d hh = half_extents();
  for (size_t i = 0; i < 3; i++) {
    const double p = point(i);
    const double h = hh(i);
    n(i) = p > h ? p - h : (p < h ? p + h : 0.);
  }
  return n.normalized();
}

TriMesh::TriMesh(const std::string& filename)
    : Shape(ncollide3d_shape_trimesh_file(filename.c_str())) {}

TriMesh::TriMesh(const std::vector<double[3]>& points, const std::vector<size_t[3]>& indices)
    : Shape(ncollide3d_shape_trimesh_new(points.data(), points.size(),
                                         indices.data(), indices.size())) {}

std::shared_ptr<ncollide2d::shape::Shape> TriMesh::project_2d() const {
  return std::make_shared<ncollide2d::shape::Ball>(0.);
}

Eigen::Vector3d TriMesh::normal(const Eigen::Vector3d& point) const {
  throw std::runtime_error("Not implemented yet.");
  return Eigen::Vector3d::Zero();
}

}  // namespace shape
}  // namespace ncollide3d

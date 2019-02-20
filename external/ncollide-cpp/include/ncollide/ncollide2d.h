/**
 * ncollide2d.h
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

#ifndef EXTERNAL_NCOLLIDE_CPP_NCOLLIDE2D_H_
#define EXTERNAL_NCOLLIDE_CPP_NCOLLIDE2D_H_

#include <memory>  // std::shared_ptr, std::unique_ptr

#if __cplusplus > 201402L
#include <optional>  // std::optional
#else
#include "LogicOpt/utils/optional.h"
#endif

#include <Eigen/Eigen>

struct ncollide2d_shape_t;

namespace ncollide2d {
namespace query {
namespace point_internal {

struct PointProjection {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool is_inside;
  Eigen::Vector2d point;
};

}  // point_internal
}  // query

namespace shape {

class Shape {

 public:

  Shape() : ptr_(nullptr) {}
  Shape(ncollide2d_shape_t* ptr);

  virtual ~Shape() = default;

  const ncollide2d_shape_t* ptr() const { return ptr_.get(); }
  ncollide2d_shape_t* ptr() { return ptr_.get(); }
  void set_ptr(ncollide2d_shape_t* ptr);

  query::point_internal::PointProjection project_point(const Eigen::Isometry2d& m,
                                                       const Eigen::Vector2d& pt, bool solid) const;

  double distance_to_point(const Eigen::Isometry2d& m, const Eigen::Vector2d& pt, bool solid) const;

  bool contains_point(const Eigen::Isometry2d& m, const Eigen::Vector2d& pt) const;

 private:

  std::shared_ptr<ncollide2d_shape_t> ptr_;

};

class Cuboid : public Shape {

 public:

  Cuboid(const Eigen::Vector2d& half_extents);
  Cuboid(double x, double y);

  Eigen::Map<const Eigen::Vector2d> half_extents() const;

};

class Ball : public Shape {

 public:

  Ball(double radius);

  double radius() const;

};

class Compound : public Shape {

 public:

  Compound(const std::vector<std::pair<Eigen::Isometry2d, std::unique_ptr<Shape>>>& shapes);

};

class ConvexPolygon : public Shape {

 public:

  ConvexPolygon(const std::vector<double[2]>& points);

};

}  // namespace shape

namespace query {

struct Contact {

  Contact() = default;
  Contact(Eigen::Ref<const Eigen::Vector2d> world1, Eigen::Ref<const Eigen::Vector2d> world2,
          Eigen::Ref<const Eigen::Vector2d> normal, double depth)
      : world1(world1), world2(world2), normal(normal), depth(depth) {}

  Eigen::Vector2d world1;
  Eigen::Vector2d world2;
  Eigen::Vector2d normal;
  double depth;

};

enum class Proximity {
  Intersecting,
  WithinMargin,
  Disjoint
};

struct ClosestPoints {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class Status {
    Intersecting,
    WithinMargin,
    Disjoint
  };

  Status status;
  Eigen::Vector2d point1;
  Eigen::Vector2d point2;
};

/**
 * Computes the pair of closest points between two shapes.
 *
 * Returns None if the objects are separated by a distance greater than
 * max_dist.
 */
// ClosestPoints closest_points(const Eigen::Isometry2d& m1, const shape::Shape& g1,
//                              const Eigen::Isometry2d& m2, const shape::Shape& g2,
//                              double max_dist);
/**
 * Computes the minimum distance separating two shapes.
 *
 * Returns `0.0` if the objects are touching or penetrating.
 */
// double distance(const Eigen::Isometry2d& m1, const shape::Shape& g1,
//                 const Eigen::Isometry2d& m2, const shape::Shape& g2);

/**
 * Computes one contact point between two shapes.
 *
 * Returns None if the objects are separated by a distance greater than prediction.
 */
// std::optional<Contact> contact(const Eigen::Isometry2d& m1, const shape::Shape& g1,
//                                const Eigen::Isometry2d& m2, const shape::Shape& g2,
//                                double prediction);

/**
 * Tests whether two shapes are in intersecting or separated by a distance
 * smaller than margin.
 */
// Proximity proximity(const Eigen::Isometry2d& m1, const shape::Shape& g1,
//                     const Eigen::Isometry2d& m2, const shape::Shape& g2,
//                     double margin);

/**
 * Computes the smallest time of impact of two shapes under translational
 * movement.
 *
 * Returns 0.0 if the objects are touching or penetrating.
 */
// std::optional<double> time_of_impact(const Eigen::Isometry2d& m1, const Eigen::Vector2d& v1,
//                                      const shape::Shape& g1,
//                                      const Eigen::Isometry2d& m2, const Eigen::Vector2d& v2,
//                                      const shape::Shape& g2);

}  // namespace query
}  // namespace ncollide2d

#endif  // EXTERNAL_NCOLLIDE_CPP_NCOLLIDE2D_H_

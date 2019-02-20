/**
 * ncollide3d.h
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 11, 2019
 * Authors: Toki Migimatsu
 */

#ifndef EXTERNAL_NCOLLIDE_CPP_NCOLLIDE3D_H_
#define EXTERNAL_NCOLLIDE_CPP_NCOLLIDE3D_H_

#include <Eigen/Eigen>

#include <memory>  // std::shared_ptr, std::unique_ptr

#if __cplusplus > 201402L
#include <optional>  // std::optional
#else
#include "LogicOpt/utils/optional.h"
#endif

#include "ncollide2d.h"

struct ncollide3d_shape_t;

namespace ncollide3d {
namespace query {
namespace point_internal {

struct PointProjection {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool is_inside;
  Eigen::Vector3d point;

};

}  // point_internal
}  // query

namespace shape {

class Shape {

 public:

  Shape() : ptr_(nullptr) {}
  Shape(ncollide3d_shape_t* ptr);

  virtual ~Shape() = default;

  const ncollide3d_shape_t* ptr() const { return ptr_.get(); }
  ncollide3d_shape_t* ptr() { return ptr_.get(); }
  void set_ptr(ncollide3d_shape_t* ptr);

  query::point_internal::PointProjection project_point(const Eigen::Isometry3d& m,
                                                       const Eigen::Vector3d& pt, bool solid) const;

  double distance_to_point(const Eigen::Isometry3d& m, const Eigen::Vector3d& pt, bool solid) const;

  bool contains_point(const Eigen::Isometry3d& m, const Eigen::Vector3d& pt) const;

  virtual std::shared_ptr<ncollide2d::shape::Shape> project_2d() const = 0;

 private:

  std::shared_ptr<ncollide3d_shape_t> ptr_;

};

class Cuboid : public Shape {

 public:

  Cuboid(const Eigen::Vector3d& half_extents);
  Cuboid(double x, double y, double z);

  Eigen::Map<const Eigen::Vector3d> half_extents() const;

  virtual std::shared_ptr<ncollide2d::shape::Shape> project_2d() const override;

};

class Ball : public Shape {

 public:

  Ball(double radius);

  double radius() const;

  virtual std::shared_ptr<ncollide2d::shape::Shape> project_2d() const override;

};

class Compound : public Shape {

 public:

  Compound(const std::vector<std::pair<Eigen::Isometry3d, std::unique_ptr<Shape>>>& shapes);

  virtual std::shared_ptr<ncollide2d::shape::Shape> project_2d() const override;

};

class TriMesh : public Shape {

 public:

  TriMesh(const std::string& filename);
  TriMesh(const std::vector<double[3]>& points, const std::vector<size_t[3]>& indices);

  virtual std::shared_ptr<ncollide2d::shape::Shape> project_2d() const override;

};

}  // namespace shape

namespace query {

struct Contact {

  Contact() = default;
  Contact(Eigen::Ref<const Eigen::Vector3d> world1, Eigen::Ref<const Eigen::Vector3d> world2,
          Eigen::Ref<const Eigen::Vector3d> normal, double depth)
      : world1(world1), world2(world2), normal(normal), depth(depth) {}

  Eigen::Vector3d world1;
  Eigen::Vector3d world2;
  Eigen::Vector3d normal;
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
  Eigen::Vector3d point1;
  Eigen::Vector3d point2;
};

/**
 * Computes the pair of closest points between two shapes.
 *
 * Returns None if the objects are separated by a distance greater than
 * max_dist.
 */
ClosestPoints closest_points(const Eigen::Isometry3d& m1, const shape::Shape& g1,
                             const Eigen::Isometry3d& m2, const shape::Shape& g2,
                             double max_dist);
/**
 * Computes the minimum distance separating two shapes.
 *
 * Returns `0.0` if the objects are touching or penetrating.
 */
double distance(const Eigen::Isometry3d& m1, const shape::Shape& g1,
                const Eigen::Isometry3d& m2, const shape::Shape& g2);

/**
 * Computes one contact point between two shapes.
 *
 * Returns None if the objects are separated by a distance greater than prediction.
 */
std::optional<Contact> contact(const Eigen::Isometry3d& m1, const shape::Shape& g1,
                               const Eigen::Isometry3d& m2, const shape::Shape& g2,
                               double prediction);

/**
 * Tests whether two shapes are in intersecting or separated by a distance
 * smaller than margin.
 */
Proximity proximity(const Eigen::Isometry3d& m1, const shape::Shape& g1,
                    const Eigen::Isometry3d& m2, const shape::Shape& g2,
                    double margin);

/**
 * Computes the smallest time of impact of two shapes under translational
 * movement.
 *
 * Returns 0.0 if the objects are touching or penetrating.
 */
std::optional<double> time_of_impact(const Eigen::Isometry3d& m1, const Eigen::Vector3d& v1,
                                     const shape::Shape& g1,
                                     const Eigen::Isometry3d& m2, const Eigen::Vector3d& v2,
                                     const shape::Shape& g2);

/*
class ContactManifold {

 public:

  size_t len() const;

  const_iterator contacts() const;
  iterator contacts_mut();

  std::optional<TrackedContact> deepest_contact() const;

  void clear(IdAllocator& gen);

  ContactTrackingMode tracking_mode() const;
  void set_tracking_mode(ContactTrackingMode mode);

  void save_cache_and_clear(IdAllocator& gen);

  bool push(Contact contact, ContactKinematic kinematic, Eigen::Vector3d tracking_pt,
            std::optional<ContactPreprocessor> preprocessor1,
            std::optional<ContactPreprocessor> preprocessor2,
            IdAllocator& gen);

};
*/

}  // namespace query

/*
namespace narrow_phase {

using ContactAlgorithm = ncollide3d_narrow_phase_contact_algorithm_t;

class ContactDispatcher {

 public:

  virtual std::optional<ContactAlgorithm>
  get_contact_algorithm(const shape::Shape& a, const shape::Shape& b) const;

};

class DefaultContactDispatcher : public ContactDispatcher {

 public:

  virtual std::optional<ContactAlgorithm>
  get_contact_algorithm(const shape::Shape& a, const shape::Shape& b) const override;

};

class ContactManifoldGenerator {

 public:

  query::ContactManifold init_manifold();

  bool generate_contacts(ContactDispatcher& dispatcher,
                         const Eigen::Isometry3d& ma, const shape::Shape& a,
                         const std::optional<ContactPreprocessor>& proc1
                         const Eigen::Isometry3d& mb, const shape::Shape& b,
                         const std::optional<ContactPreprocessor>& proc2,
                         ContactPrediction& prediction, IdAllocator& id_alloc,
                         query::ContactManifold& manifold);

};

}  // namespace narrow_phase
*/
}  // namespace ncollide3d

#endif  // EXTERNAL_NCOLLIDE_CPP_NCOLLIDE3D_H_

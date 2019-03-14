/**
 * ncollide_ffi.h
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

#ifndef EXTERNAL_NCOLLIDE_CPP_NCOLLIDE_FFI_H_
#define EXTERNAL_NCOLLIDE_CPP_NCOLLIDE_FFI_H_

struct ncollide3d_bounding_volume_aabb_t;

struct ncollide2d_shape_t;
struct ncollide3d_shape_t;

struct ncollide2d_math_isometry_t {
  double translation[2];
  double angle;
};
struct ncollide3d_math_isometry_t {
  double translation[3];
  double axisangle[3];
};

struct ncollide2d_query_contact_t {
  double world1[2];
  double world2[2];
  double normal[2];
  double depth;
};
struct ncollide3d_query_contact_t {
  double world1[3];
  double world2[3];
  double normal[3];
  double depth;
};

struct ncollide2d_query_point_projection_t {
  bool is_inside;
  double point[2];
};
struct ncollide3d_query_point_projection_t {
  bool is_inside;
  double point[3];
};

struct ncollide2d_query_ray_t;
struct ncollide3d_query_ray_t;

struct ncollide3d_narrow_phase_contact_algorithm_t;

enum ncollide2d_query_proximity_t {
  ncollide2d_query_proximity_Intersecting,
  ncollide2d_query_proximity_WithinMargin,
  ncollide2d_query_proximity_Disjoint
};
enum ncollide3d_query_proximity_t {
  ncollide3d_query_proximity_Intersecting,
  ncollide3d_query_proximity_WithinMargin,
  ncollide3d_query_proximity_Disjoint
};

enum ncollide2d_query_closest_points_t {
  ncollide2d_query_closest_points_Intersecting,
  ncollide2d_query_closest_points_WithinMargin,
  ncollide2d_query_closest_points_Disjoint
};
enum ncollide3d_query_closest_points_t {
  ncollide3d_query_closest_points_Intersecting,
  ncollide3d_query_closest_points_WithinMargin,
  ncollide3d_query_closest_points_Disjoint
};

extern "C" {

ncollide3d_bounding_volume_aabb_t*
ncollide3d_bounding_volume_aabb(const ncollide3d_shape_t* g, const ncollide3d_math_isometry_t* m);

void ncollide3d_bounding_volume_aabb_delete(const ncollide3d_bounding_volume_aabb_t* aabb);

const double* ncollide3d_bounding_volume_aabb_maxs(const ncollide3d_bounding_volume_aabb_t* aabb);
const double* ncollide3d_bounding_volume_aabb_mins(const ncollide3d_bounding_volume_aabb_t* aabb);

ncollide2d_shape_t* ncollide2d_shape_ball_new(double radius);
ncollide3d_shape_t* ncollide3d_shape_ball_new(double radius);

ncollide2d_shape_t* ncollide2d_shape_compound_new(const ncollide2d_math_isometry_t* transforms,
                                                  const ncollide2d_shape_t** shapes, size_t n);
ncollide3d_shape_t* ncollide3d_shape_compound_new(const ncollide3d_math_isometry_t* transforms,
                                                  const ncollide3d_shape_t** shapes, size_t n);

ncollide2d_shape_t* ncollide2d_shape_convex_polygon_try_from_points(const double(* points)[2],
                                                                    size_t n);

ncollide2d_shape_t* ncollide2d_shape_cuboid_new(double x, double y);
ncollide3d_shape_t* ncollide3d_shape_cuboid_new(double x, double y, double z);

ncollide3d_shape_t* ncollide3d_shape_trimesh_new(const double(* points)[3], size_t npoints,
                                                 const size_t(* indices)[3], size_t nfaces);
ncollide3d_shape_t* ncollide3d_shape_trimesh_file(const char* filename);

void ncollide2d_shape_delete(ncollide2d_shape_t* shape);
void ncollide3d_shape_delete(ncollide3d_shape_t* shape);

const double* ncollide2d_shape_cuboid_half_extents(const ncollide2d_shape_t* cuboid);
const double* ncollide3d_shape_cuboid_half_extents(const ncollide3d_shape_t* cuboid);

const double ncollide2d_shape_ball_radius(const ncollide2d_shape_t* ball);
const double ncollide3d_shape_ball_radius(const ncollide3d_shape_t* ball);

ncollide2d_query_ray_t* ncollide2d_query_ray_new(const double origin[2], const double dir[2]);
ncollide3d_query_ray_t* ncollide3d_query_ray_new(const double origin[3], const double dir[3]);

void ncollide2d_query_ray_delete(ncollide2d_query_ray_t* ray);
void ncollide3d_query_ray_delete(ncollide3d_query_ray_t* ray);

void ncollide2d_query_project_point(const ncollide2d_shape_t* shape,
                                    const ncollide2d_math_isometry_t* m1,
                                    const double pt[2], bool solid,
                                    ncollide2d_query_point_projection_t* out_projection);
void ncollide3d_query_project_point(const ncollide3d_shape_t* shape,
                                    const ncollide3d_math_isometry_t* m1,
                                    const double pt[3], bool solid,
                                    ncollide3d_query_point_projection_t* out_projection);

double ncollide2d_query_distance_to_point(const ncollide2d_shape_t* shape,
                                          const ncollide2d_math_isometry_t* m1,
                                          const double pt[2], bool solid);
double ncollide3d_query_distance_to_point(const ncollide3d_shape_t* shape,
                                          const ncollide3d_math_isometry_t* m1,
                                          const double pt[3], bool solid);

double ncollide2d_query_contains_point(const ncollide2d_shape_t* shape,
                                       const ncollide2d_math_isometry_t* m1,
                                       const double pt[2]);
double ncollide3d_query_contains_point(const ncollide3d_shape_t* shape,
                                       const ncollide3d_math_isometry_t* m1,
                                       const double pt[3]);

ncollide2d_query_closest_points_t
ncollide2d_query_closest_points(const ncollide2d_math_isometry_t* m1, const ncollide2d_shape_t* g1,
                                const ncollide2d_math_isometry_t* m2, const ncollide2d_shape_t* g2,
                                double max_dist, double out_p1[2], double out_p2[2]);
ncollide3d_query_closest_points_t
ncollide3d_query_closest_points(const ncollide3d_math_isometry_t* m1, const ncollide3d_shape_t* g1,
                                const ncollide3d_math_isometry_t* m2, const ncollide3d_shape_t* g2,
                                double max_dist, double out_p1[3], double out_p2[3]);

double ncollide2d_query_distance(const ncollide2d_math_isometry_t* m1, const ncollide2d_shape_t* g1,
                                 const ncollide2d_math_isometry_t* m2, const ncollide2d_shape_t* g2);
double ncollide3d_query_distance(const ncollide3d_math_isometry_t* m1, const ncollide3d_shape_t* g1,
                                 const ncollide3d_math_isometry_t* m2, const ncollide3d_shape_t* g2);

bool ncollide2d_query_contact(const ncollide2d_math_isometry_t* m1, const ncollide2d_shape_t* g1,
                              const ncollide2d_math_isometry_t* m2, const ncollide2d_shape_t* g2,
                              double prediction, ncollide2d_query_contact_t* out_contact);
bool ncollide3d_query_contact(const ncollide3d_math_isometry_t* m1, const ncollide3d_shape_t* g1,
                              const ncollide3d_math_isometry_t* m2, const ncollide3d_shape_t* g2,
                              double prediction, ncollide3d_query_contact_t* out_contact);

ncollide2d_query_proximity_t ncollide2d_query_proximity(const ncollide2d_math_isometry_t* m1,
                                                        const ncollide2d_shape_t* g1,
                                                        const ncollide2d_math_isometry_t* m2,
                                                        const ncollide2d_shape_t* g2,
                                                        double margin);
ncollide3d_query_proximity_t ncollide3d_query_proximity(const ncollide3d_math_isometry_t* m1,
                                                        const ncollide3d_shape_t* g1,
                                                        const ncollide3d_math_isometry_t* m2,
                                                        const ncollide3d_shape_t* g2,
                                                        double margin);

bool ncollide2d_query_time_of_impact(const ncollide2d_math_isometry_t* m1, const double v1[2],
                                     const ncollide2d_shape_t* g1,
                                     const ncollide2d_math_isometry_t* m2, const double v2[2],
                                     const ncollide2d_shape_t* g2, double* out_time);
bool ncollide3d_query_time_of_impact(const ncollide3d_math_isometry_t* m1, const double v1[3],
                                     const ncollide3d_shape_t* g1,
                                     const ncollide3d_math_isometry_t* m2, const double v2[3],
                                     const ncollide3d_shape_t* g2, double* out_time);

bool ncollide2d_query_toi_with_ray(const ncollide2d_shape_t* shape,
                                   const ncollide2d_math_isometry_t* m,
                                   const ncollide2d_query_ray_t* ray, bool solid,
                                   double* out_toi);
bool ncollide3d_query_toi_with_ray(const ncollide3d_shape_t* shape,
                                   const ncollide3d_math_isometry_t* m,
                                   const ncollide3d_query_ray_t* ray, bool solid,
                                   double* out_toi);

}  // extern "C"

#endif  // EXTERNAL_NCOLLIDE_CPP_NCOLLIDE_FFI_H_

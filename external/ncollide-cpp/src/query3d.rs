/**
 * query3d.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;
extern crate ncollide3d as nc;

use super::math3d::*;

#[repr(C)]
pub enum ncollide3d_query_proximity_t {
    Intersecting,
    WithinMargin,
    Disjoint
}

#[repr(C)]
pub enum ncollide3d_query_closest_points_t {
    Intersecting,
    WithinMargin,
    Disjoint
}

#[repr(C)]
pub struct ncollide3d_query_contact_t {
    world1: [f64; 3],
    world2: [f64; 3],
    normal: [f64; 3],
    depth: f64
}

#[repr(C)]
pub struct ncollide3d_query_point_projection_t {
    is_inside: bool,
    point: [f64; 3]
}

#[no_mangle]
pub extern fn ncollide3d_query_distance(
        m1: Option<&ncollide3d_math_isometry_t>, g1: Option<&nc::shape::ShapeHandle<f64>>,
        m2: Option<&ncollide3d_math_isometry_t>, g2: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    let m1 = isometry_from_raw(m1.unwrap());
    let m2 = isometry_from_raw(m2.unwrap());
    let g1 = g1.unwrap().as_ref();
    let g2 = g2.unwrap().as_ref();
    ncollide3d::query::distance(&m1, g1, &m2, g2)
}

#[no_mangle]
pub extern fn ncollide3d_query_project_point(shape: Option<&nc::shape::ShapeHandle<f64>>,
                                             m: Option<&ncollide3d_math_isometry_t>,
                                             pt: Option<&[f64; 3]>, solid: bool,
                                             mut out_projection: Option<&mut ncollide3d_query_point_projection_t>) {
    use nc::query::PointQuery;

    let shape = shape.unwrap().as_ref();
    let m = isometry_from_raw(m.unwrap());
    let pt = na::Point3::<f64>::from(na::Vector3::from_column_slice_generic(na::U3, na::U1, pt.unwrap()));
    let projection = shape.project_point(&m, &pt, solid);
    match out_projection {
        Some(ref mut out_projection) => {
            out_projection.is_inside = projection.is_inside;
            out_projection.point.copy_from_slice(projection.point.coords.data.as_slice());
        },
        None => {}
    };
}

#[no_mangle]
pub extern fn ncollide3d_query_distance_to_point(shape: Option<&nc::shape::ShapeHandle<f64>>,
                                                 m: Option<&ncollide3d_math_isometry_t>,
                                                 pt: Option<&[f64; 3]>, solid: bool) -> f64 {
    use nc::query::PointQuery;

    let shape = shape.unwrap().as_ref();
    let m = isometry_from_raw(m.unwrap());
    let pt = na::Point3::<f64>::from(na::Vector3::from_column_slice_generic(na::U3, na::U1, pt.unwrap()));
    shape.distance_to_point(&m, &pt, solid)
}

#[no_mangle]
pub extern fn ncollide3d_query_contains_point(shape: Option<&nc::shape::ShapeHandle<f64>>,
                                              m: Option<&ncollide3d_math_isometry_t>,
                                              pt: Option<&[f64; 3]>) -> bool {
    use nc::query::PointQuery;

    let shape = shape.unwrap().as_ref();
    let m = isometry_from_raw(m.unwrap());
    let pt = na::Point3::<f64>::from(na::Vector3::from_column_slice_generic(na::U3, na::U1, pt.unwrap()));
    shape.contains_point(&m, &pt)
}

#[no_mangle]
pub extern fn ncollide3d_query_closest_points(
        m1: Option<&ncollide3d_math_isometry_t>, g1: Option<&nc::shape::ShapeHandle<f64>>,
        m2: Option<&ncollide3d_math_isometry_t>, g2: Option<&nc::shape::ShapeHandle<f64>>,
        max_dist: f64, mut out_p1: Option<&mut [f64; 3]>, mut out_p2: Option<&mut [f64; 3]>)
        -> ncollide3d_query_closest_points_t {
    let m1 = isometry_from_raw(m1.unwrap());
    let m2 = isometry_from_raw(m2.unwrap());
    let g1 = g1.unwrap().as_ref();
    let g2 = g2.unwrap().as_ref();

    let result = ncollide3d::query::closest_points(&m1, g1, &m2, g2, max_dist);
    match result {
        ncollide3d::query::ClosestPoints::Intersecting => ncollide3d_query_closest_points_t::Intersecting,
        ncollide3d::query::ClosestPoints::WithinMargin(ref p1, ref p2) => {
            match out_p1 {
                Some(ref mut out_p1) => {
                    out_p1.copy_from_slice(p1.coords.data.as_slice());
                },
                None => {}
            };
            match out_p2 {
                Some(ref mut out_p2) => {
                    out_p2.copy_from_slice(p2.coords.data.as_slice());
                },
                None => {}
            };
            ncollide3d_query_closest_points_t::WithinMargin
        },
        ncollide3d::query::ClosestPoints::Disjoint => ncollide3d_query_closest_points_t::Disjoint
    }
}

#[no_mangle]
pub extern fn ncollide3d_query_contact(
        m1: Option<&ncollide3d_math_isometry_t>, g1: Option<&nc::shape::ShapeHandle<f64>>,
        m2: Option<&ncollide3d_math_isometry_t>, g2: Option<&nc::shape::ShapeHandle<f64>>,
        prediction: f64, mut out_contact: Option<&mut ncollide3d_query_contact_t>) -> bool {
    let m1 = isometry_from_raw(m1.unwrap());
    let m2 = isometry_from_raw(m2.unwrap());
    let g1 = g1.unwrap().as_ref();
    let g2 = g2.unwrap().as_ref();

    let result = ncollide3d::query::contact(&m1, g1, &m2, g2, prediction);
    match result {
        Some(ref contact) => {
            match out_contact {
                Some(ref mut out_contact) => {
                    out_contact.world1.copy_from_slice(contact.world1.coords.data.as_slice());
                    out_contact.world2.copy_from_slice(contact.world2.coords.data.as_slice());
                    out_contact.normal.copy_from_slice(contact.normal.data.as_slice());
                    out_contact.depth = contact.depth;
                },
                None => {}
            };
            true
        },
        None => false
    }
}

#[no_mangle]
pub extern fn ncollide3d_query_proximity(
        m1: Option<&ncollide3d_math_isometry_t>, g1: Option<&nc::shape::ShapeHandle<f64>>,
        m2: Option<&ncollide3d_math_isometry_t>, g2: Option<&nc::shape::ShapeHandle<f64>>,
        margin: f64) -> ncollide3d_query_proximity_t {
    let m1 = isometry_from_raw(m1.unwrap());
    let m2 = isometry_from_raw(m2.unwrap());
    let g1 = g1.unwrap().as_ref();
    let g2 = g2.unwrap().as_ref();

    let result = ncollide3d::query::proximity(&m1, g1, &m2, g2, margin);
    match result {
        ncollide3d::query::Proximity::Intersecting => ncollide3d_query_proximity_t::Intersecting,
        ncollide3d::query::Proximity::WithinMargin => ncollide3d_query_proximity_t::WithinMargin,
        ncollide3d::query::Proximity::Disjoint => ncollide3d_query_proximity_t::Disjoint,
    }
}

#[no_mangle]
pub extern fn ncollide3d_query_time_of_impact(
        m1: Option<&ncollide3d_math_isometry_t>, v1: Option<&[f64; 3]>,
        g1: Option<&nc::shape::ShapeHandle<f64>>,
        m2: Option<&ncollide3d_math_isometry_t>, v2: Option<&[f64; 3]>,
        g2: Option<&nc::shape::ShapeHandle<f64>>,
        mut out_time: Option<&mut f64>) -> bool {
    let m1 = isometry_from_raw(m1.unwrap());
    let m2 = isometry_from_raw(m2.unwrap());
    let v1 = na::Vector3::from_column_slice_generic(na::U3, na::U1, v1.unwrap());
    let v2 = na::Vector3::from_column_slice_generic(na::U3, na::U1, v2.unwrap());
    let g1 = g1.unwrap().as_ref();
    let g2 = g2.unwrap().as_ref();

    let result = ncollide3d::query::time_of_impact(&m1, &v1, g1, &m2, &v2, g2);
    match result {
        Some(time) => {
            match out_time {
                Some(ref mut out_time) => { **out_time = time; },
                None => {}
            };
            true
        },
        None => false
    }
}

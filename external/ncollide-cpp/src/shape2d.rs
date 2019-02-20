/**
 * shape2d.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;
extern crate ncollide2d as nc;

use super::math2d::*;

#[no_mangle]
pub extern fn ncollide2d_shape_ball_new(radius: f64) -> *mut nc::shape::ShapeHandle<f64> {
    let ball = nc::shape::Ball::new(radius);
    let handle = nc::shape::ShapeHandle::new(ball);
    Box::into_raw(Box::new(handle))
}

#[no_mangle]
pub extern fn ncollide2d_shape_cuboid_new(x: f64, y: f64) -> *mut nc::shape::ShapeHandle<f64> {
    let cuboid = nc::shape::Cuboid::new(na::Vector2::new(x, y));
    let handle = nc::shape::ShapeHandle::new(cuboid);
    Box::into_raw(Box::new(handle))
}

#[no_mangle]
pub extern fn ncollide2d_shape_compound_new(ptr_transforms: *const ncollide2d_math_isometry_t,
                                            ptr_shapes: *const Option<&nc::shape::ShapeHandle<f64>>,
                                            n: usize) -> *mut nc::shape::ShapeHandle<f64> {
    use nc::math::Isometry;
    use nc::shape::ShapeHandle;

    let transforms = unsafe { std::slice::from_raw_parts(ptr_transforms, n) };
    let transforms = transforms.iter().map(|x| isometry_from_raw(x));

    let shapes = unsafe { std::slice::from_raw_parts(ptr_shapes, n) };
    let shapes = shapes.iter().map(|x| x.unwrap().clone());

    let transforms_shapes: Vec<(Isometry<f64>, ShapeHandle<f64>)> = transforms.zip(shapes).collect();

    let compound = nc::shape::Compound::new(transforms_shapes);
    let handle = ShapeHandle::new(compound);
    Box::into_raw(Box::new(handle))
}

#[no_mangle]
pub extern fn ncollide2d_shape_convex_polygon_try_from_points(ptr_points: *const [f64; 2], n: usize)
        -> *mut nc::shape::ShapeHandle<f64> {
    use nc::math::Point;

    let points = unsafe { std::slice::from_raw_parts(ptr_points, n) };
    let points: Vec<_> = points.iter().map(|x| Point::<f64>::from_slice(x)).collect();

    let polygon = nc::shape::ConvexPolygon::try_from_points(&points).unwrap();
    let handle = nc::shape::ShapeHandle::new(polygon);
    Box::into_raw(Box::new(handle))
}

#[no_mangle]
pub unsafe extern fn ncollide2d_shape_delete(ptr: *mut nc::shape::ShapeHandle<f64>) {
    Box::from_raw(ptr);
}

#[no_mangle]
pub extern fn ncollide2d_shape_cuboid_half_extents(shape: Option<&nc::shape::ShapeHandle<f64>>)
        -> *const f64 {
    use na::storage::Storage;

    let maybe_cuboid = shape.unwrap().as_shape::<nc::shape::Cuboid<f64>>();
    match maybe_cuboid {
        Some(ref cuboid) => { cuboid.half_extents().data.ptr() },
        None => { std::ptr::null() }
    }
}

#[no_mangle]
pub extern fn ncollide2d_shape_ball_radius(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    let maybe_ball = shape.unwrap().as_shape::<nc::shape::Ball<f64>>();
    match maybe_ball {
        Some(ref ball) => { ball.radius() },
        None => { 0. }
    }
}

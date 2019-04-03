/**
 * shape3d.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;
extern crate ncollide3d as nc;

use super::math3d::*;
use super::rounded_cuboid3d::*;

#[no_mangle]
pub extern fn ncollide3d_shape_ball_new(radius: f64) -> *mut nc::shape::ShapeHandle<f64> {
    let ball = nc::shape::Ball::new(radius);
    let handle = nc::shape::ShapeHandle::new(ball);
    Box::into_raw(Box::new(handle))
}

#[no_mangle]
pub extern fn ncollide3d_shape_ball_radius(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    let maybe_ball = shape.unwrap().as_shape::<nc::shape::Ball<f64>>();
    match maybe_ball {
        Some(ref ball) => { ball.radius() },
        None => { 0. }
    }
}

#[no_mangle]
pub extern fn ncollide3d_shape_capsule_new(half_height: f64, radius: f64)
        -> *mut nc::shape::ShapeHandle<f64> {
    let capsule = nc::shape::Capsule::new(half_height, radius);
    let handle = nc::shape::ShapeHandle::new(capsule);
    Box::into_raw(Box::new(handle))
}

#[no_mangle]
pub extern fn ncollide3d_shape_capsule_half_height(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    let maybe_capsule = shape.unwrap().as_shape::<nc::shape::Capsule<f64>>();
    match maybe_capsule {
        Some(ref capsule) => { capsule.half_height() },
        None => { 0. }
    }
}

#[no_mangle]
pub extern fn ncollide3d_shape_capsule_radius(shape: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    let maybe_capsule = shape.unwrap().as_shape::<nc::shape::Capsule<f64>>();
    match maybe_capsule {
        Some(ref capsule) => { capsule.radius() },
        None => { 0. }
    }
}

#[no_mangle]
pub extern fn ncollide3d_shape_cuboid_new(x: f64, y: f64, z: f64) -> *mut nc::shape::ShapeHandle<f64> {
    let cuboid = nc::shape::Cuboid::new(na::Vector3::new(x, y, z));
    let handle = nc::shape::ShapeHandle::new(cuboid);
    Box::into_raw(Box::new(handle))
}

#[no_mangle]
pub extern fn ncollide3d_shape_cuboid_half_extents(shape: Option<&nc::shape::ShapeHandle<f64>>)
        -> *const f64 {
    use na::storage::Storage;

    let maybe_cuboid = shape.unwrap().as_shape::<nc::shape::Cuboid<f64>>();
    match maybe_cuboid {
        Some(ref cuboid) => { cuboid.half_extents().data.ptr() },
        None => { std::ptr::null() }
    }
}

#[no_mangle]
pub extern fn ncollide3d_shape_compound_new(ptr_transforms: *const ncollide3d_math_isometry_t,
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
pub extern fn ncollide3d_shape_rounded_cuboid_new(x: f64, y: f64, z: f64, radius: f64)
        -> *mut nc::shape::ShapeHandle<f64> {
    let cuboid = RoundedCuboid::new(na::Vector3::new(x, y, z), radius);
    let handle = nc::shape::ShapeHandle::new(cuboid);
    Box::into_raw(Box::new(handle))
}

#[no_mangle]
pub extern fn ncollide3d_shape_rounded_cuboid_half_extents(shape: Option<&nc::shape::ShapeHandle<f64>>)
        -> *const f64 {
    use na::storage::Storage;

    let maybe_cuboid = shape.unwrap().as_shape::<RoundedCuboid<f64>>();
    match maybe_cuboid {
        Some(ref cuboid) => { cuboid.half_extents().data.ptr() },
        None => { std::ptr::null() }
    }
}

#[no_mangle]
pub extern fn ncollide3d_shape_trimesh_new(ptr_points: *const [f64; 3], npoints: usize,
                                           ptr_indices: *const [usize; 3], nfaces: usize)
        -> *mut nc::shape::ShapeHandle<f64> {
    use nc::math::Point;
    use na::geometry::Point3;

    let points = unsafe { std::slice::from_raw_parts(ptr_points, npoints) };
    let points: Vec<_> = points.iter().map(|x| Point::<f64>::from_slice(x)).collect();

    let indices = unsafe { std::slice::from_raw_parts(ptr_indices, nfaces) };
    let indices: Vec<_> = indices.iter().map(|x| Point3::<usize>::from_slice(x)).collect();

    let trimesh = nc::shape::TriMesh::new(points, indices, None);
    let handle = nc::shape::ShapeHandle::new(trimesh);
    Box::into_raw(Box::new(handle))
}

#[derive(Debug)]
struct ParseVectorError;
impl std::fmt::Display for ParseVectorError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result { write!(f, "invalid vector string") }
}
impl std::error::Error for ParseVectorError {
    fn description(&self) -> &str { "invalid vector string" }
    fn cause(&self) -> Option<&std::error::Error> { None }
}

fn parse_obj_vertex(mut tokens: std::str::SplitWhitespace)
        -> Result<nc::math::Point<f64>, Box<std::error::Error>> {
    let mut v = unsafe { nc::math::Point::<f64>::new_uninitialized() };
    for i in 0..3 {
        let token = tokens.next().ok_or(ParseVectorError)?;
        v[i] = token.parse::<f64>()?;
    }
    Ok(v)
}

fn parse_obj_face(mut tokens: std::str::SplitWhitespace)
        -> Result<na::geometry::Point3<usize>, Box<std::error::Error>> {
    let mut f = unsafe { na::geometry::Point3::<usize>::new_uninitialized() };
    for i in 0..3 {
        let token = tokens.next().ok_or(ParseVectorError)?;
        f[i] = token.split('/').next().ok_or(ParseVectorError)?.parse::<usize>()?;
    }
    Ok(f)
}

fn parse_obj(filename: &str) -> (Vec<nc::math::Point<f64>>, Vec<na::geometry::Point3<usize>>) {
    use std::io::{BufReader, BufRead};

    let file = std::fs::File::open(filename).expect(&format!("unable to open {}", filename));

    let mut points = Vec::<nc::math::Point<f64>>::new();
    let mut indices = Vec::<na::geometry::Point3<usize>>::new();

    for line in BufReader::new(file).lines() {
        let line = line.expect(&format!("unable to read line in {}", filename));
        let mut tokens = line.split_whitespace();
        match tokens.next() {
            Some("v") => {
                let v = parse_obj_vertex(tokens).expect(&format!("unable to parse vertex from {}", line));
                points.push(v);
            },
            Some("f") => {
                let f = parse_obj_face(tokens).expect(&format!("unable to parse face from {}", line));
                indices.push(f);
            }
            _ => {}
        };
        println!("{}", line);
    }

    (points, indices)
}

#[no_mangle]
pub extern fn ncollide3d_shape_trimesh_file(filename: &str) -> *mut nc::shape::ShapeHandle<f64> {
    let (points, indices) = match &filename[filename.len()-4..] {
        ".obj" => { parse_obj(filename) },
        _ => { panic!("unsupported file format") }
    };

    let trimesh = nc::shape::TriMesh::new(points, indices, None);
    let handle = nc::shape::ShapeHandle::new(trimesh);
    Box::into_raw(Box::new(handle))
}

#[no_mangle]
pub unsafe extern fn ncollide3d_shape_delete(ptr: *mut nc::shape::ShapeHandle<f64>) {
    Box::from_raw(ptr);
}

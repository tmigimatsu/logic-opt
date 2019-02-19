extern crate nalgebra as na;
extern crate ncollide3d as nc;

#[repr(C)]
pub struct ncollide3d_math_isometry_t {
    translation: [f64; 3],
    axisangle: [f64; 3]
}

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

fn isometry_from_raw(isometry: &ncollide3d_math_isometry_t) -> nc::math::Isometry<f64> {
    let translation = na::Vector3::from_column_slice_generic(na::U3, na::U1, &isometry.translation);
    let axisangle = na::Vector3::from_column_slice_generic(na::U3, na::U1, &isometry.axisangle);
    nc::math::Isometry::new(translation, axisangle)
}

#[no_mangle]
pub extern fn ncollide3d_shape_ball_new(radius: f64) -> *mut nc::shape::ShapeHandle<f64> {
    let ball = nc::shape::Ball::new(radius);
    let handle = nc::shape::ShapeHandle::new(ball);
    Box::into_raw(Box::new(handle))
}

#[no_mangle]
pub extern fn ncollide3d_shape_cuboid_new(x: f64, y: f64, z: f64) -> *mut nc::shape::ShapeHandle<f64> {
    let cuboid = nc::shape::Cuboid::new(na::Vector3::new(x, y, z));
    let handle = nc::shape::ShapeHandle::new(cuboid);
    Box::into_raw(Box::new(handle))
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
pub extern fn ncollide3d_query_distance(
        m1: Option<&ncollide3d_math_isometry_t>, g1: Option<&nc::shape::ShapeHandle<f64>>,
        m2: Option<&ncollide3d_math_isometry_t>, g2: Option<&nc::shape::ShapeHandle<f64>>) -> f64 {
    let m1 = isometry_from_raw(m1.unwrap());
    let m2 = isometry_from_raw(m2.unwrap());
    let g1 = g1.unwrap().as_ref();
    let g2 = g2.unwrap().as_ref();
    ncollide3d::query::distance(&m1, g1, &m2, g2)
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

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}

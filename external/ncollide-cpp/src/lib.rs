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

fn isometry_from_ptr(ptr: *const ncollide3d_math_isometry_t) -> nc::math::Isometry<f64> {
    isometry_from_raw(unsafe { &*ptr })
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
                                            ptr_shapes: *const *const nc::shape::ShapeHandle<f64>,
                                            n: usize) -> *mut nc::shape::ShapeHandle<f64> {
    use nc::math::Isometry;
    use nc::shape::ShapeHandle;

    let transforms = unsafe { std::slice::from_raw_parts(ptr_transforms, n) };
    let transforms = transforms.iter().map(|x| isometry_from_raw(x));

    let shapes = unsafe { std::slice::from_raw_parts(ptr_shapes, n) };
    let shapes = shapes.iter().map(|x| unsafe { (**x).clone() });

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
pub extern fn ncollide3d_shape_cuboid_half_extents(ptr: *const nc::shape::ShapeHandle<f64>)
        -> *const f64 {
    use na::storage::Storage;

    let shape = unsafe { &*ptr };
    let maybe_cuboid = shape.as_shape::<nc::shape::Cuboid<f64>>();
    match maybe_cuboid {
        Some(ref cuboid) => { cuboid.half_extents().data.ptr() },
        None => { std::ptr::null() }
    }
}

#[no_mangle]
pub extern fn ncollide3d_query_distance(
        ptr_m1: *const ncollide3d_math_isometry_t, ptr_g1: *const nc::shape::ShapeHandle<f64>,
        ptr_m2: *const ncollide3d_math_isometry_t, ptr_g2: *const nc::shape::ShapeHandle<f64>)
        -> f64 {
    let m1 = isometry_from_ptr(ptr_m1);
    let m2 = isometry_from_ptr(ptr_m2);
    let g1 = unsafe { &*ptr_g1 };
    let g2 = unsafe { &*ptr_g2 };
    ncollide3d::query::distance(&m1, g1.as_ref(), &m2, g2.as_ref())
}

#[repr(C)]
pub struct ncollide3d_query_contact_t {
    world1: [f64; 3],
    world2: [f64; 3],
    normal: [f64; 3],
    depth: f64
}

#[no_mangle]
pub extern fn ncollide3d_query_closest_points(
        ptr_m1: *const ncollide3d_math_isometry_t, ptr_g1: *const nc::shape::ShapeHandle<f64>,
        ptr_m2: *const ncollide3d_math_isometry_t, ptr_g2: *const nc::shape::ShapeHandle<f64>,
        max_dist: f64, out_p1: &mut [f64; 3], out_p2: &mut [f64; 3])
        -> ncollide3d_query_closest_points_t {
    let m1 = isometry_from_ptr(ptr_m1);
    let m2 = isometry_from_ptr(ptr_m2);
    let g1 = unsafe { &*ptr_g1 };
    let g2 = unsafe { &*ptr_g2 };

    let result = ncollide3d::query::closest_points(&m1, g1.as_ref(), &m2, g2.as_ref(), max_dist);
    match result {
        ncollide3d::query::ClosestPoints::Intersecting => ncollide3d_query_closest_points_t::Intersecting,
        ncollide3d::query::ClosestPoints::WithinMargin(ref p1, ref p2) => {
            out_p1[0] = p1.x; out_p1[1] = p1.y; out_p1[2] = p1.z;
            out_p2[0] = p2.x; out_p2[1] = p2.y; out_p2[2] = p2.z;
            ncollide3d_query_closest_points_t::WithinMargin
        },
        ncollide3d::query::ClosestPoints::Disjoint => ncollide3d_query_closest_points_t::Disjoint
    }
}

#[no_mangle]
pub extern fn ncollide3d_query_contact(
        ptr_m1: *const ncollide3d_math_isometry_t, ptr_g1: *const nc::shape::ShapeHandle<f64>,
        ptr_m2: *const ncollide3d_math_isometry_t, ptr_g2: *const nc::shape::ShapeHandle<f64>,
        prediction: f64, ptr_out_contact: *mut ncollide3d_query_contact_t) -> bool {
    let m1 = isometry_from_ptr(ptr_m1);
    let m2 = isometry_from_ptr(ptr_m2);
    let g1 = unsafe { &*ptr_g1 };
    let g2 = unsafe { &*ptr_g2 };

    let result = ncollide3d::query::contact(&m1, g1.as_ref(), &m2, g2.as_ref(), prediction);
    match result {
        Some(ref contact) => {
            if !ptr_out_contact.is_null() {
                let out_contact = unsafe { &mut *ptr_out_contact };
                out_contact.world1 = [contact.world1.x, contact.world1.y, contact.world1.z];
                out_contact.world2 = [contact.world2.x, contact.world2.y, contact.world2.z];
                out_contact.normal = [contact.normal.x, contact.normal.y, contact.normal.z];
                out_contact.depth = contact.depth;
            }
            true
        },
        None => false
    }
}

#[no_mangle]
pub extern fn ncollide3d_query_proximity(
        ptr_m1: *const ncollide3d_math_isometry_t, ptr_g1: *const nc::shape::ShapeHandle<f64>,
        ptr_m2: *const ncollide3d_math_isometry_t, ptr_g2: *const nc::shape::ShapeHandle<f64>,
        margin: f64) -> ncollide3d_query_proximity_t {
    let m1 = isometry_from_ptr(ptr_m1);
    let m2 = isometry_from_ptr(ptr_m2);
    let g1 = unsafe { &*ptr_g1 };
    let g2 = unsafe { &*ptr_g2 };

    let result = ncollide3d::query::proximity(&m1, g1.as_ref(), &m2, g2.as_ref(), margin);
    match result {
        ncollide3d::query::Proximity::Intersecting => ncollide3d_query_proximity_t::Intersecting,
        ncollide3d::query::Proximity::WithinMargin => ncollide3d_query_proximity_t::WithinMargin,
        ncollide3d::query::Proximity::Disjoint => ncollide3d_query_proximity_t::Disjoint,
    }
}

#[no_mangle]
pub extern fn ncollide3d_query_time_of_impact(
        ptr_m1: *const ncollide3d_math_isometry_t, v1: &[f64; 3],
        ptr_g1: *const nc::shape::ShapeHandle<f64>,
        ptr_m2: *const ncollide3d_math_isometry_t, v2: &[f64; 3],
        ptr_g2: *const nc::shape::ShapeHandle<f64>,
        ptr_out_time: *mut f64) -> bool {
    let m1 = isometry_from_ptr(ptr_m1);
    let m2 = isometry_from_ptr(ptr_m2);
    let v1 = na::Vector3::from_column_slice_generic(na::U3, na::U1, v1);
    let v2 = na::Vector3::from_column_slice_generic(na::U3, na::U1, v2);
    let g1 = unsafe { &*ptr_g1 };
    let g2 = unsafe { &*ptr_g2 };

    let result = ncollide3d::query::time_of_impact(&m1, &v1, g1.as_ref(), &m2, &v2, g2.as_ref());
    match result {
        Some(time) => {
            if !ptr_out_time.is_null() {
                unsafe { *ptr_out_time = time };
            }
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

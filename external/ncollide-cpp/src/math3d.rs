/**
 * math3d.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;
extern crate ncollide3d as nc;

#[repr(C)]
pub struct ncollide3d_math_isometry_t {
    translation: [f64; 3],
    axisangle: [f64; 3]
}

pub fn isometry_from_raw(isometry: &ncollide3d_math_isometry_t) -> nc::math::Isometry<f64> {
    let translation = na::Vector3::from_column_slice_generic(na::U3, na::U1, &isometry.translation);
    let axisangle = na::Vector3::from_column_slice_generic(na::U3, na::U1, &isometry.axisangle);
    nc::math::Isometry::new(translation, axisangle)
}


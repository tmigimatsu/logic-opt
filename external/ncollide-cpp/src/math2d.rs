/**
 * math2d.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 19, 2019
 * Authors: Toki Migimatsu
 */

extern crate nalgebra as na;
extern crate ncollide2d as nc;

#[repr(C)]
pub struct ncollide2d_math_isometry_t {
    translation: [f64; 2],
    angle: f64
}

pub fn isometry_from_raw(isometry: &ncollide2d_math_isometry_t) -> nc::math::Isometry<f64> {
    let translation = na::Vector2::from_column_slice_generic(na::U2, na::U1, &isometry.translation);
    nc::math::Isometry::new(translation, isometry.angle)
}


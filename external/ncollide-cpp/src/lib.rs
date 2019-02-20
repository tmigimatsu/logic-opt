/**
 * lib.rs
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 12, 2019
 * Authors: Toki Migimatsu
 */

mod math3d;
mod shape3d;
mod query3d;

mod math2d;
mod shape2d;
mod query2d;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}

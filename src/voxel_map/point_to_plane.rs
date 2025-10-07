use rust_livo2_macros::uncertainties;

use crate::voxel_map::{plane::PlaneUncertainties, point::WorldPointUncertainties};

pub struct UncertainPoint2Plane {
    pub covariance: Point2PlaneUncertainties,
}

#[uncertainties]
#[derive(Debug, Clone)]
pub struct Point2PlaneUncertainties {
    plane: PlaneUncertainties,
    world_point: WorldPointUncertainties,
}

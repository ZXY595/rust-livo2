use nalgebra::Vector3;

use crate::esikf;

pub struct State {
    /// estimated velocity, from imu frame to world frame
    velocity: Vector3<f64>,
    /// gyroscope bias
    bias_gyro: Vector3<f64>,
    /// accelerator bias
    bias_acc: Vector3<f64>,
    /// the estimated gravity acceleration
    gravity: Vector3<f64>,
}

fn gravity_alignment(state: &mut esikf::State) {
    
}

fn undistort_pcl() {

}

fn forward_propagation() {

}

fn backward_propagation() {

}

fn covariance_propagation(state: &mut esikf::State) {

} 

fn imu_attitude_propagation(state: &mut esikf::State) {

} 

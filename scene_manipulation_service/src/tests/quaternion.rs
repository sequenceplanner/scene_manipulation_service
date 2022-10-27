#![allow(unused_imports)]
#![allow(dead_code)]
use r2r::geometry_msgs::msg::Quaternion;
use glam::Quat;

#[tokio::test]
async fn test_glam_mul() {
    let q1 = Quat::from_xyzw(0.2657277, 0.6643192, 0.4428795, 0.5403023);
    let q2 = Quat::from_xyzw(0.8596911, 0.0256624, 0.2951178, -0.4161468);

    // order reversed here because in the function we reverse the chain
    let q3 = q1.mul_quat(q2);
    println!("{:?}", q3);
}

#[tokio::test]
async fn test_glam_mul_and_norm() {
    let q1 = Quat::from_xyzw(0.2657277, 0.6643192, 0.4428795, 0.5403023);
    let q2 = Quat::from_xyzw(0.8596911, 0.0256624, 0.2951178, -0.4161468);

    // order reversed here because in the function we reverse the chain
    let q3 = q1.mul_quat(q2);
    println!("{:?}", q3);
    println!("{:?}", q3.normalize());
}

#[tokio::test]
async fn test_glam_mul_and_norm_and_inverse() {
    let q1 = Quat::from_xyzw(0.2657277, 0.6643192, 0.4428795, 0.5403023);
    let q2 = Quat::from_xyzw(0.8596911, 0.0256624, 0.2951178, -0.4161468);

    // order reversed here because in the function we reverse the chain
    let q3 = q1.mul_quat(q2);
    println!("{:?}", q3);
    println!("{:?}", q3.normalize());
    println!("{:?}", q3.normalize().inverse());
}
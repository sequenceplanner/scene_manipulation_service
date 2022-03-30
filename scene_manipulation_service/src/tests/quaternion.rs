#![allow(unused_imports)]
#![allow(dead_code)]
use r2r::geometry_msgs::msg::Quaternion;
use glam::Quat;

#[tokio::test]
async fn test_multiply() {
    let q1 = r2r::geometry_msgs::msg::Quaternion {
        x: 0.2657277,
        y: 0.6643192,
        z: 0.4428795,
        w: 0.5403023
    };
    let q2 = r2r::geometry_msgs::msg::Quaternion {
        x: 0.8596911,
        y: 0.0256624,
        z: 0.2951178,
        w: -0.4161468
    };

    // order reversed here because in the function we reverse the chain
    let q3 = crate::multiply_quaternion(&q2, &q1);
    assert_eq!(q3, r2r::geometry_msgs::msg::Quaternion {
        x: 0.53859841651413,
        y: 0.03972973478434999,
        z: -0.5891401538703,
        w: -0.60103846063429
    });
}

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

#[tokio::test]
async fn test_multiply_and_normalize() {
    let q1 = r2r::geometry_msgs::msg::Quaternion {
        x: 0.2657277,
        y: 0.6643192,
        z: 0.4428795,
        w: 0.5403023
    };
    let q2 = r2r::geometry_msgs::msg::Quaternion {
        x: 0.8596911,
        y: 0.0256624,
        z: 0.2951178,
        w: -0.4161468
    };

    // order reversed here because in the function we reverse the chain
    let q3 = crate::multiply_and_normalize(&q2, &q1);
    assert_eq!(q3, r2r::geometry_msgs::msg::Quaternion {
        x: 0.5385984008512792,
        y: 0.03972973362897914,
        z: -0.5891401367376571,
        w: -0.6010384431556353
    });
}


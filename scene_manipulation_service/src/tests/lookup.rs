#![allow(unused_imports)]
#![allow(dead_code)]
use r2r::geometry_msgs::msg::{Quaternion, Transform, Vector3};

#[tokio::test]
async fn test_lookup() {
    let mut buffer = crate::HashMap::<String, crate::ExtendedFrameData>::new();

    buffer.insert(
        "dummy_1".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
                parent_frame_id: "world".to_string(),
                child_frame_id: "dummy_1".to_string(),
                transform: r2r::geometry_msgs::msg::Transform {
                    translation: r2r::geometry_msgs::msg::Vector3 {
                        x: 1.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    rotation: r2r::geometry_msgs::msg::Quaternion {
                        x: 0.2657277,
                        y: 0.6643192,
                        z: 0.4428795,
                        w: 0.5403023,
                    },
                },
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    buffer.insert(
        "dummy_2".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
                parent_frame_id: "dummy_1".to_string(),
                child_frame_id: "dummy_2".to_string(),
                transform: r2r::geometry_msgs::msg::Transform {
                    translation: r2r::geometry_msgs::msg::Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: 1.0,
                    },
                    rotation: r2r::geometry_msgs::msg::Quaternion {
                        x: 0.8596911,
                        y: 0.0256624,
                        z: 0.2951178,
                        w: -0.4161468,
                    },
                },
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    let buffer_arc = crate::Arc::new(crate::Mutex::new(buffer));

    let res = crate::lookup_transform("world", "dummy_2", &buffer_arc)
        .await
        .unwrap();
    assert_eq!(
        res,
        Transform {
            translation: Vector3 {
                x: 1.0,
                y: 0.0,
                z: 1.0
            },
            rotation: Quaternion {
                x: 0.5385984008512792,
                y: 0.03972973362897914,
                z: -0.5891401367376571,
                w: -0.6010384431556353
            }
        }
    );
}

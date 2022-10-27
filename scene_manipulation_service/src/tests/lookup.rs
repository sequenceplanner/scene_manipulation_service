#![allow(unused_imports)]
#![allow(dead_code)]
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

use r2r::geometry_msgs::msg::{Quaternion, Transform, Vector3};

use scene_manipulation_service::{
    common::{
        files::{list_frames_in_dir, load_scenario, reload_scenario},
        frame_data::FrameData,
    },
    core::lookup::{check_would_produce_cycle, lookup_transform},
};

#[tokio::test]
async fn test_lookup_1() {
    let mut buffer = HashMap::<String, FrameData>::new();

    // add the world frame explicitly so that transforms can be looked up to and from it
    buffer.insert(
        "world".to_string(),
        FrameData {
            parent_frame_id: "world_origin".to_string(),
            child_frame_id: "world".to_string(),
            active: Some(false),
            ..Default::default()
        },
    );

    buffer.insert(
        "dummy_1".to_string(),
        FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_1".to_string(),
            transform: r2r::geometry_msgs::msg::Transform {
                translation: r2r::geometry_msgs::msg::Vector3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                ..Default::default()
            },
            ..Default::default()
        },
    );

    buffer.insert(
        "dummy_2".to_string(),
        FrameData {
            parent_frame_id: "dummy_1".to_string(),
            child_frame_id: "dummy_2".to_string(),
            transform: r2r::geometry_msgs::msg::Transform {
                translation: r2r::geometry_msgs::msg::Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 1.0,
                },
                ..Default::default()
            },
            ..Default::default()
        },
    );

    let buffer_arc = Arc::new(Mutex::new(buffer));

    let res = lookup_transform("world", "dummy_2", &buffer_arc)
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
            ..Default::default()
        }
    );
}

#[tokio::test]
async fn test_lookup_2() {
    let mut buffer = HashMap::<String, FrameData>::new();

    // add the world frame explicitly so that transforms can be looked up to and from it
    buffer.insert(
        "world".to_string(),
        FrameData {
            parent_frame_id: "world_origin".to_string(),
            child_frame_id: "world".to_string(),
            active: Some(false),
            ..Default::default()
        },
    );

    buffer.insert(
        "dummy_1".to_string(),
        FrameData {
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
            ..Default::default()
        },
    );

    buffer.insert(
        "dummy_2".to_string(),
        FrameData {
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
            ..Default::default()
        },
    );

    let buffer_arc = Arc::new(Mutex::new(buffer));

    let res = lookup_transform("world", "dummy_2", &buffer_arc)
        .await
        .unwrap();
    assert_eq!(
        res,
        Transform {
            translation: Vector3 {
                x: 1.95323708521262,
                y: 0.30128013530538,
                z: -0.023862420071860013
            },
            rotation: Quaternion {
                x: -0.5385983808201009,
                y: -0.03972973937227523,
                z: 0.5891401390261436,
                w: 0.6010384440005098
            }
        }
    );
}

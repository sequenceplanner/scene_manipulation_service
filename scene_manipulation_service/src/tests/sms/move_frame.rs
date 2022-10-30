#![allow(unused_imports)]
#![allow(dead_code)]

use futures::{stream::Stream, StreamExt};
use r2r::scene_manipulation_msgs::srv::ManipulateScene;
use r2r::ServiceRequest;
use serde_json::json;
use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};

use scene_manipulation_service::common::errors::{main_error_response, main_success_response};
use scene_manipulation_service::common::frame_data::FrameData;
use scene_manipulation_service::core::sms::move_frame;
use scene_manipulation_service::{check_would_produce_cycle, lookup_transform, ExtraData};

fn make_initial_setup() -> HashMap<String, FrameData> {
    let mut test_setup = HashMap::<String, FrameData>::new();

    test_setup.insert(
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
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
            extra_data: ExtraData {
                zone: Some(123.123), 
                active: Some(true),
                ..Default::default()
            },
        },
    );

    test_setup.insert(
        "dummy_2".to_string(),
        FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_2".to_string(),
            transform: r2r::geometry_msgs::msg::Transform {
                translation: r2r::geometry_msgs::msg::Vector3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                },
                rotation: r2r::geometry_msgs::msg::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
            extra_data: ExtraData {
                active: Some(false),
                ..Default::default()
            },
        },
    );

    test_setup.insert(
        "dummy_3".to_string(),
        FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_3".to_string(),
            transform: r2r::geometry_msgs::msg::Transform {
                translation: r2r::geometry_msgs::msg::Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 1.0,
                },
                rotation: r2r::geometry_msgs::msg::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
            extra_data: ExtraData {
                active: Some(true),
                ..Default::default()
            },
        },
    );
    test_setup
}

#[tokio::test]
async fn test_move_frame() {
    let initial_frames = make_initial_setup();
    let tf_frames = initial_frames.clone();

    let broadcasted_frames = Arc::new(Mutex::new(initial_frames));
    let buffered_frames = Arc::new(Mutex::new(tf_frames));

    let message = ManipulateScene::Request {
        command: "move".to_string(),
        child_frame_id: "dummy_1".to_string(),
        parent_frame_id: "dummy_3".to_string(),
        extra: json!({
            
        }).to_string(),
        ..Default::default()
    };

    let response = move_frame(&message, &broadcasted_frames, &buffered_frames, "").await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: true,
            info: "Frame 'dummy_1' moved to 'dummy_3'.".to_string()
        }
    );
    {
        let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
        let buffered_local = buffered_frames.lock().unwrap();

        assert_eq!(
            buffered_local.get("dummy_3").unwrap().transform.translation,
            broadcasted_local
                .get("dummy_1")
                .unwrap()
                .transform
                .translation
        );
        assert_eq!(
            broadcasted_local
                .get("dummy_1")
                .unwrap()
                .extra_data
                .zone, Some(123.123)
        );
    }
}

#[tokio::test]
async fn test_move_frame_static() {
    let initial_frames = make_initial_setup();
    let tf_frames = initial_frames.clone();

    let broadcasted_frames = Arc::new(Mutex::new(initial_frames));
    let buffered_frames = Arc::new(Mutex::new(tf_frames));

    let message = ManipulateScene::Request {
        command: "move".to_string(),
        child_frame_id: "dummy_2".to_string(),
        parent_frame_id: "dummy_3".to_string(),
        extra: json!({
            
        }).to_string(),
        ..Default::default()
    };

    let response = move_frame(&message, &broadcasted_frames, &buffered_frames, "").await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: false,
            info: "Can't manipulate static frames.".to_string()
        }
    );
    {
        let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
        let buffered_local = buffered_frames.lock().unwrap();

        assert_ne!(
            buffered_local.get("dummy_3").unwrap().transform.translation,
            broadcasted_local
                .get("dummy_2")
                .unwrap()
                .transform
                .translation
        );
    }
}

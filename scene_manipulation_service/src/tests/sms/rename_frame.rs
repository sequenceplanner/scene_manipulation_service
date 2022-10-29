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
use scene_manipulation_service::core::sms::rename_frame;
use scene_manipulation_service::ExtraData;

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
                active: Some(true),
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
                    z: 123.0,
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
                zone: Some(34.56),
                ..Default::default()
            },
        },
    );
    test_setup
}

#[tokio::test]
async fn test_rename_frame() {
    let initial_frames = make_initial_setup();
    let tf_frames = initial_frames.clone();

    let broadcasted_frames = Arc::new(Mutex::new(initial_frames.clone()));
    let buffered_frames = Arc::new(Mutex::new(tf_frames));
    let dummy_3 = initial_frames.get("dummy_3").unwrap();
    
    let message = ManipulateScene::Request {
        command: "rename".to_string(),
        child_frame_id: "dummy_3".to_string(),
        parent_frame_id: "world".to_string(),
        new_frame_id: "new_name".to_string(),
        extra: json!({}).to_string(),
        ..Default::default()
    };
    let response = rename_frame(&message, &broadcasted_frames, &buffered_frames).await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: true,
            info: "Successfully renamed frame 'dummy_3' to 'new_name'.".to_string()
        }
    );

    let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
    let buffered_local = buffered_frames.lock().unwrap();

    let new_name = broadcasted_local.get("new_name").unwrap();

    assert!(!broadcasted_local.contains_key("dummy_3"));
    assert!(!buffered_local.contains_key("dummy_3"));

    assert!(broadcasted_local.contains_key("new_name"));
    assert_eq!(dummy_3.transform, new_name.transform);
    assert_eq!(dummy_3.parent_frame_id, new_name.parent_frame_id);
    assert_eq!(dummy_3.extra_data.zone, new_name.extra_data.zone);
    assert_eq!(dummy_3.extra_data.next, new_name.extra_data.next);
    assert_eq!(dummy_3.extra_data.active, new_name.extra_data.active);
}
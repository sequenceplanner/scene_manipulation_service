#![allow(unused_imports)]
#![allow(dead_code)]

use futures::{stream::Stream, StreamExt};
use r2r::scene_manipulation_msgs::srv::ManipulateScene;
use r2r::ServiceRequest;
use serde_json::json;
use std::collections::{HashMap, HashSet};
use std::env;
use std::sync::{Arc, Mutex};

use scene_manipulation_service::common::errors::{main_error_response, main_success_response};
use scene_manipulation_service::common::frame_data::FrameData;
use scene_manipulation_service::core::sms::remove_frame;
use scene_manipulation_service::{ExtraData, add_frame};

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
async fn test_remove_frame() {
    let initial_frames = make_initial_setup();
    let tf_frames = initial_frames.clone();
    
    let broadcasted_frames = Arc::new(Mutex::new(initial_frames));
    let buffered_frames = Arc::new(Mutex::new(tf_frames));

    let message = ManipulateScene::Request {
        command: "remove".to_string(),
        child_frame_id: "dummy_3".to_string(),
        parent_frame_id: "world".to_string(),
        extra: json!({}).to_string(),
        ..Default::default()
    };
    let response = remove_frame(&message, &broadcasted_frames, &buffered_frames, "").await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: true,
            info: "Frame 'dummy_3' temporarily removed from the scene.".to_string()
        }
    );
    let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
    let buffered_local = buffered_frames.lock().unwrap();

    assert!(!broadcasted_local.contains_key("dummy_3"));
    assert!(!buffered_local.contains_key("dummy_3"));
}

#[tokio::test]
async fn test_remove_frame_and_persist() {
    let mut initial_frames = make_initial_setup();
    let mut tf_frames = initial_frames.clone();
    
    let broadcasted_frames = Arc::new(Mutex::new(initial_frames.clone()));
    let buffered_frames = Arc::new(Mutex::new(tf_frames.clone()));

    let message = ManipulateScene::Request {
        command: "add".to_string(),
        child_frame_id: "dummy_5".to_string(),
        parent_frame_id: "world".to_string(),
        extra: json!({
            "zone": 1234.1234
        }).to_string(),
        persist: true,
        ..Default::default()
    };
    let dir = env::temp_dir();
    let response = add_frame(&message, &broadcasted_frames, &buffered_frames, dir.to_str().unwrap()).await;

    initial_frames.insert(
        "dummy_5".to_string(),
        FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_5".to_string(),
            transform: r2r::geometry_msgs::msg::Transform {
                translation: r2r::geometry_msgs::msg::Vector3 {
                    x: 1.0,
                    y: 1.0,
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

    tf_frames.insert(
        "dummy_5".to_string(),
        FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_5".to_string(),
            transform: r2r::geometry_msgs::msg::Transform {
                translation: r2r::geometry_msgs::msg::Vector3 {
                    x: 1.0,
                    y: 1.0,
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

    let broadcasted_frames = Arc::new(Mutex::new(initial_frames));
    let buffered_frames = Arc::new(Mutex::new(tf_frames.clone()));

    if response.success {
        let message = ManipulateScene::Request {
            command: "remove".to_string(),
            child_frame_id: "dummy_5".to_string(),
            parent_frame_id: "world".to_string(),
            extra: json!({
                "zone": 1234.1234
            }).to_string(),
            persist: true,
            ..Default::default()
        };
        let response = remove_frame(&message, &broadcasted_frames, &buffered_frames, dir.to_str().unwrap()).await;
        assert_eq!(
            response,
            ManipulateScene::Response {
                success: true,
                info: "Frame 'dummy_5' permanently removed from the scene.".to_string()
            }
        );
        let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
        let buffered_local = buffered_frames.lock().unwrap();
    
        assert!(!broadcasted_local.contains_key("dummy_5"));
        assert!(!buffered_local.contains_key("dummy_5"));
    } else {
        println!("ASDFASDFASDFASDFASDF")
    }

    // tokio::time::sleep(std::time::Duration::from_millis(1000)).await;

    
}

#[tokio::test]
async fn test_remove_frame_world() {
    let initial_frames = make_initial_setup();
    let tf_frames = initial_frames.clone();

    let broadcasted_frames = Arc::new(Mutex::new(initial_frames));
    let buffered_frames = Arc::new(Mutex::new(tf_frames));

    let message = ManipulateScene::Request {
        command: "add".to_string(),
        child_frame_id: "world".to_string(),
        parent_frame_id: "dummy_2".to_string(),
        extra: json!({}).to_string(),
        ..Default::default()
    };

    let response = remove_frame(&message, &broadcasted_frames, &buffered_frames, "").await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: false,
            info: "Frame 'world' is reserved as the universal tree root.".to_string()
        }
    );

    let message = ManipulateScene::Request {
        command: "add".to_string(),
        child_frame_id: "world_origin".to_string(),
        parent_frame_id: "dummy_2".to_string(),
        extra: json!({}).to_string(),
        ..Default::default()
    };

    let response = remove_frame(&message, &broadcasted_frames, &buffered_frames, "").await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: false,
            info: "Frame 'world_origin' is reserved as the universal tree root.".to_string()
        }
    );
}


#[tokio::test]
async fn test_remove_frame_doesnt_exist() {
    let initial_frames = make_initial_setup();
    let tf_frames = initial_frames.clone();
    
    let broadcasted_frames = Arc::new(Mutex::new(initial_frames));
    let buffered_frames = Arc::new(Mutex::new(tf_frames));

    let message = ManipulateScene::Request {
        command: "remove".to_string(),
        child_frame_id: "dummy_4".to_string(),
        parent_frame_id: "world".to_string(),
        extra: json!({}).to_string(),
        ..Default::default()
    };
    let response = remove_frame(&message, &broadcasted_frames, &buffered_frames, "").await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: false,
            info: "Frame doesn't exist in tf, nor is it published by this broadcaster.".to_string()
        }
    );
    let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
    let buffered_local = buffered_frames.lock().unwrap();

    assert!(!broadcasted_local.contains_key("dummy_4"));
    assert!(!buffered_local.contains_key("dummy_4"));
}

#[tokio::test]
async fn test_remove_frame_exists_only_in_broadcaster() {
    let mut initial_frames = make_initial_setup();
    let tf_frames = initial_frames.clone();

    initial_frames.insert(
        "dummy_4".to_string(),
        FrameData {
            parent_frame_id: "dummy_3".to_string(),
            child_frame_id: "dummy_4".to_string(),
            ..Default::default()
        }
    );
    
    let broadcasted_frames = Arc::new(Mutex::new(initial_frames));
    let buffered_frames = Arc::new(Mutex::new(tf_frames));

    let message = ManipulateScene::Request {
        command: "remove".to_string(),
        child_frame_id: "dummy_4".to_string(),
        parent_frame_id: "world".to_string(),
        extra: json!({}).to_string(),
        ..Default::default()
    };
    let response = remove_frame(&message, &broadcasted_frames, &buffered_frames, "").await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: false,
            info: "Frame doesn't exist in tf, but it is published by this broadcaster? Investigate.".to_string()
        }
    );
    let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
    let buffered_local = buffered_frames.lock().unwrap();

    assert!(broadcasted_local.contains_key("dummy_4"));
    assert!(!buffered_local.contains_key("dummy_4"));
}

#[tokio::test]
async fn test_remove_frame_exists_only_in_buffer() {
    let initial_frames = make_initial_setup();
    let mut tf_frames = initial_frames.clone();

    tf_frames.insert(
        "dummy_4".to_string(),
        FrameData {
            parent_frame_id: "dummy_3".to_string(),
            child_frame_id: "dummy_4".to_string(),
            ..Default::default()
        }
    );
    
    let broadcasted_frames = Arc::new(Mutex::new(initial_frames));
    let buffered_frames = Arc::new(Mutex::new(tf_frames));

    let message = ManipulateScene::Request {
        command: "remove".to_string(),
        child_frame_id: "dummy_4".to_string(),
        parent_frame_id: "world".to_string(),
        extra: json!({}).to_string(),
        ..Default::default()
    };
    let response = remove_frame(&message, &broadcasted_frames, &buffered_frames, "").await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: false,
            info: "Frame exists in the tf, but it can't be removed since it is not published by sms.".to_string()
        }
    );
    let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
    let buffered_local = buffered_frames.lock().unwrap();

    assert!(!broadcasted_local.contains_key("dummy_4"));
    assert!(buffered_local.contains_key("dummy_4"));
}

#[tokio::test]
async fn test_remove_static_frame() {
    let mut initial_frames = make_initial_setup();
    let mut tf_frames = initial_frames.clone();

    tf_frames.insert(
        "dummy_4".to_string(),
        FrameData {
            parent_frame_id: "dummy_3".to_string(),
            child_frame_id: "dummy_4".to_string(),
            extra_data: ExtraData {
                active: Some(false),
                ..Default::default()
            },
            ..Default::default()
        }
    );

    initial_frames.insert(
        "dummy_4".to_string(),
        FrameData {
            parent_frame_id: "dummy_3".to_string(),
            child_frame_id: "dummy_4".to_string(),
            extra_data: ExtraData {
                active: Some(false),
                ..Default::default()
            },
            ..Default::default()
        }
    );
    
    let broadcasted_frames = Arc::new(Mutex::new(initial_frames));
    let buffered_frames = Arc::new(Mutex::new(tf_frames));

    let message = ManipulateScene::Request {
        command: "remove".to_string(),
        child_frame_id: "dummy_4".to_string(),
        parent_frame_id: "world".to_string(),
        extra: json!({}).to_string(),
        ..Default::default()
    };
    let response = remove_frame(&message, &broadcasted_frames, &buffered_frames, "").await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: false,
            info: "Can't remove static frame 'dummy_4'.".to_string()
        }
    );
    let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
    let buffered_local = buffered_frames.lock().unwrap();

    assert!(broadcasted_local.contains_key("dummy_4"));
    assert!(buffered_local.contains_key("dummy_4"));
}
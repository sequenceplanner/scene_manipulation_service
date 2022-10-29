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
use scene_manipulation_service::core::sms::add_frame;
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
async fn test_add_frame() {
    let initial_frames = make_initial_setup();
    let mut tf_frames = initial_frames.clone();
    tf_frames.insert(
        "dummy_4".to_string(),
        FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_4".to_string(),
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
    let buffered_frames = Arc::new(Mutex::new(tf_frames));

    let message = ManipulateScene::Request {
        command: "add".to_string(),
        child_frame_id: "dummy_5".to_string(),
        parent_frame_id: "world".to_string(),
        extra: json!({}).to_string(),
        ..Default::default()
    };
    let response = add_frame(&message, &broadcasted_frames, &buffered_frames).await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: true,
            info: "Frame 'dummy_5' added to the scene.".to_string()
        }
    );
    let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
    let buffered_local = buffered_frames.lock().unwrap();

    assert!(broadcasted_local.contains_key("dummy_5"));
    assert!(!buffered_local.contains_key("dummy_5"));
}

#[tokio::test]
async fn test_add_frame_world() {
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

    let response = add_frame(&message, &broadcasted_frames, &buffered_frames).await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: false,
            info: "Frame 'world' is reserved as the universal tree root.".to_string()
        }
    );
    {
        let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
        let buffered_local = buffered_frames.lock().unwrap();

        assert!(!broadcasted_local.contains_key("world"));
        assert!(!buffered_local.contains_key("world"));
    }

    let message = ManipulateScene::Request {
        command: "add".to_string(),
        child_frame_id: "world_origin".to_string(),
        parent_frame_id: "dummy_2".to_string(),
        extra: json!({}).to_string(),
        ..Default::default()
    };

    let response = add_frame(&message, &broadcasted_frames, &buffered_frames).await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: false,
            info: "Frame 'world_origin' is reserved as the universal tree root.".to_string()
        }
    );
    let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
    let buffered_local = buffered_frames.lock().unwrap();

    assert!(!broadcasted_local.contains_key("world_origin"));
    assert!(!buffered_local.contains_key("world_origin"));
}

#[tokio::test]
async fn test_add_frame_already_in_buffer() {
    let initial_frames = make_initial_setup();
    let mut tf_frames = initial_frames.clone();

    tf_frames.insert(
        "dummy_4".to_string(),
        FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_4".to_string(),
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
    let buffered_frames = Arc::new(Mutex::new(tf_frames));

    let message = ManipulateScene::Request {
        command: "add".to_string(),
        child_frame_id: "dummy_4".to_string(),
        parent_frame_id: "world".to_string(),
        extra: json!({}).to_string(),
        ..Default::default()
    };

    let response = add_frame(&message, &broadcasted_frames, &buffered_frames).await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: false,
            info: "Frame already exists in the tf, will not proceed to broadcasted locally."
                .to_string()
        }
    );
    {
        let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
        let buffered_local = buffered_frames.lock().unwrap();

        assert!(!broadcasted_local.contains_key("dummy_4"));
        assert!(buffered_local.contains_key("dummy_4"));
    }
}

#[tokio::test]
async fn test_add_frame_already_in_broadcaster() {
    let mut initial_frames = make_initial_setup();
    let mut tf_frames = initial_frames.clone();

    initial_frames.insert(
        "dummy_4".to_string(),
        FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_4".to_string(),
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

    let broadcasted_frames = Arc::new(Mutex::new(initial_frames.clone()));
    let buffered_frames = Arc::new(Mutex::new(tf_frames.clone()));

    let message = ManipulateScene::Request {
        command: "add".to_string(),
        child_frame_id: "dummy_4".to_string(),
        parent_frame_id: "world".to_string(),
        extra: json!({}).to_string(),
        ..Default::default()
    };

    let response = add_frame(&message, &broadcasted_frames, &buffered_frames).await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: false,
            info:
                "Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."
                    .to_string()
        }
    );
    {
        let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
        let buffered_local = buffered_frames.lock().unwrap();

        assert!(broadcasted_local.contains_key("dummy_4"));
        assert!(!buffered_local.contains_key("dummy_4"));
    }

    tf_frames.insert(
        "dummy_4".to_string(),
        FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_4".to_string(),
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

    let response = add_frame(&message, &broadcasted_frames, &buffered_frames).await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: false,
            info: "Frame already exists in the tf, will not proceed to broadcasted locally."
                .to_string()
        }
    );
    {
        let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
        let buffered_local = buffered_frames.lock().unwrap();

        assert!(broadcasted_local.contains_key("dummy_4"));
        assert!(buffered_local.contains_key("dummy_4"));
    }
}

#[tokio::test]
async fn test_add_frame_with_extras() {
    let initial_frames = make_initial_setup();
    let tf_frames = initial_frames.clone();

    let broadcasted_frames = Arc::new(Mutex::new(initial_frames.clone()));
    let buffered_frames = Arc::new(Mutex::new(tf_frames.clone()));

    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
    let now = clock.get_now().unwrap();
    let time_stamp = r2r::Clock::to_builtin_time(&now);

    let message = ManipulateScene::Request {
        command: "add".to_string(),
        child_frame_id: "dummy_4".to_string(),
        parent_frame_id: "world".to_string(),
        extra: json!({
            "time_stamp": time_stamp,
            "zone": 32.67,
            "next": HashSet::from(["A".to_string(), "B".to_string(), "C".to_string()]),
        })
        .to_string(),
        ..Default::default()
    };

    let response = add_frame(&message, &broadcasted_frames, &buffered_frames).await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: true,
            info:
                "Frame 'dummy_4' added to the scene."
                    .to_string()
        }
    );
    {
        let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
        let buffered_local = buffered_frames.lock().unwrap();

        assert!(broadcasted_local.contains_key("dummy_4"));
        assert!(!buffered_local.contains_key("dummy_4"));
        assert_eq!(broadcasted_local.get("dummy_4").unwrap().extra_data.zone, Some(32.67));
        assert_eq!(broadcasted_local.get("dummy_4").unwrap().extra_data.mesh_path, None);
    }
}

#[tokio::test]
async fn test_add_frame_would_produce_cycle() {
    let initial_frames = make_initial_setup();
    let mut tf_frames = initial_frames.clone();

    tf_frames.insert(
        "dummy_5".to_string(),
        FrameData {
            parent_frame_id: "dummy_4".to_string(),
            child_frame_id: "dummy_5".to_string(),
            ..Default::default()
        }
    );

    tf_frames.insert(
        "dummy_6".to_string(),
        FrameData {
            parent_frame_id: "dummy_5".to_string(),
            child_frame_id: "dummy_6".to_string(),
            ..Default::default()
        }
    );

    let broadcasted_frames = Arc::new(Mutex::new(initial_frames.clone()));
    let buffered_frames = Arc::new(Mutex::new(tf_frames.clone()));

    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
    let now = clock.get_now().unwrap();
    let time_stamp = r2r::Clock::to_builtin_time(&now);

    let message = ManipulateScene::Request {
        command: "add".to_string(),
        child_frame_id: "dummy_4".to_string(),
        parent_frame_id: "dummy_6".to_string(),
        extra: json!({
            "time_stamp": time_stamp,
            "zone": 32.67,
            "next": HashSet::from(["A".to_string(), "B".to_string(), "C".to_string()]),
        })
        .to_string(),
        ..Default::default()
    };

    let response = add_frame(&message, &broadcasted_frames, &buffered_frames).await;
    assert_eq!(
        response,
        ManipulateScene::Response {
            success: false,
            info:
                "Adding frame 'dummy_4' would produce a cycle."
                    .to_string()
        }
    );
    {
        let broadcasted_local = broadcasted_frames.lock().unwrap().clone();
        let buffered_local = buffered_frames.lock().unwrap();

        assert!(!broadcasted_local.contains_key("dummy_4"));
        assert!(!buffered_local.contains_key("dummy_4"));
    }
}

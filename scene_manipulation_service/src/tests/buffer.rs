#![allow(unused_imports)]
#![allow(dead_code)]
use r2r::geometry_msgs::msg::{Quaternion, Transform, Vector3};
use std::{process::exit, collections::HashMap};

use crate::ExtendedFrameData;

pub static NODE_ID: &'static str = "buffer_test";

// #[tokio::main]
// async fn main() -> Result<(), Box<dyn std::error::Error>> {
//     let ctx = r2r::Context::create()?;
//     let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

//     let client = node.create_client::<crate::ManipulateScene::Service>("manipulate_scene")?;
//     let waiting = node.is_available(&client)?;

//     let handle = std::thread::spawn(move || loop {
//         &node.spin_once(std::time::Duration::from_millis(100));
//     });

//     r2r::log_warn!(NODE_ID, "Waiting for Scene Manipulation Service...");
//     waiting.await?;
//     r2r::log_info!(NODE_ID, "Scene Manipulation Service available.");
//     r2r::log_info!(
//         NODE_ID,
//         "Node started."
//     );

//     std::thread::sleep(std::time::Duration::from_millis(3000));

//     let mut messages = vec![];

//     let message_1 = ManipulateScene::Request {
//         command: "update".to_string(),
//         parent_frame: "frame_4".to_string(),
//         child_frame: "frame_2".to_string(),
//         same_position_in_world: true,
//         transform: Transform::default(),
//     };
//     messages.push(message_1);

//     let message_2 = ManipulateScene::Request {
//         command: "update".to_string(),
//         parent_frame: "world".to_string(),
//         child_frame: "frame_4".to_string(),
//         same_position_in_world: false,
//         transform: Transform::default(),
//     };
//     messages.push(message_2);

//     let message_3 = ManipulateScene::Request {
//         command: "remove".to_string(),
//         parent_frame: "world".to_string(),
//         child_frame: "frame_4".to_string(),
//         same_position_in_world: false,
//         transform: Transform::default(),
//     };
//     messages.push(message_3);

//     for message in messages {
//         sms_test(
//             &client,
//             message
//         ).await?;
//     }
    
//     handle.join().unwrap();

//     Ok(())
// }

// async fn sms_test(
//     client: &r2r::Client<ManipulateScene::Service>,
//     message: ManipulateScene::Request,
// ) -> Result<(), Box<dyn std::error::Error>> {

//     let response = client
//         .request(&message)
//         .expect("Could not send SMS request.")
//         .await
//         .expect("Cancelled.");

//     r2r::log_info!("sms_test", "Request to SMS sent.");

//     match response.success {
//         true => {
//             r2r::log_info!("sms_test", "Got sms response: {}", response.success);
//         }
//         false => {
//             r2r::log_error!(
//                 "sms_test",
//                 "Couldn't manipulate scene for command command '{}'.",
//                 message.command
//             );
//         }
//     }

//     std::thread::sleep(std::time::Duration::from_millis(3000));

//     Ok(())
// }

#[tokio::test]
async fn test_maintain_buffer() -> Result<(), Box<dyn std::error::Error>> {
    let mut buffer = crate::HashMap::<String, crate::ExtendedFrameData>::new();
    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
    let now = clock.get_now().unwrap();
    let current_time = r2r::Clock::to_builtin_time(&now);

    let dummy_active_folder_fd = crate::ExtendedFrameData {
        frame_data: crate::FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_active_folder".to_string(),
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
        time_stamp: current_time.clone()
    };

    buffer.insert(
        "dummy_active_folder".to_string(),
        dummy_active_folder_fd.clone()
    );

    let dummy_static_folder_fd = crate::ExtendedFrameData {
        frame_data: crate::FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_static_folder".to_string(),
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
            active: false,
        },
        folder_loaded: true,
        time_stamp: current_time.clone()
    };

    buffer.insert(
        "dummy_static_folder".to_string(),
        dummy_static_folder_fd.clone()
    );

    let dummy_active_in_tf_fd = crate::ExtendedFrameData {
        frame_data: crate::FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_active_in_tf".to_string(),
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
        folder_loaded: false,
        time_stamp: current_time.clone()
    };

    buffer.insert(
        "dummy_active_in_tf".to_string(),
        dummy_active_in_tf_fd.clone()
    );

    let dummy_static_in_tf_fd = crate::ExtendedFrameData {
        frame_data: crate::FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_static_in_tf".to_string(),
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
            active: false,
        },
        folder_loaded: false,
        time_stamp: current_time.clone()
    };

    buffer.insert(
        "dummy_static_in_tf".to_string(),
        dummy_static_in_tf_fd.clone()
    );

    let ctx = r2r::Context::create().unwrap();
    let mut node = r2r::Node::create(ctx, crate::NODE_ID, "").unwrap();

    let buffer_maintain_timer = node.create_wall_timer(std::time::Duration::from_millis(crate::BUFFER_MAINTAIN_RATE)).unwrap();
    let buffer_arc = crate::Arc::new(crate::Mutex::new(buffer));

    // spawn a tokio task to maintain the buffer
    let buffered_frames_clone_1 = buffer_arc.clone();
    let handle1 = tokio::task::spawn(async move {
        match crate::maintain_buffer(buffer_maintain_timer, &buffered_frames_clone_1).await {
            Ok(()) => (),
            Err(e) => r2r::log_error!(crate::NODE_ID, "Buffer maintainer failed with: '{}'.", e),
        };
    });

    // keep the node alive
    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(1000));
    });

    let buffered_frames_clone_2 = buffer_arc.clone();

    let mut t: u64 = 0;
    loop {
        std::thread::sleep(std::time::Duration::from_millis(100));
        let mut frames = buffered_frames_clone_2.lock().unwrap().clone();

        if t < 50 { // all frames should be here for 5 more seconds
            assert_eq!(frames.get("dummy_active_folder"), Some(&dummy_active_folder_fd));
            assert_eq!(frames.get("dummy_static_folder"), Some(&dummy_static_folder_fd));
            assert_eq!(frames.get("dummy_active_in_tf"), Some(&dummy_active_in_tf_fd));
            assert_eq!(frames.get("dummy_static_in_tf"), Some(&dummy_static_in_tf_fd));
        }
        if t > 71 { // the dummy_active_in_tf should be removed from the buffer after 5 seconds
            assert_eq!(frames.get("dummy_active_folder"), Some(&dummy_active_folder_fd));
            assert_eq!(frames.get("dummy_static_folder"), Some(&dummy_static_folder_fd));
            assert_eq!(frames.get("dummy_active_in_tf"), None);
            assert_eq!(frames.get("dummy_static_in_tf"), Some(&dummy_static_in_tf_fd));
        }
        if t > 101 { // the dummy_static_in_tf should be removed from the buffer after 10 seconds
            assert_eq!(frames.get("dummy_active_folder"), Some(&dummy_active_folder_fd));
            assert_eq!(frames.get("dummy_static_folder"), Some(&dummy_static_folder_fd));
            assert_eq!(frames.get("dummy_active_in_tf"), None);
            assert_eq!(frames.get("dummy_static_in_tf"), None);
            handle1.abort();
            break
        } 
        t = t + 1;
    }

    // r2r::log_info!(NODE_ID, "Node started.");

    handle.join().unwrap();

    Ok(())
}
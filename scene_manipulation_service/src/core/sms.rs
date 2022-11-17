use futures::{stream::Stream, StreamExt};
use r2r::scene_manipulation_msgs::srv::ManipulateScene;
use r2r::ServiceRequest;
use serde_json::json;
use std::collections::HashMap;
use std::fs;
use std::sync::{Arc, Mutex};

use crate::common::errors::{main_error_response, main_success_response};
use crate::common::frame_data::FrameData;
use crate::{check_would_produce_cycle, lookup_transform, ExtraData};

pub async fn scene_manipulation_server(
    mut service: impl Stream<Item = ServiceRequest<ManipulateScene::Service>> + Unpin,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    persist_path: &str,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(request) => match request.message.command.as_str() {
                "add" => {
                    r2r::log_info!(node_id, "Got 'add' request: {:?}.", request.message);
                    let response = add_frame(
                        &request.message,
                        &broadcasted_frames,
                        &buffered_frames,
                        &persist_path,
                    )
                    .await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "remove" => {
                    r2r::log_info!(node_id, "Got 'remove' request: {:?}.", request.message);
                    let response = remove_frame(
                        &request.message,
                        &broadcasted_frames,
                        &buffered_frames,
                        &persist_path,
                    )
                    .await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "rename" => {
                    r2r::log_info!(node_id, "Got 'rename' request: {:?}.", request.message);
                    let response = rename_frame(
                        &request.message,
                        &broadcasted_frames,
                        &buffered_frames,
                        &persist_path,
                    )
                    .await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "move" => {
                    r2r::log_info!(node_id, "Got 'move' request: {:?}.", request.message);
                    let response = move_frame(
                        &request.message,
                        &broadcasted_frames,
                        &buffered_frames,
                        &persist_path,
                    )
                    .await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "reparent" => {
                    r2r::log_info!(node_id, "Got 'reparent' request: {:?}.", request.message);
                    let response = reparent_frame(
                        &request.message,
                        &broadcasted_frames,
                        &buffered_frames,
                        &persist_path,
                    )
                    .await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "clone" => {
                    r2r::log_info!(node_id, "Got 'clone' request: {:?}.", request.message);
                    let response = clone_frame(
                        &request.message,
                        &broadcasted_frames,
                        &buffered_frames,
                        &persist_path,
                    )
                    .await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                _ => {
                    r2r::log_error!(node_id, "No such command.");
                    continue;
                }
            },
            None => {}
        }
    }
}

// Add a new frame
pub async fn add_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    persist_path: &str,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let mut local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
    let now = clock.get_now().unwrap();
    let time_stamp = r2r::Clock::to_builtin_time(&now);

    match &message.child_frame_id == "" || &message.parent_frame_id == "" {
        true => main_error_response("Child and parent frame ids can't be empty"),
        false => match serde_json::from_str(&message.extra) {
        Ok::<ExtraData, _>(mut extras) => {
            extras.time_stamp = Some(time_stamp);
            let frame_to_add = FrameData {
                parent_frame_id: message.parent_frame_id.clone(),
                child_frame_id: message.child_frame_id.clone(),
                transform: message.transform.clone(),
                extra_data: extras
            };
            match &message.child_frame_id == "world" || &message.child_frame_id == "world_origin" || &message.child_frame_id == "teaching_marker" {
                false => match local_buffered_frames.contains_key(&message.child_frame_id) {
                    false => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                        false => match local_buffered_frames.contains_key(&message.parent_frame_id) {
                            false => {
                                local_broadcasted_frames
                                    .insert(message.child_frame_id.clone(), frame_to_add.clone());
                                *broadcasted_frames.lock().unwrap() = local_broadcasted_frames;
                                match &message.persist {
                                    false => main_success_response(&format!(
                                        "Frame '{}' temporarily added to the scene.",
                                        message.child_frame_id
                                    )),
                                    true => {
                                        match fs::write(
                                            &format!("{}/{}.json", persist_path, message.child_frame_id),
                                            serde_json::to_string_pretty(&frame_to_add.clone()).unwrap(),
                                        ) {
                                            Ok(()) =>  main_success_response(&format!(
                                                "Frame '{}' permanently added to the scene.",
                                                message.child_frame_id
                                            )),
                                            Err(_) => main_success_response(&format!(
                                                        "Frame '{}' temporairly added to the scene, but adding the json file failed. Investigate?",
                                                        message.child_frame_id
                                                    )),
                                        }
                                    }
                                }
                            }
                            true => {
                                match check_would_produce_cycle(&frame_to_add, &local_buffered_frames) {
                                    (false, _) => {
                                        local_broadcasted_frames
                                            .insert(message.child_frame_id.clone(), frame_to_add);
                                        *broadcasted_frames.lock().unwrap() = local_broadcasted_frames;
                                        main_success_response(&format!(
                                            "Frame '{}' added to the scene.",
                                            message.child_frame_id
                                        ))
                                    }
                                    (true, _) => main_error_response(&format!(
                                        "Adding frame '{}' would produce a cycle.",
                                        &message.child_frame_id,
                                    )),
                                }
                            }
                        },
                        true => {
                            tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
                            match local_buffered_frames.contains_key(&message.child_frame_id) {
                                false => main_error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                                true => main_error_response("Frame already exists in the tf."),
                            }
                        }
                    },
                    true => main_error_response("Frame already exists in the tf, will not proceed to broadcasted locally."),
                },
                true => main_error_response(&format!("Frame '{}' is reserved.", &message.child_frame_id)),
            }
        },
        Err(cause) => main_error_response(&format!(
            "Failed to decode message.extras string. The 'extra_data' field is probably missing in the message. Not added: '{}'",
            cause
        ))
    }
}
}

// Remove the frame from the scene
pub async fn remove_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    persist_path: &str,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    fn inner(
        message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
        broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
        buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
        persist_path: &str,
    ) -> ManipulateScene::Response {
        let mut local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
        let mut local_buffered_frames = buffered_frames.lock().unwrap().clone();
        match local_broadcasted_frames.get(&message.child_frame_id) {
            Some(frame) => match frame.extra_data.active {
                Some(true) | None => match local_broadcasted_frames.remove(&message.child_frame_id)
                {
                    Some(_) => match local_buffered_frames.remove(&message.child_frame_id) {
                        Some(_) => {
                            *broadcasted_frames.lock().unwrap() = local_broadcasted_frames;
                            *buffered_frames.lock().unwrap() = local_buffered_frames;
                            match &message.persist {
                                false => main_success_response(&format!(
                                    "Frame '{}' temporarily removed from the scene.",
                                    message.child_frame_id
                                )),
                                true => {
                                    match fs::remove_file(
                                        &format!("{}/{}.json", persist_path, message.child_frame_id)
                                    ) {
                                        Ok(()) =>  main_success_response(&format!(
                                            "Frame '{}' permanently removed from the scene.",
                                            message.child_frame_id
                                        )),
                                        Err(_) => main_success_response(&format!(
                                                    "Frame '{}' temporairly removed to the scene, but removing the json file failed. Investigate?",
                                                    message.child_frame_id
                                                )),
                                    }
                                }
                            }
                        }
                        None => main_error_response(&format!(
                            "Failed to remove frame '{}' from the tf buffer.",
                            message.child_frame_id
                        )),
                    },
                    None => main_error_response(&format!(
                        "Failed to remove frame '{}' from the broadcast buffer.",
                        message.child_frame_id
                    )),
                },
                Some(false) => main_error_response(&format!(
                    "Can't remove static frame '{}'.",
                    message.child_frame_id
                )),
            },
            None => main_error_response("Failed to get frame data from broadcaster hashmap."),
        }
    }

    match &message.child_frame_id == "world" || &message.child_frame_id == "world_origin" || &message.child_frame_id == "teaching_marker" {
        false => match local_buffered_frames.contains_key(&message.child_frame_id) {
            false => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => main_error_response("Frame doesn't exist in tf, nor is it published by this broadcaster."),
                true => {
                    tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
                    match local_buffered_frames.contains_key(&message.child_frame_id) {
                        false => main_error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                        true => inner(message, broadcasted_frames, buffered_frames, persist_path)
                    }
                }
            },
            true => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => main_error_response("Frame exists in the tf, but it can't be removed since it is not published by sms."),
                true => inner(message, broadcasted_frames, buffered_frames, persist_path)
            }
        },
        true => main_error_response(&format!("Frame '{}' is reserved.", &message.child_frame_id)),
    }
}

// Rename the child frame to the new frame id
pub async fn rename_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    persist_path: &str,
) -> ManipulateScene::Response {
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
    let mut remove_request = message.clone();
    let mut add_request = message.clone();
    remove_request.command = "remove".to_string();
    add_request.command = "add".to_string();

    let remove_response = remove_frame(
        &remove_request,
        &broadcasted_frames,
        &buffered_frames,
        &persist_path,
    )
    .await;

    match remove_response.success {
        true => match local_broadcasted_frames.get(&message.child_frame_id) {
            None => main_error_response(&format!(
                "Failed to fetch frame '{}' from the broadcasted hashmap.",
                message.child_frame_id.to_string(),
            )),
            Some(frame) => {
                let add_response = add_frame(
                    &ManipulateScene::Request {
                        command: "add".to_string(),
                        child_frame_id: message.new_frame_id.to_string(),
                        parent_frame_id: frame.parent_frame_id.to_string(),
                        new_frame_id: message.new_frame_id.to_string(),
                        transform: frame.transform.clone(),
                        persist: message.persist,
                        extra: json!({
                            "time_stamp": frame.extra_data.time_stamp,
                            "zone": frame.extra_data.zone,
                            "next": frame.extra_data.next,
                            "frame_type": frame.extra_data.frame_type,
                            "active": frame.extra_data.active,
                            "show_mesh": frame.extra_data.show_mesh,
                            "mesh_type": frame.extra_data.mesh_type,
                            "mesh_path": frame.extra_data.mesh_path,
                            "mesh_scale": frame.extra_data.mesh_scale,
                            // "mesh_color": frame.extra_data.mesh_color
                            "mesh_r": frame.extra_data.mesh_r,
                            "mesh_g": frame.extra_data.mesh_g,
                            "mesh_b": frame.extra_data.mesh_b,
                            "mesh_a": frame.extra_data.mesh_a,

                        })
                        .to_string(),
                    },
                    &broadcasted_frames,
                    &buffered_frames,
                    &persist_path,
                )
                .await;
                match add_response.success {
                    false => add_response,
                    true => main_success_response(&format!(
                        "Successfully renamed frame '{}' to '{}'.",
                        message.child_frame_id.to_string(),
                        message.new_frame_id.to_string()
                    )),
                }
            }
        },
        false => return remove_response,
    }
}

// Move the child marker to the position of the currently selected parent frame
pub async fn move_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    persist_path: &str,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    async fn inner(
        message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
        broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
        buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
        persist_path: &str,
    ) -> ManipulateScene::Response {
        let mut local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
        let local_broadcasted_frames_clone = local_broadcasted_frames.clone();
        let local_buffered_frames = buffered_frames.lock().unwrap().clone();

        match local_buffered_frames.get(&message.parent_frame_id.clone()) {
            Some(_) =>
                match local_broadcasted_frames_clone.get(&message.child_frame_id.clone()) {
                    Some(child) => match child.extra_data.active {
                        Some(true) | None => {
                            match lookup_transform(
                                &child.parent_frame_id,
                                &message.parent_frame_id,
                                &buffered_frames,
                            ).await {
                                Some(transform) => {
                                    local_broadcasted_frames.insert(
                                        child.child_frame_id.clone(),
                                        FrameData {
                                            parent_frame_id: child.parent_frame_id.clone(),
                                            child_frame_id: child.child_frame_id.clone(),
                                            transform,
                                            extra_data: child.extra_data.clone()
                                        }
                                    );
                                    *broadcasted_frames.lock().unwrap() = local_broadcasted_frames;
                                    match &message.persist {
                                        false => main_success_response(&format!(
                                            "Frame '{}' temporarily moved to '{}'.",
                                            message.child_frame_id, message.parent_frame_id
                                        )),
                                        true => {
                                            match fs::remove_file(
                                                &format!("{}/{}.json", &persist_path, message.child_frame_id)
                                            ) {
                                                Ok(()) =>  main_success_response(&format!(
                                                    "Frame '{}' permanently moved to '{}'.",
                                                    message.child_frame_id, message.parent_frame_id
                                                )),
                                                Err(_) => main_success_response(&format!(
                                                        "Frame '{}' temporairly moved to '{}' in the scene, but manipulating the json file failed. Investigate?",
                                                        message.child_frame_id, message.parent_frame_id
                                                    )),
                                                }
                                            }
                                        }
                                    }
                                    None => main_error_response(&format!("Failed to lookup frame '{}'.", message.parent_frame_id)) 
                                }
                            }
                        Some(false) => main_error_response("Can't manipulate static frames.") 
                    },
                    None => main_error_response(&format!("Can't move the frame '{}' to the position of the '{}' frame, '{}' is not published locally.", 
                    &message.child_frame_id, &message.parent_frame_id, &message.child_frame_id)),
                },
            None => main_error_response(&format!("Can't move the frame '{}' to the position of the '{}' frame, '{}' doens't exist in tf.", 
            &message.child_frame_id, &message.parent_frame_id, &message.parent_frame_id)),
        }
    }

    match &message.child_frame_id == "world" || &message.child_frame_id == "world_origin" {
        false => match local_buffered_frames.contains_key(&message.child_frame_id) {
            false => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => main_error_response("Frame doesn't exist."),
                true => {
                    tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
                    match local_buffered_frames.contains_key(&message.child_frame_id) {
                        false => main_error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                        true => inner(&message, &broadcasted_frames, &buffered_frames, &persist_path).await
                    }
                }
            }
            true => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => main_error_response("Frame exists in the tf, but it can't be moved since it is not published by this broadcaster."),
                true => inner(&message, &broadcasted_frames, &buffered_frames, &persist_path).await
            }
        },
        true => main_error_response(&format!("Frame '{}' is reserved.", &message.child_frame_id)),
    }
}

// TODO: test and add persist
pub async fn reparent_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    persist_path: &str,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    async fn inner(
        message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
        broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
        buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    ) -> ManipulateScene::Response {
        let local_buffered_frames = buffered_frames.lock().unwrap().clone();
        let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
        let mut local_broadcasted_frames_clone = local_broadcasted_frames.clone();

        // let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        // let now = clock.get_now().unwrap();
        // let time_stamp = r2r::Clock::to_builtin_time(&now);

        match check_would_produce_cycle(
            &FrameData {
                parent_frame_id: message.parent_frame_id.to_string(),
                child_frame_id: message.child_frame_id.to_string(),
                transform: message.transform.clone(),
                ..Default::default()
            },
            &local_buffered_frames,
        ) {
            (false, _) => match lookup_transform(
                &message.parent_frame_id,
                &message.child_frame_id,
                &buffered_frames,
            )
            .await
            {
                None => main_error_response("Failed to lookup transform."),
                Some(transform) => {
                    match local_broadcasted_frames.get(&message.child_frame_id.clone()) {
                        Some(frame) => {
                            local_broadcasted_frames_clone.insert(
                                message.child_frame_id.clone(),
                                FrameData {
                                    parent_frame_id: message.parent_frame_id.clone(),
                                    child_frame_id: message.child_frame_id.clone(),
                                    transform,
                                    extra_data: frame.extra_data.clone()
                                },
                            );
                            *broadcasted_frames.lock().unwrap() = local_broadcasted_frames_clone;
                            main_success_response(&format!(
                                "Frame '{}' reparented to '{}'.",
                                message.child_frame_id, message.parent_frame_id
                            ))
                        }
                        None => main_error_response(&format!(
                            "Frame '{}' in the tf buffer, but not in broadcaster, investigate.'",
                            &message.child_frame_id
                        )),
                    }
                }
            },
            (true, cause) => main_error_response(&format!(
                "Adding frame '{}' would produce a cycle. Not added, cause: '{}'",
                &message.child_frame_id, cause
            )),
        }
    }

    match &message.child_frame_id == "" || &message.parent_frame_id == "" {
        true => main_error_response("Child and parent frame ids can't be empty"),
        false => match &message.child_frame_id == "world" || &message.child_frame_id == "world_origin" || &message.child_frame_id == "teaching_marker" {
            false => match local_buffered_frames.contains_key(&message.child_frame_id) {
                false => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                    false => main_error_response("Frame doesn't exist."),
                    true => {
                        tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
                        match local_buffered_frames.contains_key(&message.child_frame_id) {
                            false => main_error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                            true => inner(&message, &broadcasted_frames, &buffered_frames).await
                        }
                    }
                }
                true => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                    false => main_error_response("Frame exists in the tf, but it can't be moved since it is not published by this broadcaster."),
                    true => inner(&message, &broadcasted_frames, &buffered_frames).await
                }
            },
            true => main_error_response(&format!("Frame '{}' is reserved.", &message.child_frame_id)),
        }
    }
}

// TODO: test and add persist
async fn clone_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    persist_path: &str,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    async fn inner(
        message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
        broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
        buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    ) -> ManipulateScene::Response {
        let local_buffered_frames = buffered_frames.lock().unwrap().clone();
        let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
        let mut local_broadcasted_frames_clone = local_broadcasted_frames.clone();

        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let frame_to_clone = FrameData {
            parent_frame_id: message.parent_frame_id.clone(),
            child_frame_id: message.new_frame_id.clone(),
            transform: message.transform.clone(),
            extra_data: ExtraData {
                active: Some(true),
                time_stamp: Some(time_stamp.clone()),
                ..Default::default()
            },
        };

        match check_would_produce_cycle(&frame_to_clone, &local_buffered_frames) {
            (false, _) => match lookup_transform(
                &message.parent_frame_id,
                &message.child_frame_id,
                &buffered_frames,
            )
            .await
            {
                None => main_error_response("Failed to lookup transform."),
                Some(transform) => {
                    match local_broadcasted_frames.get(&message.child_frame_id.clone()) {
                        Some(frame) => {
                            local_broadcasted_frames_clone.insert(
                                message.new_frame_id.clone(),
                                FrameData {
                                    parent_frame_id: message.parent_frame_id.clone(),
                                    child_frame_id: message.new_frame_id.clone(),
                                    transform,
                                    extra_data: frame.extra_data.clone(),
                                },
                            );
                            *broadcasted_frames.lock().unwrap() = local_broadcasted_frames_clone;
                            main_success_response(&format!(
                                "Frame '{}' cloned as '{}'.",
                                message.child_frame_id, message.new_frame_id
                            ))
                        }
                        None => main_error_response(&format!(
                            "Frame '{}' in the tf buffer, but not in broadcaster, investigate.'",
                            &message.child_frame_id
                        )),
                    }
                }
            },
            (true, cause) => main_error_response(&format!(
                "Adding frame '{}' would produce a cycle. Not added, cause: '{}'",
                &message.child_frame_id, cause
            )),
        }
    }

    match &message.child_frame_id == ""
        || &message.parent_frame_id == ""
        || &message.new_frame_id == ""
    {
        true => main_error_response("Child and parent frame ids can't be empty"),
        false => match local_buffered_frames.contains_key(&message.child_frame_id) {
            false => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => main_error_response("Frame doesn't exist."),
                true => {
                    tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
                    match local_buffered_frames.contains_key(&message.child_frame_id) {
                        false => main_error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                        true => match local_buffered_frames.contains_key(&message.parent_frame_id) {
                            false => main_error_response("Parent doesn't exist."),
                            true => inner(&message, &broadcasted_frames, &buffered_frames).await
                        }
                    }
                }
            },
            true => inner(&message, &broadcasted_frames, &buffered_frames).await,
        },
    }
}

use futures::{stream::Stream, StreamExt};
use r2r::scene_manipulation_msgs::srv::ManipulateScene;
use r2r::ServiceRequest;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use crate::common::errors::{main_error_response, main_success_response};
use crate::common::frame_data::FrameData;
use crate::{check_would_produce_cycle, lookup_transform};

// A BIG TODO: disable reparenting and cloning if the parent and child is the same, breaks the tree
// TODO: add a loop check before adding frames
// TODO: sort out information in the messages when responding
pub async fn scene_manipulation_server(
    mut service: impl Stream<Item = ServiceRequest<ManipulateScene::Service>> + Unpin,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(request) => match request.message.command.as_str() {
                "add" => {
                    r2r::log_info!(node_id, "Got 'add' request: {:?}.", request.message);
                    let response =
                        add_frame(&request.message, &broadcasted_frames, &buffered_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "remove" => {
                    r2r::log_info!(node_id, "Got 'remove' request: {:?}.", request.message);
                    let response =
                        remove_frame(&request.message, &broadcasted_frames, &buffered_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "rename" => {
                    r2r::log_info!(node_id, "Got 'rename' request: {:?}.", request.message);
                    let response =
                        rename_frame(&request.message, &broadcasted_frames, &buffered_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "move" => {
                    r2r::log_info!(node_id, "Got 'move' request: {:?}.", request.message);
                    let response =
                        move_frame(&request.message, &broadcasted_frames, &buffered_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "reparent" => {
                    r2r::log_info!(node_id, "Got 'reparent' request: {:?}.", request.message);
                    let response =
                        reparent_frame(&request.message, &broadcasted_frames, &buffered_frames)
                            .await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "clone" => {
                    r2r::log_info!(node_id, "Got 'clone' request: {:?}.", request.message);
                    let response =
                        clone_frame(&request.message, &broadcasted_frames, &buffered_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "teach" => {
                    r2r::log_info!(node_id, "Got 'teach' request: {:?}.", request.message);
                    let response =
                        teach_frame(&request.message, &broadcasted_frames, &buffered_frames).await;
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

async fn add_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let mut local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
    let now = clock.get_now().unwrap();
    let time_stamp = r2r::Clock::to_builtin_time(&now);

    let frame_to_add = FrameData {
        parent_frame_id: message.parent_frame_id.clone(),
        child_frame_id: message.child_frame_id.clone(),
        transform: message.transform.clone(),
        active: Some(true),
        time_stamp: Some(time_stamp),
        zone: None,
        next: None,
        frame_type: None,
        // local: Some(true)
    };

    match &message.child_frame_id == "world" || &message.child_frame_id == "world_origin" {
        false => match local_buffered_frames.contains_key(&message.child_frame_id) {
            false => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => match local_buffered_frames.contains_key(&message.parent_frame_id) {
                    false => {
                        local_broadcasted_frames
                            .insert(message.child_frame_id.clone(), frame_to_add);
                        *broadcasted_frames.lock().unwrap() = local_broadcasted_frames;
                        main_success_response(&format!(
                            "Frame '{}' added to the scene.",
                            message.child_frame_id
                        ))
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
                            (true, cause) => main_error_response(&format!(
                                "Adding frame '{}' would produce a cycle. Not added: '{}'",
                                &message.child_frame_id, cause
                            )),
                        }
                    }
                },
                true => {
                    tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
                    match local_buffered_frames.contains_key(&message.child_frame_id) {
                        false => main_error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                        true => main_error_response("Frame already exists."),
                    }
                }
            },
            true => main_error_response("Frame already exists."),
        },
        true => main_error_response("Frame 'world' is reserved as the universal tree root."),
    }
}

// TODO: maybe remove the frame also from the buffered_frames
// TODO: a frame needs a name, can't be blank ""
async fn remove_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    fn inner(
        message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
        broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
        buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    ) -> ManipulateScene::Response {
        let mut local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
        let mut local_buffered_frames = buffered_frames.lock().unwrap().clone();
        match local_broadcasted_frames.get(&message.child_frame_id) {
            Some(frame) => match frame.active {
                Some(true) | None => match local_broadcasted_frames.remove(&message.child_frame_id)
                {
                    Some(_) => match local_buffered_frames.remove(&message.child_frame_id) {
                        Some(_) => {
                            *broadcasted_frames.lock().unwrap() = local_broadcasted_frames;
                            *buffered_frames.lock().unwrap() = local_buffered_frames;
                            main_success_response(&format!(
                                "Frame '{}' removed from the scene.",
                                message.child_frame_id
                            ))
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
                Some(false) => main_error_response("Can't manipulate_static frames."),
            },
            None => main_error_response("Failed to get frame data from broadcaster hashmap."),
        }
    }

    match &message.child_frame_id == "world" || &message.child_frame_id == "world_origin" {
        false => match local_buffered_frames.contains_key(&message.child_frame_id) {
            false => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => main_error_response("Frame doesn't exist in tf, nor is it published by this broadcaster."),
                true => {
                    tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
                    match local_buffered_frames.contains_key(&message.child_frame_id) {
                        false => main_error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                        true => inner(message, broadcasted_frames, buffered_frames)
                    }
                }
            },
            true => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => main_error_response("Frame exists in the tf, but it can't be removed since it is not published by sms."),
                true => inner(message, broadcasted_frames, buffered_frames)
            }
        },
        true => main_error_response("Frame 'world' is reserved as the universal tree root."),
    }
}

async fn rename_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> ManipulateScene::Response {
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    let remove_response = remove_frame(
        &ManipulateScene::Request {
            command: "remove".to_string(),
            child_frame_id: message.child_frame_id.to_string(),
            parent_frame_id: message.parent_frame_id.to_string(),
            new_frame_id: message.new_frame_id.to_string(),
            transform: message.transform.clone(),
        },
        &broadcasted_frames,
        &buffered_frames,
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
                    },
                    &broadcasted_frames,
                    &buffered_frames,
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

// see if it is local
async fn move_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let mut local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
    let local_broadcasted_frames_clone = broadcasted_frames.lock().unwrap().clone();

    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
    let now = clock.get_now().unwrap();
    let time_stamp = r2r::Clock::to_builtin_time(&now);

    match &message.child_frame_id == "world" || &message.child_frame_id == "world_origin" {
        false => match local_buffered_frames.contains_key(&message.child_frame_id) {
            false => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => main_error_response("Frame doesn't exist."),
                true => {
                    tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
                    match local_buffered_frames.contains_key(&message.child_frame_id) {
                        false => main_error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                        true => {
                            match local_broadcasted_frames_clone.get(&message.child_frame_id.clone()) {
                                Some(frame) => {
                                    local_broadcasted_frames.insert(
                                        message.child_frame_id.clone(),
                                        FrameData {
                                            parent_frame_id: message.parent_frame_id.clone(),
                                            child_frame_id: message.child_frame_id.clone(),
                                            transform: message.transform.clone(),
                                            active: Some(true),
                                            time_stamp: Some(time_stamp),
                                            zone: frame.zone.clone(),
                                            next: frame.next.clone(),
                                            frame_type: frame.frame_type.clone(),
                                            // local: Some(true)
                                        }
                                    );
                                    *broadcasted_frames.lock().unwrap() = local_broadcasted_frames;
                            main_success_response(&format!(
                                "Frame '{}' added to the scene.",
                                message.child_frame_id
                            ))
                                },
                                None => main_error_response("Frame exists in the tf, but it can't be moved since it is not published by this broadcaster."),
                            }
                        }
                    }
                }
            }
            true => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => main_error_response("Frame exists in the tf, but it can't be moved since it is not published by this broadcaster."),
                true => {
                    match local_broadcasted_frames_clone.get(&message.child_frame_id.clone()) {
                        Some(frame) => {
                            local_broadcasted_frames.insert(
                                message.child_frame_id.clone(),
                                FrameData {
                                    parent_frame_id: message.parent_frame_id.clone(),
                                    child_frame_id: message.child_frame_id.clone(),
                                    transform: message.transform.clone(),
                                    active: Some(true),
                                    time_stamp: Some(time_stamp),
                                    zone: frame.zone.clone(),
                                    next: frame.next.clone(),
                                    frame_type: frame.frame_type.clone(),
                                }
                            );
                            *broadcasted_frames.lock().unwrap() = local_broadcasted_frames;
                    main_success_response(&format!(
                        "Frame '{}' added to the scene.",
                        message.child_frame_id
                    ))
                        },
                        None => main_error_response("Frame exists in the tf, but it can't be moved since it is not published by this broadcaster."),
                    }
                }
            }
        },
        true => main_error_response("Frame 'world' is reserved as the universal tree root."),
    }
}

async fn teach_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    async fn inner(
        message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
        broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
        buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    ) -> ManipulateScene::Response {
        let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
        let mut local_broadcasted_frames_clone = broadcasted_frames.lock().unwrap().clone();

        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        match lookup_transform(
            &message.parent_frame_id,
            "teaching_marker",
            &buffered_frames,
        )
        .await
        {
            Some(transform) => {
                match local_broadcasted_frames.get(&message.child_frame_id.clone()) {
                    Some(frame) => {
                        //match frame.local {
                        // Some(true) | None => {
                        local_broadcasted_frames_clone.insert(
                            message.child_frame_id.clone(),
                            FrameData {
                                parent_frame_id: message.parent_frame_id.clone(),
                                child_frame_id: message.child_frame_id.clone(),
                                transform,
                                active: Some(true),
                                time_stamp: Some(time_stamp),
                                zone: frame.zone.clone(),
                                next: frame.next.clone(),
                                frame_type: frame.frame_type.clone(),
                                // local: Some(true)
                            },
                        );
                        *broadcasted_frames.lock().unwrap() = local_broadcasted_frames_clone;
                        main_success_response(&format!(
                            "Frame '{}' was taught a new pose.",
                            message.child_frame_id
                        ))
                        // },
                        // Some(false) => {
                        //     main_error_response(&format!(
                        //         "Frame '{}' can't be taught since it is published elsewhere.",
                        //         message.child_frame_id
                        //     ))
                        // }
                    }
                    None => main_error_response(&format!(
                        "Frame '{}' can't be taught since it is published elsewhere.",
                        message.child_frame_id
                    )),
                }
            }
            None => main_error_response("Failed to lookup transform."),
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
                        true => inner(&message, &broadcasted_frames, &buffered_frames).await
                    }
                }
            }
            true => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => main_error_response("Frame exists in the tf, but it can't be moved since it is not published by this broadcaster."),
                true => inner(&message, &broadcasted_frames, &buffered_frames).await
            }
        },
        true => main_error_response("Frame 'world' is reserved as the universal tree root."),        
    }
}

async fn reparent_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
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

        match check_would_produce_cycle(
            &FrameData {
                parent_frame_id: message.parent_frame_id.to_string(),
                child_frame_id: message.child_frame_id.to_string(),
                transform: message.transform.clone(),
                time_stamp: None,
                zone: None,
                next: None,
                frame_type: None,
                active: None,
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
                                    active: Some(true),
                                    time_stamp: Some(time_stamp),
                                    zone: frame.zone.clone(),
                                    next: frame.next.clone(),
                                    frame_type: frame.frame_type.clone(),
                                    // local: Some(true)
                                },
                            );
                            *broadcasted_frames.lock().unwrap() = local_broadcasted_frames_clone;
                            main_success_response(&format!(
                                "Frame '{}' added to the scene.",
                                message.child_frame_id
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

    match &message.child_frame_id == "world" || &message.child_frame_id == "world_origin" {
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
        true => main_error_response("Frame 'world' is reserved as the universal tree root."),        
    }
}

async fn clone_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
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
            active: Some(true),
            time_stamp: Some(time_stamp.clone()),
            zone: None,
            next: None,
            frame_type: None,
        };

        // let new_message = ManipulateScene::Request {
        //     command: "clone".to_string(),
        //     child_frame_id: message.new_frame_id.to_string(),
        //     parent_frame_id: message.parent_frame_id.to_string(),
        //     new_frame_id: message.new_frame_id.to_string(),
        //     transform: message.transform.clone(),
        // };
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
                    match local_broadcasted_frames.get(&frame_to_clone.child_frame_id.clone()) {
                        Some(frame) => {
                            local_broadcasted_frames_clone.insert(
                                message.child_frame_id.clone(),
                                FrameData {
                                    parent_frame_id: message.parent_frame_id.clone(),
                                    child_frame_id: message.new_frame_id.clone(),
                                    transform,
                                    active: Some(true),
                                    time_stamp: Some(time_stamp),
                                    zone: frame.zone.clone(),
                                    next: frame.next.clone(),
                                    frame_type: frame.frame_type.clone(),
                                    // local: Some(true)
                                },
                            );
                            *broadcasted_frames.lock().unwrap() = local_broadcasted_frames_clone;
                            main_success_response(&format!(
                                "Frame '{}' added to the scene.",
                                message.child_frame_id
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

    match local_buffered_frames.contains_key(&message.child_frame_id) {
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
    }
}

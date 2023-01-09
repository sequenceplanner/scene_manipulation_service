use crate::common::errors::{extra_error_response, extra_success_response};
use futures::{stream::Stream, StreamExt};
use r2r::geometry_msgs::msg::TransformStamped;
use r2r::scene_manipulation_msgs::msg::{TFExtra, TFExtraData};
use r2r::scene_manipulation_msgs::srv::ManipulateExtras;
use r2r::std_msgs::msg::Header;
use r2r::ServiceRequest;

use crate::{lookup_transform, ExtraData};
use serde_json::Value;
use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};

use crate::common::{files::reload_scenario, frame_data::FrameData};

pub async fn extra_features_server(
    mut service: impl Stream<Item = ServiceRequest<ManipulateExtras::Service>> + Unpin,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(request) => match request.message.command.as_str() {
                "reload_scenario" => {
                    r2r::log_info!(
                        node_id,
                        "Got 'reload_scenario' request: {:?}.",
                        request.message
                    );
                    let response =
                        reload_scenario(&request.message, &broadcasted_frames, node_id).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "set_zone" => {
                    r2r::log_info!(node_id, "Got 'set_zone' request: {:?}.", request.message);
                    let response = set_zone(&request.message, &broadcasted_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "enable_path" => {
                    r2r::log_info!(node_id, "Got 'enable_path' request: {:?}.", request.message);
                    let response =
                        enable_or_disable_path(&request.message, &broadcasted_frames, true).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "disable_path" => {
                    r2r::log_info!(
                        node_id,
                        "Got 'disable_path' request: {:?}.",
                        request.message
                    );
                    let response =
                        enable_or_disable_path(&request.message, &broadcasted_frames, false).await;
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

// should work only for frames published by sms
// if the frame is not published by sms, the zones can be sent out on the /tf_extra topic
pub async fn set_zone(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateExtras::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> ManipulateExtras::Response {
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
    let mut local_broadcasted_frames_clone = local_broadcasted_frames.clone();
    match local_broadcasted_frames.get(&message.child_frame_id) {
        Some(frame) => {
            // match frame.local {
            // Some(true) | None => {
            local_broadcasted_frames_clone.insert(
                message.child_frame_id.clone(),
                FrameData {
                    parent_frame_id: frame.parent_frame_id.clone(),
                    child_frame_id: frame.child_frame_id.clone(),
                    transform: frame.transform.clone(),
                    extra_data: ExtraData {
                        active: frame.extra_data.active,
                        time_stamp: frame.extra_data.time_stamp.clone(),
                        zone: Some(message.size),
                        next: frame.extra_data.next.clone(),
                        frame_type: frame.extra_data.frame_type.clone(),
                        ..Default::default()
                    },
                },
            );
            *broadcasted_frames.lock().unwrap() = local_broadcasted_frames_clone;
            extra_success_response(&format!(
                "Zone for '{}' has been set to '{}'.",
                &frame.child_frame_id, &message.size
            ))
        }
        None => extra_error_response("Frame doesn't exist."),
    }
}

// TODO: should work only for frames published by sms.
// if the frame is not published by sms, the enabled paths can be sent out on the /tf_extra topic
pub async fn enable_or_disable_path(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateExtras::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    enable: bool,
) -> ManipulateExtras::Response {
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
    let mut local_broadcasted_frames_clone = local_broadcasted_frames.clone();
    match local_broadcasted_frames.get(&message.parent_frame_id) {
        Some(parent) => {
            match local_broadcasted_frames.get(&message.child_frame_id) {
                Some(child) => {
                    let new_next = match parent.extra_data.next.clone() {
                        Some(mut n) => {
                            match enable {
                                true => n.insert(child.child_frame_id.clone()),
                                false => n.remove(&child.child_frame_id),
                            };
                            n
                        },
                        None => {
                            let mut n = HashSet::new();
                            match enable {
                                true => {
                                    n.insert(child.child_frame_id.clone());
                                },
                                false => ()
                            };
                            n
                        }
                    };
                    local_broadcasted_frames_clone.insert(
                        parent.child_frame_id.clone(),
                        FrameData {
                            parent_frame_id: parent.parent_frame_id.clone(),
                            child_frame_id: parent.child_frame_id.clone(),
                            transform: parent.transform.clone(),
                            extra_data: ExtraData {
                                active: parent.extra_data.active,
                                time_stamp: parent.extra_data.time_stamp.clone(),
                                zone: parent.extra_data.zone,
                                next: Some(new_next),
                                frame_type: parent.extra_data.frame_type.clone(),
                                ..Default::default()
                            }
                        }
                    );
                    *broadcasted_frames.lock().unwrap() = local_broadcasted_frames_clone;
                    let reply_info = match enable {
                        true => "enabled",
                        false => "disabled"
                    };
                    extra_success_response(&format!(
                        "Path between '{}' and '{}' has been {}.",
                        &parent.child_frame_id, &child.child_frame_id, reply_info
                    ))
                }
                None => extra_error_response("Child frame is not published by the local broadcaster. Paths can't be enabled between frames published elsewhere."),
            }
        }
        None => extra_error_response("Parent frame is not published by the local broadcaster. Paths can't be enabled between frames published elsewhere."),
    }
}
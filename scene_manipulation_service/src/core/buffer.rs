use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

use futures::{Stream, StreamExt};
use r2r::{
    geometry_msgs::msg::TransformStamped, scene_manipulation_msgs::srv::GetAllTransforms,
    std_msgs::msg::Header, tf2_msgs::msg::TFMessage, ServiceRequest,
};

use crate::{lookup_transform, FrameData};

pub async fn get_all_transforms_server(
    mut service: impl Stream<Item = ServiceRequest<GetAllTransforms::Service>> + Unpin,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(request) => {
                let frames_local = buffered_frames.lock().unwrap().clone();
                let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
                let now = clock.get_now().unwrap();
                let time_stamp = r2r::Clock::to_builtin_time(&now);
                let mut transforms_to_publish = vec![];

                for frame in frames_local {
                    match request.message.parent_frame_id.as_str() {
                        "" => transforms_to_publish.push(TransformStamped {
                            header: Header {
                                stamp: time_stamp.clone(),
                                frame_id: frame.1.parent_frame_id.clone(),
                            },
                            child_frame_id: frame.1.child_frame_id.clone(),
                            transform: frame.1.transform.clone(),
                        }),
                        _ => match lookup_transform(
                            &request.message.parent_frame_id,
                            &frame.1.child_frame_id,
                            &buffered_frames,
                        )
                        .await
                        {
                            Some(transform) => transforms_to_publish.push(TransformStamped {
                                header: Header {
                                    stamp: time_stamp.clone(),
                                    frame_id: request.message.parent_frame_id.clone(),
                                },
                                child_frame_id: frame.1.child_frame_id.clone(),
                                transform: transform.clone(),
                            }),
                            None => (),
                        },
                    }
                }

                let response = GetAllTransforms::Response {
                    success: true,
                    info: "".to_string(),
                    transforms: transforms_to_publish,
                };
                request
                    .respond(response)
                    .expect("Could not send service response.");
                continue;
            }
            None => {}
        }
    }
}

// broadcast static frames
pub async fn static_frame_broadcaster_callback(
    publisher: r2r::Publisher<TFMessage>,
    mut timer: r2r::Timer,
    frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let transforms_local = frames.lock().unwrap().clone();
        let mut updated_transforms = vec![];

        transforms_local.iter().for_each(|(_, v)| match v.active {
            Some(false) => {
                updated_transforms.push(TransformStamped {
                    header: Header {
                        stamp: time_stamp.clone(),
                        frame_id: v.parent_frame_id.clone(),
                    },
                    child_frame_id: v.child_frame_id.clone(),
                    transform: v.transform.clone(),
                });
            }
            Some(true) | None => (),
        });

        let msg = TFMessage {
            transforms: updated_transforms,
        };

        match publisher.publish(&msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    node_id,
                    "Static broadcaster failed to send a message with: '{}'",
                    e
                );
            }
        };
        timer.tick().await?;
    }
}

// broadcast active frames
pub async fn active_frame_broadcaster_callback(
    publisher: r2r::Publisher<TFMessage>,
    mut timer: r2r::Timer,
    frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let transforms_local = frames.lock().unwrap().clone();
        let mut updated_transforms = vec![];

        transforms_local.iter().for_each(|(_, v)| match v.active {
            Some(true) | None => {
                updated_transforms.push(TransformStamped {
                    header: Header {
                        stamp: time_stamp.clone(),
                        frame_id: v.parent_frame_id.clone(),
                    },
                    child_frame_id: v.child_frame_id.clone(),
                    transform: v.transform.clone(),
                });
            }
            Some(false) => (),
        });

        let msg = TFMessage {
            transforms: updated_transforms,
        };

        match publisher.publish(&msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    node_id,
                    "Active broadcaster failed to send a message with: '{}'",
                    e
                );
            }
        };
        timer.tick().await?;
    }
}

// updates the buffer with active frames from the tf topic
// TODO: if a stale active frame is on the tf for some reason, don't include it
pub async fn active_tf_listener_callback(
    mut subscriber: impl Stream<Item = TFMessage> + Unpin,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let mut frames_local = buffered_frames.lock().unwrap().clone();
                message.transforms.iter().for_each(|t| {
                    frames_local.insert(
                        t.child_frame_id.clone(),
                        FrameData {
                            parent_frame_id: t.header.frame_id.clone(),
                            child_frame_id: t.child_frame_id.clone(),
                            transform: t.transform.clone(),
                            active: Some(true),
                            time_stamp: Some(t.header.stamp.clone()),
                            zone: None,
                            next: None,
                            frame_type: None,
                        },
                    );
                });
                *buffered_frames.lock().unwrap() = frames_local;
            }
            None => {
                r2r::log_error!(node_id, "Subscriber did not get the message?");
            }
        }
    }
}

// updates the buffer with static frames from the tf_static topic
pub async fn static_tf_listener_callback(
    mut subscriber: impl Stream<Item = TFMessage> + Unpin,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let mut frames_local = buffered_frames.lock().unwrap().clone();
                message.transforms.iter().for_each(|t| {
                    // static frames are always true, so we don't need to check their timestamp
                    frames_local.insert(
                        t.child_frame_id.clone(),
                        FrameData {
                            parent_frame_id: t.header.frame_id.clone(),
                            child_frame_id: t.child_frame_id.clone(),
                            transform: t.transform.clone(),
                            active: Some(false),
                            time_stamp: Some(t.header.stamp.clone()),
                            zone: None,
                            next: None,
                            frame_type: None,
                        },
                    );
                });
                *buffered_frames.lock().unwrap() = frames_local;
            }
            None => {
                r2r::log_error!(node_id, "Subscriber did not get the message?");
            }
        }
    }
}

// task to remove all stale active frames older than ACTIVE_FRAME_LIFETIME
pub async fn maintain_buffer(
    mut timer: r2r::Timer,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    active_frame_lifetime: i32,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let frames_local = buffered_frames.lock().unwrap().clone();
        let mut frames_local_reduced = frames_local.clone();
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let current_time = r2r::Clock::to_builtin_time(&now);
        frames_local.iter().for_each(|(k, v)| {
            let stamp = v.clone().time_stamp.unwrap(); // TODO: handle this nicer
            match v.active {
                Some(true) | None => match current_time.sec > stamp.sec + active_frame_lifetime {
                    true => {
                        frames_local_reduced.remove(k);
                    }
                    false => (), // do nothing if the frame is fresh
                },
                Some(false) => (), // maybe we should remove static frames older than STATIC_FRAME_LIFETIME
            }
        });
        *buffered_frames.lock().unwrap() = frames_local_reduced;
        timer.tick().await?;
    }
}

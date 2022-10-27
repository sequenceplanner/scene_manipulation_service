// A BIG TODO: disable reparenting and cloning if the parent and child is the same, breaks the tree

use futures::{stream::Stream, StreamExt};
use r2r::geometry_msgs::msg::TransformStamped;
use r2r::scene_manipulation_msgs::msg::{TFExtra, TFExtraData};
use r2r::scene_manipulation_msgs::srv::{
    ExtraFeatures, GetAllTransforms, LookupTransform, ManipulateScene,
};
use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::{ParameterValue, QosProfile, ServiceRequest};
use scene_manipulation_service::common::errors::{extra_error_response, extra_success_response};
use serde_json::Value;
use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::sync::{Arc, Mutex};

use scene_manipulation_service::{
    common::{
        files::{list_frames_in_dir, load_scenario, reload_scenario},
        frame_data::FrameData,
    },
    core::lookup::{check_would_produce_cycle, lookup_transform},
};

pub static NODE_ID: &'static str = "scene_manipulation_service";
pub static STATIC_BROADCAST_RATE: u64 = 1000;
pub static ACTIVE_BROADCAST_RATE: u64 = 100;
pub static EXTRA_BROADCAST_RATE: u64 = 100;
pub static BUFFER_MAINTAIN_RATE: u64 = 100;
pub static ACTIVE_FRAME_LIFETIME: i32 = 3; //seconds
pub static STATIC_FRAME_LIFETIME: i32 = 10; //seconds
pub static MAX_TRANSFORM_CHAIN: u64 = 100;

mod tests;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // setup the node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    // handle parameters passed on from the launch files
    let params = node.params.clone();
    let params_things = params.lock().unwrap(); // OK to panic
    let scenario_path = params_things.get("scenario_path");

    let path_param = match scenario_path {
        Some(p) => match p {
            ParameterValue::String(path_param) => path_param.clone(),
            _ => {
                r2r::log_warn!(
                    NODE_ID,
                    "Parameter 'scenario_path' has to be of type String. Empty scenario will be launched."
                );
                "".to_string()
            }
        },
        None => {
            r2r::log_warn!(
                NODE_ID,
                "Parameter 'scenario_path' not specified. Empty scenario will be launched."
            );
            "".to_string()
        }
    };

    let scenario_res = list_frames_in_dir(&path_param, NODE_ID).await;

    let init_loaded = match scenario_res {
        Ok(scenario) => {
            let loaded = load_scenario(&scenario, NODE_ID);
            r2r::log_info!(
                NODE_ID,
                "Initial frames added to the scene: '{:?}'.",
                loaded.keys()
            );
            loaded
        }
        Err(_) => {
            r2r::log_warn!(NODE_ID, "No initial frames added to the scene.");
            HashMap::<String, FrameData>::new()
        }
    };

    // a buffer of frames that are published by this broadcaster
    let broadcasted_frames = Arc::new(Mutex::new(init_loaded));

    // a buffer of frames that exist on the tf and tf_static topic
    let buffered_frames = Arc::new(Mutex::new(HashMap::<String, FrameData>::new()));

    // listen to the active frames on the tf topic to see if a frame exists before broadcasting it
    // spawn a tokio task to listen to the active frames and add them to the buffer
    let active_tf_listener =
        node.subscribe::<TFMessage>("tf", QosProfile::best_effort(QosProfile::default()))?;
    let buffered_frames_clone = buffered_frames.clone();
    tokio::task::spawn(async move {
        match active_tf_listener_callback(active_tf_listener, &buffered_frames_clone.clone()).await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Active tf listener failed with: '{}'.", e),
        };
    });

    // listen to the static frames on the static tf topic to see if a frame exists before broadcasting it
    // spawn a tokio task to listen to the static frames and add them to the buffer
    let static_tf_listener = node.subscribe::<TFMessage>(
        "tf_static",
        QosProfile::transient_local(QosProfile::default()),
    )?;
    let buffered_frames_clone = buffered_frames.clone();
    tokio::task::spawn(async move {
        match static_tf_listener_callback(static_tf_listener, &buffered_frames_clone.clone()).await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Static tf listener failed with: '{}'.", e),
        };
    });

    // publish the active frames to tf
    // spawn a tokio task to handle publishing active frames
    let active_pub_timer =
        node.create_wall_timer(std::time::Duration::from_millis(ACTIVE_BROADCAST_RATE))?;
    let active_frame_broadcaster = node
        .create_publisher::<TFMessage>("tf", QosProfile::transient_local(QosProfile::default()))?;
    let broadcasted_frames_clone = broadcasted_frames.clone();
    tokio::task::spawn(async move {
        match active_frame_broadcaster_callback(
            active_frame_broadcaster,
            active_pub_timer,
            &broadcasted_frames_clone,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Active frame broadcaster failed with: '{}'.", e),
        };
    });

    // publish the static frames to tf_static
    // spawn a tokio task to handle publishing static frames
    let static_pub_timer =
        node.create_wall_timer(std::time::Duration::from_millis(STATIC_BROADCAST_RATE))?;
    let static_frame_broadcaster = node.create_publisher::<TFMessage>(
        "tf_static",
        QosProfile::transient_local(QosProfile::default()),
    )?;
    let broadcasted_frames_clone = broadcasted_frames.clone();
    tokio::task::spawn(async move {
        match static_frame_broadcaster_callback(
            static_frame_broadcaster,
            static_pub_timer,
            &broadcasted_frames_clone,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Static frame broadcaster failed with: '{}'.", e),
        };
    });

    // publish the extra frame data to tf_extra
    // spawn a tokio task to handle publishing extra frame data
    let extra_pub_timer =
        node.create_wall_timer(std::time::Duration::from_millis(EXTRA_BROADCAST_RATE))?;
    let extra_frame_broadcaster = node.create_publisher::<TFExtra>(
        "tf_extra",
        QosProfile::transient_local(QosProfile::default()),
    )?;

    // sed out all, otherwise the extra node can't look up the transforms...
    let broadcasted_frames_clone = broadcasted_frames.clone();
    let buffered_frames_clone = buffered_frames.clone();
    tokio::task::spawn(async move {
        match extra_frame_broadcaster_callback(
            extra_frame_broadcaster,
            extra_pub_timer,
            &buffered_frames_clone,
            &broadcasted_frames_clone,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Extra frame broadcaster failed with: '{}'.", e),
        };
    });

    // offer the scene manipulation service
    let scene_manipulation_service =
        node.create_service::<ManipulateScene::Service>("manipulate_scene")?;
    let broadcasted_frames_clone = broadcasted_frames.clone();
    let buffered_frames_clone = buffered_frames.clone();
    tokio::task::spawn(async move {
        let result = scene_manipulation_server(
            scene_manipulation_service,
            &broadcasted_frames_clone,
            &buffered_frames_clone,
        )
        .await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Scene Manipulation Service call succeeded."),
            Err(e) => r2r::log_error!(
                NODE_ID,
                "Scene Manipulation Service call failed with: {}.",
                e
            ),
        };
    });

    // offer the transform lookup service
    let transform_lookup_service =
        node.create_service::<LookupTransform::Service>("lookup_transform")?;
    let buffered_frames_clone = buffered_frames.clone();
    tokio::task::spawn(async move {
        let result =
            transform_lookup_server(transform_lookup_service, &buffered_frames_clone).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Transform Lookup Service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Transform Lookup Service call failed with: {}.", e),
        };
    });

    // offer a service to get all frames from tf (local buffer)
    let get_all_transforms_service =
        node.create_service::<GetAllTransforms::Service>("get_all_transforms")?;
    let buffered_frames_clone = buffered_frames.clone();
    tokio::task::spawn(async move {
        let result =
            get_all_transforms_server(get_all_transforms_service, &buffered_frames_clone.clone())
                .await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Get All Frames Service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Get All Frames Service call failed with: {}.", e),
        };
    });

    // spawn a tokio task to maintain the tf buffer by removing stale frames and adding fresh ones
    let buffer_maintain_timer =
        node.create_wall_timer(std::time::Duration::from_millis(BUFFER_MAINTAIN_RATE))?;
    let buffered_frames_clone = buffered_frames.clone();
    tokio::task::spawn(async move {
        match maintain_buffer(buffer_maintain_timer, &buffered_frames_clone).await {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Buffer maintainer failed with: '{}'.", e),
        };
    });

    // offer a service for extra features, like the teaching marker, zones, paths, etc.
    let extra_features_service = node.create_service::<ExtraFeatures::Service>("extra_features")?;
    let broadcasted_frames_clone = broadcasted_frames.clone();
    tokio::task::spawn(async move {
        let result = extra_features_server(extra_features_service, &broadcasted_frames_clone).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Load Scenario Service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Load Scenario Service call failed with: {}.", e),
        };
    });

    // keep the node alive
    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(1000));
    });

    r2r::log_warn!(NODE_ID, "Node started.");

    handle.join().unwrap();

    Ok(())
}

// load or reload the scenario from json frame files
// TODO: add a loop check before adding frames

// TODO: sort out information in the messages when responding
async fn scene_manipulation_server(
    mut service: impl Stream<Item = ServiceRequest<ManipulateScene::Service>> + Unpin,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(request) => match request.message.command.as_str() {
                "add" => {
                    r2r::log_info!(NODE_ID, "Got 'add' request: {:?}.", request.message);
                    let response =
                        add_frame(&request.message, &broadcasted_frames, &buffered_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "remove" => {
                    r2r::log_info!(NODE_ID, "Got 'remove' request: {:?}.", request.message);
                    let response =
                        remove_frame(&request.message, &broadcasted_frames, &buffered_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "rename" => {
                    r2r::log_info!(NODE_ID, "Got 'rename' request: {:?}.", request.message);
                    let response =
                        rename_frame(&request.message, &broadcasted_frames, &buffered_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "move" => {
                    r2r::log_info!(NODE_ID, "Got 'move' request: {:?}.", request.message);
                    let response =
                        move_frame(&request.message, &broadcasted_frames, &buffered_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "reparent" => {
                    r2r::log_info!(NODE_ID, "Got 'reparent' request: {:?}.", request.message);
                    let response =
                        reparent_frame(&request.message, &broadcasted_frames, &buffered_frames)
                            .await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "clone" => {
                    r2r::log_info!(NODE_ID, "Got 'clone' request: {:?}.", request.message);
                    let response =
                        clone_frame(&request.message, &broadcasted_frames, &buffered_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "teach" => {
                    r2r::log_info!(NODE_ID, "Got 'teach' request: {:?}.", request.message);
                    let response =
                        teach_frame(&request.message, &broadcasted_frames, &buffered_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                _ => {
                    r2r::log_error!(NODE_ID, "No such command.");
                    continue;
                }
            },
            None => {}
        }
    }
}

async fn extra_features_server(
    mut service: impl Stream<Item = ServiceRequest<ExtraFeatures::Service>> + Unpin,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(request) => match request.message.command.as_str() {
                "reload_scenario" => {
                    r2r::log_info!(
                        NODE_ID,
                        "Got 'reload_scenario' request: {:?}.",
                        request.message
                    );
                    let response =
                        reload_scenario(&request.message, &broadcasted_frames, NODE_ID).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "set_zone" => {
                    r2r::log_info!(NODE_ID, "Got 'set_zone' request: {:?}.", request.message);
                    let response = set_zone(&request.message, &broadcasted_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "enable_path" => {
                    r2r::log_info!(NODE_ID, "Got 'enable_path' request: {:?}.", request.message);
                    let response =
                        enable_or_disable_path(&request.message, &broadcasted_frames, true).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "disable_path" => {
                    r2r::log_info!(
                        NODE_ID,
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
                    r2r::log_error!(NODE_ID, "No such command.");
                    continue;
                }
            },
            None => {}
        }
    }
}

// should work only for frames published by sms
// if the frame is not published by sms, the zones can be sent out on the /tf_extra topic
async fn set_zone(
    message: &r2r::scene_manipulation_msgs::srv::ExtraFeatures::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> ExtraFeatures::Response {
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
                    active: frame.active,
                    time_stamp: frame.time_stamp.clone(),
                    zone: Some(message.size),
                    next: frame.next.clone(),
                    frame_type: frame.frame_type.clone(),
                    // local: frame.local
                },
            );
            *broadcasted_frames.lock().unwrap() = local_broadcasted_frames_clone;
            extra_success_response(&format!(
                "Zone for '{}' has been set to '{}'.",
                &frame.child_frame_id, &message.size
            ))
            // }
            // Some(false) => extra_error_response(&format!(
            //     "Zone for '{}' can't be set since it is published elsewhere.",
            //     &frame.child_frame_id,
            //                 ))
            //         }
        }
        None => extra_error_response("Frame doesn't exist."),
    }
}

// TODO: should work only for frames published by sms.
// if the frame is not published by sms, the enabled paths can be sent out on the /tf_extra topic
async fn enable_or_disable_path(
    message: &r2r::scene_manipulation_msgs::srv::ExtraFeatures::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    enable: bool,
) -> ExtraFeatures::Response {
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
    let mut local_broadcasted_frames_clone = local_broadcasted_frames.clone();
    match local_broadcasted_frames.get(&message.parent_frame_id) {
        Some(parent) => {
            match local_broadcasted_frames.get(&message.child_frame_id) {
                Some(child) => {
                    let new_next = match parent.next.clone() {
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
                            active: parent.active,
                            time_stamp: parent.time_stamp.clone(),
                            zone: parent.zone,
                            next: Some(new_next),
                            frame_type: parent.frame_type.clone(),
                            // local: parent.local
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

async fn transform_lookup_server(
    mut service: impl Stream<Item = ServiceRequest<LookupTransform::Service>> + Unpin,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(request) => match lookup_transform(
                &request.message.parent_frame_id,
                &request.message.child_frame_id,
                &buffered_frames,
            )
            .await
            {
                Some(transform) => {
                    let info = &format!(
                        "Found transform from '{}' to '{}'.",
                        &request.message.parent_frame_id, &request.message.child_frame_id
                    );
                    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
                    let now = clock.get_now().unwrap();
                    let time_stamp = r2r::Clock::to_builtin_time(&now);
                    let response = LookupTransform::Response {
                        success: true,
                        info: info.to_string(),
                        transform: TransformStamped {
                            header: Header {
                                stamp: time_stamp,
                                frame_id: request.message.parent_frame_id.clone(),
                            },
                            child_frame_id: request.message.child_frame_id.clone(),
                            transform,
                        },
                    };
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                None => {
                    let info = &format!(
                        "Failed to lookup transform from '{}' to '{}'.",
                        &request.message.parent_frame_id, &request.message.child_frame_id
                    );
                    r2r::log_warn!(NODE_ID, "{}", info);
                    let response = LookupTransform::Response {
                        success: false,
                        info: info.to_string(),
                        ..Default::default()
                    };
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
            },
            None => {}
        }
    }
}

async fn get_all_transforms_server(
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

fn main_error_response(msg: &str) -> ManipulateScene::Response {
    let info = msg.to_string();
    // r2r::log_error!(NODE_ID, "{}", info);
    ManipulateScene::Response {
        success: false,
        info,
    }
}

fn main_success_response(msg: &str) -> ManipulateScene::Response {
    let info = msg.to_string();
    // r2r::log_info!(NODE_ID, "{}", info);
    ManipulateScene::Response {
        success: true,
        info,
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

// broadcast static frames
async fn static_frame_broadcaster_callback(
    publisher: r2r::Publisher<TFMessage>,
    mut timer: r2r::Timer,
    frames: &Arc<Mutex<HashMap<String, FrameData>>>,
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
                    NODE_ID,
                    "Static broadcaster failed to send a message with: '{}'",
                    e
                );
            }
        };
        timer.tick().await?;
    }
}

// broadcast active frames
async fn active_frame_broadcaster_callback(
    publisher: r2r::Publisher<TFMessage>,
    mut timer: r2r::Timer,
    frames: &Arc<Mutex<HashMap<String, FrameData>>>,
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
                    NODE_ID,
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
async fn active_tf_listener_callback(
    mut subscriber: impl Stream<Item = TFMessage> + Unpin,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
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
                r2r::log_error!(NODE_ID, "Subscriber did not get the message?");
            }
        }
    }
}

// updates the buffer with static frames from the tf_static topic
async fn static_tf_listener_callback(
    mut subscriber: impl Stream<Item = TFMessage> + Unpin,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
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
                r2r::log_error!(NODE_ID, "Subscriber did not get the message?");
            }
        }
    }
}

// task to remove all stale active frames older than ACTIVE_FRAME_LIFETIME
async fn maintain_buffer(
    mut timer: r2r::Timer,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
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
                Some(true) | None => match current_time.sec > stamp.sec + ACTIVE_FRAME_LIFETIME {
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

async fn extra_frame_broadcaster_callback(
    publisher: r2r::Publisher<TFExtra>,
    mut timer: r2r::Timer,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let mut buffered_frames_local = buffered_frames.lock().unwrap().clone();
        let broadcasted_frames_local = broadcasted_frames.lock().unwrap().clone();

        // collect frames from tf and overwrite with data from the broadcaster buffer
        broadcasted_frames_local.iter().for_each(|(k, v)| {
            // println!("{:?}", v);
            buffered_frames_local.insert(k.to_string(), v.clone());
        });
        let mut to_publish = vec![];

        buffered_frames_local.iter().for_each(|(_, v)| {
            to_publish.push(TFExtraData {
                transform: TransformStamped {
                    header: Header {
                        stamp: time_stamp.clone(),
                        frame_id: v.parent_frame_id.clone(),
                    },
                    child_frame_id: v.child_frame_id.clone(),
                    transform: v.transform.clone(),
                },
                extra: {
                    let mut extra_map = serde_json::Map::<String, Value>::new();
                    extra_map.insert(
                        "frame_type".to_string(),
                        Value::from(v.frame_type.clone().unwrap_or_default()),
                    );
                    extra_map.insert("zone".to_string(), Value::from(v.zone.unwrap_or_default()));
                    extra_map.insert(
                        "next".to_string(),
                        Value::from(
                            v.next
                                .clone()
                                .unwrap_or_default()
                                .iter()
                                .map(|value| value.clone())
                                .collect::<Vec<String>>(),
                        ),
                    );
                    // println!("map: {:?}", extra_map);
                    serde_json::Value::Object(extra_map).to_string()
                },
            });
        });

        let msg = TFExtra { data: to_publish };

        match publisher.publish(&msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    NODE_ID,
                    "Extra frame broadcaster failed to send a message with: '{}'",
                    e
                );
            }
        };
        timer.tick().await?;
    }
}

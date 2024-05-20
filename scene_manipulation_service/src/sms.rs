use r2r::scene_manipulation_msgs::msg::TFExtra;
use r2r::scene_manipulation_msgs::srv::*;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::{ParameterValue, QosProfile};
use scene_manipulation_service::core::sms::scene_manipulation_server;
use scene_manipulation_service::{
    active_frame_broadcaster_callback, active_tf_listener_callback, extra_features_server,
    extra_frame_broadcaster_callback, get_all_transforms_server, static_frame_broadcaster_callback,
    static_tf_listener_callback, transform_lookup_server, get_extra_server
};
use std::collections::HashMap;
use std::error::Error;
use std::sync::{Arc, Mutex};

use scene_manipulation_service::{
    common::{
        files::{list_frames_in_dir, load_scenario},
        frame_data::FrameData,
    },
    core::buffer::maintain_buffer,
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
    let mesh_path = params_things.get("mesh_path").and_then(|mp| {
        if let ParameterValue::String(s) = &mp.value {
            Some(s.to_owned())
        } else {
            None
        }
    });

    let path_param = match scenario_path {
        Some(p) => match &p.value {
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
        match active_tf_listener_callback(
            active_tf_listener,
            &buffered_frames_clone.clone(),
            NODE_ID,
        )
        .await
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
        match static_tf_listener_callback(
            static_tf_listener,
            &buffered_frames_clone.clone(),
            NODE_ID,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Static tf listener failed with: '{}'.", e),
        };
    });

    // this should go to gpss
    // offer a service to get all extras from the broadcaster
    // let get_all_extras_service = node.create_service::<GetAllExtra::Service>("get_all_extras")?;
    // let buffered_frames_clone = buffered_frames.clone();
    // tokio::task::spawn(async move {
    //     let result =
    //         get_all_extras_server(get_all_extras_service, &buffered_frames_clone.clone()).await;
    //     match result {
    //         Ok(()) => r2r::log_info!(NODE_ID, "Get All Extras Service call succeeded."),
    //         Err(e) => r2r::log_error!(NODE_ID, "Get All Extras Service call failed with: {}.", e),
    //     };
    // });

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
            NODE_ID,
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
            NODE_ID,
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

    let broadcasted_frames_clone = broadcasted_frames.clone();
    let buffered_frames_clone = buffered_frames.clone();
    tokio::task::spawn(async move {
        match extra_frame_broadcaster_callback(
            extra_frame_broadcaster,
            extra_pub_timer,
            &buffered_frames_clone,
            &broadcasted_frames_clone,
            NODE_ID,
            mesh_path,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Extra frame broadcaster failed with: '{}'.", e),
        };
    });

    // offer the scene manipulation service
    let scene_manipulation_service =
        node.create_service::<ManipulateScene::Service>("manipulate_scene", QosProfile::default())?;
    let broadcasted_frames_clone = broadcasted_frames.clone();
    let buffered_frames_clone = buffered_frames.clone();
    tokio::task::spawn(async move {
        let result = scene_manipulation_server(
            scene_manipulation_service,
            &broadcasted_frames_clone,
            &buffered_frames_clone,
            &path_param,
            NODE_ID,
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
        node.create_service::<LookupTransform::Service>("lookup_transform", QosProfile::default())?;
    let buffered_frames_clone = buffered_frames.clone();
    tokio::task::spawn(async move {
        let result =
            transform_lookup_server(transform_lookup_service, &buffered_frames_clone, NODE_ID)
                .await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Transform Lookup Service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Transform Lookup Service call failed with: {}.", e),
        };
    });

    // offer a service to get all frames from tf (local buffer)
    let get_all_transforms_service =
        node.create_service::<GetAllTransforms::Service>("get_all_transforms", QosProfile::default())?;
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

    let get_extra_service =
        node.create_service::<GetExtra::Service>("get_extra", QosProfile::default())?;
    let frames_clone = broadcasted_frames.clone();
    tokio::task::spawn(async move {
        let result =
            get_extra_server(get_extra_service, &frames_clone.clone()).await;
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
        match maintain_buffer(
            buffer_maintain_timer,
            &buffered_frames_clone,
            ACTIVE_FRAME_LIFETIME,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Buffer maintainer failed with: '{}'.", e),
        };
    });

    // this can remain here since it is connected to the GUI
    // offer a service for extra features, like the teaching marker, zones, paths, etc.
    let extra_features_service =
        node.create_service::<ManipulateExtras::Service>("extra_features", QosProfile::default())?;
    let broadcasted_frames_clone = broadcasted_frames.clone();
    tokio::task::spawn(async move {
        let result =
            extra_features_server(extra_features_service, &broadcasted_frames_clone, NODE_ID).await;
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

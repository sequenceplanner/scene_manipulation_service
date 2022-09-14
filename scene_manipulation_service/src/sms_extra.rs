use futures::{stream::Stream, StreamExt};
use glam::{DAffine3, DQuat, DVec3};
use r2r::builtin_interfaces::msg::{Duration, Time};
use r2r::geometry_msgs::msg::{Point, Pose, Quaternion, Transform, TransformStamped, Vector3};
use r2r::scene_manipulation_msgs::msg::{TFExtra, TFExtraData};
use r2r::scene_manipulation_msgs::srv::{
    ExtraFeatures, GetAllExtra, GetAllTransforms, LookupTransform, ManipulateScene,
};
use r2r::std_msgs::msg::{ColorRGBA, Header};
use r2r::tf2_msgs::msg::TFMessage;
use r2r::visualization_msgs::msg::{Marker, MarkerArray};
use r2r::{ParameterValue, QosProfile, ServiceRequest};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fs::{self, File};
use std::io::BufReader;
use std::sync::{Arc, Mutex};
use std::{default, fmt};

use scene_manipulation_service::common::lookup::{
    check_would_produce_cycle, lookup_transform, FrameData,
};

pub static NODE_ID: &'static str = "scene_manipulation_service_extra";
pub static STATIC_BROADCAST_RATE: u64 = 1000;
pub static ACTIVE_BROADCAST_RATE: u64 = 100;
pub static BUFFER_MAINTAIN_RATE: u64 = 100;
pub static ACTIVE_FRAME_LIFETIME: i32 = 3; //seconds
pub static STATIC_FRAME_LIFETIME: i32 = 10; //seconds
pub static MAX_TRANSFORM_CHAIN: u64 = 100;

// #[derive(Debug, Default, Serialize, Deserialize, Clone, PartialEq)]
// pub struct FrameData {
//     pub parent_frame_id: String,        // the id of the frame's parent frame
//     pub child_frame_id: String,         // the id of the frame
//     pub transform: Transform,           // where is the child frame defined in the parent
//     pub active: bool,                   // only active frames are manipulatable
//     pub zone: Option<f64>,              // when are you "at" the frame, threshold, in meters
//     pub next: Option<HashSet<String>>,  // this list can be used to store data for planners and visualizers
//     pub time_stamp: Option<Time>,       // the idea is that all frames should have this, but some don't
//     pub frame_type: Option<String>,     // can be used to distinguish if a frame is a waypoint, tag, human, etc.
//     pub local: bool                     // is the frame published by sms or elsewhere, i.e. is it manipulatable or not
// }

// the extra stuff that is arriving - everything should be option because it can be anything or nothing
#[derive(Debug, Default, Serialize, Deserialize, Clone, PartialEq)]
pub struct ExtraData {
    pub zone: Option<f64>,              // when are you "at" the frame, threshold, in meters
    pub next: Option<HashSet<String>>,  // this list can be used to store data for planners and visualizers
    pub time_stamp: Option<Time>,       // the idea is that all frames should have this, but some don't
    pub frame_type: Option<String>,     // can be used to distinguish if a frame is a waypoint, tag, human, etc.
}

// the testing module
// mod tests;

// some error handling utils
#[derive(Debug, Clone)]
struct ErrorMsg {
    info: String,
}

impl ErrorMsg {
    fn new(info: &str) -> ErrorMsg {
        ErrorMsg {
            info: info.to_string(),
        }
    }
}

impl fmt::Display for ErrorMsg {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.info)
    }
}

impl Error for ErrorMsg {
    fn description(&self) -> &str {
        &self.info
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // setup the node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    // frames that exist on the tf_extra
    let buffered_frames = Arc::new(Mutex::new(HashMap::<String, FrameData>::new()));

    // listen to the active frames on the tf topic to see if a frame exists before broadcasting it
    // spawn a tokio task to listen to the active frames and add them to the buffer
    let extra_tf_listener =
        node.subscribe::<TFExtra>("tf_extra", QosProfile::best_effort(QosProfile::default()))?;
    let buffered_frames_clone = buffered_frames.clone();
    tokio::task::spawn(async move {
        match extra_tf_listener_callback(extra_tf_listener, &buffered_frames_clone.clone()).await {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Extra tf listener failed with: '{}'.", e),
        };
    });

    // offer a service to get all extras from the broadcaster
    let get_all_extras_service = node.create_service::<GetAllExtra::Service>("get_all_extras")?;

    // publish the visualization markers for zone size for each frame
    let zone_marker_publisher = node
        .create_publisher::<r2r::visualization_msgs::msg::MarkerArray>(
            "zone_markers",
            QosProfile::default(),
        )?;

    // publish the visualization markers enabled paths for each frame's 'next' pair
    let path_marker_publisher = node
        .create_publisher::<r2r::visualization_msgs::msg::MarkerArray>(
            "path_markers",
            QosProfile::default(),
        )?;

    let marker_timer = node.create_wall_timer(std::time::Duration::from_millis(200))?;
    let buffered_frames_clone = buffered_frames.clone();
    tokio::task::spawn(async move {
        match marker_publisher_callback(
            zone_marker_publisher,
            path_marker_publisher,
            &buffered_frames_clone,
            marker_timer,
        )
        .await
        {
            Ok(()) => println!("done."),
            Err(e) => println!("error: {}", e),
        };
    });

    // frequency of how often to refresh (add or remove) frames to the local tf buffer
    let buffer_maintain_timer =
        node.create_wall_timer(std::time::Duration::from_millis(BUFFER_MAINTAIN_RATE))?;

    // spawn a tokio task to maintain the buffer by removing stale active frames
    let buffered_frames_clone = buffered_frames.clone();
    tokio::task::spawn(async move {
        match maintain_buffer(buffer_maintain_timer, &buffered_frames_clone).await {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Buffer maintainer failed with: '{}'.", e),
        };
    });

    let buffered_frames_clone = buffered_frames.clone();
    tokio::task::spawn(async move {
        let result =
            get_all_extras_server(get_all_extras_service, &buffered_frames_clone.clone()).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Get All Extras Service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Get All Extras Service call failed with: {}.", e),
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

// fn get_time_stamp() ->

fn tf_extra_to_frame_data(extras: HashMap<String, TFExtraData>) -> HashMap<String, FrameData> {
    let mut map = HashMap::<String, FrameData>::new();

    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
    let now = clock.get_now().unwrap();
    let time_stamp = r2r::Clock::to_builtin_time(&now);

    extras.iter().for_each(|(k, v)| {
        let extra_json = serde_json::json!(v.extra);
        match serde_json::from_value(extra_json) {
            Ok::<FrameData, _>(extra) => {
                map.insert(
                    k.clone(),
                    FrameData {
                        parent_frame_id: v.transform.header.frame_id.clone(),
                        child_frame_id: v.transform.child_frame_id.clone(),
                        transform: v.transform.transform.clone(),
                        time_stamp: Some(time_stamp.clone()),
                        zone: extra.zone,
                        next: extra.next,
                        frame_type: extra.frame_type,
                        active: extra.active,
                    },
                );
            }
            Err(_) => (),
        };
    });
    map
}

// fn frame_data_to_tf_extra(frames: HashMap<String, FrameData>) -> HashMap<String, TFExtraData> {
//     let mut map = HashMap::<String, TFExtraData>::new();

//     let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
//     let now = clock.get_now().unwrap();
//     let time_stamp = r2r::Clock::to_builtin_time(&now);

//     frames.iter().for_each(|(k, v)| {
//         let extra_json = serde_json::json!(v.extra);
//         match serde_json::from_value(extra_json) {
//             Ok::<FrameData, _>(extra) => {
//                 map.insert(
//                     k.clone(),
//                     FrameData {
//                         parent_frame_id: v.transform.header.frame_id.clone(),
//                         child_frame_id: v.transform.child_frame_id.clone(),
//                         transform: v.transform.transform.clone(),
//                         time_stamp: Some(time_stamp),
//                         zone: extra.zone,
//                         next: extra.next,
//                         frame_type: extra.frame_type,
//                         active: extra.active,
//                     },
//                 );
//             }
//             Err(_) => (),
//         };
//     });
//     map
// }

async fn get_all_extras_server(
    mut service: impl Stream<Item = ServiceRequest<GetAllExtra::Service>> + Unpin,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(request) => {
                let frames_local = buffered_frames.lock().unwrap().clone();
                // let frames_local = tf_extra_to_frame_data(frames_local_pre);
                let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
                let now = clock.get_now().unwrap();
                let time_stamp = r2r::Clock::to_builtin_time(&now);
                let mut filtered_correct_type: Vec<FrameData> = vec![];
                let mut extras_list = vec![];

                frames_local
                    .iter()
                    .for_each(|frame| match &frame.1.frame_type {
                        Some(has_type) => match request.message.frame_type.as_str() {
                            "" => filtered_correct_type.push(frame.1.clone()),
                            _ => match has_type == &request.message.frame_type {
                                true => filtered_correct_type.push(frame.1.clone()),
                                false => (),
                            },
                        },
                        None => (),
                    });

                for filtered in filtered_correct_type {
                    match lookup_transform(
                        &request.message.parent_frame_id,
                        &filtered.child_frame_id,
                        &Arc::new(Mutex::new(frames_local.clone())),
                    )
                    .await
                    {
                        Some(found) => extras_list.push(TFExtraData {
                            transform: TransformStamped {
                                header: Header {
                                    stamp: time_stamp.clone(),
                                    frame_id: request.message.parent_frame_id.clone(),
                                },
                                child_frame_id: filtered.child_frame_id.clone(),
                                transform: found.clone(),
                            },
                            extra: {
                                let mut extra_map = serde_json::Map::<String, Value>::new();
                                let frame_type_data = match filtered.frame_type {
                                    Some(frame_type) => frame_type,
                                    None => "".to_string(),
                                };
                                let zone_data = match filtered.zone {
                                    Some(zone) => zone,
                                    None => 0.0,
                                };
                                let next_data = match filtered.next {
                                    Some(next) => next.into_iter().collect(),
                                    None => vec![],
                                };
                                extra_map.insert(
                                    "frame_type".to_string(),
                                    Value::from(frame_type_data.clone()),
                                );
                                extra_map.insert("zone".to_string(), Value::from(zone_data));
                                extra_map.insert(
                                    "next".to_string(),
                                    Value::from(
                                        next_data
                                            .clone()
                                            .iter()
                                            .map(|value| value.clone())
                                            .collect::<Vec<String>>(),
                                    ),
                                );
                                serde_json::Value::Object(extra_map).to_string()
                            },
                        }),
                        None => (),
                    }
                }

                let response = GetAllExtra::Response {
                    success: true,
                    info: "".to_string(),
                    extras: TFExtra { data: extras_list },
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

fn extra_error_response(msg: &str) -> ExtraFeatures::Response {
    let info = msg.to_string();
    // r2r::log_error!(NODE_ID, "{}", info);
    ExtraFeatures::Response {
        success: false,
        info,
    }
}

fn extra_success_response(msg: &str) -> ExtraFeatures::Response {
    let info = msg.to_string();
    // r2r::log_info!(NODE_ID, "{}", info);
    ExtraFeatures::Response {
        success: true,
        info,
    }
}

// updates the broadcaster buffer with frames from the tf extra topic
async fn extra_tf_listener_callback(
    mut subscriber: impl Stream<Item = TFExtra> + Unpin,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let mut frames_local = buffered_frames.lock().unwrap().clone();

                message.data.iter().for_each(|v| {
                    // println!("encoded {:?}", v);
                    // let extra_json = serde_json::json!(v.extra);
                    // match serde_json::from_value(extra_json) {
                    match serde_json::from_str(&v.extra) {
                        Ok::<ExtraData, _>(extra) => {
                            let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
                            let now = clock.get_now().unwrap();
                            let time_stamp = r2r::Clock::to_builtin_time(&now);
                            // println!("decoded {:?}", extra.zone);
                            frames_local.insert(
                                v.transform.child_frame_id.clone(),
                                FrameData {
                                    parent_frame_id: v.transform.header.frame_id.clone(),
                                    child_frame_id: v.transform.child_frame_id.clone(),
                                    transform: v.transform.transform.clone(),
                                    time_stamp: Some(time_stamp),
                                    zone: extra.zone,
                                    next: extra.next,
                                    frame_type: extra.frame_type,
                                    active: Some(true)
                                },
                            );
                        }
                        Err(_) => () //println!("Doesn't work because: {:?}", e),
                    };
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
        // let frames_local = tf_extra_to_frame_data(frames_local_pre);
        let mut frames_local_reduced = frames_local.clone();
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let current_time = r2r::Clock::to_builtin_time(&now);
        frames_local.iter().for_each(|(k, v)| match v.active {
            Some(true) | None => match &v.time_stamp {
                Some(stamp) => match current_time.sec > stamp.sec + ACTIVE_FRAME_LIFETIME {
                    true => {
                        frames_local_reduced.remove(k);
                    }
                    false => (), // do nothing if the frame is fresh
                },
                None => {
                    frames_local_reduced.remove(k);
                    r2r::log_warn!(
                        NODE_ID,
                        "Active frame shouldn't have 'None' timestamp. Investigate."
                    );
                }
            },
            Some(false) => (), // maybe we should remove static frames older than STATIC_FRAME_LIFETIME
        });
        *buffered_frames.lock().unwrap() = frames_local_reduced;
        timer.tick().await?;
    }
}

async fn marker_publisher_callback(
    zone_publisher: r2r::Publisher<MarkerArray>,
    path_publisher: r2r::Publisher<MarkerArray>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    mut timer: r2r::Timer,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {

        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let mut zone_markers: Vec<Marker> = vec![];
        let mut path_markers: Vec<Marker> = vec![];
        let frames_local = buffered_frames.lock().unwrap().clone();
        // let decoded = serde_json::json!()
        let mut id = 0;
        for frame in frames_local {
            // let extra_json = serde_json::json!(frame.1.extra);
            // let extra: FrameData = serde_json::from_value(extra_json).unwrap_or_default();
            match frame.1.zone {
                Some(z) => {
                    id = id + 1;
                    let indiv_marker = Marker {
                        header: Header {
                            stamp: time_stamp.clone(),
                            frame_id: frame.1.child_frame_id.to_string(),
                        },
                        ns: "".to_string(),
                        id,
                        type_: 3,
                        action: 0,
                        pose: Pose {
                            position: Point {
                                x: 0.0,
                                y: 0.0,
                                z: 0.0,
                            },
                            orientation: Quaternion {
                                x: 0.0,
                                y: 0.0,
                                z: 0.0,
                                w: 1.0,
                            },
                        },
                        lifetime: Duration { sec: 2, nanosec: 0 },
                        scale: Vector3 {
                            x: z,
                            y: z,
                            z: 0.01,
                        },
                        color: ColorRGBA {
                            r: 0.0,
                            g: 255.0,
                            b: 0.0,
                            a: 0.15,
                        },
                        ..Marker::default()
                    };
                    zone_markers.push(indiv_marker)
                }
                None => (),
            }
            match frame.1.next {
                Some(n) => {
                    for enabled in n {
                        match lookup_transform(
                            &frame.1.child_frame_id,
                            &enabled,
                            &buffered_frames,
                        )
                        .await
                        {
                            Some(end_in_world) => {
                                id = id + 1;
                                let indiv_marker = Marker {
                                    header: Header {
                                        stamp: r2r::builtin_interfaces::msg::Time {
                                            sec: 0,
                                            nanosec: 0,
                                        },
                                        frame_id: frame.1.child_frame_id.to_string(),
                                    },
                                    ns: "".to_string(),
                                    id,
                                    type_: 0,
                                    action: 0,
                                    points: vec![
                                        Point {
                                            x: 0.0,
                                            y: 0.0,
                                            z: 0.0,
                                        },
                                        Point {
                                            x: end_in_world.translation.x,
                                            y: end_in_world.translation.y,
                                            z: end_in_world.translation.z,
                                        },
                                    ],
                                    lifetime: Duration { sec: 2, nanosec: 0 },
                                    scale: Vector3 {
                                        x: 0.1,
                                        y: 0.2,
                                        z: 0.5,
                                    },
                                    color: ColorRGBA {
                                        r: 0.0,
                                        g: 255.0,
                                        b: 0.0,
                                        a: 0.15,
                                    },
                                    ..Marker::default()
                                };
                                path_markers.push(indiv_marker)
                            }
                            None => (),
                        }
                    }
                }
                None => (),
            }
        }

        let zone_array_msg = MarkerArray {
            markers: zone_markers,
        };
        let path_array_msg = MarkerArray {
            markers: path_markers,
        };

        match zone_publisher.publish(&zone_array_msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    NODE_ID,
                    "Publisher failed to send zone marker message with: {}",
                    e
                );
            }
        };

        match path_publisher.publish(&path_array_msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    NODE_ID,
                    "Publisher failed to send path marker message with: {}",
                    e
                );
            }
        };

        timer.tick().await?;
    }
}

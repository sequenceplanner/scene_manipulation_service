use futures::{stream::Stream, StreamExt};
use glam::{DAffine3, DQuat, DVec3};
use r2r::builtin_interfaces::msg::{Duration, Time};
use r2r::geometry_msgs::msg::{Point, Pose, Quaternion, Transform, TransformStamped, Vector3};
use r2r::scene_manipulation_msgs::msg::{TFExtra, TFExtraData};
use r2r::scene_manipulation_msgs::srv::{
    ExtraFeatures, GetAllTransforms, LookupTransform, ManipulateScene, GetAllExtra
};
use r2r::std_msgs::msg::{ColorRGBA, Header};
use r2r::tf2_msgs::msg::TFMessage;
use r2r::visualization_msgs::msg::{Marker, MarkerArray};
use r2r::{ParameterValue, QosProfile, ServiceRequest};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fs::{self, File};
use std::io::BufReader;
use std::sync::{Arc, Mutex};
use std::{default, fmt};

pub static NODE_ID: &'static str = "scene_manipulation_service";
pub static STATIC_BROADCAST_RATE: u64 = 1000;
pub static ACTIVE_BROADCAST_RATE: u64 = 100;
pub static BUFFER_MAINTAIN_RATE: u64 = 100;
pub static ACTIVE_FRAME_LIFETIME: i32 = 3; //seconds
pub static STATIC_FRAME_LIFETIME: i32 = 10; //seconds
pub static MAX_TRANSFORM_CHAIN: u64 = 100;

#[derive(Debug, Default, Serialize, Deserialize, Clone, PartialEq)]
pub struct FrameData {
    pub parent_frame_id: String,
    pub child_frame_id: String,
    pub transform: Transform,
    pub active: bool,
    pub zone: Option<f64>,
    pub next: Option<HashSet<String>>,
    pub time_stamp: Option<Time>,
    pub frame_type: Option<String>,
    // pub local: bool     // is it published by sms or elsewhere
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

    let scenario_res = list_frames_in_dir(&path_param).await;

    let init_loaded = match scenario_res {
        Ok(scenario) => {
            let loaded = load_scenario(&scenario).await;
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

    // frames that are published by this broadcaster
    let broadcasted_frames = Arc::new(Mutex::new(init_loaded));

    // frames that exist on the tf and tf_static topic, i.e. this is a tf buffer
    let buffered_frames = Arc::new(Mutex::new(HashMap::<String, FrameData>::new()));

    // listen to the active frames on the tf topic to see if a frame exists before broadcasting it
    let active_tf_listener =
        node.subscribe::<TFMessage>("tf", QosProfile::best_effort(QosProfile::default()))?;

    // listen to the active frames on the static tf topic to see if a frame exists before broadcasting it
    let static_tf_listener = node.subscribe::<TFMessage>(
        "tf_static",
        QosProfile::transient_local(QosProfile::default()),
    )?;

    // listen to the frames on the tf extra topic. Some nodes need to publish some additional info
    // about frames, so this is where we can pick that up and maintain it and publish it to tf
    let extra_tf_listener =
        node.subscribe::<TFExtra>("tf_extra", QosProfile::best_effort(QosProfile::default()))?;

    // publish the static frames to tf_static
    let static_pub_timer =
        node.create_wall_timer(std::time::Duration::from_millis(STATIC_BROADCAST_RATE))?;
    let static_frame_broadcaster = node.create_publisher::<TFMessage>(
        "tf_static",
        QosProfile::transient_local(QosProfile::default()),
    )?;

    // publish the active frames to tf
    let active_pub_timer =
        node.create_wall_timer(std::time::Duration::from_millis(ACTIVE_BROADCAST_RATE))?;
    let active_frame_broadcaster = node
        .create_publisher::<TFMessage>("tf", QosProfile::transient_local(QosProfile::default()))?;

    // offer the scene manipulation service
    let scene_manipulation_service =
        node.create_service::<ManipulateScene::Service>("manipulate_scene")?;

    // offer the transform lookup service
    let transform_lookup_service =
        node.create_service::<LookupTransform::Service>("lookup_transform")?;

    // offer a service for extra features
    let extra_features_service = node.create_service::<ExtraFeatures::Service>("extra_features")?;

    // offer a service to get all frames from tf (local buffer)
    // TODO: add a parent id in the request so that the frames are tranformed in it 
    let get_all_transforms_service =
        node.create_service::<GetAllTransforms::Service>("get_all_transforms")?;

    // offer a service to get all extras from the broadcaster
    let get_all_extras_service =
        node.create_service::<GetAllExtra::Service>("get_all_extras")?;

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
    let broadcasted_frames_clone_16 = broadcasted_frames.clone();
    let buffered_frames_clone_16 = buffered_frames.clone();
    tokio::task::spawn(async move {
        match marker_publisher_callback(
            zone_marker_publisher,
            path_marker_publisher,
            &broadcasted_frames_clone_16,
            &buffered_frames_clone_16,
            marker_timer,
        )
        .await
        {
            Ok(()) => println!("done."),
            Err(e) => println!("error: {}", e),
        };
    });

    // spawn a tokio task to handle publishing static frames
    let broadcasted_frames_clone_1 = broadcasted_frames.clone();
    tokio::task::spawn(async move {
        match static_frame_broadcaster_callback(
            static_frame_broadcaster,
            static_pub_timer,
            &broadcasted_frames_clone_1,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Static frame broadcaster failed with: '{}'.", e),
        };
    });

    // spawn a tokio task to handle publishing active frames
    let broadcasted_frames_clone_2 = broadcasted_frames.clone();
    tokio::task::spawn(async move {
        match active_frame_broadcaster_callback(
            active_frame_broadcaster,
            active_pub_timer,
            &broadcasted_frames_clone_2,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Active frame broadcaster failed with: '{}'.", e),
        };
    });

    // offer a service to manipulate the scene
    let broadcasted_frames_clone_4 = broadcasted_frames.clone();
    let buffered_frames_clone_1 = buffered_frames.clone();
    tokio::task::spawn(async move {
        let result = scene_manipulation_server(
            scene_manipulation_service,
            &broadcasted_frames_clone_4,
            &buffered_frames_clone_1,
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

    // offer a service for clients that want to lookup the transform between two frames from the local buffer
    let buffered_frames_clone_2 = buffered_frames.clone();
    tokio::task::spawn(async move {
        // std::thread::sleep(std::time::Duration::from_millis(2000));
        let result =
            transform_lookup_server(transform_lookup_service, &buffered_frames_clone_2).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Transform Lookup Service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Transform Lookup Service call failed with: {}.", e),
        };
    });

    // offer a service for clients that want to load/reload the scenario
    let broadcasted_frames_clone_5 = broadcasted_frames.clone();
    tokio::task::spawn(async move {
        let result =
            extra_features_server(extra_features_service, &broadcasted_frames_clone_5).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Load Scenario Service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Load Scenario Service call failed with: {}.", e),
        };
    });

    // spawn a tokio task to listen to the active frames and add them to the buffer
    let buffered_frames_clone_2 = buffered_frames.clone();
    tokio::task::spawn(async move {
        match active_tf_listener_callback(active_tf_listener, &buffered_frames_clone_2.clone())
            .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Active tf listener failed with: '{}'.", e),
        };
    });

    // spawn a tokio task to listen to the active frames and add them to the buffer
    let buffered_frames_clone_7 = buffered_frames.clone();
    tokio::task::spawn(async move {
        match static_tf_listener_callback(static_tf_listener, &buffered_frames_clone_7.clone())
            .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Static tf listener failed with: '{}'.", e),
        };
    });

    // let's see if this will work out...
    let boadcasted_frames_clone_6 = broadcasted_frames.clone();
    tokio::task::spawn(async move {
        match extra_tf_listener_callback(extra_tf_listener, &boadcasted_frames_clone_6.clone())
            .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Extra tf listener failed with: '{}'.", e),
        };
    });


    // frequency of how often to refresh (add or remove) frames to the local tf buffer
    let buffer_maintain_timer =
        node.create_wall_timer(std::time::Duration::from_millis(BUFFER_MAINTAIN_RATE))?;

    // spawn a tokio task to maintain the buffer by removing stale active frames
    let buffered_frames_clone_3 = buffered_frames.clone();
    tokio::task::spawn(async move {
        match maintain_buffer(buffer_maintain_timer, &buffered_frames_clone_3).await {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Buffer maintainer failed with: '{}'.", e),
        };
    });

    // offer a service for clients that want to get all transforms from the local buffer and their names
    let buffered_frames_clone_31 = buffered_frames.clone();
    tokio::task::spawn(async move {
        let result =
            get_all_transforms_server(get_all_transforms_service, &buffered_frames_clone_31.clone()).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Get All Frames Service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Get All Frames Service call failed with: {}.", e),
        };
    });

    tokio::task::spawn(async move {
        let result =
            get_all_extras_server(get_all_extras_service, &broadcasted_frames.clone(), &buffered_frames.clone()).await;
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

async fn list_frames_in_dir(path: &str) -> Result<Vec<String>, Box<dyn std::error::Error + Send>> {
    let mut scenario = vec![];
    match fs::read_dir(path) {
        Ok(dir) => dir.for_each(|file| match file {
            Ok(entry) => match entry.path().to_str() {
                Some(valid) => scenario.push(valid.to_string()),
                None => r2r::log_warn!(NODE_ID, "Path is not valid unicode."),
            },
            Err(e) => r2r::log_warn!(NODE_ID, "Reading entry failed with '{}'.", e),
        }),
        Err(e) => {
            r2r::log_warn!(
                NODE_ID,
                "Reading the scenario directory failed with: '{}'.",
                e
            );
            r2r::log_warn!(NODE_ID, "Empty scenario is loaded/reloaded.");
            return Err(Box::new(ErrorMsg::new(&format!(
                "Reading the scenario directory failed with: '{}'. 
                    Empty scenario is loaded/reloaded.",
                e
            ))));
        }
    }
    Ok(scenario)
}

// load or reload the scenario from json frame files
// TODO: add a loop check before adding frames
async fn load_scenario(scenario: &Vec<String>) -> HashMap<String, FrameData> {
    let mut frame_datas: HashMap<String, FrameData> = HashMap::new();

    // add the world frame explicitly so that transforms can be looked up to and from it
    frame_datas.insert(
        "world".to_string(),
        FrameData {
            parent_frame_id: "world_origin".to_string(),
            child_frame_id: "world".to_string(),
            active: false,
            transform: Transform {
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                translation: Vector3 {
                    ..Default::default()
                },
            },
            ..Default::default()
        },
    );

    scenario.iter().for_each(|x| match File::open(x) {
        Ok(file) => {
            let reader = BufReader::new(file);
            match serde_json::from_reader(reader) {
                Ok::<FrameData, _>(json) => {
                    frame_datas.insert(json.child_frame_id.clone(), json.clone());
                }
                Err(e) => {
                    r2r::log_warn!(NODE_ID, "Serde failed with: '{}'.", e);
                }
            }
        }
        Err(e) => r2r::log_warn!(NODE_ID, "Opening json file failed with: '{}'.", e),
    });
    frame_datas
}

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
                    let response = reload_scenario(&request.message, &broadcasted_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "set_zone" => {
                    r2r::log_info!(
                        NODE_ID,
                        "Got 'set_zone' request: {:?}.",
                        request.message
                    );
                    let response = set_zone(&request.message, &broadcasted_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "enable_path" => {
                    r2r::log_info!(
                        NODE_ID,
                        "Got 'enable_path' request: {:?}.",
                        request.message
                    );
                    let response = enable_or_disable_path(&request.message, &broadcasted_frames, true).await;
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
                    let response = enable_or_disable_path(&request.message, &broadcasted_frames, false).await;
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

async fn reload_scenario(
    message: &r2r::scene_manipulation_msgs::srv::ExtraFeatures::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> ExtraFeatures::Response {
    match list_frames_in_dir(&message.scenario_path).await {
        Ok(scenario) => {
            let loaded = load_scenario(&scenario).await;
            let mut local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
            for x in &loaded {
                local_broadcasted_frames.insert(x.0.clone(), x.1.clone());
            }
            *broadcasted_frames.lock().unwrap() = local_broadcasted_frames;
            extra_success_response(&format!(
                "Reloaded frames in the scene: '{:?}'.",
                loaded.keys()
            ))
        }
        Err(e) => {
            extra_error_response(&format!("Reloading the scenario failed with: '{:?}'.", e))
        }
    }
}

async fn set_zone(
    message: &r2r::scene_manipulation_msgs::srv::ExtraFeatures::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> ExtraFeatures::Response {
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
    let mut local_broadcasted_frames_clone = local_broadcasted_frames.clone();
    match local_broadcasted_frames.get(&message.child_frame_id) {
        Some(frame) => {
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
                }
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

async fn enable_or_disable_path(
    message: &r2r::scene_manipulation_msgs::srv::ExtraFeatures::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    enable: bool
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
                let mut transforms_to_publish = vec!();

                for frame in frames_local {
                    match request.message.parent_frame_id.as_str() {
                        "" => transforms_to_publish.push(
                            TransformStamped {
                                header: Header {
                                    stamp: time_stamp.clone(),
                                    frame_id: frame.1.parent_frame_id.clone(),
                                },
                                child_frame_id: frame.1.child_frame_id.clone(),
                                transform: frame.1.transform.clone(),
                            }
                        ),
                        _ => match lookup_transform(
                            &request.message.parent_frame_id,
                            &frame.1.child_frame_id,
                            &buffered_frames,
                        ).await {
                            Some(transform) => transforms_to_publish.push(
                                TransformStamped {
                                    header: Header {
                                        stamp: time_stamp.clone(),
                                        frame_id: request.message.parent_frame_id.clone(),
                                    },
                                    child_frame_id: frame.1.child_frame_id.clone(),
                                    transform: transform.clone(),
                                }
                            ),
                            None => ()
                        }
                    } 
                };

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

async fn get_all_extras_server(
    mut service: impl Stream<Item = ServiceRequest<GetAllExtra::Service>> + Unpin,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(request) => {
                let frames_local = broadcasted_frames.lock().unwrap().clone();
                let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
                let now = clock.get_now().unwrap();
                let time_stamp = r2r::Clock::to_builtin_time(&now);
                let mut filtered_correct_type: Vec<FrameData>  = vec!();
                let mut extras_list = vec!();

                frames_local.iter().for_each(|frame| match &frame.1.frame_type {
                    Some(has_type) => match request.message.frame_type.as_str() {
                        "" => filtered_correct_type.push(frame.1.clone()),
                        _ => match has_type == &request.message.frame_type {
                            true => filtered_correct_type.push(frame.1.clone()),
                            false => ()
                        }
                    }
                    None => ()
                });

                for filtered in filtered_correct_type {
                    match lookup_transform(
                        &request.message.parent_frame_id,
                        &filtered.child_frame_id,
                        &buffered_frames,
                    )
                    .await {
                        Some(found) => {
                            extras_list.push(
                                TFExtraData {
                                    transform: TransformStamped {
                                        header: Header {
                                            stamp: time_stamp.clone(),
                                            frame_id: request.message.parent_frame_id.clone()
                                        },
                                        child_frame_id: filtered.child_frame_id.clone(),
                                        transform: found.clone()
                                    },
                                    frame_type: match filtered.frame_type {
                                        Some(frame_type) => frame_type,
                                        None => "".to_string()
                                    },
                                    next: match filtered.next {
                                        Some(next) => next.into_iter().collect(),
                                        None => vec!()
                                    },
                                    zone: match filtered.zone {
                                        Some(zone) => zone,
                                        None => 0.0
                                    }
                                }
                            )
                        },
                        None =>()
                    }
                }                

                let response = GetAllExtra::Response {
                    success: true,
                    info: "".to_string(),
                    extras: TFExtra { data: extras_list } ,
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

fn make_broadcasted_frame_data(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
) -> FrameData {
    // let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
    // let now = clock.get_now().unwrap();
    // let time_stamp = r2r::Clock::to_builtin_time(&now);
    FrameData {
        parent_frame_id: message.parent_frame_id.clone(),
        child_frame_id: message.child_frame_id.clone(),
        transform: message.transform.clone(),
        active: true,
        time_stamp: None, // added just before broadcasting
        zone: None,
        next: None,
        frame_type: None
    }
}

fn add_with_msg_tf(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> ManipulateScene::Response {
    let mut local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
    local_broadcasted_frames.insert(
        message.child_frame_id.clone(),
        make_broadcasted_frame_data(message),
    );
    *broadcasted_frames.lock().unwrap() = local_broadcasted_frames;
    main_success_response(&format!(
        "Frame '{}' added to the scene.",
        message.child_frame_id
    ))
}

fn add_with_lookup_tf(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    transform: Transform,
    time_stamp: Option<Time>,
    zone: Option<f64>,
    next: Option<HashSet<String>>,
    frame_type: Option<String>
) -> ManipulateScene::Response {
    let mut local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
    // let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
    // let now = clock.get_now().unwrap();
    // let time_stamp = r2r::Clock::to_builtin_time(&now);
    local_broadcasted_frames.insert(
        message.child_frame_id.clone(),
        FrameData {
            parent_frame_id: message.parent_frame_id.clone(),
            child_frame_id: message.child_frame_id.clone(),
            transform,
            active: true,
            time_stamp,
            zone,
            next,
            frame_type
        },
    );
    *broadcasted_frames.lock().unwrap() = local_broadcasted_frames;
    main_success_response(&format!(
        "Frame '{}' added to the scene.",
        message.child_frame_id
    ))
}

async fn add_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    match &message.child_frame_id == "world" || &message.child_frame_id == "world_origin" {
        false => match local_buffered_frames.contains_key(&message.child_frame_id) {
            false => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => match local_buffered_frames.contains_key(&message.parent_frame_id) {
                    false => add_with_msg_tf(message, broadcasted_frames), //add warning
                    true => match check_would_produce_cycle(
                        &make_broadcasted_frame_data(&message),
                        &local_buffered_frames,
                    ) {
                        (false, _) => add_with_msg_tf(message, broadcasted_frames),
                        (true, cause) => main_error_response(&format!(
                            "Adding frame '{}' would produce a cycle. Not added: '{}'",
                            &message.child_frame_id, cause
                        )),
                    },
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
                true => match local_broadcasted_frames.remove(&message.child_frame_id) {
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
                false => main_error_response("Can't manipulate_static frames."),
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

    // let saved_transform = match lookup_transform(
    //     &message.parent_frame_id,
    //     &message.child_frame_id,
    //     &buffered_frames,
    // )
    // .await {
    //     Some(t) => t,
    //     None => Transform::default()
    // };

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

async fn move_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    match &message.child_frame_id == "world" || &message.child_frame_id == "world_origin" {
        false => match local_buffered_frames.contains_key(&message.child_frame_id) {
            false => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => main_error_response("Frame doesn't exist."),
                true => {
                    tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
                    match local_buffered_frames.contains_key(&message.child_frame_id) {
                        false => main_error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                        true => add_with_msg_tf(message, broadcasted_frames),
                    }
                }
            }
            true => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => main_error_response("Frame exists in the tf, but it can't be moved since it is not published by this broadcaster."),
                true => add_with_msg_tf(message, broadcasted_frames)
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
        match lookup_transform(
            &message.parent_frame_id,
            "teaching_marker",
            &buffered_frames,
        )
        .await
        {
            None => main_error_response("Failed to lookup transform."),
            Some(transform) => {
                match local_broadcasted_frames.get(&message.child_frame_id.clone()) {
                    Some(frame_data) => add_with_lookup_tf(
                        message,
                        broadcasted_frames,
                        transform,
                        frame_data.time_stamp.clone(),
                        frame_data.zone,
                        frame_data.next.clone(),
                        frame_data.frame_type.clone()
                    ),
                    None => {
                        add_with_lookup_tf(message, broadcasted_frames, transform, None, None, None, None)
                    }
                }
            }
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
        match check_would_produce_cycle(
            &make_broadcasted_frame_data(&ManipulateScene::Request {
                command: "reparent".to_string(),
                child_frame_id: message.child_frame_id.to_string(),
                parent_frame_id: message.parent_frame_id.to_string(),
                new_frame_id: message.new_frame_id.to_string(),
                transform: message.transform.clone(),
            }),
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
                        Some(frame_data) => add_with_lookup_tf(
                            message,
                            broadcasted_frames,
                            transform,
                            frame_data.time_stamp.clone(),
                            frame_data.zone,
                            frame_data.next.clone(),
                            frame_data.frame_type.clone()
                        ),
                        None => add_with_lookup_tf(
                            message,
                            broadcasted_frames,
                            transform,
                            None,
                            None,
                            None,
                            None
                        ),
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
        let new_message = ManipulateScene::Request {
            command: "clone".to_string(),
            child_frame_id: message.new_frame_id.to_string(),
            parent_frame_id: message.parent_frame_id.to_string(),
            new_frame_id: message.new_frame_id.to_string(),
            transform: message.transform.clone(),
        };
        match check_would_produce_cycle(
            &make_broadcasted_frame_data(&new_message),
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
                    match local_broadcasted_frames.get(&new_message.child_frame_id.clone()) {
                        Some(frame_data) => add_with_lookup_tf(
                            &new_message,
                            broadcasted_frames,
                            transform,
                            frame_data.time_stamp.clone(),
                            frame_data.zone,
                            frame_data.next.clone(),
                            frame_data.frame_type.clone()
                        ),
                        None => add_with_lookup_tf(
                            &new_message,
                            broadcasted_frames,
                            transform,
                            None,
                            None,
                            None,
                            None
                        ),
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
    // service_loaded: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let transforms_local = frames.lock().unwrap().clone();
        let mut updated_transforms = vec![];

        transforms_local.iter().for_each(|(_, v)| match v.active {
            false => {
                updated_transforms.push(TransformStamped {
                    header: Header {
                        stamp: time_stamp.clone(),
                        frame_id: v.parent_frame_id.clone(),
                    },
                    child_frame_id: v.child_frame_id.clone(),
                    transform: v.transform.clone(),
                });
            }
            true => (),
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
    // service_loaded: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let transforms_local = frames.lock().unwrap().clone();
        let mut updated_transforms = vec![];

        transforms_local.iter().for_each(|(_, v)| match v.active {
            true => {
                updated_transforms.push(TransformStamped {
                    header: Header {
                        stamp: time_stamp.clone(),
                        frame_id: v.parent_frame_id.clone(),
                    },
                    child_frame_id: v.child_frame_id.clone(),
                    transform: v.transform.clone(),
                });
            }
            false => (),
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
                            active: true,
                            time_stamp: Some(t.header.stamp.clone()),
                            zone: None,
                            next: None,
                            frame_type: None
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
                            active: true,
                            time_stamp: Some(t.header.stamp.clone()),
                            zone: None,
                            next: None,
                            frame_type: None
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

// updates the broadcaster buffer with frames from the tf extra topic
async fn extra_tf_listener_callback(
    mut subscriber: impl Stream<Item = TFExtra> + Unpin,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let mut frames_local = broadcasted_frames.lock().unwrap().clone();
                message.data.iter().for_each(|t| {
                    match frames_local.get(&t.transform.child_frame_id.clone()) {
                        Some(exists) => {
                            frames_local.insert(
                                t.transform.child_frame_id.clone(),
                                FrameData {
                                    parent_frame_id: t.transform.header.frame_id.clone(),
                                    child_frame_id: t.transform.child_frame_id.clone(),
                                    transform: t.transform.transform.clone(),
                                    active: true,
                                    time_stamp: Some(t.transform.header.stamp.clone()),
                                    zone: exists.zone.clone(),
                                    next: exists.next.clone(),
                                    frame_type: Some(t.frame_type.clone())
                                },
                            );
                        }, 
                        None => {
                            frames_local.insert(
                                t.transform.child_frame_id.clone(),
                                FrameData {
                                    parent_frame_id: t.transform.header.frame_id.clone(),
                                    child_frame_id: t.transform.child_frame_id.clone(),
                                    transform: t.transform.transform.clone(),
                                    active: true,
                                    time_stamp: Some(t.transform.header.stamp.clone()),
                                    zone: None,
                                    next: None,
                                    frame_type: Some(t.frame_type.clone())
                                },
                            );
                        }, 
                    }
                    
                });
                *broadcasted_frames.lock().unwrap() = frames_local;
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
        frames_local.iter().for_each(|(k, v)| match v.active {
            true => match &v.time_stamp {
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
            false => (), // maybe we should remove static frames older than STATIC_FRAME_LIFETIME
        });
        *buffered_frames.lock().unwrap() = frames_local_reduced;
        timer.tick().await?;
    }
}

// easier to manipulate transforms in glam's affine format
fn tf_to_affine(t: &Transform) -> DAffine3 {
    DAffine3::from_rotation_translation(
        DQuat::from_xyzw(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
        DVec3::new(t.translation.x, t.translation.y, t.translation.z),
    )
}

// back to ros's Transform type when needed
fn affine_to_tf(a: &DAffine3) -> Transform {
    match a.to_scale_rotation_translation() {
        (_, r, t) => Transform {
            translation: Vector3 {
                x: t.x,
                y: t.y,
                z: t.z,
            },
            rotation: Quaternion {
                x: r.x,
                y: r.y,
                z: r.z,
                w: r.w,
            },
        },
    }
}

async fn lookup_transform(
    parent_frame_id: &str,
    child_frame_id: &str,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Option<Transform> {
    let frames_local = buffered_frames.lock().unwrap().clone();
    let mut chain = vec![];
    match is_cyclic_all(&frames_local) {
        (false, _) => match parent_to_world(parent_frame_id, &frames_local) {
            Some(up_chain) => match world_to_child(child_frame_id, &frames_local) {
                Some(down_chain) => {
                    chain.extend(up_chain);
                    chain.extend(down_chain);
                    Some(affine_to_tf(&chain.iter().product::<DAffine3>()))
                }
                None => None,
            },
            None => None,
        },
        (true, _cause) => None,
    }
}

// when looking up a frame in a parent, first we have to find a common parent.
// we know that the world is always there so we go up until the world frame.
fn parent_to_world(
    parent_frame_id: &str,
    frames: &HashMap<String, FrameData>,
) -> Option<Vec<DAffine3>> {
    let mut current_parent = parent_frame_id.to_string();
    let mut path = vec![];
    let mut lenght = 0;
    loop {
        match lenght > MAX_TRANSFORM_CHAIN {
            false => {
                lenght = lenght + 1;
                match frames.get(&current_parent.clone()) {
                    Some(parent) => {
                        let a = tf_to_affine(&parent.transform);
                        path.push(a.inverse());
                        let p = parent.parent_frame_id.clone();
                        match p == "world_origin".to_string() {
                            true => break Some(path),
                            false => {
                                let p = parent.parent_frame_id.clone();
                                current_parent = p;
                                continue;
                            }
                        }
                    }
                    None => break None, // log err
                }
            }
            true => {
                r2r::log_error!(
                    NODE_ID,
                    "TF lookup failed with: 'MAX_TRANSFORM_CHAIN reached'."
                );
                break None;
            }
        }
    }
}

// basically BFS to get the path to the child...
// even simpler since we check for cycles beforehand and its a tree, so not tracking visited...
// TODO: add max depth check though just in case.
fn world_to_child(
    child_frame_id: &str,
    frames: &HashMap<String, FrameData>,
) -> Option<Vec<DAffine3>> {
    let mut stack = vec![];
    get_frame_children_3("world_origin", frames)
        .iter()
        .for_each(|(k, v)| {
            stack.push((
                k.to_string(),
                vec![k.to_string()],
                vec![tf_to_affine(&v.transform)],
            ))
        });
    loop {
        // r2r::log_warn!(
        //     NODE_ID,
        //     "WORLD TO CHILD LOOP."
        // );
        // println!(
        //     "WORLD TO CHILD: {:?}",
        //     stack.iter().map(|x| x.0.clone()).collect::<Vec<String>>()
        // );
        match stack.pop() {
            Some((frame, path, chain)) => match frame == child_frame_id {
                true => {
                    // println!("WORLD TO CHILD PATH: {:?}", path);
                    break Some(chain);
                }
                false => {
                    get_frame_children_3(&frame, frames)
                        .iter()
                        .for_each(|(k, v)| {
                            let mut prev_path = path.clone();
                            let mut prev_chain = chain.clone();
                            prev_path.push(k.clone());
                            prev_chain.push(tf_to_affine(&v.transform));
                            stack.insert(0, (k.to_string(), prev_path.clone(), prev_chain.clone()));
                        });

                    continue;
                }
            },
            None => break None,
        };
    }
}

// get all children frames of a frame
fn get_frame_children(frame: &str, frames: &HashMap<String, FrameData>) -> Vec<String> {
    frames
        .iter()
        .filter(|(_, v)| frame == v.parent_frame_id)
        .map(|(k, _)| k.clone())
        .collect()
}

// get all children frames of a frame
fn get_frame_children_3(
    frame: &str,
    frames: &HashMap<String, FrameData>,
) -> Vec<(String, FrameData)> {
    frames
        .iter()
        .filter(|(_, v)| frame == v.parent_frame_id)
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect()
}

// TODO: should return option. should check also max depth
// check for cycles in the tree segment starting from this frame
fn is_cyclic(frame: &str, frames: &HashMap<String, FrameData>) -> (bool, String) {
    let mut stack = vec![];
    let mut visited = vec![];
    stack.push(frame.to_string());
    loop {
        // r2r::log_warn!(
        //     NODE_ID,
        //     "STACK = {:?}", stack
        // );
        // r2r::log_warn!(
        //     NODE_ID,
        //     "VISITED = {:?}", visited
        // );
        match stack.pop() {
            Some(current_frame) => match visited.contains(&current_frame) {
                true => break (true, current_frame),
                false => {
                    visited.push(current_frame.clone());
                    for child in get_frame_children(&current_frame, frames) {
                        stack.push(child);
                        // continue;
                    }
                }
            },
            None => break (false, "".to_string()),
        }
    }
}

// check for all cycles including all frames even if tree is segmented
fn is_cyclic_all(frames: &HashMap<String, FrameData>) -> (bool, String) {
    for (k, _) in frames {
        let (cyclic, cause) = is_cyclic(k, frames);
        match cyclic {
            true => return (cyclic, cause),
            false => continue,
        }
    }
    (false, "".to_string())
}

// check if adding the frame to the tree would produce a cycle
fn check_would_produce_cycle(
    frame: &FrameData,
    frames: &HashMap<String, FrameData>,
) -> (bool, String) {
    let mut frames_local = frames.clone();
    frames_local.insert(frame.child_frame_id.clone(), frame.clone());
    is_cyclic_all(&frames_local)
}

async fn marker_publisher_callback(
    zone_publisher: r2r::Publisher<MarkerArray>,
    path_publisher: r2r::Publisher<MarkerArray>,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    mut timer: r2r::Timer,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut zone_markers: Vec<Marker> = vec![];
        let mut path_markers: Vec<Marker> = vec![];
        let frames_local = broadcasted_frames.lock().unwrap().clone();
        let mut id = 0;
        for frame in frames_local {
            match frame.1.zone {
                Some(z) => {
                    id = id + 1;
                    let indiv_marker = Marker {
                        header: Header {
                            stamp: r2r::builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
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
                        match lookup_transform(&frame.1.child_frame_id, &enabled, &buffered_frames)
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

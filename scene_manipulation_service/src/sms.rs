use futures::{stream::Stream, StreamExt};
use glam::{DAffine3, DQuat, DVec3};
use r2r::builtin_interfaces::msg::{Time, Duration};
use r2r::geometry_msgs::msg::{Point, Pose, Quaternion, Transform, TransformStamped, Vector3};
use r2r::scene_manipulation_msgs::srv::{
    GetAllTransforms, LookupTransform, ManipulateScene, ReloadScenario,
};
use r2r::std_msgs::msg::{ColorRGBA, Header};
use r2r::tf2_msgs::msg::TFMessage;
use r2r::visualization_msgs::msg::{Marker, MarkerArray};
use r2r::{ParameterValue, QosProfile, ServiceRequest};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::error::Error;
use std::fmt;
use std::fs::{self, File};
use std::io::BufReader;
use std::sync::{Arc, Mutex};

pub static NODE_ID: &'static str = "scene_manipulation_service";
// pub static FOLDER_RELOAD_SCENE_RATE: u64 = 3000;
pub static STATIC_BROADCAST_RATE: u64 = 1000;
pub static ACTIVE_BROADCAST_RATE: u64 = 100;
pub static BUFFER_MAINTAIN_RATE: u64 = 100;
pub static ACTIVE_FRAME_LIFETIME: i32 = 3; //seconds
pub static STATIC_FRAME_LIFETIME: i32 = 10; //seconds
pub static MAX_TRANSFORM_CHAIN: u64 = 100;

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub struct FrameData {
    pub parent_frame_id: String,
    pub child_frame_id: String,
    pub transform: Transform,
    pub active: bool,
}

// Hack to include the zone into the broadcasted but not buffered shared state
#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub struct ZonedFrameData {
    pub parent_frame_id: String,
    pub child_frame_id: String,
    pub transform: Transform,
    pub active: bool,
    pub zone: Option<f64>,
}

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub struct BufferedFrameData {
    pub frame_data: FrameData,
    pub time_stamp: Time,
}

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub struct BroadcastedFrameData {
    pub frame_data: FrameData,
    pub time_stamp: Time,
    pub zone: Option<f64>,
}

// the testing module
mod tests;

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

    // let path = Arc::new(Mutex::new(path_param.clone()));
    let scenario_res = list_frames_in_dir(&path_param, true).await;

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
            HashMap::<String, BroadcastedFrameData>::new()
        }
    };

    // frames that are published by this broadcaster
    let broadcasted_frames = Arc::new(Mutex::new(init_loaded));

    // frames that exist on the tf and tf_static topic, i.e. this is a local tf buffer
    let buffered_frames = Arc::new(Mutex::new(HashMap::<String, BufferedFrameData>::new()));

    // listen to the active frames on the tf topic to see if a frame exists before broadcasting it
    let active_tf_listener =
        node.subscribe::<TFMessage>("tf", QosProfile::best_effort(QosProfile::default()))?;

    // listen to the active frames on the static tf topic to see if a frame exists before broadcasting it
    let static_tf_listener = node.subscribe::<TFMessage>(
        "tf_static",
        QosProfile::transient_local(QosProfile::default()),
    )?;

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

    // offer the scenario loading/reloading service
    let load_scenario_service =
        node.create_service::<ReloadScenario::Service>("reload_scenario")?;

    // offer a service to get all frames from tf (local buffer)
    let get_all_transforms_service =
        node.create_service::<GetAllTransforms::Service>("get_all_transforms")?;

    // // occasionally look into the folder specified by the path to see if there are changes
    // let reload_timer =
    //     node.create_wall_timer(std::time::Duration::from_millis(FOLDER_RELOAD_SCENE_RATE))?;

    let zone_marker_publisher = node
        .create_publisher::<r2r::visualization_msgs::msg::MarkerArray>(
            "zone_markers",
            QosProfile::default(),
        )?;

    let zone_marker_timer = node.create_wall_timer(std::time::Duration::from_millis(50))?;
    let broadcasted_frames_clone_16 = broadcasted_frames.clone();
    tokio::task::spawn(async move {
        match zone_marker_publisher_callback(
            zone_marker_publisher,
            &broadcasted_frames_clone_16,
            zone_marker_timer,
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

    // // spawn a tokio task to handle reloading the scene when a new frame is to be added manually
    // // actually, the folder callback should always read the folder and not only when there is a change
    // let broadcasted_frames_clone_3 = broadcasted_frames.clone();
    // let path_clone_1 = path.clone();
    // tokio::task::spawn(async move {
    //     match folder_manupulation_callback(reload_timer, &broadcasted_frames_clone_3, &path_clone_1)
    //         .await
    //     {
    //         Ok(()) => (),
    //         Err(e) => r2r::log_error!(NODE_ID, "Active frame broadcaster failed with: '{}'.", e),
    //     };
    // });

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
        let result = load_scenario_server(load_scenario_service, &broadcasted_frames_clone_5).await;
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

    // frequency of how often to refresh (add or remove) frames to the local buffer
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
    // let buffered_frames_clone_4 = buffered_frames.clone();
    tokio::task::spawn(async move {
        let result =
            get_all_transforms_server(get_all_transforms_service, &buffered_frames.clone()).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Get All Frames Service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Get All Frames Service call failed with: {}.", e),
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

async fn list_frames_in_dir(
    path: &str,
    launch: bool,
) -> Result<Vec<String>, Box<dyn std::error::Error + Send>> {
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

async fn load_scenario(scenario: &Vec<String>) -> HashMap<String, BroadcastedFrameData> {
    let mut frame_datas: HashMap<String, BroadcastedFrameData> = HashMap::new();
    scenario.iter().for_each(|x| match File::open(x) {
        Ok(file) => {
            let reader = BufReader::new(file);
            match serde_json::from_reader(reader) {
                Ok(jsonl) => {
                    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
                    let now = clock.get_now().unwrap();
                    let time_stamp = r2r::Clock::to_builtin_time(&now);
                    let zoned: ZonedFrameData = jsonl;
                    let json = BroadcastedFrameData {
                        frame_data: FrameData {
                            parent_frame_id: zoned.parent_frame_id.clone(),
                            child_frame_id: zoned.child_frame_id.clone(),
                            transform: zoned.transform.clone(),
                            active: zoned.active.clone(),
                        },
                        time_stamp: time_stamp.clone(),
                        zone: match zoned.zone {
                            Some(z) => Some(z),
                            None => None,
                        }
                        .clone(), // 0.0 // jsonl.zone.clone(), // should check if zone exists
                    };
                    frame_datas.insert(json.frame_data.child_frame_id.clone(), json.clone());
                    frame_datas.insert(
                        "world".to_string(),
                        BroadcastedFrameData {
                            frame_data: FrameData {
                                parent_frame_id: "world_origin".to_string(), //"origin",
                                child_frame_id: "world".to_string(),
                                transform: Transform {
                                    translation: Vector3 {
                                        x: 0.0,
                                        y: 0.0,
                                        z: 0.0,
                                    },
                                    rotation: Quaternion {
                                        x: 0.0,
                                        y: 0.0,
                                        z: 0.0,
                                        w: 1.0,
                                    },
                                },
                                active: true,
                            },
                            time_stamp,
                            zone: None,
                        },
                    );
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

// maybe add a loop check before adding frames
async fn load_scenario_server(
    mut service: impl Stream<Item = ServiceRequest<ReloadScenario::Service>> + Unpin,
    broadcasted_frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(request) => {
                let scenario_res = list_frames_in_dir(&request.message.scenario_path, true).await;
                match scenario_res {
                    Ok(scenario) => {
                        let loaded = load_scenario(&scenario).await;
                        r2r::log_info!(
                            NODE_ID,
                            "Reloaded frames in the scene: '{:?}'.",
                            loaded.keys()
                        );
                        let mut local_broadcasted_frames =
                            broadcasted_frames.lock().unwrap().clone();
                        for x in &loaded {
                            local_broadcasted_frames.insert(x.0.clone(), x.1.clone());
                        }
                        *broadcasted_frames.lock().unwrap() = local_broadcasted_frames;
                        request
                            .respond(ReloadScenario::Response {
                                success: true,
                                info: format!(
                                    "Reloaded frames in the scene: '{:?}'.",
                                    loaded.keys()
                                )
                                .to_string(),
                            })
                            .expect("Could not send service response.");
                        continue;
                    }
                    Err(e) => {
                        request
                            .respond(ReloadScenario::Response {
                                success: false,
                                info: format!("Reloading the scenario failed with: '{:?}'.", e)
                                    .to_string(),
                            })
                            .expect("Could not send service response.");
                        continue;
                    }
                }
            }
            None => (),
        }
    }
}

async fn scene_manipulation_server(
    mut service: impl Stream<Item = ServiceRequest<ManipulateScene::Service>> + Unpin,
    broadcasted_frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, BufferedFrameData>>>,
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
                _ => {
                    r2r::log_error!(NODE_ID, "No such command.");
                    continue;
                }
            },
            None => {}
        }
    }
}

async fn transform_lookup_server(
    mut service: impl Stream<Item = ServiceRequest<LookupTransform::Service>> + Unpin,
    buffered_frames: &Arc<Mutex<HashMap<String, BufferedFrameData>>>,
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
    buffered_frames: &Arc<Mutex<HashMap<String, BufferedFrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(request) => {
                let frames_local = buffered_frames.lock().unwrap().clone();
                let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
                let now = clock.get_now().unwrap();
                let time_stamp = r2r::Clock::to_builtin_time(&now);
                let response = GetAllTransforms::Response {
                    success: true,
                    info: "".to_string(),
                    names: frames_local.iter().map(|x| x.0.to_string()).collect(),
                    transforms: frames_local
                        .iter()
                        .map(|x| TransformStamped {
                            header: Header {
                                stamp: time_stamp.clone(),
                                frame_id: x.1.frame_data.parent_frame_id.clone(),
                            },
                            child_frame_id: x.1.frame_data.child_frame_id.clone(),
                            transform: x.1.frame_data.transform.clone(),
                        })
                        .collect(),
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

fn error_response(msg: &str) -> ManipulateScene::Response {
    let info = msg.to_string();
    r2r::log_error!(NODE_ID, "{}", info);
    ManipulateScene::Response {
        success: false,
        info,
    }
}

fn success_response(msg: &str) -> ManipulateScene::Response {
    let info = msg.to_string();
    // r2r::log_info!(NODE_ID, "{}", info);
    ManipulateScene::Response {
        success: true,
        info,
    }
}

fn make_broadcasted_frame_data(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
) -> BroadcastedFrameData {
    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
    let now = clock.get_now().unwrap();
    let time_stamp = r2r::Clock::to_builtin_time(&now);
    BroadcastedFrameData {
        frame_data: FrameData {
            parent_frame_id: message.parent_frame_id.clone(),
            child_frame_id: message.child_frame_id.clone(),
            transform: message.transform.clone(),
            active: true,
        },
        time_stamp,
        zone: Some(message.zone.clone()),
    }
}

fn add_with_msg_tf(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
) -> ManipulateScene::Response {
    let mut local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
    local_broadcasted_frames.insert(
        message.child_frame_id.clone(),
        make_broadcasted_frame_data(message),
    );
    *broadcasted_frames.lock().unwrap() = local_broadcasted_frames;
    success_response(&format!(
        "Frame '{}' added to the scene.",
        message.child_frame_id
    ))
}

fn add_with_lookup_tf(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
    transform: Transform,
) -> ManipulateScene::Response {
    let mut local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
    let now = clock.get_now().unwrap();
    let time_stamp = r2r::Clock::to_builtin_time(&now);
    local_broadcasted_frames.insert(
        message.child_frame_id.clone(),
        BroadcastedFrameData {
            frame_data: FrameData {
                parent_frame_id: message.parent_frame_id.clone(),
                child_frame_id: message.child_frame_id.clone(),
                transform,
                active: true,
            },
            time_stamp,
            zone: Some(message.zone.clone()),
        },
    );
    *broadcasted_frames.lock().unwrap() = local_broadcasted_frames;
    success_response(&format!(
        "Frame '{}' added to the scene.",
        message.child_frame_id
    ))
}

async fn add_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, BufferedFrameData>>>,
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
                        (true, cause) => error_response(&format!(
                            "Adding frame '{}' would produce a cycle. Not added: '{}'",
                            &message.child_frame_id, cause
                        )),
                    },
                },
                true => {
                    tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
                    match local_buffered_frames.contains_key(&message.child_frame_id) {
                        false => error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                        true => error_response("Frame already exists."),
                    }
                }
            },
            true => error_response("Frame already exists."),
        },
        true => error_response("Frame 'world' is reserved as the universal tree root."),
    }
}

async fn remove_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, BufferedFrameData>>>,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    fn inner(
        message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
        broadcasted_frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
    ) -> ManipulateScene::Response {
        let mut local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();
        match local_broadcasted_frames.get(&message.child_frame_id) {
            Some(frame) => match frame.frame_data.active {
                true => match local_broadcasted_frames.remove(&message.child_frame_id) {
                    Some(_) => {
                        *broadcasted_frames.lock().unwrap() = local_broadcasted_frames;
                        success_response(&format!(
                            "Frame '{}' removed from the scene.",
                            message.child_frame_id
                        ))
                    }
                    None => error_response(&format!(
                        "Failed to remove frame '{}' from the scene.",
                        message.child_frame_id
                    )),
                },
                false => error_response("Can't manipulate_static frames."),
            },
            None => error_response("Failed to get frame data from broadcaster hashmap."),
        }
    }

    match &message.child_frame_id == "world" || &message.child_frame_id == "world_origin" {
        false => match local_buffered_frames.contains_key(&message.child_frame_id) {
            false => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => error_response("Frame doesn't exist in tf, nor is it published by this broadcaster."),
                true => {
                    tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
                    match local_buffered_frames.contains_key(&message.child_frame_id) {
                        false => error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                        true => inner(message, broadcasted_frames)
                    }
                }
            },
            true => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => error_response("Frame exists in the tf, but it can't be removed since it is not published by sms."),
                true => inner(message, broadcasted_frames)
            }
        },
        true => error_response("Frame 'world' is reserved as the universal tree root."),
    }
}

async fn rename_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, BufferedFrameData>>>,
) -> ManipulateScene::Response {
    // let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    let remove_response = remove_frame(
        &ManipulateScene::Request {
            command: "remove".to_string(),
            child_frame_id: message.child_frame_id.to_string(),
            parent_frame_id: message.parent_frame_id.to_string(),
            new_frame_id: message.new_frame_id.to_string(),
            transform: message.transform.clone(),
            zone: message.zone.clone(),
        },
        &broadcasted_frames,
        &buffered_frames,
    )
    .await;

    match remove_response.success {
        true => match local_broadcasted_frames.get(&message.child_frame_id) {
            None => error_response(&format!(
                "Failed to fetch frame '{}' from the broadcasted hashmap.",
                message.child_frame_id.to_string(),
            )),
            Some(frame) => {
                let add_response = add_frame(
                    &ManipulateScene::Request {
                        command: "add".to_string(),
                        child_frame_id: message.new_frame_id.to_string(),
                        parent_frame_id: frame.frame_data.parent_frame_id.to_string(),
                        new_frame_id: message.new_frame_id.to_string(),
                        transform: message.transform.clone(),
                        zone: message.zone.clone(),
                    },
                    &broadcasted_frames,
                    &buffered_frames,
                )
                .await;
                match add_response.success {
                    false => add_response,
                    true => success_response(&format!(
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
    broadcasted_frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, BufferedFrameData>>>,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    match &message.child_frame_id == "world" || &message.child_frame_id == "world_origin" {
        false => match local_buffered_frames.contains_key(&message.child_frame_id) {
            false => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => error_response("Frame doesn't exist."),
                true => {
                    tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
                    match local_buffered_frames.contains_key(&message.child_frame_id) {
                        false => error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                        true => add_with_msg_tf(message, broadcasted_frames),
                    }
                }
            }
            true => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => error_response("Frame exists in the tf, but it can't be moved since it is not published by this broadcaster."),
                true => add_with_msg_tf(message, broadcasted_frames)
            }
        },
        true => error_response("Frame 'world' is reserved as the universal tree root."),        
    }
}

async fn reparent_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, BufferedFrameData>>>,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    async fn inner(
        message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
        broadcasted_frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
        buffered_frames: &Arc<Mutex<HashMap<String, BufferedFrameData>>>,
    ) -> ManipulateScene::Response {
        let local_buffered_frames = buffered_frames.lock().unwrap().clone();
        match check_would_produce_cycle(
            &make_broadcasted_frame_data(&ManipulateScene::Request {
                command: "reparent".to_string(),
                child_frame_id: message.child_frame_id.to_string(),
                parent_frame_id: message.parent_frame_id.to_string(),
                new_frame_id: message.new_frame_id.to_string(),
                transform: message.transform.clone(),
                zone: message.zone.clone(),
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
                None => error_response("Frailed to lookup transform."),
                Some(transform) => add_with_lookup_tf(message, broadcasted_frames, transform),
            },
            (true, cause) => error_response(&format!(
                "Adding frame '{}' would produce a cycle. Not added, cause: '{}'",
                &message.child_frame_id, cause
            )),
        }
    }

    match &message.child_frame_id == "world" || &message.child_frame_id == "world_origin" {
        false => match local_buffered_frames.contains_key(&message.child_frame_id) {
            false => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => error_response("Frame doesn't exist."),
                true => {
                    tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
                    match local_buffered_frames.contains_key(&message.child_frame_id) {
                        false => error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                        true => inner(&message, &broadcasted_frames, &buffered_frames).await
                    }
                }
            }
            true => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
                false => error_response("Frame exists in the tf, but it can't be moved since it is not published by this broadcaster."),
                true => inner(&message, &broadcasted_frames, &buffered_frames).await
            }
        },
        true => error_response("Frame 'world' is reserved as the universal tree root."),        
    }
}

async fn clone_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, BufferedFrameData>>>,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let local_broadcasted_frames = broadcasted_frames.lock().unwrap().clone();

    async fn inner(
        message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
        broadcasted_frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
        buffered_frames: &Arc<Mutex<HashMap<String, BufferedFrameData>>>,
    ) -> ManipulateScene::Response {
        let local_buffered_frames = buffered_frames.lock().unwrap().clone();
        let new_message = ManipulateScene::Request {
            command: "clone".to_string(),
            child_frame_id: message.new_frame_id.to_string(),
            parent_frame_id: message.parent_frame_id.to_string(),
            new_frame_id: message.new_frame_id.to_string(),
            transform: message.transform.clone(),
            zone: message.zone.clone(),
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
                None => error_response("Frailed to lookup transform."),
                Some(transform) => add_with_lookup_tf(&new_message, broadcasted_frames, transform),
            },
            (true, cause) => error_response(&format!(
                "Adding frame '{}' would produce a cycle. Not added, cause: '{}'",
                &message.child_frame_id, cause
            )),
        }
    }

    match local_buffered_frames.contains_key(&message.child_frame_id) {
        false => match local_broadcasted_frames.contains_key(&message.child_frame_id) {
            false => error_response("Frame doesn't exist."),
            true => {
                tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
                match local_buffered_frames.contains_key(&message.child_frame_id) {
                    false => error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                    true => match local_buffered_frames.contains_key(&message.parent_frame_id) {
                        false => error_response("Parent doesn't exist."),
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
    frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
    // service_loaded: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let transforms_local = frames.lock().unwrap().clone();
        let mut updated_transforms = vec![];

        transforms_local
            .iter()
            .for_each(|(_, v)| match v.frame_data.active {
                false => {
                    updated_transforms.push(TransformStamped {
                        header: Header {
                            stamp: time_stamp.clone(),
                            frame_id: v.frame_data.parent_frame_id.clone(),
                        },
                        child_frame_id: v.frame_data.child_frame_id.clone(),
                        transform: v.frame_data.transform.clone(),
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
    frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
    // service_loaded: &Arc<Mutex<HashMap<String, FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let transforms_local = frames.lock().unwrap().clone();
        let mut updated_transforms = vec![];

        transforms_local
            .iter()
            .for_each(|(_, v)| match v.frame_data.active {
                true => {
                    updated_transforms.push(TransformStamped {
                        header: Header {
                            stamp: time_stamp.clone(),
                            frame_id: v.frame_data.parent_frame_id.clone(),
                        },
                        child_frame_id: v.frame_data.child_frame_id.clone(),
                        transform: v.frame_data.transform.clone(),
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

// BETTER TO HAVE RELOAD SCENE SERVICE
// // occasionally look at the scenario folder specified by the path and if there
// // have been changes, add or remove the the frames that have been manually added
// // to the folder or removed from the folder
// async fn folder_manupulation_callback(
//     mut timer: r2r::Timer,
//     frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
//     path: &Arc<Mutex<String>>,
// ) -> Result<(), Box<dyn std::error::Error>> {
//     loop {
//         // let mut to_be_added = vec![];
//         let path = path.lock().unwrap().clone();
//         let dir_frames = list_frames_in_dir(&path, false).await.clone();

//         // differentiate between frames loaded from folder and through services
//         // for example, if we add a frame through the service call, it will be
//         // removed every RELOAD_SCENE_RATE since it is not in the folder
//         let mut folder_scenario = load_scenario(&dir_frames).await;
//         folder_scenario.retain(|_, v| v.folder_loaded);
//         let mut frames_local = frames.lock().unwrap().clone();
//         frames_local.retain(|_, v| v.folder_loaded);

//         let folder_vec = folder_scenario
//             .iter()
//             .map(|(k, _)| k.clone())
//             .collect::<Vec<String>>();
//         let loaded_vec = frames_local
//             .iter()
//             .map(|(k, _)| k.clone())
//             .collect::<Vec<String>>();

//         let folder_set: HashSet<String> =
//             HashSet::from_iter(folder_vec.clone().iter().map(|x| x.clone()));
//         let local_set: HashSet<String> =
//             HashSet::from_iter(loaded_vec.clone().iter().map(|x| x.clone()));

//         let to_be_added_names: Vec<String> = folder_set
//             .difference(&local_set)
//             .map(|x| x.clone())
//             .collect();
//         let to_be_removed_names: Vec<String> = local_set
//             .difference(&folder_set)
//             .map(|x| x.clone())
//             .collect();

//         let mut to_be_added = HashMap::<String, ExtendedFrameData>::new();
//         let mut to_be_removed = HashMap::<String, ExtendedFrameData>::new();
//         let mut transforms_local = frames.lock().unwrap().clone();

//         to_be_added_names.iter().for_each(|name| {
//             match folder_scenario.iter().find(|(json_k, _)| *json_k == name) {
//                 Some((data_k, data_v)) => {
//                     to_be_added.insert(data_k.to_string(), data_v.clone());
//                 }
//                 None => (),
//             }
//         });

//         to_be_removed_names.iter().for_each(|name| {
//             match frames_local.iter().find(|(json_k, _)| *json_k == name) {
//                 Some((data_k, data_v)) => {
//                     to_be_removed.insert(data_k.to_string(), data_v.clone());
//                 }
//                 None => (),
//             }
//         });

//         // TODO: check if parent exists in the world
//         // TODO: check if this addition will produce a cycle
//         // TODO: check if it is folder loaded or if that has been changed in the meantime

//         to_be_added
//             .iter()
//             .for_each(|(k, v)| match frames_local.get(k) {
//                 Some(frame_data) => match frame_data.frame_data.active {
//                     false => {
//                         let info = format!("Can't manipulate static frame '{}'.", k);
//                         r2r::log_error!(NODE_ID, "{}", info);
//                     }
//                     true => {
//                         transforms_local.insert(k.clone(), v.clone());
//                         r2r::log_info!(NODE_ID, "Updated active frame: '{}'.", k);
//                     }
//                 },
//                 None => {
//                     transforms_local.insert(k.clone(), v.clone());
//                     r2r::log_info!(NODE_ID, "Added new frame: '{}'.", k);
//                 }
//             });

//         to_be_removed
//             .iter()
//             .for_each(|(k, _)| match frames_local.get(k) {
//                 Some(frame_data) => match frame_data.frame_data.active {
//                     false => {
//                         let info = format!("Can't manipulate static frame '{}'.", k);
//                         r2r::log_error!(NODE_ID, "{}", info);
//                     }
//                     true => {
//                         transforms_local.remove(k);
//                         r2r::log_info!(NODE_ID, "Removed active frame: '{}'.", k);
//                     }
//                 },
//                 None => {
//                     let info = format!(
//                         "Couldn't find the frame '{}' in this broadcaster. None removed.",
//                         k
//                     );
//                     r2r::log_warn!(NODE_ID, "{}", info);
//                 }
//             });

//         *frames.lock().unwrap() = transforms_local;

//         timer.tick().await?;
//     }
// }

// update the buffer with active frames from the tf topic
async fn active_tf_listener_callback(
    mut subscriber: impl Stream<Item = TFMessage> + Unpin,
    buffered_frames: &Arc<Mutex<HashMap<String, BufferedFrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let mut frames_local = buffered_frames.lock().unwrap().clone();
                message.transforms.iter().for_each(|t| {
                    // before adding an active frame to the buffer, check if the frame is
                    // already stale as defined with ACTIVE_FRAME_LIFETIME
                    // let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
                    // let now = clock.get_now().unwrap();
                    // let current_time = r2r::Clock::to_builtin_time(&now);

                    // match current_time.sec > t.header.stamp.sec + ACTIVE_FRAME_LIFETIME {
                    //     false => {
                    frames_local.insert(
                        t.child_frame_id.clone(),
                        BufferedFrameData {
                            frame_data: FrameData {
                                parent_frame_id: t.header.frame_id.clone(),
                                child_frame_id: t.child_frame_id.clone(),
                                transform: t.transform.clone(),
                                active: true,
                            },
                            time_stamp: t.header.stamp.clone(),
                        },
                    );
                    // }
                    // true => (),
                    // }
                });
                *buffered_frames.lock().unwrap() = frames_local;
            }
            None => {
                r2r::log_error!(NODE_ID, "Subscriber did not get the message?");
            }
        }
    }
}

// update the buffer with static frames from the tf_static topic
async fn static_tf_listener_callback(
    mut subscriber: impl Stream<Item = TFMessage> + Unpin,
    buffered_frames: &Arc<Mutex<HashMap<String, BufferedFrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let mut frames_local = buffered_frames.lock().unwrap().clone();
                message.transforms.iter().for_each(|t| {
                    // static frames are always true, so we don't need to check their timestamp
                    frames_local.insert(
                        t.child_frame_id.clone(),
                        BufferedFrameData {
                            frame_data: FrameData {
                                parent_frame_id: t.header.frame_id.clone(),
                                child_frame_id: t.child_frame_id.clone(),
                                transform: t.transform.clone(),
                                active: false,
                            },
                            time_stamp: t.header.stamp.clone(),
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
// also, if a stale active frame is on the tf for some reason, don't include it
async fn maintain_buffer(
    mut timer: r2r::Timer,
    buffered_frames: &Arc<Mutex<HashMap<String, BufferedFrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let frames_local = buffered_frames.lock().unwrap().clone();
        let mut frames_local_reduced = frames_local.clone();
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let current_time = r2r::Clock::to_builtin_time(&now);
        frames_local
            .iter()
            .for_each(|(k, v)| match v.frame_data.active {
                true => match current_time.sec > v.time_stamp.sec + ACTIVE_FRAME_LIFETIME {
                    true => {
                        frames_local_reduced.remove(k);
                    }
                    false => (),
                },
                false => (), // do similar also for static frames
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
    buffered_frames: &Arc<Mutex<HashMap<String, BufferedFrameData>>>,
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
    frames: &HashMap<String, BufferedFrameData>,
) -> Option<Vec<DAffine3>> {
    let mut current_parent = parent_frame_id.to_string();
    let mut path = vec![];
    let mut lenght = 0;
    loop {
        match lenght > MAX_TRANSFORM_CHAIN {
            false => {
                lenght = lenght + 1;
                // let current_parent_2 = current_parent.clone();
                match frames.get(&current_parent.clone()) {
                    Some(parent) => {
                        // println!("PARENT TO WORLD: {:?}", parent.frame_data.child_frame_id);
                        let a = tf_to_affine(&parent.frame_data.transform);
                        path.push(a.inverse());
                        let p = parent.frame_data.parent_frame_id.clone();
                        match p == "world_origin".to_string() {
                            true => break Some(path),
                            false => {
                                let p = parent.frame_data.parent_frame_id.clone();
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
// add max depth though just in case.
fn world_to_child(
    child_frame_id: &str,
    frames: &HashMap<String, BufferedFrameData>,
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
fn get_frame_children(frame: &str, frames: &HashMap<String, BufferedFrameData>) -> Vec<String> {
    frames
        .iter()
        .filter(|(_, v)| frame == v.frame_data.parent_frame_id)
        .map(|(k, _)| k.clone())
        .collect()
}

// get all children frames of a frame as a hashmap
// fn get_frame_children_2(
//     frame: &str,
//     frames: &HashMap<String, ExtendedFrameData>,
// ) -> HashMap<String, ExtendedFrameData> {
//     let mut children = HashMap::<String, ExtendedFrameData>::new();
//     frames
//         .iter()
//         .filter(|(_, v)| frame == v.frame_data.parent_frame_id)
//         .for_each(|(k, v)| {
//             children.insert(k.to_string(), v.clone());
//         });
//     children
// }

// get all children frames of a frame
fn get_frame_children_3(
    frame: &str,
    frames: &HashMap<String, BufferedFrameData>,
) -> Vec<(String, FrameData)> {
    frames
        .iter()
        .filter(|(_, v)| frame == v.frame_data.parent_frame_id)
        .map(|(k, v)| (k.clone(), v.frame_data.clone()))
        .collect()
}

// TODO: should return option. should check also max depth
// check for cycles in the tree segment starting from this frame
fn is_cyclic(frame: &str, frames: &HashMap<String, BufferedFrameData>) -> (bool, String) {
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
fn is_cyclic_all(frames: &HashMap<String, BufferedFrameData>) -> (bool, String) {
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
    frame: &BroadcastedFrameData,
    frames: &HashMap<String, BufferedFrameData>,
) -> (bool, String) {
    let mut frames_local = frames.clone();
    frames_local.insert(
        frame.frame_data.child_frame_id.clone(),
        BufferedFrameData {
            frame_data: frame.frame_data.clone(),
            time_stamp: frame.time_stamp.clone(),
        },
    );
    // is_cyclic(&frame.frame_data.child_frame_id.clone(), &frames_local)
    is_cyclic_all(&frames_local)
}

async fn zone_marker_publisher_callback(
    publisher: r2r::Publisher<MarkerArray>,
    broadcasted_frames: &Arc<Mutex<HashMap<String, BroadcastedFrameData>>>,
    mut timer: r2r::Timer,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut markers: Vec<Marker> = vec![];
        let frames_local = broadcasted_frames.lock().unwrap().clone();
        let mut id = 0;
        for frame in frames_local {
            match frame.1.zone {
                Some(z) => {
                    id = id + 1;
                    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
                    let now = clock.get_now().unwrap();
                    let time_stamp = r2r::Clock::to_builtin_time(&now);
                    let indiv_marker = Marker {
                        header: Header {
                            stamp: time_stamp,
                            frame_id: frame.1.frame_data.child_frame_id.to_string(),
                        },
                        ns: "".to_string(),
                        id,
                        type_: 3,
                        action: 0,
                        pose: Pose {
                            position: Point {
                                x: 0.0, //frame.1.frame_data.transform.translation.x,
                                y: 0.0, //frame.1.frame_data.transform.translation.y,
                                z: 0.0 //frame.1.frame_data.transform.translation.z,
                            },
                            orientation: Quaternion {
                                x: 0.0, //frame.1.frame_data.transform.rotation.x,
                                y: 0.0, //frame.1.frame_data.transform.rotation.y,
                                z: 0.0, //frame.1.frame_data.transform.rotation.z,
                                w: 1.0 //frame.1.frame_data.transform.rotation.w,
                            },
                        },
                        lifetime: Duration { sec: 2, nanosec: 0 },
                        scale: Vector3 { x: z, y: z, z: 0.01 },
                        color: ColorRGBA {
                            r: 0.0,
                            g: 255.0,
                            b: 0.0,
                            a: 0.3,
                        },
                        ..Marker::default()
                    };
                    markers.push(indiv_marker)
                }
                None => (),
            }
        }

        let array_msg = MarkerArray { markers };

        match publisher.publish(&array_msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    NODE_ID,
                    "Publisher failed to send marker message with: {}",
                    e
                );
            }
        };
        timer.tick().await?;
    }
}

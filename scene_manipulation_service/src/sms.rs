use futures::{stream::Stream, StreamExt};
use glam::{DAffine3, DQuat, DVec3};
use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::{Quaternion, Transform, TransformStamped, Vector3};
use r2r::scene_manipulation_msgs::srv::{LookupTransform, ManipulateScene};
use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::{ParameterValue, QosProfile, ServiceRequest, log_warn};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::fs::{self, File};
use std::io::BufReader;
use std::sync::{Arc, Mutex};

pub static NODE_ID: &'static str = "scene_manipulation_service";
pub static FOLDER_RELOAD_SCENE_RATE: u64 = 3000;
pub static STATIC_BROADCAST_RATE: u64 = 1000;
pub static ACTIVE_BROADCAST_RATE: u64 = 100;
pub static BUFFER_MAINTAIN_RATE: u64 = 100;
pub static ACTIVE_FRAME_LIFETIME: i32 = 3; //seconds
pub static MAX_TRANSFORM_CHAIN: u64 = 100;

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub struct FrameData {
    pub parent_frame_id: String,
    pub child_frame_id: String,
    pub transform: Transform,
    pub active: bool,
}

// differentiate frames loaded through service call or loaded from folder
#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub struct ExtendedFrameData {
    pub frame_data: FrameData,
    pub folder_loaded: bool,
    pub time_stamp: Time,
}

// the testing module
mod tests;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
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

    let path = Arc::new(Mutex::new(path_param.clone()));
    let scenario = list_frames_in_dir(&path_param).await;

    // TODO: offer a service to load the scenario, i.e. if sms was launched without the specified folder
    // TODO: offer a service to get all frames from tf

    let init_loaded = load_scenario(&scenario).await;
    r2r::log_info!(
        NODE_ID,
        "Initial frames added to the scene: '{:?}'.",
        init_loaded.keys()
    );

    // frames that are published by this broadcaster
    let broadcaster_frames = Arc::new(Mutex::new(init_loaded));

    // frames that exist on the tf and tf_static topic, i.e. this is a local tf buffer
    let buffered_frames = Arc::new(Mutex::new(HashMap::<String, ExtendedFrameData>::new()));

    // listen to the active frames on the tf topic to see if a frame exists before broadcasting it
    let active_tf_listener = node.subscribe::<TFMessage>("tf", QosProfile::default())?;

    // listen to the active frames on the tf topic to see if a frame exists before broadcasting it
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

    let scene_manipulation_service =
        node.create_service::<ManipulateScene::Service>("manipulate_scene")?;

    let transform_lookup_service =
        node.create_service::<LookupTransform::Service>("lookup_transform")?;

    // occasionally look into the folder specified by the path to see if there are changes
    let reload_timer =
        node.create_wall_timer(std::time::Duration::from_millis(FOLDER_RELOAD_SCENE_RATE))?;

    // spawn a tokio task to handle publishing static frames
    let broadcaster_frames_clone_1 = broadcaster_frames.clone();
    tokio::task::spawn(async move {
        match static_frame_broadcaster_callback(
            static_frame_broadcaster,
            static_pub_timer,
            &broadcaster_frames_clone_1,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Static frame broadcaster failed with: '{}'.", e),
        };
    });

    // spawn a tokio task to handle publishing active frames
    let broadcaster_frames_clone_2 = broadcaster_frames.clone();
    tokio::task::spawn(async move {
        match active_frame_broadcaster_callback(
            active_frame_broadcaster,
            active_pub_timer,
            &broadcaster_frames_clone_2,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Active frame broadcaster failed with: '{}'.", e),
        };
    });

    // spawn a tokio task to handle reloading the scene when a new frame is to be added manually
    let broadcaster_frames_clone_3 = broadcaster_frames.clone();
    let path_clone_1 = path.clone();
    tokio::task::spawn(async move {
        match folder_manupulation_callback(reload_timer, &broadcaster_frames_clone_3, &path_clone_1)
            .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Active frame broadcaster failed with: '{}'.", e),
        };
    });

    // offer a service to manipulate the scene. This means adding (loading), updating, and removing frames through a service call.
    let broadcaster_frames_clone_4 = broadcaster_frames.clone();
    let buffered_frames_clone_1 = buffered_frames.clone();
    tokio::task::spawn(async move {
        let result = scene_manipulation_server(
            scene_manipulation_service,
            &broadcaster_frames_clone_4,
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

    // spawn a tokio task to listen to the active frames and add them to the buffer
    let buffered_frames_clone_2 = buffered_frames.clone();
    tokio::task::spawn(async move {
        match active_tf_listener_callback(active_tf_listener, &buffered_frames_clone_2).await {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Active tf listener failed with: '{}'.", e),
        };
    });

    // spawn a tokio task to listen to the active frames and add them to the buffer
    let buffered_frames_clone_2 = buffered_frames.clone();
    tokio::task::spawn(async move {
        match static_tf_listener_callback(static_tf_listener, &buffered_frames_clone_2).await {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Static tf listener failed with: '{}'.", e),
        };
    });

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

    // keep the node alive
    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(1000));
    });

    r2r::log_warn!(NODE_ID, "Node started.");

    handle.join().unwrap();

    Ok(())
}

async fn list_frames_in_dir(path: &str) -> Vec<String> {
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
            println!("{:?}", path);
            r2r::log_error!(
                NODE_ID,
                "Reading the scenario directory failed with: '{}'. Empty scenario will be launched.",
                e
            );
        }
    }
    scenario
}

async fn load_scenario(scenario: &Vec<String>) -> HashMap<String, ExtendedFrameData> {
    let mut frame_datas: HashMap<String, ExtendedFrameData> = HashMap::new();
    scenario.iter().for_each(|x| match File::open(x) {
        Ok(file) => {
            let reader = BufReader::new(file);
            match serde_json::from_reader(reader) {
                Ok(jsonl) => {
                    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
                    let now = clock.get_now().unwrap();
                    let time_stamp = r2r::Clock::to_builtin_time(&now);
                    let json = ExtendedFrameData {
                        frame_data: jsonl,
                        folder_loaded: true,
                        time_stamp,
                    };
                    frame_datas.insert(json.frame_data.child_frame_id.clone(), json.clone());
                }
                Err(e) => r2r::log_warn!(NODE_ID, "Serde failed with: '{}'.", e),
            }
        }
        Err(e) => r2r::log_warn!(NODE_ID, "Opening json file failed with: '{}'.", e),
    });
    frame_datas
}

async fn scene_manipulation_server(
    mut service: impl Stream<Item = ServiceRequest<ManipulateScene::Service>> + Unpin,
    broadcaster_frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(request) => match request.message.command.as_str() {
                "update" => {
                    r2r::log_info!(NODE_ID, "Got 'update' request: {:?}.", request.message);
                    let response =
                        update_frame(&request.message, &broadcaster_frames, &buffered_frames).await;
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
                "remove" => {
                    r2r::log_info!(NODE_ID, "Got 'remove' request: {:?}.", request.message);
                    let response =
                        remove_frame(&request.message, &broadcaster_frames, &buffered_frames).await;
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
    buffered_frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
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
                    r2r::log_info!(
                        NODE_ID,
                        "Found transform from '{}' to '{}': {:?}",
                        &request.message.parent_frame_id,
                        &request.message.child_frame_id,
                        transform
                    );
                    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
                    let now = clock.get_now().unwrap();
                    let time_stamp = r2r::Clock::to_builtin_time(&now);
                    let response = LookupTransform::Response {
                        success: true,
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
                    r2r::log_warn!(
                        NODE_ID,
                        "Failed to lookup transform from '{}' to '{}'.",
                        &request.message.parent_frame_id,
                        &request.message.child_frame_id,
                    );
                    let response = LookupTransform::Response {
                        success: false,
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
    r2r::log_info!(NODE_ID, "{}", info);
    ManipulateScene::Response {
        success: true,
        info,
    }
}

async fn update_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcaster_frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let local_broadcaster_frames = broadcaster_frames.lock().unwrap().clone();

    fn make_extended_frame_data(
        message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    ) -> ExtendedFrameData {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);
        ExtendedFrameData {
            frame_data: FrameData {
                parent_frame_id: message.parent_frame_id.clone(),
                child_frame_id: message.child_frame_id.clone(),
                transform: message.transform.clone(),
                active: true,
            },
            folder_loaded: false,
            time_stamp,
        }
    }

    fn add_with_msg_tf(
        message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
        broadcaster_frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
    ) -> ManipulateScene::Response {
        let mut local_broadcaster_frames = broadcaster_frames.lock().unwrap().clone();
        local_broadcaster_frames.insert(
            message.child_frame_id.clone(),
            make_extended_frame_data(message),
        );
        *broadcaster_frames.lock().unwrap() = local_broadcaster_frames;
        success_response(&format!(
            "Frame '{}' added to the scene.",
            message.child_frame_id
        ))
    }

    fn add_with_lookup_tf(
        message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
        broadcaster_frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
        transform: Transform,
    ) -> ManipulateScene::Response {
        let mut local_broadcaster_frames = broadcaster_frames.lock().unwrap().clone();
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);
        local_broadcaster_frames.insert(
            message.child_frame_id.clone(),
            ExtendedFrameData {
                frame_data: FrameData {
                    parent_frame_id: message.parent_frame_id.clone(),
                    child_frame_id: message.child_frame_id.clone(),
                    transform: transform,
                    active: true,
                },
                folder_loaded: false,
                time_stamp,
            },
        );
        *broadcaster_frames.lock().unwrap() = local_broadcaster_frames;
        success_response(&format!(
            "Frame '{}' added to the scene.",
            message.child_frame_id
        ))
    }

    async fn inner(
        message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
        broadcaster_frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
        buffered_frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
    ) -> ManipulateScene::Response {
        let local_broadcaster_frames = broadcaster_frames.lock().unwrap().clone();
        let local_buffered_frames = buffered_frames.lock().unwrap().clone();

        match local_broadcaster_frames.get(&message.child_frame_id) {
            Some(frame) => match frame.frame_data.active {
                false => match local_buffered_frames.contains_key(&message.parent_frame_id) {
                    false => {
                        // log_warn!()
                        add_with_msg_tf(message, broadcaster_frames)
                    }, //add warning
                    true => match check_would_produce_cycle(
                        &make_extended_frame_data(&message),
                        &local_buffered_frames,
                    ) {
                        (false, _) => match message.same_position_in_world {
                            false => add_with_msg_tf(message, broadcaster_frames),
                            true => match lookup_transform(
                                &message.parent_frame_id,
                                &message.child_frame_id,
                                &buffered_frames,
                            )
                            .await
                            {
                                None => error_response("Frailed to lookup transform."),
                                Some(transform) => {
                                    add_with_lookup_tf(message, broadcaster_frames, transform)
                                }
                            },
                        },
                        (true, cause) => error_response(&format!(
                            "Adding frame '{}' would produce a cycle. Not added",
                            cause
                        )),
                    },
                },
                true => error_response("Can't manipulate_static frames."),
            },
            None => error_response("Failed to get frame data from broadcaster hashmap."),
        }
    }

    match &message.child_frame_id == "world" {
        false => match local_buffered_frames.contains_key(&message.child_frame_id) {
            false => match local_broadcaster_frames.contains_key(&message.child_frame_id) {
                false => match local_buffered_frames.contains_key(&message.parent_frame_id) {
                    false => add_with_msg_tf(message, broadcaster_frames), //add warning
                    true => match check_would_produce_cycle(&make_extended_frame_data(&message), &local_buffered_frames) {
                        (false, _) => add_with_msg_tf(message, broadcaster_frames),
                        (true, cause) => error_response(&format!("Adding frame '{}' would produce a cycle. Not added", cause))
                    }
                }
                true => {
                    std::thread::sleep(std::time::Duration::from_millis(2000));
                    match local_buffered_frames.contains_key(&message.child_frame_id) {
                        false => error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                        true => inner(message, broadcaster_frames, buffered_frames).await
                    }
                }
            },
            true => match local_broadcaster_frames.contains_key(&message.child_frame_id) {
                false => error_response("Frame exists in the tf, but it can't be updated since it is not published by this broadcaster."),
                true => inner(message, broadcaster_frames, buffered_frames).await
            }
        },
        true => error_response("Frame 'world' is reserved as the universal tree root."),
    }
}

async fn remove_frame(
    message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
    broadcaster_frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
    buffered_frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
) -> ManipulateScene::Response {
    let local_buffered_frames = buffered_frames.lock().unwrap().clone();
    let local_broadcaster_frames = broadcaster_frames.lock().unwrap().clone();

    fn inner(
        message: &r2r::scene_manipulation_msgs::srv::ManipulateScene::Request,
        broadcaster_frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
    ) -> ManipulateScene::Response {
        let mut local_broadcaster_frames = broadcaster_frames.lock().unwrap().clone();
        match local_broadcaster_frames.get(&message.child_frame_id) {
            Some(frame) => match frame.frame_data.active {
                false => match frame.folder_loaded {
                    false => match local_broadcaster_frames.remove(&message.child_frame_id) {
                        Some(_) => {
                            *broadcaster_frames.lock().unwrap() = local_broadcaster_frames;
                            success_response(&format!("Frame '{}' removed from the scene.", message.child_frame_id))
                        }
                        None => error_response(&format!(
                            "Failed to remove frame '{}' from the scene.",
                            message.child_frame_id
                        ))
                    },
                    true => error_response("Frame is loaded via the folder and not manipulated afterwards. Frame won't be removed.")
                },
                true => error_response("Can't manipulate_static frames.")
            },
            None => error_response("Failed to get frame data from broadcaster hashmap.")
        }
    }

    match &message.child_frame_id == "world" {
        false => match local_buffered_frames.contains_key(&message.child_frame_id) {
            false => match local_broadcaster_frames.contains_key(&message.child_frame_id) {
                false => error_response("Frame doesn't exist in tf, nor is it published by this broadcaster."),
                true => {
                    std::thread::sleep(std::time::Duration::from_millis(2000));
                    match local_buffered_frames.contains_key(&message.child_frame_id) {
                        false => error_response("Frame doesn't exist in tf, but it is published by this broadcaster? Investigate."),
                        true => inner(message, broadcaster_frames)
                    }
                }
            },
            true => match local_broadcaster_frames.contains_key(&message.child_frame_id) {
                false => error_response("Frame exists in the tf, but it can't be removed since it is not published by this broadcaster."),
                true => inner(message, broadcaster_frames)
            }
        },
        true => error_response("Frame 'world' is reserved as the universal tree root."),
    }
}

// broadcast static frames
async fn static_frame_broadcaster_callback(
    publisher: r2r::Publisher<TFMessage>,
    mut timer: r2r::Timer,
    frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
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
                    "Active broadcaster failed to send a message with: '{}'",
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
    frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
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

// occasionally look at the scenario folder specified by the path and if there
// have been changes, add or remove the the frames that have been manually added
// to the folder or removed from the folder
async fn folder_manupulation_callback(
    mut timer: r2r::Timer,
    frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
    path: &Arc<Mutex<String>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        // let mut to_be_added = vec![];
        let path = path.lock().unwrap().clone();
        let dir_frames = list_frames_in_dir(&path).await.clone();

        // differentiate between frames loaded from folder and through services
        // for example, if we add a frame through the service call, it will be
        // removed every RELOAD_SCENE_RATE since it is not in the folder
        let mut folder_scenario = load_scenario(&dir_frames).await;
        folder_scenario.retain(|_, v| v.folder_loaded);
        let mut frames_local = frames.lock().unwrap().clone();
        frames_local.retain(|_, v| v.folder_loaded);

        let folder_vec = folder_scenario
            .iter()
            .map(|(k, _)| k.clone())
            .collect::<Vec<String>>();
        let loaded_vec = frames_local
            .iter()
            .map(|(k, _)| k.clone())
            .collect::<Vec<String>>();

        let folder_set: HashSet<String> =
            HashSet::from_iter(folder_vec.clone().iter().map(|x| x.clone()));
        let local_set: HashSet<String> =
            HashSet::from_iter(loaded_vec.clone().iter().map(|x| x.clone()));

        let to_be_added_names: Vec<String> = folder_set
            .difference(&local_set)
            .map(|x| x.clone())
            .collect();
        let to_be_removed_names: Vec<String> = local_set
            .difference(&folder_set)
            .map(|x| x.clone())
            .collect();

        let mut to_be_added = HashMap::<String, ExtendedFrameData>::new();
        let mut to_be_removed = HashMap::<String, ExtendedFrameData>::new();
        let mut transforms_local = frames.lock().unwrap().clone();

        to_be_added_names.iter().for_each(|name| {
            match folder_scenario.iter().find(|(json_k, _)| *json_k == name) {
                Some((data_k, data_v)) => {
                    to_be_added.insert(data_k.to_string(), data_v.clone());
                }
                None => (),
            }
        });

        to_be_removed_names.iter().for_each(|name| {
            match frames_local.iter().find(|(json_k, _)| *json_k == name) {
                Some((data_k, data_v)) => {
                    to_be_removed.insert(data_k.to_string(), data_v.clone());
                }
                None => (),
            }
        });

        // TODO: check if parent exists in the world
        // TODO: check if this addition will produce a cycle
        // TODO: check if it is folder loaded or if that has been changed in the meantime

        to_be_added
            .iter()
            .for_each(|(k, v)| match frames_local.get(k) {
                Some(frame_data) => match frame_data.frame_data.active {
                    false => {
                        let info = format!("Can't manipulate static frame '{}'.", k);
                        r2r::log_error!(NODE_ID, "{}", info);
                    }
                    true => {
                        transforms_local.insert(k.clone(), v.clone());
                        r2r::log_info!(NODE_ID, "Updated active frame: '{}'.", k);
                    }
                },
                None => {
                    transforms_local.insert(k.clone(), v.clone());
                    r2r::log_info!(NODE_ID, "Added new frame: '{}'.", k);
                }
            });

        to_be_removed
            .iter()
            .for_each(|(k, _)| match frames_local.get(k) {
                Some(frame_data) => match frame_data.frame_data.active {
                    false => {
                        let info = format!("Can't manipulate static frame '{}'.", k);
                        r2r::log_error!(NODE_ID, "{}", info);
                    }
                    true => {
                        transforms_local.remove(k);
                        r2r::log_info!(NODE_ID, "Removed active frame: '{}'.", k);
                    }
                },
                None => {
                    let info = format!(
                        "Couldn't find the frame '{}' in this broadcaster. None removed.",
                        k
                    );
                    r2r::log_warn!(NODE_ID, "{}", info);
                }
            });

        *frames.lock().unwrap() = transforms_local;

        timer.tick().await?;
    }
}

// update the buffer with active frames from the tf topic
async fn active_tf_listener_callback(
    mut subscriber: impl Stream<Item = TFMessage> + Unpin,
    frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let mut frames_local = frames.lock().unwrap().clone();
                message.transforms.iter().for_each(|t| {
                    // before adding an active frame to the buffer, check if the frame is
                    // already stale as defined with ACTIVE_FRAME_LIFETIME
                    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
                    let now = clock.get_now().unwrap();
                    let current_time = r2r::Clock::to_builtin_time(&now);

                    // match current_time.sec > t.header.stamp.sec + ACTIVE_FRAME_LIFETIME {
                    //     false => {
                    frames_local.insert(
                        t.child_frame_id.clone(),
                        ExtendedFrameData {
                            frame_data: FrameData {
                                parent_frame_id: t.header.frame_id.clone(),
                                child_frame_id: t.child_frame_id.clone(),
                                transform: t.transform.clone(),
                                active: true,
                            },
                            folder_loaded: false, // should be option None
                            time_stamp: t.header.stamp.clone(),
                        },
                    );
                    // }
                    // true => (),
                    // }
                });
                *frames.lock().unwrap() = frames_local;
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
    frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let mut frames_local = frames.lock().unwrap().clone();
                message.transforms.iter().for_each(|t| {
                    // static frames are always true, so we don't need to check their timestamp
                    frames_local.insert(
                        t.child_frame_id.clone(),
                        ExtendedFrameData {
                            frame_data: FrameData {
                                parent_frame_id: t.header.frame_id.clone(),
                                child_frame_id: t.child_frame_id.clone(),
                                transform: t.transform.clone(),
                                active: false,
                            },
                            folder_loaded: false, // should be option None, but lazy...
                            time_stamp: t.header.stamp.clone(),
                        },
                    );
                });
                *frames.lock().unwrap() = frames_local;
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
    frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let frames_local = frames.lock().unwrap().clone();
        let mut frames_local_reduced = frames_local.clone();
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let current_time = r2r::Clock::to_builtin_time(&now);
        frames_local.iter().for_each(|(k, v)| {
            match v.folder_loaded {
                false => {
                    match v.frame_data.active {
                        true => match current_time.sec > v.time_stamp.sec + ACTIVE_FRAME_LIFETIME {
                            true => {
                                frames_local_reduced.remove(k);
                            }
                            false => (),
                        },
                        false => (),
                    };
                }
                true => {
                    match v.frame_data.active {
                        true => match current_time.sec > v.time_stamp.sec + ACTIVE_FRAME_LIFETIME {
                            true => {
                                frames_local_reduced.remove(k);
                            }
                            false => (),
                        },
                        false => ()
                    }
                }
            }
        });
        *frames.lock().unwrap() = frames_local_reduced;
        timer.tick().await?;
    }
}

// fn quaternion_conjugate(q: &Quaternion) -> Quaternion {
//     Quaternion {
//         x: -q.x,
//         y: -q.y,
//         z: -q.z,
//         w: q.w,
//     }
// }

// // to apply the rotation of one quaternion to a pose, multiply the previous quaternion of the pose
// // by the quaternion representing the desired rotation
// // the order of this multiplication matters
// fn multiply_quaternion(q1: &Quaternion, q2: &Quaternion) -> Quaternion {
//     Quaternion {
//         x: q2.x * q1.w + q2.y * q1.z - q2.z * q1.y + q2.w * q1.x,
//         y: -q2.x * q1.z + q2.y * q1.w + q2.z * q1.x + q2.w * q1.y,
//         z: q2.x * q1.y - q2.y * q1.x + q2.z * q1.w + q2.w * q1.z,
//         w: -q2.x * q1.x - q2.y * q1.y - q2.z * q1.z + q2.w * q1.w,
//     }
// }

// // the magnitude of a quaternion should always be one
// // if numerical errors cause a quaternion magnitude other than one, ROS 2 will print warnings
// // to avoid these warnings, normalize the quaternion
// fn normalize_quaternion(q: &Quaternion) -> Quaternion {
//     let norm = (q.x.powi(2) + q.y.powi(2) + q.z.powi(2) + q.w.powi(2)).sqrt();
//     Quaternion {
//         x: q.x / norm,
//         y: q.y / norm,
//         z: q.z / norm,
//         w: q.w / norm,
//     }
// }

// fn multiply_and_normalize(q1: &Quaternion, q2: &Quaternion) -> Quaternion {
//     normalize_quaternion(&multiply_quaternion(q1, q2))
// }

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
    frames: &Arc<Mutex<HashMap<String, ExtendedFrameData>>>,
) -> Option<Transform> {
    let frames_local = frames.lock().unwrap().clone();
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
        (true, cause) => None,
    }
}

// when looking up a frame in a parent, first we have to find a common parent.
// we know that the world is always there so we go up until the world frame.
fn parent_to_world(
    parent_frame_id: &str,
    frames: &HashMap<String, ExtendedFrameData>,
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
                        println!("PARENT TO WORLD: {:?}", parent.frame_data.child_frame_id);
                        let a = tf_to_affine(&parent.frame_data.transform);
                        path.push(a.inverse());
                        let p = parent.frame_data.parent_frame_id.clone();
                        match p == "world".to_string() {
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
    frames: &HashMap<String, ExtendedFrameData>,
) -> Option<Vec<DAffine3>> {
    let mut stack = vec![];
    get_frame_children_3("world", frames)
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
        println!(
            "WORLD TO CHILD: {:?}",
            stack.iter().map(|x| x.0.clone()).collect::<Vec<String>>()
        );
        match stack.pop() {
            Some((frame, path, chain)) => match frame == child_frame_id {
                true => {
                    println!("WORLD TO CHILD PATH: {:?}", path);
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
// [tested]
fn get_frame_children(frame: &str, frames: &HashMap<String, ExtendedFrameData>) -> Vec<String> {
    frames
        .iter()
        .filter(|(_, v)| frame == v.frame_data.parent_frame_id)
        .map(|(k, _)| k.clone())
        .collect()
}

// get all children frames of a frame as a hashmap
fn get_frame_children_2(
    frame: &str,
    frames: &HashMap<String, ExtendedFrameData>,
) -> HashMap<String, ExtendedFrameData> {
    let mut children = HashMap::<String, ExtendedFrameData>::new();
    frames
        .iter()
        .filter(|(_, v)| frame == v.frame_data.parent_frame_id)
        .for_each(|(k, v)| {
            children.insert(k.to_string(), v.clone());
        });
    children
}

// get all children frames of a frame
fn get_frame_children_3(
    frame: &str,
    frames: &HashMap<String, ExtendedFrameData>,
) -> Vec<(String, FrameData)> {
    frames
        .iter()
        .filter(|(_, v)| frame == v.frame_data.parent_frame_id)
        .map(|(k, v)| (k.clone(), v.frame_data.clone()))
        .collect()
}

// TODO: should return option. should check also max depth
// check for cycles in the tree segment starting from this frame
// [tested]
fn is_cyclic(frame: &str, frames: &HashMap<String, ExtendedFrameData>) -> (bool, String) {
    let mut stack = vec![];
    let mut visited = vec![];
    stack.push(frame.to_string());
    loop {
        // r2r::log_warn!(
        //     NODE_ID,
        //     "IS CYCLIC LOOP."
        // );
        match stack.pop() {
            Some(current_frame) => match visited.contains(&current_frame) {
                true => break (true, current_frame),
                false => {
                    visited.push(current_frame.clone());
                    for child in get_frame_children(&current_frame, frames) {
                        stack.push(child);
                        continue;
                    }
                }
            },
            None => break (false, "".to_string()),
        }
    }
}

// check for all cycles including all frames even if tree is segmented
// [tested]
fn is_cyclic_all(frames: &HashMap<String, ExtendedFrameData>) -> (bool, String) {
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
// [tested]
fn check_would_produce_cycle(
    frame: &ExtendedFrameData,
    frames: &HashMap<String, ExtendedFrameData>,
) -> (bool, String) {
    let mut frames_local = frames.clone();
    frames_local.insert(frame.frame_data.child_frame_id.clone(), frame.clone());
    is_cyclic_all(&frames_local)
}

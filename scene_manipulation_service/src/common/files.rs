use r2r::geometry_msgs::msg::{Quaternion, Transform, Vector3};
use r2r::scene_manipulation_msgs::srv::ExtraFeatures;
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::BufReader;
use std::sync::{Arc, Mutex};

use crate::common::{errors::ErrorMsg, frame_data::FrameData};

use super::errors::{extra_error_response, extra_success_response};

pub async fn list_frames_in_dir(
    path: &str,
    node_id: &str,
) -> Result<Vec<String>, Box<dyn std::error::Error + Send>> {
    let mut scenario = vec![];
    match fs::read_dir(path) {
        Ok(dir) => dir.for_each(|file| match file {
            Ok(entry) => match entry.path().to_str() {
                Some(valid) => scenario.push(valid.to_string()),
                None => r2r::log_warn!(node_id, "Path is not valid unicode."),
            },
            Err(e) => r2r::log_warn!(node_id, "Reading entry failed with '{}'.", e),
        }),
        Err(e) => {
            r2r::log_warn!(
                node_id,
                "Reading the scenario directory failed with: '{}'.",
                e
            );
            r2r::log_warn!(node_id, "Empty scenario is loaded/reloaded.");
            return Err(Box::new(ErrorMsg::new(&format!(
                "Reading the scenario directory failed with: '{}'. 
                    Empty scenario is loaded/reloaded.",
                e
            ))));
        }
    }
    Ok(scenario)
}

pub fn load_scenario(scenario: &Vec<String>, node_id: &str) -> HashMap<String, FrameData> {
    let mut frame_datas: HashMap<String, FrameData> = HashMap::new();

    // add the world frame explicitly so that transforms can be looked up to and from it
    frame_datas.insert(
        "world".to_string(),
        FrameData {
            parent_frame_id: "world_origin".to_string(),
            child_frame_id: "world".to_string(),
            active: Some(false),
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
                    r2r::log_warn!(node_id, "Serde failed with: '{}'.", e);
                }
            }
        }
        Err(e) => r2r::log_warn!(node_id, "Opening json file failed with: '{}'.", e),
    });
    frame_datas
}

pub async fn reload_scenario(
    message: &r2r::scene_manipulation_msgs::srv::ExtraFeatures::Request,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    node_id: &str,
) -> ExtraFeatures::Response {
    match list_frames_in_dir(&message.scenario_path, node_id).await {
        Ok(scenario) => {
            let loaded = load_scenario(&scenario, node_id);
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
        Err(e) => extra_error_response(&format!("Reloading the scenario failed with: '{:?}'.", e)),
    }
}

// async fn persist_frame_change(path: &str, frame: FrameData) -> bool {
//     match fs::read_dir(path) {
//         Ok(dir) => dir.for_each(|file| match file {
//             Ok(entry) => match entry.path().to_str() {
//                 Some(valid) => match valid.to_string() == format!("{}{}", path, frame.child_frame_id.clone()) {
//                     true => {
//                         println!("Changing existing frame {} permanently", frame.child_frame_id.clone());
//                         match File::open(valid.clone()) {
//                             Ok(file) =>
//                         }
//                         let writer = BufWriter::;
//                     // }
//                     },
//                     false => {}
//                 }
//                 None => r2r::log_warn!(NODE_ID, "Path is not valid unicode."),
//             },
//             Err(e) => r2r::log_warn!(NODE_ID, "Reading entry failed with '{}'.", e),
//         }),
//         Err(e) => {
//             r2r::log_warn!(
//                 NODE_ID,
//                 "Reading the scenario directory failed with: '{}'.",
//                 e
//             );
//             r2r::log_warn!(NODE_ID, "Empty scenario is loaded/reloaded.");
//             return false
//         }
//     }
//     true
// }

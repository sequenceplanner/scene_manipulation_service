use futures::{stream::Stream, StreamExt};
use glam::{DAffine3, DQuat, DVec3};
use r2r::builtin_interfaces::msg::{Time};
use r2r::geometry_msgs::msg::{Quaternion, Transform, TransformStamped, Vector3};
use r2r::scene_manipulation_msgs::msg::{TFExtra, TFExtraData};
use r2r::scene_manipulation_msgs::srv::{
    ExtraFeatures, GetAllTransforms, LookupTransform, ManipulateScene,
};
use r2r::std_msgs::msg::{Header};
use r2r::tf2_msgs::msg::TFMessage;
use r2r::{ParameterValue, QosProfile, ServiceRequest};
use serde::{Deserialize, Serialize};
use serde_json::{Value};
use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fs::{self, File};
use std::io::BufReader;
use std::sync::{Arc, Mutex};
use std::{fmt};

pub static MAX_TRANSFORM_CHAIN: u64 = 100;

#[derive(Debug, Default, Serialize, Deserialize, Clone, PartialEq)]
pub struct FrameData {
    // mandatory fields in the json files
    pub parent_frame_id: String, // the id of the frame's parent frame
    pub child_frame_id: String,  // the id of the frame
    pub transform: Transform,    // where is the child frame defined in the parent
    // optional fields in the json files. will be encoded to a json string before sent out
    pub time_stamp: Option<Time>, // the idea is that all frames should have this, but some don't
    pub zone: Option<f64>,        // when are you "at" the frame, threshold, in meters
    pub next: Option<HashSet<String>>, // this list can be used to store data for planners and visualizers
    pub frame_type: Option<String>, // can be used to distinguish if a frame is a waypoint, tag, human, etc.
    pub active: Option<bool>, // only active frames are manipulatable. undefined will be added as active
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

pub async fn lookup_transform(
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
            //     r2r::log_error!(
            //         NODE_ID,
            //         "TF lookup failed with: 'MAX_TRANSFORM_CHAIN reached'."
            //     );
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
        match stack.pop() {
            Some((frame, path, chain)) => match frame == child_frame_id {
                true => {
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
        match stack.pop() {
            Some(current_frame) => match visited.contains(&current_frame) {
                true => break (true, current_frame),
                false => {
                    visited.push(current_frame.clone());
                    for child in get_frame_children(&current_frame, frames) {
                        stack.push(child);
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
pub fn check_would_produce_cycle(
    frame: &FrameData,
    frames: &HashMap<String, FrameData>,
) -> (bool, String) {
    let mut frames_local = frames.clone();
    frames_local.insert(frame.child_frame_id.clone(), frame.clone());
    is_cyclic_all(&frames_local)
}
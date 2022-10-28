use crate::common::frame_data::FrameData;
use futures::{Stream, StreamExt};
use glam::{DAffine3, DQuat, DVec3};
use r2r::geometry_msgs::msg::{Quaternion, Transform, TransformStamped, Vector3};
use r2r::scene_manipulation_msgs::srv::LookupTransform;
use r2r::std_msgs::msg::Header;
use r2r::ServiceRequest;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

pub static MAX_TRANSFORM_CHAIN: u64 = 100;

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
pub fn parent_to_world(
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
                break None;
            }
        }
    }
}

// basically BFS to get the path to the child...
// even simpler since we check for cycles beforehand and its a tree, so not tracking visited...
// TODO: add max depth check though just in case.
pub fn world_to_child(
    child_frame_id: &str,
    frames: &HashMap<String, FrameData>,
) -> Option<Vec<DAffine3>> {
    let mut stack = vec![];
    get_frame_children("world_origin", frames)
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
                    get_frame_children(&frame, frames)
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

// get all children frame_names of a frame
pub fn get_frame_children_ids(frame: &str, frames: &HashMap<String, FrameData>) -> Vec<String> {
    frames
        .iter()
        .filter(|(_, v)| frame == v.parent_frame_id)
        .map(|(k, _)| k.clone())
        .collect()
}

// get all children frames of a frame
pub fn get_frame_children(
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
pub fn is_cyclic(frame: &str, frames: &HashMap<String, FrameData>) -> (bool, String) {
    let mut stack = vec![];
    let mut visited = vec![];
    stack.push(frame.to_string());
    loop {
        match stack.pop() {
            Some(current_frame) => match visited.contains(&current_frame) {
                true => break (true, current_frame),
                false => {
                    visited.push(current_frame.clone());
                    for child in get_frame_children_ids(&current_frame, frames) {
                        stack.push(child);
                    }
                }
            },
            None => break (false, "".to_string()),
        }
    }
}

// check for all cycles including all frames even if tree is segmented
pub fn is_cyclic_all(frames: &HashMap<String, FrameData>) -> (bool, String) {
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

pub async fn transform_lookup_server(
    mut service: impl Stream<Item = ServiceRequest<LookupTransform::Service>> + Unpin,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    node_id: &str,
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
                    r2r::log_warn!(node_id, "{}", info);
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

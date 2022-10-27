use r2r::geometry_msgs::msg::{Quaternion, Transform};
use r2r::{builtin_interfaces::msg::Time, geometry_msgs::msg::Vector3};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;

pub static MAX_TRANSFORM_CHAIN: u64 = 100;

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
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

impl Default for FrameData {
    fn default() -> Self {
        FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "UNKNOWN".to_string(),
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
            time_stamp: None,
            zone: None,
            next: None,
            frame_type: None,
            active: None,
        }
    }
}

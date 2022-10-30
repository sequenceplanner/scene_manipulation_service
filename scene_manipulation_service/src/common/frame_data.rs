use r2r::geometry_msgs::msg::{Quaternion, Transform};
use r2r::std_msgs::msg::ColorRGBA;
use r2r::{builtin_interfaces::msg::Time, geometry_msgs::msg::Vector3};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub struct LoadedFrameData {
    // mandatory fields in the json files
    pub parent_frame_id: String, // the id of the frame's parent frame
    pub child_frame_id: String,  // the id of the frame
    pub transform: Transform,    // where is the child frame defined in the parent
    // optional fields in the json files. will be encoded to a json string before sent out
    pub extra_data: Option<ExtraData>,
}

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub struct FrameData {
    // mandatory fields in the json files
    pub parent_frame_id: String, // the id of the frame's parent frame
    pub child_frame_id: String,  // the id of the frame
    pub transform: Transform,    // where is the child frame defined in the parent
    // optional fields in the json files. will be encoded to a json string before sent out
    pub extra_data: ExtraData,
}

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub struct ExtraData {
    pub time_stamp: Option<Time>, // the idea is that all frames should have this, but some don't
    pub zone: Option<f64>,        // when are you "at" the frame, threshold, in meters
    pub next: Option<HashSet<String>>, // this list can be used to store data for planners and visualizers
    pub frame_type: Option<String>, // can be used to distinguish if a frame is a waypoint, tag, human, etc.
    pub active: Option<bool>, // only active frames are manipulatable. undefined will be added as active
    pub show_mesh: Option<bool>, // if the frame should also visualize something or not
    pub mesh_type: Option<i32>, // 1 - cube, 2 - sphere, 3 - cylinder or 10 - mesh (provide path)
    pub mesh_path: Option<String>, // where to find the mesh path if mesh_type was 10
    pub mesh_scale: Option<f32>, // not all meshes are in mm values
    pub mesh_color: Option<ColorRGBA>, // color for the mesh, A is transparency
}

impl Default for ExtraData {
    fn default() -> Self {
        ExtraData {
            time_stamp: None,
            zone: None,
            next: None,
            frame_type: None,
            active: None,
            show_mesh: None,
            mesh_type: None,
            mesh_path: None,
            mesh_scale: None,
            mesh_color: None,
        }
    }
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
            // json_path: "".to_string(),
            extra_data: ExtraData::default(),
        }
    }
}

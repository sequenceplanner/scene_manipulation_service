use r2r::builtin_interfaces::msg::Time;
use serde::{Deserialize, Serialize};
use std::collections::HashSet;

// the extra stuff that is arriving - everything should be option because it can be anything or nothing
#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub struct ExtraData {
    pub zone: Option<f64>, // when are you "at" the frame, threshold, in meters
    pub next: Option<HashSet<String>>, // this list can be used to store data for planners and visualizers
    pub time_stamp: Option<Time>, // the idea is that all frames should have this, but some don't
    pub frame_type: Option<String>, // can be used to distinguish if a frame is a waypoint, tag, human, etc.
}

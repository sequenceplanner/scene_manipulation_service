use std::error::Error;
use std::fmt;

use r2r::scene_manipulation_msgs::srv::{ManipulateExtras, ManipulateScene};

#[derive(Debug, Clone)]
pub struct ErrorMsg {
    info: String,
}

impl ErrorMsg {
    pub fn new(info: &str) -> ErrorMsg {
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

pub fn extra_error_response(msg: &str) -> ManipulateExtras::Response {
    let info = msg.to_string();
    // r2r::log_error!(NODE_ID, "{}", info);
    ManipulateExtras::Response {
        success: false,
        info,
    }
}

pub fn extra_success_response(msg: &str) -> ManipulateExtras::Response {
    let info = msg.to_string();
    // r2r::log_info!(NODE_ID, "{}", info);
    ManipulateExtras::Response {
        success: true,
        info,
    }
}

pub fn main_error_response(msg: &str) -> ManipulateScene::Response {
    let info = msg.to_string();
    // r2r::log_error!(NODE_ID, "{}", info);
    ManipulateScene::Response {
        success: false,
        info,
    }
}

pub fn main_success_response(msg: &str) -> ManipulateScene::Response {
    let info = msg.to_string();
    // r2r::log_info!(NODE_ID, "{}", info);
    ManipulateScene::Response {
        success: true,
        info,
    }
}

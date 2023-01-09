use futures::{stream::Stream, StreamExt};
use r2r::geometry_msgs::msg::TransformStamped;
use r2r::scene_manipulation_msgs::msg::{TFExtra, TFExtraData};
use r2r::std_msgs::msg::Header;
use serde_json::{json, Value};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use crate::common::frame_data::FrameData;
use crate::ExtraData;

pub async fn extra_frame_broadcaster_callback(
    publisher: r2r::Publisher<TFExtra>,
    mut timer: r2r::Timer,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    broadcasted_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let mut buffered_frames_local = buffered_frames.lock().unwrap().clone();
        let broadcasted_frames_local = broadcasted_frames.lock().unwrap().clone();

        // collect frames from tf and overwrite with data from the broadcaster buffer
        broadcasted_frames_local.iter().for_each(|(k, v)| {
            // println!("{:?}", v);
            buffered_frames_local.insert(k.to_string(), v.clone());
        });
        let mut to_publish = vec![];

        buffered_frames_local.iter().for_each(|(_, v)| {
            to_publish.push(TFExtraData {
                transform: TransformStamped {
                    header: Header {
                        stamp: time_stamp.clone(),
                        frame_id: v.parent_frame_id.clone(),
                    },
                    child_frame_id: v.child_frame_id.clone(),
                    transform: v.transform.clone(),
                },
                extra: {
                        let mut extra_map = serde_json::Map::<String, Value>::new();
                        // extra_map.insert(
                        //     "time_stamp".to_string(),
                        //     Value::from(v.extra_data.time_stamp.clone().unwrap_or_default()),
                        // );
                        extra_map.insert("zone".to_string(), Value::from(v.extra_data.zone.unwrap_or_default()));
                        extra_map.insert(
                            "next".to_string(),
                            Value::from(
                                v.extra_data.clone().next
                                    .clone()
                                    .unwrap_or_default()
                                    .iter()
                                    .map(|value| value.clone())
                                    .collect::<Vec<String>>(),
                            ),
                        );
                        extra_map.insert(
                            "frame_type".to_string(),
                            Value::from(v.extra_data.frame_type.clone().unwrap_or_default()),
                        );
                        extra_map.insert(
                            "active".to_string(),
                            Value::from(v.extra_data.active.clone().unwrap_or_default()),
                        );
                        extra_map.insert(
                            "show_mesh".to_string(),
                            Value::from(v.extra_data.show_mesh.clone().unwrap_or_default()),
                        );
                        extra_map.insert(
                            "mesh_type".to_string(),
                            Value::from(v.extra_data.mesh_type.clone().unwrap_or_default()),
                        );
                        extra_map.insert(
                            "mesh_path".to_string(),
                            Value::from(v.extra_data.mesh_path.clone().unwrap_or_default()),
                        );
                        extra_map.insert(
                            "mesh_scale".to_string(),
                            Value::from(v.extra_data.mesh_scale.clone().unwrap_or_default()),
                        );
                        extra_map.insert(
                            "mesh_r".to_string(),
                            Value::from(v.extra_data.mesh_r.clone().unwrap_or_default()),
                        );
                        extra_map.insert(
                            "mesh_g".to_string(),
                            Value::from(v.extra_data.mesh_g.clone().unwrap_or_default()),
                        );
                        extra_map.insert(
                            "mesh_b".to_string(),
                            Value::from(v.extra_data.mesh_b.clone().unwrap_or_default()),
                        );
                        extra_map.insert(
                            "mesh_a".to_string(),
                            Value::from(v.extra_data.mesh_a.clone().unwrap_or_default()),
                        );
                        serde_json::Value::Object(extra_map).to_string()
                    },
                                  
            });
        });

        let msg = TFExtra { data: to_publish };

        match publisher.publish(&msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    node_id,
                    "Extra frame broadcaster failed to send a message with: '{}'",
                    e
                );
            }
        };
        timer.tick().await?;
    }
}
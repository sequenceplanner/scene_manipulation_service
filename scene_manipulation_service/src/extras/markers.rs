use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

use r2r::{
    builtin_interfaces::msg::Duration,
    geometry_msgs::msg::{Point, Pose, Quaternion, Vector3},
    std_msgs::msg::{ColorRGBA, Header},
    visualization_msgs::msg::{Marker, MarkerArray},
};

use crate::{lookup_transform, FrameData};

pub async fn marker_publisher_callback(
    zone_publisher: r2r::Publisher<MarkerArray>,
    path_publisher: r2r::Publisher<MarkerArray>,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    mut timer: r2r::Timer,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let mut zone_markers: Vec<Marker> = vec![];
        let mut path_markers: Vec<Marker> = vec![];
        let frames_local = buffered_frames.lock().unwrap().clone();
        // let decoded = serde_json::json!()
        let mut id = 0;
        for frame in frames_local {
            // let extra_json = serde_json::json!(frame.1.extra);
            // let extra: FrameData = serde_json::from_value(extra_json).unwrap_or_default();
            match frame.1.zone {
                Some(z) => {
                    id = id + 1;
                    let indiv_marker = Marker {
                        header: Header {
                            stamp: time_stamp.clone(),
                            frame_id: frame.1.child_frame_id.to_string(),
                        },
                        ns: "".to_string(),
                        id,
                        type_: 3,
                        action: 0,
                        pose: Pose {
                            position: Point {
                                x: 0.0,
                                y: 0.0,
                                z: 0.0,
                            },
                            orientation: Quaternion {
                                x: 0.0,
                                y: 0.0,
                                z: 0.0,
                                w: 1.0,
                            },
                        },
                        lifetime: Duration { sec: 2, nanosec: 0 },
                        scale: Vector3 {
                            x: z,
                            y: z,
                            z: 0.01,
                        },
                        color: ColorRGBA {
                            r: 0.0,
                            g: 255.0,
                            b: 0.0,
                            a: 0.15,
                        },
                        ..Marker::default()
                    };
                    zone_markers.push(indiv_marker)
                }
                None => (),
            }
            match frame.1.next {
                Some(n) => {
                    for enabled in n {
                        match lookup_transform(&frame.1.child_frame_id, &enabled, &buffered_frames)
                            .await
                        {
                            Some(end_in_world) => {
                                id = id + 1;
                                let indiv_marker = Marker {
                                    header: Header {
                                        stamp: r2r::builtin_interfaces::msg::Time {
                                            sec: 0,
                                            nanosec: 0,
                                        },
                                        frame_id: frame.1.child_frame_id.to_string(),
                                    },
                                    ns: "".to_string(),
                                    id,
                                    type_: 0,
                                    action: 0,
                                    points: vec![
                                        Point {
                                            x: 0.0,
                                            y: 0.0,
                                            z: 0.0,
                                        },
                                        Point {
                                            x: end_in_world.translation.x,
                                            y: end_in_world.translation.y,
                                            z: end_in_world.translation.z,
                                        },
                                    ],
                                    lifetime: Duration { sec: 2, nanosec: 0 },
                                    scale: Vector3 {
                                        x: 0.1,
                                        y: 0.2,
                                        z: 0.5,
                                    },
                                    color: ColorRGBA {
                                        r: 0.0,
                                        g: 255.0,
                                        b: 0.0,
                                        a: 0.15,
                                    },
                                    ..Marker::default()
                                };
                                path_markers.push(indiv_marker)
                            }
                            None => (),
                        }
                    }
                }
                None => (),
            }
        }

        let zone_array_msg = MarkerArray {
            markers: zone_markers,
        };
        let path_array_msg = MarkerArray {
            markers: path_markers,
        };

        match zone_publisher.publish(&zone_array_msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    node_id,
                    "Publisher failed to send zone marker message with: {}",
                    e
                );
            }
        };

        match path_publisher.publish(&path_array_msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    node_id,
                    "Publisher failed to send path marker message with: {}",
                    e
                );
            }
        };

        timer.tick().await?;
    }
}

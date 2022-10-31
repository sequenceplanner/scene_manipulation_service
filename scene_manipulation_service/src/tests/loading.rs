#![allow(unused_imports)]
#![allow(dead_code)]
use scene_manipulation_service::FrameData;
use serde_json::json;
use std::{fs::File, io::Write};

use crate::{list_frames_in_dir, load_scenario};

fn make_frame(name: &str, x: f64, y: f64, z: f64) -> String {
    json!({
        "active": true,
        "child_frame_id": name,
        "parent_frame_id": "world",
        "transform": {
            "translation": {
                "x": x,
                "y": y,
                "z": z
            },
            "rotation": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "w": 1.0
            }
        },
        "extra_data": {}
    })
    .to_string()
}

#[tokio::test]
async fn test_list_frames_in_dir() {
    // create the temp directory to store the dummy files in
    let dir = match tempfile::tempdir() {
        Ok(d) => d,
        Err(e) => {
            println!(
                "Failed to generate temporary dummy directory with: '{}'.",
                e
            );
            panic!()
        }
    };

    // create the temporary dummy files
    let dummy_file_1 = dir.path().join("dummy_1.urdf");
    let dummy_file_2 = dir.path().join("dummy_2.txt");
    let dummy_file_3 = dir.path().join("dummy_3.json");
    let pathbufs = vec![
        dummy_file_1.clone(),
        dummy_file_2.clone(),
        dummy_file_3.clone(),
    ];
    pathbufs.iter().for_each(|pb| {
        File::create(pb.clone()).unwrap();
    });

    let list = list_frames_in_dir(
        &format!("{}", dir.path().as_os_str().to_str().unwrap()),
        "some_node",
    )
    .await;

    assert_eq!(
        list.unwrap().sort(),
        vec!(
            dummy_file_1.as_path().as_os_str().to_str().unwrap(),
            dummy_file_2.as_path().as_os_str().to_str().unwrap(),
            dummy_file_3.as_path().as_os_str().to_str().unwrap()
        )
        .sort()
    );

    pathbufs.iter().for_each(|x| drop(x));

    match dir.close() {
        Ok(()) => (),
        Err(_) => panic!(),
    };
}

#[tokio::test]
async fn test_load_scenario() {
    // create the temp directory to store the dummy files in
    let dir = match tempfile::tempdir() {
        Ok(d) => d,
        Err(e) => {
            println!(
                "Failed to generate temporary dummy directory with: '{}'.",
                e
            );
            panic!()
        }
    };

    // create the temporary dummy files
    let dummy_file_1 = dir.path().join("dummy_1.json");
    let dummy_file_2 = dir.path().join("dummy_2.json");
    let dummy_file_3 = dir.path().join("dummy_3.json");

    let mut file_1 = File::create(dummy_file_1.clone()).unwrap();
    write!(file_1, "{}", make_frame("dummy_1", 1.0, 0.0, 0.0)).unwrap();
    let mut file_2 = File::create(dummy_file_2.clone()).unwrap();
    write!(file_2, "{}", make_frame("dummy_2", 0.0, 1.0, 0.0)).unwrap();
    let mut file_3 = File::create(dummy_file_3.clone()).unwrap();
    write!(file_3, "{}", make_frame("dummy_3", 0.0, 0.0, 1.0)).unwrap();

    let mut list = crate::list_frames_in_dir(
        &format!("{}", dir.path().as_os_str().to_str().unwrap()),
        "some_node",
    )
    .await
    .unwrap();

    assert_eq!(
        list.sort(),
        vec!(
            dummy_file_1.as_path().as_os_str().to_str().unwrap(),
            dummy_file_2.as_path().as_os_str().to_str().unwrap(),
            dummy_file_3.as_path().as_os_str().to_str().unwrap()
        )
        .sort()
    );

    let scenario = load_scenario(&list.clone(), "some_node");

    let mut test_scenario = crate::HashMap::<String, FrameData>::new();

    test_scenario.insert(
        "dummy_1".to_string(),
        FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_1".to_string(),
            transform: r2r::geometry_msgs::msg::Transform {
                translation: r2r::geometry_msgs::msg::Vector3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                rotation: r2r::geometry_msgs::msg::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
            // active: Some(true),
            ..Default::default()
        },
    );

    test_scenario.insert(
        "dummy_2".to_string(),
        FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_2".to_string(),
            transform: r2r::geometry_msgs::msg::Transform {
                translation: r2r::geometry_msgs::msg::Vector3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                },
                rotation: r2r::geometry_msgs::msg::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
            // active: Some(true),
            ..Default::default()
        },
    );

    test_scenario.insert(
        "dummy_3".to_string(),
        FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_3".to_string(),
            transform: r2r::geometry_msgs::msg::Transform {
                translation: r2r::geometry_msgs::msg::Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 1.0,
                },
                rotation: r2r::geometry_msgs::msg::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
            // active: Some(true),
            ..Default::default()
        },
    );

    assert_eq!(
        scenario.get("dummy_1").unwrap().clone(),
        test_scenario.get("dummy_1").unwrap().clone()
    );
    assert_eq!(
        scenario.get("dummy_2").unwrap().clone(),
        test_scenario.get("dummy_2").unwrap().clone()
    );
    assert_eq!(
        scenario.get("dummy_3").unwrap().clone(),
        test_scenario.get("dummy_3").unwrap().clone()
    );

    drop(dummy_file_1);
    drop(dummy_file_2);
    drop(dummy_file_3);

    match dir.close() {
        Ok(()) => (),
        Err(_) => panic!(),
    };
}

#![allow(unused_imports)]
#![allow(dead_code)]
use r2r::geometry_msgs::msg::{Quaternion, Transform, Vector3};
use std::{
    collections::HashMap,
    default,
    sync::{Arc, Mutex},
};

use scene_manipulation_service::{
    common::{
        files::{list_frames_in_dir, load_scenario, reload_scenario},
        frame_data::FrameData,
    },
    core::lookup::{
        check_would_produce_cycle, get_frame_children_ids, is_cyclic, is_cyclic_all,
        lookup_transform,
    },
};

fn world_frame() -> FrameData {
    FrameData {
        parent_frame_id: "world_origin".to_string(),
        child_frame_id: "world".to_string(),
        // active: Some(false),
        ..Default::default()
    }
}

fn dummy_1_frame() -> FrameData {
    FrameData {
        parent_frame_id: "world".to_string(),
        child_frame_id: "dummy_1".to_string(),
        transform: r2r::geometry_msgs::msg::Transform {
            translation: r2r::geometry_msgs::msg::Vector3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            ..Default::default()
        },
        ..Default::default()
    }
}

fn dummy_2_frame() -> FrameData {
    FrameData {
        parent_frame_id: "dummy_1".to_string(),
        child_frame_id: "dummy_2".to_string(),
        transform: r2r::geometry_msgs::msg::Transform {
            translation: r2r::geometry_msgs::msg::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            ..Default::default()
        },
        ..Default::default()
    }
}

fn dummy_3_frame() -> FrameData {
    FrameData {
        parent_frame_id: "dummy_1".to_string(),
        child_frame_id: "dummy_3".to_string(),
        transform: r2r::geometry_msgs::msg::Transform {
            translation: r2r::geometry_msgs::msg::Vector3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            ..Default::default()
        },
        ..Default::default()
    }
}

#[tokio::test]
async fn test_get_frame_children() {
    let mut buffer = HashMap::<String, FrameData>::new();

    buffer.insert("world".to_string(), world_frame());
    buffer.insert("dummy_1".to_string(), dummy_1_frame());

    //          w
    //          |
    //          d1

    assert_eq!(get_frame_children_ids("world", &buffer), vec!("dummy_1"));

    buffer.insert("dummy_2".to_string(), dummy_2_frame());

    //          w
    //          |
    //          d1
    //          |
    //          d2

    assert_eq!(
        get_frame_children_ids("dummy_1", &buffer).sort(),
        vec!("dummy_2").sort()
    );

    buffer.insert("dummy_3".to_string(), dummy_3_frame());

    //          w
    //          |
    //          d1
    //         /  \
    //       d2    d3

    assert_eq!(
        get_frame_children_ids("dummy_1", &buffer).sort(),
        vec!("dummy_2, dummy_3").sort()
    );

    buffer.insert(
        "dummy_1".to_string(),
        FrameData {
            parent_frame_id: "dummy_2".to_string(),
            child_frame_id: "dummy_1".to_string(),
            transform: r2r::geometry_msgs::msg::Transform {
                translation: r2r::geometry_msgs::msg::Vector3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                ..Default::default()
            },
            ..Default::default()
        },
    );

    //          w
    //
    //          d1
    //        //   \
    //       d2    d3

    assert_eq!(
        get_frame_children_ids("world", &buffer),
        Vec::<String>::new()
    );
}

#[tokio::test]
async fn test_is_cyclic_1() {
    let mut buffer = HashMap::<String, FrameData>::new();
    buffer.insert(
        "dummy_2".to_string(),
        FrameData {
            parent_frame_id: "dummy_1".to_string(),
            child_frame_id: "dummy_2".to_string(),
            transform: r2r::geometry_msgs::msg::Transform {
                translation: r2r::geometry_msgs::msg::Vector3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                rotation: r2r::geometry_msgs::msg::Quaternion {
                    x: 0.2657277,
                    y: 0.6643192,
                    z: 0.4428795,
                    w: 0.5403023,
                },
            },
            ..Default::default()
        },
    );

    //          d1
    //          |
    //          d2

    let res = is_cyclic("dummy_1", &buffer);
    assert_eq!(res.0, false);

    buffer.insert(
        "dummy_1".to_string(),
        FrameData {
            parent_frame_id: "dummy_2".to_string(),
            child_frame_id: "dummy_1".to_string(),
            transform: r2r::geometry_msgs::msg::Transform {
                translation: r2r::geometry_msgs::msg::Vector3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                rotation: r2r::geometry_msgs::msg::Quaternion {
                    x: 0.2657277,
                    y: 0.6643192,
                    z: 0.4428795,
                    w: 0.5403023,
                },
            },
            ..Default::default()
        },
    );

    //          d1
    //        //
    //       d2

    let res = is_cyclic("dummy_1", &buffer);
    assert_eq!(res.0, true);
}

#[tokio::test]
async fn test_is_cyclic_2() {
    let mut buffer = HashMap::<String, FrameData>::new();
    buffer.insert("dummy_2".to_string(), dummy_2_frame());

    //          d1
    //          |
    //          d2

    let res = is_cyclic("dummy_1", &buffer);
    assert_eq!(res.0, false);

    let res = is_cyclic("dummy_2", &buffer);
    assert_eq!(res.0, false);

    buffer.insert(
        "dummy_3".to_string(),
        FrameData {
            parent_frame_id: "dummy_2".to_string(),
            child_frame_id: "dummy_3".to_string(),
            ..Default::default()
        },
    );

    //          d1
    //        /
    //       d2 -- d3

    let res = is_cyclic("dummy_1", &buffer);
    assert_eq!(res.0, false);

    let res = is_cyclic("dummy_2", &buffer);
    assert_eq!(res.0, false);

    let res = is_cyclic("dummy_3", &buffer);
    assert_eq!(res.0, false);

    buffer.insert(
        "dummy_1".to_string(),
        FrameData {
            parent_frame_id: "dummy_3".to_string(),
            child_frame_id: "dummy_1".to_string(),
            ..Default::default()
        },
    );

    //          d1
    //        /    \
    //       d2 -- d3

    let res = is_cyclic("dummy_1", &buffer);
    assert_eq!(res.0, true);

    let res = is_cyclic("dummy_2", &buffer);
    assert_eq!(res.0, true);

    let res = is_cyclic("dummy_3", &buffer);
    assert_eq!(res.0, true);
}

#[tokio::test]
async fn test_is_cyclic_all_1() {
    let mut buffer = HashMap::<String, FrameData>::new();
    buffer.insert(
        "dummy_2".to_string(),
        FrameData {
            parent_frame_id: "dummy_1".to_string(),
            child_frame_id: "dummy_2".to_string(),
            ..Default::default()
        },
    );

    //          d1
    //          |
    //          d2

    let res = is_cyclic_all(&buffer);
    assert_eq!(res.0, false);

    buffer.insert(
        "dummy_3".to_string(),
        FrameData {
            parent_frame_id: "dummy_2".to_string(),
            child_frame_id: "dummy_3".to_string(),
            ..Default::default()
        },
    );

    //          d1
    //        /
    //       d2 -- d3

    let res = is_cyclic_all(&buffer);
    assert_eq!(res.0, false);

    buffer.insert(
        "dummy_1".to_string(),
        FrameData {
            parent_frame_id: "dummy_3".to_string(),
            child_frame_id: "dummy_1".to_string(),
            ..Default::default()
        },
    );

    //          d1
    //        /    \
    //       d2 -- d3

    let res = is_cyclic_all(&buffer);
    assert_eq!(res.0, true);
}

#[tokio::test]
async fn test_is_cyclic_all_2() {
    let mut buffer = HashMap::<String, FrameData>::new();
    buffer.insert(
        "dummy_1".to_string(),
        FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_1".to_string(),
            ..Default::default()
        },
    );

    //          w
    //          |
    //          d1

    let res = is_cyclic_all(&buffer);
    assert_eq!(res.0, false);

    buffer.insert(
        "dummy_2".to_string(),
        FrameData {
            parent_frame_id: "dummy_1".to_string(),
            child_frame_id: "dummy_2".to_string(),
            ..Default::default()
        },
    );

    //          w
    //          |
    //          d1
    //          |
    //          d2

    let res = is_cyclic_all(&buffer);
    assert_eq!(res.0, false);

    buffer.insert(
        "dummy_3".to_string(),
        FrameData {
            parent_frame_id: "dummy_1".to_string(),
            child_frame_id: "dummy_3".to_string(),
            ..Default::default()
        },
    );

    //          w
    //          |
    //          d1
    //         /  \
    //       d2    d3

    let res = is_cyclic_all(&buffer);
    assert_eq!(res.0, false);

    buffer.insert(
        "dummy_5".to_string(),
        FrameData {
            parent_frame_id: "dummy_4".to_string(),
            child_frame_id: "dummy_5".to_string(),
            ..Default::default()
        },
    );

    //          w       d4
    //          |       |
    //          d       d5
    //         /  \
    //       d2    d3

    let res = is_cyclic_all(&buffer);
    assert_eq!(res.0, false);

    buffer.insert(
        "dummy_6".to_string(),
        FrameData {
            parent_frame_id: "dummy_5".to_string(),
            child_frame_id: "dummy_6".to_string(),
            ..Default::default()
        },
    );

    //          w       d4
    //          |       |
    //          d       d5
    //         /  \     |
    //       d2    d3   d6

    let res = is_cyclic_all(&buffer);
    assert_eq!(res.0, false);

    buffer.insert(
        "dummy_4".to_string(),
        FrameData {
            parent_frame_id: "dummy_6".to_string(),
            child_frame_id: "dummy_4".to_string(),
            ..Default::default()
        },
    );

    //          w          d4
    //          |        /    \
    //          d       d5 -- d6
    //         /  \
    //       d2    d3

    let res = is_cyclic_all(&buffer);
    assert_eq!(res.0, true);

    buffer.insert(
        "dummy_4".to_string(),
        FrameData {
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_4".to_string(),
            ..Default::default()
        },
    );

    //          w---------d4
    //          |        /
    //          d       d5 -- d6
    //         /  \
    //       d2    d3

    let res = is_cyclic_all(&buffer);
    assert_eq!(res.0, false);
}

#[tokio::test]
async fn test_check_would_produce_cycle() {
    let mut buffer = HashMap::<String, FrameData>::new();
    buffer.insert(
        "dummy_2".to_string(),
        FrameData {
            parent_frame_id: "dummy_1".to_string(),
            child_frame_id: "dummy_2".to_string(),
            ..Default::default()
        },
    );

    //          d1
    //          |
    //          d2

    let frame_3 = FrameData {
        parent_frame_id: "dummy_2".to_string(),
        child_frame_id: "dummy_3".to_string(),
        ..Default::default()
    };

    //          d1
    //        /
    //       d2 -- d3

    let res = check_would_produce_cycle(&frame_3, &buffer);
    assert_eq!(res.0, false);

    let frame_1 = FrameData {
        parent_frame_id: "dummy_2".to_string(),
        child_frame_id: "dummy_1".to_string(),
        ..Default::default()
    };

    //          d1
    //          ||
    //          d2

    let res = check_would_produce_cycle(&frame_1, &buffer);
    assert_eq!(res.0, true);
}

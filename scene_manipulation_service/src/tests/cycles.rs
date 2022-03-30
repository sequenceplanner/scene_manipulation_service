#![allow(unused_imports)]
#![allow(dead_code)]
use r2r::geometry_msgs::msg::{Quaternion, Transform, Vector3};

#[tokio::test]
async fn test_get_frame_children() {
    let mut buffer = crate::HashMap::<String, crate::ExtendedFrameData>::new();
    buffer.insert(
        "dummy_1".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
                parent_frame_id: "world".to_string(),
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          w
    //          |
    //          d1

    assert_eq!(crate::get_frame_children("world", &buffer), vec!("dummy_1"));

    buffer.insert(
        "dummy_2".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          w
    //          |
    //          d1
    //          |
    //          d2

    assert_eq!(
        crate::get_frame_children("dummy_1", &buffer).sort(),
        vec!("dummy_2").sort()
    );

    buffer.insert(
        "dummy_3".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
                parent_frame_id: "dummy_1".to_string(),
                child_frame_id: "dummy_3".to_string(),
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          w
    //          |
    //          d1
    //         /  \
    //       d2    d3

    assert_eq!(
        crate::get_frame_children("dummy_1", &buffer).sort(),
        vec!("dummy_2, dummy_3").sort()
    );

    buffer.insert(
        "dummy_1".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          w
    //
    //          d1
    //        //   \
    //       d2    d3

    assert_eq!(
        crate::get_frame_children("world", &buffer),
        Vec::<String>::new()
    );
}

#[tokio::test]
async fn test_is_cyclic_1() {
    let mut buffer = crate::HashMap::<String, crate::ExtendedFrameData>::new();
    buffer.insert(
        "dummy_2".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          d1
    //          |
    //          d2

    let res = crate::is_cyclic("dummy_1", &buffer);
    assert_eq!(res.0, false);

    buffer.insert(
        "dummy_1".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          d1
    //        //
    //       d2

    let res = crate::is_cyclic("dummy_1", &buffer);
    assert_eq!(res.0, true);
}

#[tokio::test]
async fn test_is_cyclic_2() {
    let mut buffer = crate::HashMap::<String, crate::ExtendedFrameData>::new();
    buffer.insert(
        "dummy_2".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          d1
    //          |
    //          d2

    let res = crate::is_cyclic("dummy_1", &buffer);
    assert_eq!(res.0, false);

    let res = crate::is_cyclic("dummy_2", &buffer);
    assert_eq!(res.0, false);

    buffer.insert(
        "dummy_3".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
                parent_frame_id: "dummy_2".to_string(),
                child_frame_id: "dummy_3".to_string(),
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          d1
    //        /
    //       d2 -- d3

    let res = crate::is_cyclic("dummy_1", &buffer);
    assert_eq!(res.0, false);

    let res = crate::is_cyclic("dummy_2", &buffer);
    assert_eq!(res.0, false);

    let res = crate::is_cyclic("dummy_3", &buffer);
    assert_eq!(res.0, false);

    buffer.insert(
        "dummy_1".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
                parent_frame_id: "dummy_3".to_string(),
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          d1
    //        /    \
    //       d2 -- d3

    let res = crate::is_cyclic("dummy_1", &buffer);
    assert_eq!(res.0, true);

    let res = crate::is_cyclic("dummy_2", &buffer);
    assert_eq!(res.0, true);

    let res = crate::is_cyclic("dummy_3", &buffer);
    assert_eq!(res.0, true);
}

#[tokio::test]
async fn test_is_cyclic_all_1() {
    let mut buffer = crate::HashMap::<String, crate::ExtendedFrameData>::new();
    buffer.insert(
        "dummy_2".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          d1
    //          |
    //          d2

    let res = crate::is_cyclic_all(&buffer);
    assert_eq!(res, None);

    buffer.insert(
        "dummy_3".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
                parent_frame_id: "dummy_2".to_string(),
                child_frame_id: "dummy_3".to_string(),
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          d1
    //        /
    //       d2 -- d3

    let res = crate::is_cyclic_all(&buffer);
    assert_eq!(res, None);

    buffer.insert(
        "dummy_1".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
                parent_frame_id: "dummy_3".to_string(),
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          d1
    //        /    \
    //       d2 -- d3

    let res = crate::is_cyclic_all(&buffer);
    assert_eq!(res.unwrap().0, true);
}

#[tokio::test]
async fn test_is_cyclic_all_2() {
    let mut buffer = crate::HashMap::<String, crate::ExtendedFrameData>::new();
    buffer.insert(
        "dummy_1".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
                parent_frame_id: "world".to_string(),
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          w
    //          |
    //          d1

    let res = crate::is_cyclic_all(&buffer);
    assert_eq!(res, None);

    buffer.insert(
        "dummy_2".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          w
    //          |
    //          d1
    //          |
    //          d2

    let res = crate::is_cyclic_all(&buffer);
    assert_eq!(res, None);

    buffer.insert(
        "dummy_3".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
                parent_frame_id: "dummy_1".to_string(),
                child_frame_id: "dummy_3".to_string(),
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          w
    //          |
    //          d1
    //         /  \
    //       d2    d3

    let res = crate::is_cyclic_all(&buffer);
    assert_eq!(res, None);

    buffer.insert(
        "dummy_5".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
                parent_frame_id: "dummy_4".to_string(),
                child_frame_id: "dummy_5".to_string(),
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          w       d4
    //          |       |
    //          d       d5
    //         /  \
    //       d2    d3

    let res = crate::is_cyclic_all(&buffer);
    assert_eq!(res, None);

    buffer.insert(
        "dummy_6".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
                parent_frame_id: "dummy_5".to_string(),
                child_frame_id: "dummy_6".to_string(),
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          w       d4
    //          |       |
    //          d       d5
    //         /  \     |
    //       d2    d3   d6

    let res = crate::is_cyclic_all(&buffer);
    assert_eq!(res, None);

    buffer.insert(
        "dummy_4".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
                parent_frame_id: "dummy_6".to_string(),
                child_frame_id: "dummy_4".to_string(),
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          w          d4
    //          |        /    \
    //          d       d5 -- d6
    //         /  \
    //       d2    d3

    let res = crate::is_cyclic_all(&buffer);
    assert_eq!(res.unwrap().0, true);

    buffer.insert(
        "dummy_4".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
                parent_frame_id: "world".to_string(),
                child_frame_id: "dummy_4".to_string(),
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          w---------d4
    //          |        /
    //          d       d5 -- d6
    //         /  \
    //       d2    d3

    let res = crate::is_cyclic_all(&buffer);
    assert_eq!(res, None);
}

#[tokio::test]
async fn test_check_would_produce_cycle() {
    let mut buffer = crate::HashMap::<String, crate::ExtendedFrameData>::new();
    buffer.insert(
        "dummy_2".to_string(),
        crate::ExtendedFrameData {
            frame_data: crate::FrameData {
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
                active: true,
            },
            folder_loaded: true,
            time_stamp: crate::Time { sec: 0, nanosec: 0 },
        },
    );

    //          d1
    //          |
    //          d2

    let frame_3 = crate::ExtendedFrameData {
        frame_data: crate::FrameData {
            parent_frame_id: "dummy_2".to_string(),
            child_frame_id: "dummy_3".to_string(),
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
            active: true,
        },
        folder_loaded: true,
        time_stamp: crate::Time { sec: 0, nanosec: 0 },
    };

    //          d1
    //        /
    //       d2 -- d3

    let res = crate::check_would_produce_cycle( &frame_3,&buffer);
    assert_eq!(res, None);

    let frame_1 = crate::ExtendedFrameData {
        frame_data: crate::FrameData {
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
            active: true,
        },
        folder_loaded: true,
        time_stamp: crate::Time { sec: 0, nanosec: 0 },
    };

    //          d1
    //          ||
    //          d2

    let res = crate::check_would_produce_cycle( &frame_1,&buffer);
    assert_eq!(res.unwrap().0, true);
}

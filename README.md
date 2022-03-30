# Scene Manipulation Service

This is the new and integrated scene manipulation service, that also offers a lookup service.

![Alt text](/description/sms_chart.png "")

### Documentation: 

Pending

<!-- In the ROS2 world, the `tf` maintains the relationships between different coordinate frames in a tree structure buffered in time. One can include frames to that tree by `broadcasting` frame data to the `tf`. One can also `lookup` relationships between frames from this tree. Using the broadcasting and lookup tools, one can then use the `Scene Manipulation Service` to manipulate the scene in different ways.

To address a certain robotic or manufacturing setup, we use the notion of `scenarios`. A `scenario` is represented by its resources, their arrangement,and the task that they are going to execute. To initialize a scenario, the scenario folder contains a collection of frame description files in a `.json` format. 

## Features
1. Load initial frames from a scenario folder using `.json` description files while launching the service.
2. Add, update, or remove scenario frames by manually adding or removing `.json` description files to or from the scenario folder. How does this work? The frames that are added throught the scenario folder have a `folder_loaded` tag that is meant to differentiate them from the frames added through the service call. If the frame was updated through the service, it is not `folder_loaded` anymore, and thus will not be removed from the current scene if manually removed from the folder.
3. Add, update, or remove scenario frames using the service. 

Let's look at an example scenario located in the bringup package `scenario_1`, where we want to initialize the scene with two frames. The descriptions of these two frames are:

```
frame_1.json:                       frame_2.json:
{                                   {                       
    "active": true,                     "active": false,
    "child_frame_id": "frame_1",        "child_frame_id": "frame_2",
    "parent_frame_id": "world",         "parent_frame_id": "frame_1",
    "transform": {                      "transform": {
        "translation": {                    "translation": {
            "x": 1.0,                           "x": 0.0,
            "y": 0.0,                           "y": 1.0,
            "z": 0.0                            "z": 0.0
        },                                  },
        "rotation": {                       "rotation": {
            "x": 0.0,                           "x": 0.0,
            "y": 0.0,                           "y": 0.0,
            "z": 0.0,                           "z": 0.0,
            "w": 1.0                            "w": 1.0
        }                                   }
    }                                   }
}                                   }                      
```
There are two main ways one can manipulate the scene:
1.  By manually adding or removing .json files from the scenario folder 
2.  By requesting scene manipulations with a service call

## Buffer
Since we are not doing any path planning, i.e. we don't need any frame interpolation in time to generate trajectories, 
the buffer that is maintained here is simpler than the buffer that one would use for motion planning. Contrary to keeping all incomming frames in the buffer
for `DEFAULT_BUFFER_TIMESPAN` which is 10 seconds, the buffer is a simple hashmap which overwrites the previous value if a fresher one comes in. 
Let's take the two frames as an example. The buffer has two parameters: `ACTIVE_FRAME_LIFETIME` and `DEFAULT_BUFFER_TIMESPAN`. 

## TODO: add to the flowchart that `exists?` must be checked with the new fast lookup in the future instead of the local hashmap in the broadcaster

## Test checklist:
- [ ] service -> remove -> not_in_buffer -> not_in_bc -> FAULT
- [ ] service -> remove -> not_in_buffer -> in_bc -> wait -> not_in_buffer -> FAULT
- [ ] service -> remove -> not_in_buffer -> in_bc -> wait -> in_buffer -> static -> FAULT
- [ ] service -> remove -> not_in_buffer -> in_bc -> wait -> in_buffer -> active -> folder_tag -> FAULT
- [ ] service -> remove -> not_in_buffer -> in_bc -> wait -> in_buffer -> active -> not_folder_tag -> SUCCESS
- [ ] service -> remove -> in_buffer -> not_in_bc -> FAULT
- [ ] service -> remove -> in_buffer -> in_bc -> static -> FAULT
- [ ] service -> remove -> in_buffer -> in_bc -> active -> folder_tag -> FAULT
- [ ] service -> remove -> in_buffer -> in_bc -> active -> not_folder_tag -> SUCCESS

## Affine transformations
An affine transformation is any transformation that preserves collinearity (i.e., all points lying on a line initially still lie on a line after transformation) and ratios of distances (e.g., the midpoint of a line segment remains the midpoint after transformation). In this sense, affine indicates a special class of projective transformations that do not move any objects from the affine space R^3 to the plane at infinity or conversely. An affine transformation is also called an affinity.

Geometric contraction, expansion, dilation, reflection, rotation, shear, similarity transformations, spiral similarities, and translation are all affine transformations, as are their combinations. In general, an affine transformation is a composition of rotations, translations, dilations, and shears.

While an affine transformation preserves proportions on lines, it does not necessarily preserve angles or lengths. Any triangle can be transformed into any other by an affine transformation, so all triangles are affine and, in this sense, affine is a generalization of congruent and similar. -->

### Acknowledgements:
Christian Larsen - FCC
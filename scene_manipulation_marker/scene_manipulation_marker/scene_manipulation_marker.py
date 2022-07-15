import rclpy
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
from builtin_interfaces.msg import Time
from scene_manipulation_msgs.srv import PlaceMarker
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from scene_manipulation_msgs.srv import LookupTransform
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from tf2_ros import TransformBroadcaster

class SceneManipulationMarker(Node):
    def __init__(self):
        self.node_name = "scene_manipulation_marker"
        super().__init__(self.node_name)

        self.interactive_markers_server = InteractiveMarkerServer(
            self, "scene_manipulation_marker_server"
        )

        self.initial_marker_pose = TransformStamped()

        # self.initial_marker_pose = self.get_initial_marker_pose()
        self.initial_marker_pose = TransformStamped()
        self.initial_marker_pose.child_frame_id = "teaching_marker"
        self.initial_marker_pose.header.frame_id = "world"
        self.initial_marker_pose.transform.translation.x = 1.0
        self.initial_marker_pose.transform.translation.y = 1.0
        self.initial_marker_pose.transform.translation.z = 1.0
        self.initial_marker_pose.transform.rotation.x = 0.0
        self.initial_marker_pose.transform.rotation.y = 0.0
        self.initial_marker_pose.transform.rotation.z = 0.0
        self.initial_marker_pose.transform.rotation.w = 1.0

        self.marker_pose = self.initial_marker_pose

        # self.reset_service = self.create_service(PlaceMarker, 'reset_scene_manipulation_marker', self.reset_scene_manipulation_marker)
        self.broadcaster = TransformBroadcaster(self)
        # self.publisher = self.create_publisher(
        #     TransformStamped, "teaching_pose", 10
        # )

        self.lookup_client = self.create_client(LookupTransform, "lookup_transform")
        
        while not self.lookup_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("tf lookup service not available, waiting again...")

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)        

        # need a service also in gui like: put marker to "child" in "parent" when reseting and loading the marker

        # self.declare_parameter("initial_base_link_id", "default_value")
        # self.declare_parameter("initial_tcp_id", "default_value")
        # self.declare_parameter("marker_scale", "default_value")
        
        # self.base_link = self.get_parameter("initial_base_link_id").get_parameter_value().string_value
        # self.tcp_link = self.get_parameter("initial_tcp_id").get_parameter_value().string_value
        # self.marker_scale = float(self.get_parameter("marker_scale").get_parameter_value().string_value)

        self.create_interactive_marker(False, InteractiveMarkerControl.NONE, self.initial_marker_pose, True)
        self.interactive_markers_server.applyChanges()

    def timer_callback(self):
        current_time = self.get_clock().now().seconds_nanoseconds()
        self.marker_pose.header.stamp.sec = current_time[0]
        self.marker_pose.header.stamp.nanosec = current_time[1]
        self.broadcaster.sendTransform(self.marker_pose)

    # def get_initial_marker_pose(self):
    #     request = LookupTransform.Request()
    #     response = LookupTransform.Response()
    #     request.parent_frame_id = self.base_link
    #     request.child_frame_id = self.tcp_link
    #     # request.deadline = 3000
    #     future = self.client.call_async(request)
    #     self.get_logger().info(f"request sent: {request}")
    #     while rclpy.ok():
    #         rclpy.spin_once(self)
    #         if future.done():
    #             try:
    #                 response = future.result()
    #             except Exception as e:
    #                 self.get_logger().error(f"service call failed with: {(e,)}")
    #             else:
    #                 self.get_logger().info(f"lookup result: {self.response}")
    #             finally:
    #                 self.get_logger().info(f"service call completed")
    #                 return response.transform

    def create_marker_control(self, msg):
        control = InteractiveMarkerControl()
        control.always_visible = True
        msg.controls.append(control)
        return control

    def normalize_quaternion(self, quaternion_msg):
        norm = (
            quaternion_msg.x ** 2
            + quaternion_msg.y ** 2
            + quaternion_msg.z ** 2
            + quaternion_msg.w ** 2
        )
        s = norm ** (-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s

    def process_feedback(self, feedback):
        current_time = self.get_clock().now().seconds_nanoseconds()
        tf = TransformStamped()
        tf.header.frame_id = feedback.header.frame_id
        tf.header.stamp.sec = current_time[0]
        tf.header.stamp.nanosec = current_time[1]
        tf.child_frame_id = "teaching_marker"
        transf = Transform()
        transf.translation.x = feedback.pose.position.x
        transf.translation.y = feedback.pose.position.y
        transf.translation.z = feedback.pose.position.z
        transf.rotation.x = feedback.pose.orientation.x
        transf.rotation.y = feedback.pose.orientation.y
        transf.rotation.z = feedback.pose.orientation.z
        transf.rotation.w = feedback.pose.orientation.w
        tf.transform = transf
        self.marker_pose = tf        

    # lookup where the tcp is and put the marker there instead
    def create_interactive_marker(self, fixed, interaction_mode, initial_pose, show_6dof=False):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world" #self.parent_frame_id
        int_marker.pose.position.x = initial_pose.transform.translation.x
        int_marker.pose.position.y = initial_pose.transform.translation.y
        int_marker.pose.position.z = initial_pose.transform.translation.z
        int_marker.pose.orientation.x = initial_pose.transform.rotation.x
        int_marker.pose.orientation.y = initial_pose.transform.rotation.y
        int_marker.pose.orientation.z = initial_pose.transform.rotation.z
        int_marker.pose.orientation.w = initial_pose.transform.rotation.w
        int_marker.scale = 0.5 #self.marker_scale
        int_marker.name = "teaching_marker"
        int_marker.description = "teaching_marker"

        self.create_marker_control(int_marker)
        int_marker.controls[0].interaction_mode = interaction_mode
        int_marker.controls[0].always_visible = True

        if interaction_mode != InteractiveMarkerControl.NONE:
            control_modes_dict = {
                InteractiveMarkerControl.MOVE_3D: "MOVE_3D",
                InteractiveMarkerControl.ROTATE_3D: "ROTATE_3D",
                InteractiveMarkerControl.MOVE_ROTATE_3D: "MOVE_ROTATE_3D",
                InteractiveMarkerControl.FIXED: "FIXED",
                InteractiveMarkerControl.INHERIT: "INHERIT",
                InteractiveMarkerControl.MOVE_ROTATE: "MOVE_ROTATE",
            }
            int_marker.name += "_" + control_modes_dict[interaction_mode]
            int_marker.description = "3D Control"
            if show_6dof:
                int_marker.description += " + 6-DOF controls"
            int_marker.description += "\n" + control_modes_dict[interaction_mode]

        if show_6dof:
            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 1.0
            control.orientation.y = 0.0
            control.orientation.z = 0.0
            control.always_visible = True
            self.normalize_quaternion(control.orientation)
            control.name = "rotate_x"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 1.0
            control.orientation.y = 0.0
            control.orientation.z = 0.0
            control.always_visible = True
            self.normalize_quaternion(control.orientation)
            control.name = "move_x"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 1.0
            control.orientation.z = 0.0
            self.normalize_quaternion(control.orientation)
            control.name = "rotate_z"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 1.0
            control.orientation.z = 0.0
            self.normalize_quaternion(control.orientation)
            control.name = "move_z"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 0.0
            control.orientation.z = 1.0
            self.normalize_quaternion(control.orientation)
            control.name = "rotate_y"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 0.0
            control.orientation.z = 1.0
            self.normalize_quaternion(control.orientation)
            control.name = "move_y"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

        self.interactive_markers_server.insert(
            int_marker, feedback_callback=self.process_feedback
        )

    def reset_teaching_marker_callback(self, request, response):
        self.interactive_markers_server.clear()
        self.create_interactive_marker(False, InteractiveMarkerControl.NONE, self.initial_marker_pose, True)
        self.interactive_markers_server.applyChanges()
        self.get_logger().info('Got request to reset teaching marker.')
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)

    smm = SceneManipulationMarker()
    rclpy.spin(smm)
    smm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

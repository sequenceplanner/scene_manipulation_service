import sys
from urllib import response
import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from scene_manipulation_msgs.srv import LookupTransform, ManipulateScene, ReloadScenario, GetAllTransforms
from sensor_msgs.msg import JointState

import threading
import yaml

import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


class Callbacks:
    transform = TransformStamped()
    information = ""
    parent = ""
    child = ""
    new_id = ""
    frames = []
    joints = JointState()
    permanent = False
    scenario_path = ""

    trigger_refresh = None
    trigger_query = None
    trigger_remove_frame = None
    trigger_rename_frame = None
    trigger_reparent_frame = None
    trigger_clone_frame = None
    trigger_reload_scenario = None
    trigger_get_all = None

class Ros2Node(Node, Callbacks):
    def __init__(self):
        Node.__init__(self, "scene_manipulation_gui")
        Callbacks.__init__(self)

        Callbacks.trigger_query = self.trigger_query
        Callbacks.trigger_refresh = self.trigger_refresh
        Callbacks.trigger_remove_frame = self.trigger_remove_frame
        Callbacks.trigger_rename_frame = self.trigger_rename_frame
        Callbacks.trigger_reparent_frame = self.trigger_reparent_frame
        Callbacks.trigger_clone_frame = self.trigger_clone_frame
        Callbacks.trigger_reload_scenario = self.trigger_reload_scenario
        Callbacks.trigger_get_all = self.trigger_get_all

        self.tf_buffer = tf2_ros.Buffer()
        self.lf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.scenario_path = self.declare_parameter("scenario_path", "default value")
        self.scenario_path = (
            self.get_parameter("scenario_path").get_parameter_value().string_value
        )

        Callbacks.scenario_path = self.scenario_path

        self.tf_lookup_client = self.create_client(LookupTransform, "lookup_transform")
        self.tf_lookup_request = LookupTransform.Request()
        self.tf_lookup_response = LookupTransform.Response()

        while not self.tf_lookup_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("TF Lookup Service not available, waiting again...")

        self.sms_client = self.create_client(ManipulateScene, "manipulate_scene")
        self.sms_request = ManipulateScene.Request()
        self.sms_response = ManipulateScene.Response()

        while not self.sms_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "Manipulate Scene Service not available, waiting again..."
            )

        self.reload_client = self.create_client(ReloadScenario, "reload_scenario")
        self.reload_request = ReloadScenario.Request()
        self.reload_response = ReloadScenario.Response()

        while not self.reload_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "Reload Scenario Service not available, waiting again..."
            )

        self.get_all_client = self.create_client(GetAllTransforms, "get_all_transforms")
        self.get_all_request = GetAllTransforms.Request()
        self.get_all_response = GetAllTransforms.Response()

        while not self.get_all_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "Get All Transforms Service not available, waiting again..."
            )

        # initialize joint states if robot is not running
        Callbacks.joints = JointState()
        Callbacks.joints.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.jsub = self.create_subscription(
            JointState, "/joint_states", self.jsub_callback, 10
        )

        self.get_logger().info("Scene Manipulateion GUI up and running.")

    def jsub_callback(self, msg):
        Callbacks.joints = msg

    def trigger_refresh(self):
        yaml_file = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        Callbacks.frames.clear()
        if yaml_file != []:
            for i in yaml_file.keys():
                Callbacks.frames.append(i)
            Callbacks.frames.sort()

    def trigger_query(self):
        response = self.tf_lookup(Callbacks.parent, Callbacks.child)
        Callbacks.transform = response.transform
        if response.success:
            Callbacks.information = (
                "{\n"
                + '    "show": true,\n'
                + '    "active": true,\n'
                + f'    "child_frame_id": "{Callbacks.child}",\n'
                + f'    "parent_frame_id": "{Callbacks.parent}",\n'
                + '    "transform": {\n'
                + '        "translation": {\n'
                + f'            "x": {Callbacks.transform.transform.translation.x},\n'
                + f'            "y": {Callbacks.transform.transform.translation.y},\n'
                + f'            "z": {Callbacks.transform.transform.translation.z}\n'
                + "        },\n"
                + '        "rotation": {\n'
                + f'            "x": {Callbacks.transform.transform.rotation.x},\n'
                + f'            "y": {Callbacks.transform.transform.rotation.y},\n'
                + f'            "z": {Callbacks.transform.transform.rotation.z},\n'
                + f'            "w": {Callbacks.transform.transform.rotation.w}\n'
                + "        }\n"
                + "    },\n"
                + '    "preferred_joint_configuration": {\n'
                + f'        "j0": {Callbacks.joints.position[0]},\n'
                + f'        "j1": {Callbacks.joints.position[1]},\n'
                + f'        "j2": {Callbacks.joints.position[2]},\n'
                + f'        "j3": {Callbacks.joints.position[3]},\n'
                + f'        "j4": {Callbacks.joints.position[4]},\n'
                + f'        "j5": {Callbacks.joints.position[5]}\n'
                + "    }\n"
                + "}\n"
            )
        else:
            Callbacks.information = str(response.success) + ": " + response.info

    def manipulate_scene(self, command, child, parent, new_id, transform):
        self.sms_request.command = command
        self.sms_request.child_frame_id = child
        self.sms_request.parent_frame_id = parent
        self.sms_request.new_frame_id = new_id
        self.sms_request.transform = transform
        self.sms_future = self.sms_client.call_async(self.sms_request)
        while True:
            if self.sms_future.done():
                try:
                    response = self.sms_future.result()
                except Exception as e:
                    self.get_logger().info("SMS service call failed %r" % (e,))
                else:
                    self.get_logger().info("SMS succeded %r" % (response.success))
                return response

    def reload_scenario(self, path):
        self.reload_request.scenario_path = path
        future = self.reload_client.call_async(self.reload_request)
        while True:
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info("Reload service call failed %r" % (e,))
                else:
                    self.get_logger().info("Reload succeded %r" % (response.success))
                return response

    def get_all(self):
        future = self.get_all_client.call_async(self.get_all_request)
        while True:
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info("Get all service call failed %r" % (e,))
                else:
                    self.get_logger().info("Get all succeded %r" % (response.success))
                return response

    def trigger_reload_scenario(self):
        response = self.reload_scenario(Callbacks.scenario_path)
        Callbacks.information = str(response.success) + ": " + response.info

    def trigger_get_all(self):
        response = self.get_all()
        Callbacks.information = str(response)

    def trigger_remove_frame(self):
        response = self.manipulate_scene(
            "remove",
            Callbacks.child,
            Callbacks.parent,
            Callbacks.new_id,
            Callbacks.transform.transform,
        )
        Callbacks.information = str(response.success) + ": " + response.info

    def trigger_rename_frame(self):
        response = self.manipulate_scene(
            "rename",
            Callbacks.child,
            Callbacks.parent,
            Callbacks.new_id,
            Callbacks.transform.transform,
        )
        Callbacks.information = str(response.success) + ": " + response.info

    def trigger_reparent_frame(self):
        response = self.manipulate_scene(
            "reparent",
            Callbacks.child,
            Callbacks.parent,
            Callbacks.new_id,
            Callbacks.transform.transform,
        )
        Callbacks.information = str(response.success) + ": " + response.info

    def trigger_clone_frame(self):
        response = self.manipulate_scene(
            "clone",
            Callbacks.child,
            Callbacks.parent,
            Callbacks.new_id,
            Callbacks.transform.transform,
        )
        Callbacks.information = str(response.success) + ": " + response.info

    def tf_lookup(self, parent, child):
        self.tf_lookup_request.parent_frame_id = parent
        self.tf_lookup_request.child_frame_id = child

        future = self.tf_lookup_client.call_async(self.tf_lookup_request)
        self.get_logger().info("tf lookup request sent: %s" % self.tf_lookup_request)
        while rclpy.ok():
            if future.done():
                try:
                    response = future.result()
                    self.get_logger().info("lookup tf result: %s" % response)
                    return response
                except Exception as e:
                    self.get_logger().info(
                        "lookup tf service call failed with: %r" % (e,)
                    )
                    response = future.result()
                    return response


class Window(QWidget, Callbacks):
    def __init__(self):
        Callbacks.__init__(self)
        QWidget.__init__(self, None)

        grid = QGridLayout(self)
        grid.addWidget(self.make_query_tf_box())
        grid.addWidget(self.make_remove_box())
        grid.addWidget(self.make_rename_box())
        grid.addWidget(self.make_reparent_box())
        grid.addWidget(self.make_clone_box())
        grid.addWidget(self.make_extras_box())
        grid.addWidget(self.make_information_box())
        self.setLayout(grid)
        self.setWindowTitle("Scene Manipulateion GUI")
        self.resize(600, 900)

    def make_information_box(self):
        information_box = QGroupBox("information")
        information_box_layout = QGridLayout()
        self.output = QTextBrowser(information_box)
        self.output.setGeometry(QRect(10, 90, 600, 200))
        self.output.setObjectName("information")
        information_button = QPushButton("clear")
        information_button.setMaximumWidth(80)
        information_box_layout.addWidget(self.output, 0, 0)
        information_box_layout.addWidget(
            information_button, 1, 0, alignment=Qt.AlignRight
        )
        information_box.setLayout(information_box_layout)

        def information_button_clicked():
            self.output.clear()

        information_button.clicked.connect(information_button_clicked)

        return information_box

    def make_query_tf_box(self):
        combo_box = QGroupBox("")
        combo_box.setMinimumWidth(300)
        combo_box.setMaximumHeight(100)

        combo_box_layout = QGridLayout()

        combo_1_box_label = QLabel("child")
        combo_2_box_label = QLabel("parent")

        combo_1 = QComboBox()

        combo_1_box_button = QPushButton("refresh")
        combo_1_box_button.setMaximumWidth(280)
        combo_2 = QComboBox()
        combo_2.setMinimumWidth(400)
        combo_2_box_button = QPushButton("query")
        combo_2_box_button.setMaximumWidth(80)

        combo_box_layout.addWidget(combo_1_box_label, 0, 0)
        combo_box_layout.addWidget(combo_1, 0, 1)
        combo_box_layout.addWidget(combo_1_box_button, 0, 2)
        combo_box_layout.addWidget(combo_2_box_label, 1, 0)
        combo_box_layout.addWidget(combo_2, 1, 1)
        combo_box_layout.addWidget(combo_2_box_button, 1, 2)
        combo_box.setLayout(combo_box_layout)

        def combo_1_box_button_clicked():
            Callbacks.trigger_refresh()
            combo_1.clear()
            combo_1.addItems(Callbacks.frames)
            combo_2.clear()
            combo_2.addItems(Callbacks.frames)

        combo_1_box_button.clicked.connect(combo_1_box_button_clicked)

        def combo_2_box_button_clicked():
            Callbacks.child = combo_1.currentText()
            Callbacks.parent = combo_2.currentText()
            Callbacks.trigger_query()
            self.output.append(Callbacks.information)

        combo_2_box_button.clicked.connect(combo_2_box_button_clicked)

        return combo_box

    def make_remove_box(self):
        combo_box = QGroupBox("")
        combo_box.setMinimumWidth(300)
        combo_box.setMaximumHeight(100)

        combo_box_layout = QGridLayout()

        combo_2_box_label = QLabel("child")

        combo_1_box_button = QPushButton("refresh")
        combo_1_box_button.setMaximumWidth(280)
        combo_2 = QComboBox()
        combo_2.setMinimumWidth(313)
        combo_2_box_button = QPushButton("remove")
        combo_2_box_button.setMaximumWidth(80)

        combo_box_layout.addWidget(combo_2_box_label, 0, 0)
        combo_box_layout.addWidget(combo_2, 0, 1)
        combo_box_layout.addWidget(combo_1_box_button, 0, 2)
        combo_box_layout.addWidget(combo_2_box_button, 0, 3)
        combo_box.setLayout(combo_box_layout)

        def combo_1_box_button_clicked():
            Callbacks.trigger_refresh()
            combo_2.clear()
            combo_2.addItems(Callbacks.frames)

        combo_1_box_button.clicked.connect(combo_1_box_button_clicked)

        def combo_2_box_button_clicked():
            Callbacks.child = combo_2.currentText()
            Callbacks.trigger_remove_frame()
            self.output.append(Callbacks.information)

        combo_2_box_button.clicked.connect(combo_2_box_button_clicked)

        return combo_box

    def make_rename_box(self):
        combo_box = QGroupBox("")
        combo_box.setMinimumWidth(300)
        combo_box.setMaximumHeight(200)

        combo_box_layout = QGridLayout()

        combo_1_box_label = QLabel("child")
        line_edit_1_box_label = QLabel("new id")

        combo_1 = QComboBox()

        line_edit_1 = QLineEdit("")
        line_edit_1.setMaximumWidth(400)
        combo_1.setMaximumWidth(400)
        combo_1_box_button = QPushButton("refresh")
        combo_1_box_button.setMaximumWidth(280)
        combo_2_box_button = QPushButton("rename")
        combo_2_box_button.setMaximumWidth(280)

        combo_box_layout.addWidget(combo_1_box_label, 0, 0)
        combo_box_layout.addWidget(combo_1, 0, 1)
        combo_box_layout.addWidget(combo_1_box_button, 0, 2)
        combo_box_layout.addWidget(line_edit_1_box_label, 1, 0)
        combo_box_layout.addWidget(line_edit_1, 1, 1)
        combo_box_layout.addWidget(combo_2_box_button, 1, 2)
        combo_box.setLayout(combo_box_layout)

        def combo_1_box_button_clicked():
            Callbacks.trigger_refresh()
            combo_1.clear()
            combo_1.addItems(Callbacks.frames)

        combo_1_box_button.clicked.connect(combo_1_box_button_clicked)

        def combo_2_box_button_clicked():
            Callbacks.child = combo_1.currentText()
            Callbacks.new_id = line_edit_1.text()
            Callbacks.trigger_rename_frame()
            self.output.append(Callbacks.information)

        combo_2_box_button.clicked.connect(combo_2_box_button_clicked)

        return combo_box

    def make_reparent_box(self):
        combo_box = QGroupBox("")
        combo_box.setMinimumWidth(300)
        combo_box.setMaximumHeight(100)

        combo_box_layout = QGridLayout()

        combo_1_box_label = QLabel("child")
        combo_2_box_label = QLabel("to")

        combo_1 = QComboBox()

        combo_1_box_button = QPushButton("refresh")
        combo_1_box_button.setMaximumWidth(280)
        combo_2 = QComboBox()
        combo_2.setMinimumWidth(400)
        combo_2_box_button = QPushButton("reparent")
        combo_2_box_button.setMaximumWidth(80)

        combo_box_layout.addWidget(combo_1_box_label, 0, 0)
        combo_box_layout.addWidget(combo_1, 0, 1)
        combo_box_layout.addWidget(combo_1_box_button, 0, 2)
        combo_box_layout.addWidget(combo_2_box_label, 1, 0)
        combo_box_layout.addWidget(combo_2, 1, 1)
        combo_box_layout.addWidget(combo_2_box_button, 1, 2)
        combo_box.setLayout(combo_box_layout)

        def combo_1_box_button_clicked():
            Callbacks.trigger_refresh()
            combo_1.clear()
            combo_1.addItems(Callbacks.frames)
            combo_2.clear()
            combo_2.addItems(Callbacks.frames)

        combo_1_box_button.clicked.connect(combo_1_box_button_clicked)

        def combo_2_box_button_clicked():
            Callbacks.child = combo_1.currentText()
            Callbacks.parent = combo_2.currentText()
            Callbacks.trigger_reparent_frame()
            self.output.append(Callbacks.information)

        combo_2_box_button.clicked.connect(combo_2_box_button_clicked)

        return combo_box

    def make_clone_box(self):
        combo_box = QGroupBox("")
        combo_box.setMinimumWidth(300)
        combo_box.setMaximumHeight(200)

        combo_box_layout = QGridLayout()

        line_edit_1 = QLineEdit("")
        line_edit_1.setMaximumWidth(400)
        combo_1_box_label = QLabel("child")
        combo_2_box_label = QLabel("parent")
        line_edit_1_box_label = QLabel("clone")

        combo_1 = QComboBox()

        combo_1_box_button = QPushButton("refresh")
        combo_1_box_button.setMaximumWidth(280)
        combo_2 = QComboBox()
        combo_2.setMinimumWidth(400)
        combo_2_box_button = QPushButton("clone")
        combo_2_box_button.setMaximumWidth(80)

        combo_box_layout.addWidget(combo_1_box_label, 0, 0)
        combo_box_layout.addWidget(combo_1, 0, 1)
        combo_box_layout.addWidget(combo_1_box_button, 1, 2)
        combo_box_layout.addWidget(combo_2_box_label, 1, 0)
        combo_box_layout.addWidget(combo_2, 1, 1)
        combo_box_layout.addWidget(combo_2_box_button, 2, 2)
        combo_box_layout.addWidget(line_edit_1_box_label,2, 0)
        combo_box_layout.addWidget(line_edit_1, 2, 1)
        combo_box.setLayout(combo_box_layout)

        def combo_1_box_button_clicked():
            Callbacks.trigger_refresh()
            combo_1.clear()
            combo_1.addItems(Callbacks.frames)
            combo_2.clear()
            combo_2.addItems(Callbacks.frames)

        combo_1_box_button.clicked.connect(combo_1_box_button_clicked)

        def combo_2_box_button_clicked():
            Callbacks.child = combo_1.currentText()
            Callbacks.parent = combo_2.currentText()
            Callbacks.new_id = line_edit_1.text()
            Callbacks.trigger_clone_frame()
            self.output.append(Callbacks.information)

        combo_2_box_button.clicked.connect(combo_2_box_button_clicked)

        return combo_box

    def make_extras_box(self):
        combo_box = QGroupBox("")
        combo_box.setMinimumWidth(300)
        combo_box.setMaximumHeight(100)

        combo_box_layout = QGridLayout()

        combo_2_box_label = QLabel("path")

        combo_1_box_button = QPushButton("reload")
        combo_1_box_button.setMaximumWidth(280)
        line_edit_1 = QLineEdit(Callbacks.scenario_path)
        line_edit_1.setMaximumWidth(313)
        combo_2_box_button = QPushButton("get_all")
        combo_2_box_button.setMaximumWidth(80)

        combo_box_layout.addWidget(combo_2_box_label, 0, 0)
        combo_box_layout.addWidget(line_edit_1, 0, 1)
        combo_box_layout.addWidget(combo_1_box_button, 0, 2)
        combo_box_layout.addWidget(combo_2_box_button, 0, 3)
        combo_box.setLayout(combo_box_layout)

        def combo_1_box_button_clicked():
            Callbacks.scenario_path = line_edit_1.text()
            Callbacks.trigger_reload_scenario()
            self.output.append(Callbacks.information)

        combo_1_box_button.clicked.connect(combo_1_box_button_clicked)

        def combo_2_box_button_clicked():
            Callbacks.trigger_get_all()
            self.output.append(Callbacks.information)

        combo_2_box_button.clicked.connect(combo_2_box_button_clicked)

        return combo_box


def main(args=None):
    def launch_node():
        def launch_node_callback_local():
            rclpy.init(args=args)
            node = Ros2Node()
            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()

        t = threading.Thread(target=launch_node_callback_local)
        t.daemon = True
        t.start()

    # Window has to be in the main thread
    def launch_window():
        app = QApplication(sys.argv)
        clock = Window()
        clock.show()
        sys.exit(app.exec_())

    launch_node()
    launch_window()


if __name__ == "__main__":
    main()

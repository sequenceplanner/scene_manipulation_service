from ast import Call
from gc import callbacks
import sys
from urllib import response
import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from scene_manipulation_msgs.srv import LookupTransform, ManipulateScene, ManipulateExtras, GetAllTransforms
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
    persist = False
    scenario_path = ""
    zone = "0.0"
    marker_size = "2.0"

    trigger_refresh = None
    trigger_query = None
    trigger_get_all = None
    trigger_manipulate_scene = None
    trigger_manipulate_extra_features = None    

class Ros2Node(Node, Callbacks):
    def __init__(self):
        Node.__init__(self, "scene_manipulation_gui")
        Callbacks.__init__(self)

        Callbacks.trigger_refresh = self.trigger_refresh
        Callbacks.trigger_query = self.trigger_query
        Callbacks.trigger_get_all = self.trigger_get_all
        Callbacks.trigger_manipulate_scene = self.trigger_manipulate_scene
        Callbacks.trigger_manipulate_extra_features = self.trigger_manipulate_extra_features

        # self.tf_buffer = tf2_ros.Buffer(Duration(seconds=3, nanoseconds=0))
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

        self.extra_features_client = self.create_client(ManipulateExtras, "extra_features")
        self.extra_features_request = ManipulateExtras.Request()
        self.extra_features_response = ManipulateExtras.Response()

        while not self.extra_features_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "Extra Features Service not available, waiting again..."
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

    def manipulate_extra_features(self, command, child, parent, path, size):
        self.extra_features_request.command = command
        self.extra_features_request.child_frame_id = child
        self.extra_features_request.parent_frame_id = parent
        self.extra_features_request.scenario_path = path
        self.extra_features_request.size = size
        future = self.extra_features_client.call_async(self.extra_features_request)
        while True:
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info("Extra features service call failed %r" % (e,))
                else:
                    self.get_logger().info("Extra features succeded %r" % (response.success))
                return response

    def get_all(self):
        self.get_all_request.parent_frame_id = Callbacks.parent
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

    def trigger_manipulate_scene(self, command):
        response = self.manipulate_scene(
            command,
            Callbacks.child,
            Callbacks.parent,
            Callbacks.new_id,
            Callbacks.transform.transform
        )
        Callbacks.information = str(response.success) + ": " + response.info

    def trigger_manipulate_extra_features(self, command, size):
        response = self.manipulate_extra_features(
            command,
            Callbacks.child,
            Callbacks.parent,
            Callbacks.scenario_path,
            float(size)
        )
        Callbacks.information = str(response.success) + ": " + response.info

    def trigger_get_all(self):
        response = self.get_all()
        Callbacks.information = str(response)

class Window(QWidget, Callbacks):
    def __init__(self):
        Callbacks.__init__(self)
        QWidget.__init__(self, None)

        grid = QGridLayout(self)
        grid.addWidget(self.make_select_box())
        grid.addWidget(self.make_actions_box())
        grid.addWidget(self.make_extras_box())
        grid.addWidget(self.make_teaching_marker_box())
        grid.addWidget(self.make_information_box())
        self.setLayout(grid)
        self.setWindowTitle("Scene Manipulateion GUI")
        self.resize(600, 900)

    def make_select_box(self):
        combo_box = QGroupBox("select")

        combo_box_layout = QGridLayout()
        combo_1_box_label = QLabel("frame")
        combo_1_box_label.setToolTip("""Select a child frame to manipulate or use it to change the scene.""")
        self.child_select_combo = QComboBox()
        self.child_select_combo.setMinimumWidth(360)
        self.child_select_combo.setToolTip("""Select a child frame to manipulate or use it to change the scene.""")
        

        combo_2_box_label = QLabel("parent")
        combo_2_box_label.setToolTip("""Select a parent frame in which the selected child frame will be looked up, reparented, etc.""")
        self.parent_select_combo = QComboBox()
        self.parent_select_combo.setMinimumWidth(360)
        self.parent_select_combo.setToolTip("""Select a parent frame in which the selected child frame will be looked up, reparented, etc.""")

        self.refresh_button = QPushButton("refresh")
        self.refresh_button.setToolTip("""Refresh the lists to get all the latest frames in the scene.""")

        self.persist = QCheckBox("persist")
        self.persist.setChecked(False)
        self.persist.stateChanged.connect(lambda: persist_box_state_change(self.persist))
        self.persist.setToolTip("""Checking "persist" enables you to permanently save changes made while the box is checked. 
If "persist" is not checked, the changes will only affect the current session.""")

        info = QLabel("")
        info.setMaximumWidth(25)
        info_symbol = getattr(QStyle, "SP_MessageBoxInformation")
        info_icon = self.style().standardIcon(info_symbol)
        info_pixmap = info_icon.pixmap(QSize(24, 24))
        info.setPixmap(info_pixmap)
        info.setToolTip("""Hover over objects to reveal information.""")

        combo_box_layout.addWidget(combo_1_box_label, 0, 0)
        combo_box_layout.addWidget(self.child_select_combo, 0, 1)
        combo_box_layout.addWidget(self.refresh_button, 0, 2, 1, 2)
        combo_box_layout.addWidget(combo_2_box_label, 1, 0)
        combo_box_layout.addWidget(self.parent_select_combo, 1, 1)
        combo_box_layout.addWidget(self.persist, 1, 2)
        combo_box_layout.addWidget(info, 1, 3)
        combo_box.setLayout(combo_box_layout)

        def persist_box_state_change(checkbox):
            if checkbox.isChecked():
                Callbacks.persist_checked = True
            else:
                Callbacks.persist_checked = False

        def combo_1_box_button_clicked():
            Callbacks.trigger_refresh()
            self.child_select_combo.clear()
            self.child_select_combo.addItems(Callbacks.frames)
            self.parent_select_combo.clear()
            self.parent_select_combo.addItems(Callbacks.frames)

        self.refresh_button.clicked.connect(combo_1_box_button_clicked)

        return combo_box

    def make_actions_box(self):
        combo_box = QGroupBox("actions")
        combo_box_layout = QGridLayout()

        query_button = QPushButton("query")
        query_button.setToolTip("""Look up the current transformation from the selected parent frame to the selected child frame.""")
        remove_button = QPushButton("remove")
        remove_button.setToolTip("""Remove the selected child frame from the scene. Note the "persist" checkbox.""")
        reparent_button = QPushButton("reparent")
        reparent_button.setToolTip("""Change the parent of the selected child frame to the selected parent frame. 
The frame will remain in the same position in the world. Note the "persist" checkbox.""")
        teach_button = QPushButton("teach")
        teach_button.setToolTip("""Move the selected frame to the position od the teaching marker.""")
        rename_button = QPushButton("rename")
        rename_button.setToolTip("""Rename the selected frame with the new name from the line edit. Note the "persist" checkbox.""")
        clone_button = QPushButton("clone")
        clone_button.setToolTip("""Clone the selected frame with the new name provided in the line edit. 
The clone will spawn at the position where its original is, parented in the parent 
frame that is currently selected. Note the "persist" checkbox.""")
        new_id_line_edit = QLineEdit("")
        new_id_line_edit.setToolTip("""Put a new frame id here, or the name for the clone to be spawned.""")

        combo_box_layout.addWidget(query_button, 0, 0)
        combo_box_layout.addWidget(remove_button, 0, 1)
        combo_box_layout.addWidget(reparent_button, 0, 2)
        combo_box_layout.addWidget(teach_button, 0, 3)
        combo_box_layout.addWidget(new_id_line_edit, 1, 0, 1, 2)
        combo_box_layout.addWidget(rename_button, 1, 2)
        combo_box_layout.addWidget(clone_button, 1, 3)
        combo_box.setLayout(combo_box_layout)

        def query_button_clicked():
            Callbacks.child = self.child_select_combo.currentText()
            Callbacks.parent = self.parent_select_combo.currentText()
            Callbacks.trigger_query()
            self.output.append(Callbacks.information)

        query_button.clicked.connect(query_button_clicked)

        def remove_button_clicked():
            Callbacks.child = self.child_select_combo.currentText()
            Callbacks.trigger_manipulate_scene("remove")
            self.output.append(Callbacks.information)

        remove_button.clicked.connect(remove_button_clicked)

        def reparent_button_clicked():
            Callbacks.child = self.child_select_combo.currentText()
            Callbacks.parent = self.parent_select_combo.currentText()
            Callbacks.trigger_manipulate_scene("reparent")
            self.output.append(Callbacks.information)

        reparent_button.clicked.connect(reparent_button_clicked)

        def teach_button_clicked():
            Callbacks.child = self.child_select_combo.currentText()
            Callbacks.parent = self.parent_select_combo.currentText()
            Callbacks.trigger_manipulate_scene("teach")
            self.output.append(Callbacks.information)

        teach_button.clicked.connect(teach_button_clicked)

        def rename_button_clicked():
            Callbacks.child = self.child_select_combo.currentText()
            Callbacks.new_id = new_id_line_edit.text()
            Callbacks.trigger_manipulate_scene("rename")
            self.output.append(Callbacks.information)

        rename_button.clicked.connect(rename_button_clicked)

        def clone_button_clicked():
            Callbacks.child = self.child_select_combo.currentText()
            Callbacks.parent = self.parent_select_combo.currentText()
            Callbacks.new_id = new_id_line_edit.text()
            Callbacks.trigger_manipulate_scene("clone")
            self.output.append(Callbacks.information)

        clone_button.clicked.connect(clone_button_clicked)

        return combo_box

    def make_extras_box(self):
        combo_box = QGroupBox("extras")
        combo_box_layout = QGridLayout()

        enable_path_button = QPushButton("enable path")
        enable_path_button.setToolTip("""Enable the path from the selected parent to the 
selected child frame. Note the "persist" checkbox.""")
        disable_path_button = QPushButton("disable path")
        disable_path_button.setToolTip("""Disable the path from the selected parent to the 
selected child frame. Note the "persist" checkbox.""")

        set_zone_button = QPushButton("set zone")
        set_zone_button.setToolTip("""Set the "zone" of the selected frame using the line edit
on the left. Note the "persist" checkbox.""")
        
        reload_button = QPushButton("reload")
        reload_button.setToolTip("""Reload the scene from the scenario folder specified with its path in the line edit.""")
        zone_line_edit = QLineEdit(Callbacks.zone)
        zone_line_edit.setMaximumWidth(90)
        zone_line_edit.setToolTip("""Put a frame "zone" size here in meters.""")
        scenario_path_line_edit = QLineEdit(Callbacks.scenario_path)
        scenario_path_line_edit.setToolTip("""Specify the path of the scenario here.""")
        get_all_button = QPushButton("get all")
        get_all_button.setToolTip("""List all the frames that are currently in the scene.""")

        combo_box_layout.addWidget(zone_line_edit, 0, 0, 1, 1)
        combo_box_layout.addWidget(set_zone_button, 0, 1, 1, 1)
        combo_box_layout.addWidget(enable_path_button, 0, 2, 1, 2)
        combo_box_layout.addWidget(disable_path_button, 0, 4, 1, 2)
        combo_box_layout.addWidget(scenario_path_line_edit, 1, 0, 1, 4)
        combo_box_layout.addWidget(reload_button, 1, 4, 1, 1)
        combo_box_layout.addWidget(get_all_button, 1, 5, 1, 1)
        
        combo_box.setLayout(combo_box_layout)

        def set_zone_button_clicked():
            Callbacks.child = self.child_select_combo.currentText()
            Callbacks.zone = zone_line_edit.text()
            Callbacks.trigger_manipulate_extra_features("set_zone", Callbacks.zone)
            self.output.append(Callbacks.information)

        set_zone_button.clicked.connect(set_zone_button_clicked)

        def enable_path_button_clicked():
            Callbacks.child = self.child_select_combo.currentText()
            Callbacks.parent = self.parent_select_combo.currentText()
            Callbacks.trigger_manipulate_extra_features("enable_path", Callbacks.zone)
            self.output.append(Callbacks.information)

        enable_path_button.clicked.connect(enable_path_button_clicked)

        def disable_path_button_clicked():
            Callbacks.child = self.child_select_combo.currentText()
            Callbacks.parent = self.parent_select_combo.currentText()
            Callbacks.trigger_manipulate_extra_features("disable_path", Callbacks.zone)
            self.output.append(Callbacks.information)

        disable_path_button.clicked.connect(disable_path_button_clicked)

        def reload_button_button_clicked():
            Callbacks.scenario_path = scenario_path_line_edit.text()
            Callbacks.trigger_manipulate_extra_features("reload_scenario", Callbacks.zone)
            self.output.append(Callbacks.information)

        reload_button.clicked.connect(reload_button_button_clicked)

        def get_all_button_clicked():
            Callbacks.parent = self.parent_select_combo.currentText()
            Callbacks.trigger_get_all()
            self.output.append(Callbacks.information)

        get_all_button.clicked.connect(get_all_button_clicked)

        return combo_box

    def make_teaching_marker_box(self):
        combo_box = QGroupBox("marker")
        combo_box_layout = QGridLayout()

        marker_size_edit = QLineEdit(Callbacks.marker_size)
        marker_size_edit.setMaximumWidth(185)
        marker_size_edit.setToolTip("""Put the size of the teaching marker here in meters.""")

        set_size_button = QPushButton("resize")
        set_size_button.setToolTip("""Set the size of the teaching marker using the line edit.""")
        move_button = QPushButton("move")
        move_button.setToolTip("""Move the teaching marker to the position of the currently selected child frame.""")
        reset_button = QPushButton("reset")
        reset_button.setToolTip("""Reset the teaching marker to its original position and size.""")

        combo_box_layout.addWidget(marker_size_edit, 0, 0, 1, 1)
        combo_box_layout.addWidget(set_size_button, 0, 1, 1, 1)
        combo_box_layout.addWidget(move_button, 0, 2, 1, 1)
        combo_box_layout.addWidget(reset_button, 0, 3, 1, 1)
        
        combo_box.setLayout(combo_box_layout)

        def resize_marker_button_clicked():
            Callbacks.marker_size = marker_size_edit.text()
            Callbacks.trigger_manipulate_extra_features("resize_marker", Callbacks.marker_size)
            self.output.append(Callbacks.information)

        set_size_button.clicked.connect(resize_marker_button_clicked)

        def reset_marker_button_clicked():
            Callbacks.trigger_manipulate_extra_features("reset_marker", Callbacks.marker_size)
            self.output.append(Callbacks.information)

        reset_button.clicked.connect(reset_marker_button_clicked)

        def move_marker_button_clicked():
            Callbacks.child = self.child_select_combo.currentText()
            Callbacks.trigger_manipulate_extra_features("move_marker", Callbacks.marker_size)
            self.output.append(Callbacks.information)

        move_button.clicked.connect(move_marker_button_clicked)

        return combo_box

    def make_information_box(self):
        information_box = QGroupBox("info")
        information_box_layout = QGridLayout()
        self.output = QTextBrowser(information_box)
        self.output.setGeometry(QRect(10, 90, 600, 200))
        self.output.setObjectName("info")
        information_button = QPushButton("clear")
        information_button.setMaximumWidth(80)
        information_button.setToolTip("""Clear the information text box.""")
        information_box_layout.addWidget(self.output, 0, 0)
        information_box_layout.addWidget(
            information_button, 1, 0, alignment=Qt.AlignRight
        )
        information_box.setLayout(information_box_layout)

        def information_button_clicked():
            self.output.clear()

        information_button.clicked.connect(information_button_clicked)

        return information_box


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

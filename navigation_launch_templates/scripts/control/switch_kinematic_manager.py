#!/usr/bin/python3
#
# Copyright 2024 Fraunhofer Italia Research
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import io
import os
import subprocess
import sys
from typing import List, Tuple
import yaml

import rclpy
import numpy as np
from rclpy.node import Node, Publisher, Service
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import ParameterValue, Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from fhi_ros2.node_helpers import ClientsHandler
from fhi_msgs.srv import StringTrigger
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import (
    ListControllers,
    ListControllerTypes,
    SwitchController,
    ConfigureController,
    LoadController,
    UnloadController,
)
from controller_manager_msgs.msg import ControllerState
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    ReliabilityPolicy,
)
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import time


class ConfigurationException(BaseException):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


class SwitchKinematicManager(Node):
    clients_handler: ClientsHandler

    # automatic navigation
    config_folder: str = ""
    automatic_navigation: dict = {}
    kinematics: dict = {}

    # used to manage control frame tf
    root_frame: str = ""
    control_frame: str = ""
    control_frame_link: str
    control_frame_tf: TransformStamped = None
    broadcaster: StaticTransformBroadcaster
    control_frame_description_topic: str
    control_frame_description_publisher: Publisher

    # used on kinematic switch
    controller_manager: str
    controllers_namespace: str = ""
    control_tool: str
    control_tool_type: str
    initial_controller: str
    autostart: bool = True
    controllers_params_file: str
    last_active_controller: str = ""
    last_active_kinematic: str = ""

    switch_kinematic_service: Service
    list_kinematics_service: Service
    set_active_controller_service: Service

    list_controllers_timeout = 1.0
    deactivate_controller_timeout = 2.0
    parameters_timeout = 2.0
    activating: bool = True

    def __init__(self, name: str):
        Node.__init__(self, name)

    def configure(self):
        self.get_logger().info("Configuring...")
        # automatic_navigation
        self.config_folder = (
            self.declare_parameter("config_folder", rclpy.Parameter.Type.STRING)
            .get_parameter_value()
            .string_value
        )
        automatic_navigation_file = os.path.join(
            self.config_folder,
            "automatic_navigation.yaml",
        )
        try:
            with open(automatic_navigation_file, "r") as file:
                self.automatic_navigation = yaml.safe_load(file)["automatic_navigation"]
        except FileNotFoundError:
            self.get_logger().fatal(
                f"Automatic Navigation configuration file not found at path {automatic_navigation_file}"
            )
            raise ConfigurationException
        except KeyError:
            self.get_logger().fatal(
                f"Automatic Navigation configuration wrongly defined, path {automatic_navigation_file}"
            )
            raise ConfigurationException
        if not self._parse_automatic_navigation_file():
            self.get_logger().fatal(
                f"Failed to parse Automatic Navigation file, path {automatic_navigation_file}"
            )
            raise ConfigurationException

        # available and requested controllers
        self.controller_manager: str = (
            self.declare_parameter("controller_manager", rclpy.Parameter.Type.STRING)
            .get_parameter_value()
            .string_value
        )
        self.clients_handler = ClientsHandler(
            node=self,
            action_info={},
            service_info={
                self.controller_manager + "/get_parameters": GetParameters,
                self.controller_manager + "/set_parameters": SetParameters,
                self.controller_manager + "/list_controller_types": ListControllerTypes,
                self.controller_manager + "/list_controllers": ListControllers,
                self.controller_manager + "/unload_controller": UnloadController,
                self.controller_manager + "/load_controller": LoadController,
                self.controller_manager + "/configure_controller": ConfigureController,
                self.controller_manager + "/switch_controller": SwitchController,
            },
            spin=True,
        )
        if self.controller_manager.startswith("/"):
            self.controllers_namespace = self.controller_manager.split(
                "controller_manager"
            )[0]
        else:
            self.controllers_namespace = self.get_namespace() + "/"

        self.control_tool: str = (
            self.declare_parameter("controller_name", rclpy.Parameter.Type.STRING)
            .get_parameter_value()
            .string_value
        )
        try:
            self.control_tool_type = self.automatic_navigation["tools"]["control"][
                "tool"
            ]
        except KeyError:
            self.get_logger().fatal("Control Tool type not defined!")
            raise ConfigurationException
        while rclpy.ok():
            if self.check_controller(self.control_tool, self.control_tool_type):
                break

            self.get_logger().fatal(
                "Control Tool not defined!", throttle_duration_sec=10.0
            )
            time.sleep(5.0)

        self.initial_controller = (
            self.declare_parameter("initial_controller", self.control_tool)
            .get_parameter_value()
            .string_value
        )
        self.get_logger().info("Checking given paramaters...")
        self.last_active_controller = self.initial_controller
        if not self.initial_controller in [
            controller.name for controller in self.get_loaded_controllers()
        ]:
            self.get_logger().warn(
                f"Initial Controller {self.initial_controller} not loaded in controller manager {self.controller_manager}"
            )
        else:
            self.get_logger().info("Initial and Control Tool correctly defined!")
        self.clients_handler.spin = False

        self.get_logger().info("Creating services...")
        # switch service
        self.switch_kinematic_service = self.create_service(
            StringTrigger,
            "~/switch_kinematic",
            self.switch_kinematic_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.list_kinematics_service = self.create_service(
            Trigger,
            "~/list_kinematics",
            self.list_kinematics,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.actual_kinematic_service = self.create_service(
            Trigger,
            "~/get_active_kinematic",
            self.get_active_kinematic,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.set_active_controller_service = self.create_service(
            StringTrigger,
            "~/set_active_controller",
            self.set_active_controller,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.get_logger().info("Switch Kinematic Service ready!")

        # control frame manager
        self.root_frame = self.automatic_navigation["vehicle"]["root_frame"]
        self.control_frame = self.automatic_navigation["configurators_info"][
            "control_frame"
        ]
        self.control_frame_link = f"<robot><link name='{self.control_frame}'><visual><geometry><sphere radius='0.1'/></geometry><material name='green'><color rgba='0 1 0 1'/></material></visual></link></robot>"
        self.broadcaster = StaticTransformBroadcaster(self)
        self.control_frame_tf = TransformStamped()
        self.control_frame_tf.header.frame_id = self.root_frame
        self.control_frame_tf.child_frame_id = self.control_frame
        self.control_frame_description_topic = self.control_frame + "_description"
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=LivelinessPolicy.AUTOMATIC,
        )
        self.control_frame_description_publisher = self.create_publisher(
            String, self.control_frame_description_topic, qos
        )

        self.autostart = (
            self.declare_parameter("autostart", self.autostart)
            .get_parameter_value()
            .bool_value
        )
        if self.autostart:
            self.autostart_timer = self.create_timer(1.0, self.autostart_callback)
            self.get_logger().info("Automatic starting activated")

    def autostart_callback(self):
        self.get_logger().info(
            "Automatic starting with chosen automatic navigation kinematics"
        )
        request = StringTrigger.Request()
        request.data = self.automatic_navigation["vehicle"]["chosen_kinematic"]["name"]
        response = StringTrigger.Response()
        self.switch_kinematic_callback(request, response)
        if response.success:
            self.get_logger().info("Automatic starting done")
            self.destroy_timer(self.autostart_timer)
        else:
            self.get_logger().error("Automatic starting failed")

    def list_kinematics(self, request: Trigger.Request, response: Trigger.Response):
        response.success = True
        response.message = ", ".join(
            list(self.automatic_navigation["vehicle"]["possible_kinematics"].keys())
        )
        return response

    def set_active_controller(
        self, request: StringTrigger.Request, response: StringTrigger.Response
    ):
        if request.data in [
            controller.name
            for controller in self.get_loaded_controllers()
            if controller.state == "active"
        ]:
            self.last_active_controller = request.data
            response.success = True
            response.message = "Actual active controller changed"
        else:
            response.success = False
            response.message = f"Controller requested {request.data} is not active in controller manager {self.controller_manager}"

        return response

    def get_active_kinematic(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        response.success = True
        response.message = self.last_active_kinematic
        return response

    def switch_kinematic_callback(
        self, request: StringTrigger.Request, response: StringTrigger.Response
    ):
        response.success = False
        self.get_logger().info("Switch Kinematic received, switching controller")
        if request.data in self.kinematics.keys():
            if self.switch_kinematic(
                self.last_active_controller,
                self.control_tool,
                self.control_tool_type,
                request.data,
            ):
                self.get_logger().info(
                    f"Controller switched, kinematic {request.data} prepared"
                )
                try:
                    control_frame = self.automatic_navigation["vehicle"][
                        "possible_kinematics"
                    ][request.data]["control_frame"]
                    self.move_control_frame(
                        control_frame["x"], control_frame["y"], control_frame["theta"]
                    )
                    self.get_logger().info(f"Control Frame ready!")
                    response.success = True
                    response.message = "Switch completed!"
                except KeyError:
                    self.get_logger().error(f"Error on moving control frame!")
            else:
                self.get_logger().error(f"Switch Kinematic failed!")

        return response

    def move_control_frame(self, x: float, y: float, theta: float):
        if self.activating:
            self.control_frame_description_publisher.publish(
                String(data=self.control_frame_link)
            )
            self.activating = False

        self.control_frame_tf.header.stamp = self.get_clock().now().to_msg()
        self.control_frame_tf.transform.translation.x = x
        self.control_frame_tf.transform.translation.y = y
        self.control_frame_tf.transform.translation.z = 0.0

        quat = quaternion_from_euler(0.0, 0.0, theta)
        self.control_frame_tf.transform.rotation.x = quat[0]
        self.control_frame_tf.transform.rotation.y = quat[1]
        self.control_frame_tf.transform.rotation.z = quat[2]
        self.control_frame_tf.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(self.control_frame_tf)
        return

    def switch_kinematic(
        self, unload: str, load: str, load_type: str, kinematic_name: str = ""
    ) -> bool:
        # unloads last known controller
        if unload != "":
            if not self.unload_controller(unload):
                return False

        # loads or reloads desired controller
        if not self.load_controller(load, load_type):
            return False
        self.last_active_controller = self.control_tool
        # Updates Params of the new kinematics
        with open(self.controllers_params_file, "r") as file:
            try:
                all_params: dict = yaml.safe_load(file)
            except yaml.YAMLError as e:
                self.get_logger().error(f"Failed to read params from file {file}")
                return False
        if kinematic_name in all_params.keys():
            controller_params = {
                self.controllers_namespace + load: all_params[kinematic_name]
            }
            file_path = "/tmp/controller_params_list.yaml"
            with open(file_path, "w") as file:
                yaml.safe_dump(controller_params, file)
            process = subprocess.Popen(
                f"ros2 param load {self.controllers_namespace + load} {file_path}",
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            stdout, stderr = process.communicate()
            self.get_logger().info(
                f"Kinematic params {kinematic_name} of controller {load} loaded!"
            )
            self.last_active_kinematic = kinematic_name

        return self.activate_controller(load)

    def _parse_automatic_navigation_file(self):
        try:
            self.controllers_params_file = os.path.join(
                self.config_folder,
                self.automatic_navigation["tools"]["control"]["config_file"],
            )
            self.kinematics: dict = self.automatic_navigation["vehicle"][
                "possible_kinematics"
            ]
        except KeyError:
            self.get_logger().error("Missing keywords!")
            return False
        if len(self.kinematics) == 0:
            self.get_logger().error("No possible kinematics defined!")
            return False

        return True

    def check_controller(self, controller: str, type: str) -> bool:
        request = GetParameters.Request()
        request.names = [controller + ".type"]
        response: GetParameters.Response = self.clients_handler.call_sync(
            request,
            self.controller_manager + "/get_parameters",
            self.parameters_timeout,
        )
        if response:
            values: List[ParameterValue] = response.values
            if values[0].string_value == type:
                return True
            else:
                self.get_logger().warn(f"Controller Manager has not {controller}")
                if type in self.get_available_controller_types():
                    self.get_logger().info(
                        f"Controller manager has type {type} available"
                    )
                    request = SetParameters.Request()
                    request.parameters = [
                        Parameter(
                            name=str(controller + ".type"),
                            value=ParameterValue(type=4, string_value=type),
                        )
                    ]
                    response: SetParameters.Response = self.clients_handler.call_sync(
                        request,
                        self.controller_manager + "/set_parameters",
                        self.parameters_timeout,
                    )
                    if response and response.results[0].successful:
                        self.get_logger().info(
                            f"Controller {controller} with type {type} added"
                        )
                        return True
        self.get_logger().error(
            f"Controller {controller} wrongly defined in Controller Manager"
        )
        return False

    def get_loaded_controllers(self) -> List[ControllerState]:
        response: ListControllers.Response = self.clients_handler.call_sync(
            ListControllers.Request(),
            self.controller_manager + "/list_controllers",
            timeout=self.list_controllers_timeout,
        )
        if not response:
            return []
        return response.controller

    def get_available_controller_types(self) -> List[str]:
        response: ListControllerTypes.Response = self.clients_handler.call_sync(
            ListControllerTypes.Request(),
            self.controller_manager + "/list_controller_types",
            timeout=self.list_controllers_timeout,
        )
        if not response:
            return []
        return response.types

    def unload_controller(self, controller) -> bool:
        loaded_controllers = self.get_loaded_controllers()
        for loaded_controller in loaded_controllers:
            if loaded_controller.name == controller:
                if loaded_controller.state == "active":
                    # DEACTIVATE old controller
                    request = SwitchController.Request()
                    request.deactivate_controllers = [controller]
                    request.strictness = 1
                    response: SwitchController.Response = (
                        self.clients_handler.call_sync(
                            request,
                            self.controller_manager + "/switch_controller",
                            timeout=self.deactivate_controller_timeout,
                        )
                    )
                    if not (response and response.ok):
                        self.get_logger().error(
                            f"Unable to deactivate controller {controller}!"
                        )
                        return False
                # UNLOAD old controller
                request = UnloadController.Request()
                request.name = controller
                response: UnloadController.Response = self.clients_handler.call_sync(
                    request, self.controller_manager + "/unload_controller", 1.0
                )
                if not (response and response.ok):
                    self.get_logger().error(f"Unable to unload controller {controller}")
                    return False
                self.get_logger().info(f"Controller {controller} unloaded!")
                return True
        self.get_logger().debug(f"Controller {controller} already unloaded!")
        return True

    def load_controller(self, controller, controller_type) -> bool:
        # checks that controller exists
        if not self.check_controller(controller, controller_type):
            return False

        if controller in [
            loaded_controller.name
            for loaded_controller in self.get_loaded_controllers()
        ]:
            self.get_logger().warn(
                f"Controller {controller} already loaded, unloading to clear params"
            )
            if not self.unload_controller(controller):
                return False
        # LOAD new controller
        request = LoadController.Request()
        request.name = controller
        response: LoadController.Response = self.clients_handler.call_sync(
            request, self.controller_manager + "/load_controller", 1.0
        )
        if not (response and response.ok):
            self.get_logger().error(f"Unable to load controller {controller}")
            return False
        self.get_logger().info(f"Controller {controller} loaded!")

        return True

    def activate_controller(self, controller) -> bool:
        # CONFIGURE new controller
        request = ConfigureController.Request()
        request.name = controller
        response: ConfigureController.Response = self.clients_handler.call_sync(
            request, self.controller_manager + "/configure_controller", 1.0
        )
        if not (response and response.ok):
            self.get_logger().error(f"Unable to configure controller {controller}")
            return False
        self.get_logger().info(f"Controller {controller} configured!")
        # ACTIVATE new controller
        request = SwitchController.Request()
        request.activate_controllers = [controller]
        request.activate_asap = True
        request.strictness = 1
        response: SwitchController.Response = self.clients_handler.call_sync(
            request, self.controller_manager + "/switch_controller", 1.0
        )
        if not (response and response.ok):
            self.get_logger().error(f"Unable to activate controller {controller}")
            return False
        if not controller in [
            loaded_controller.name
            for loaded_controller in self.get_loaded_controllers()
            if loaded_controller.state == "active"
        ]:
            self.get_logger().error("Controller not activated!")
            return False
        self.get_logger().info(f"Controller {controller} activated!")

        return True


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = SwitchKinematicManager("switch_kinematic_manager")
    try:
        node.configure()
    except ConfigurationException:
        node.get_logger().fatal("Closing ... ")
        return

    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.destroy_node()
        node.get_logger().info("Keyboard interrupt, shutting down.\n")

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()

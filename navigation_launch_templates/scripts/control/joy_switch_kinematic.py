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
import time
from typing import List, Tuple
import yaml

import rclpy
from rclpy.node import Node, Subscription
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Joy
from fhi_ros2.node_helpers import ClientsHandler
from fhi_msgs.srv import StringTrigger
from std_srvs.srv import Trigger
from std_msgs.msg import String


class ConfigurationException(BaseException):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)


class JoySwitchKinematic(Node):
    clients_handler: ClientsHandler
    joy_subscription: Subscription

    joy_topic: str = "joy"
    sleep_time: float = 2.0
    switch_kinematic_node_name: str = "/switch_kinematic_manager"

    kinematics: List[str] = []

    def __init__(self, name: str):
        Node.__init__(self, name)

    def configure(self):
        self.joy_topic = (
            self.declare_parameter("joy_topic", self.joy_topic)
            .get_parameter_value()
            .string_value
        )
        self.sleep_time = (
            self.declare_parameter("sleep_time", self.sleep_time)
            .get_parameter_value()
            .double_value
        )

        self.switch_kinematic_node_name = (
            self.declare_parameter(
                "switch_kinematic_manager_name", self.switch_kinematic_node_name
            )
            .get_parameter_value()
            .string_value
        )

        self.clients_handler = ClientsHandler(
            node=self,
            action_info={},
            service_info={
                self.switch_kinematic_node_name + "/get_active_kinematic": Trigger,
                self.switch_kinematic_node_name + "/list_kinematics": Trigger,
                self.switch_kinematic_node_name + "/switch_kinematic": StringTrigger,
            },
            spin=True,
        )
        while rclpy.ok():
            response: Trigger.Response = self.clients_handler.call_sync(
                Trigger.Request(),
                self.switch_kinematic_node_name + "/list_kinematics",
            )
            if response:
                break
            self.get_logger().error(
                f"Switch Kinematic Node '{self.switch_kinematic_node_name}' failed to list kinematics"
            )
            time.sleep(5.0)

        kinematics = response.message.split(", ")
        self.kinematics = []
        for element in reversed(kinematics):
            self.kinematics += [element]
        if len(self.kinematics) == 0:
            self.get_logger().fatal(
                f"No kinematics defined in switch kinematic manager {self.switch_kinematic_node_name}"
            )
            raise ConfigurationException
        self.clients_handler.spin = False

        self.joy_subscription = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_callback,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.get_logger().info(f"Joy Switch Kinematic ready!")

    def joy_callback(self, msg: Joy):
        if msg.axes[7] == 0:
            return
        self.get_logger().info(f"Received a Switch Kinematic request! Switching ...")
        response: Trigger.Response = self.clients_handler.call_sync(
            Trigger.Request(),
            self.switch_kinematic_node_name + "/get_active_kinematic",
            1.0,
        )
        if not response:
            self.get_logger().error(
                f"Switch Kinematic {self.switch_kinematic_node_name} failed to return active kinematic"
            )
            return
        if not response.message in self.kinematics:
            self.get_logger().warn(
                f"Given Kinematic {response.message} is not in the kinematics list, last will be set"
            )
            index = 0
        else:
            index = self.kinematics.index(response.message) + int(msg.axes[7])
            if index >= len(self.kinematics):
                index = 0
            elif index < 0:
                index = len(self.kinematics) - 1
        request = StringTrigger.Request()
        request.data = self.kinematics[index]
        response = self.clients_handler.call_sync(
            request, self.switch_kinematic_node_name + "/switch_kinematic", timeout=15.0
        )
        if not (response and response.success):
            self.get_logger().error(
                f"Switch Kinematic {self.switch_kinematic_node_name} failed to switch kinematic"
            )
            return
        self.get_logger().info(
            f"Switch Kinematic completed! Actual kinematic is {request.data}"
        )
        time.sleep(self.sleep_time)
        return


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = JoySwitchKinematic("joy_switch_kinematic")
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

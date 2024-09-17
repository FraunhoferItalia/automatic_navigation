#!/usr/bin/python3
#
# Copyright 2022-2024 Fraunhofer Italia Research
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
import sys

import rclpy
from rclpy.node import Node, Subscription, Publisher
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from navigation_deployment.study_robot import (
    study_robot_core,
)


class ConfigurationException(BaseException):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)


class AutomaticNavigation(Node):
    robot_description_topic: str = "/robot_description"
    config_folder: str = "/tmp/study_robot_config/"
    settings: str = ""
    urdf_file: str = "/tmp/robot_description.xacro"

    robot_description_subscription: Subscription = None

    def __init__(self, name: str):
        Node.__init__(self, name)

        self.robot_description_topic = (
            self.declare_parameter(
                "robot_description_topic", self.robot_description_topic
            )
            .get_parameter_value()
            .string_value
        )
        self.config_folder = (
            self.declare_parameter("config_folder", self.config_folder)
            .get_parameter_value()
            .string_value
        )
        self.settings = (
            self.declare_parameter("settings", self.settings)
            .get_parameter_value()
            .string_value
        )

        if not os.path.isdir(self.config_folder):
            self.get_logger().error(
                "Config folder wrongly defined, default will be used. Path: "
                + self.config_folder
                + ".\n"
            )
            raise ConfigurationException()

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        try:
            self.robot_description_subscription = self.create_subscription(
                String,
                self.robot_description_topic,
                self.study_robot_callback,
                qos,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
        except rclpy.exceptions.InvalidTopicNameException:
            self.get_logger().fatal("Robot description topic name wrongly defined.\n")
            raise ConfigurationException()

    def study_robot_callback(self, msg: String):
        self.get_logger().info("Robot description received! Starting study...")
        with open(self.urdf_file, "w") as file:
            file.write(msg.data)

        output_buffer = io.StringIO()
        sys.stdout = output_buffer
        study_robot_core(
            self.urdf_file, self.config_folder, self.settings, ignore_wheels_radius=True
        )
        self.get_logger().info(output_buffer.getvalue())
        sys.stdout = sys.__stdout__
        raise ConfigurationException()


def main(args=None):
    rclpy.init(args=args)

    executor = SingleThreadedExecutor()
    try:
        node = AutomaticNavigation("automatic_navigation")
    except ConfigurationException:
        rclpy.shutdown()
        return

    executor.add_node(node)
    try:
        node.get_logger().info(
            "Waiting for robot description on topic: " + node.robot_description_topic
        )
        executor.spin()
    except KeyboardInterrupt:
        node.destroy_node()
        node.get_logger().info("Keyboard interrupt, shutting down.\n")
    except ConfigurationException:
        node.destroy_node()
        node.get_logger().info("Automatic Navigation Completed! \n")

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()

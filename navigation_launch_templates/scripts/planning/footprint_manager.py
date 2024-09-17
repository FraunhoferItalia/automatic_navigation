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
from typing import List
import rclpy
from rclpy.node import Node, Subscription, Publisher
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    ReliabilityPolicy,
)
import ast
from fhi_ros2.node_helpers.tf_manager import Transformation
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from builtin_interfaces.msg import Time
from tf2_ros import TransformException
import numpy as np


def parse_footprint_string(footprint: str) -> Polygon:
    footprint_list = ast.literal_eval(footprint)
    if isinstance(footprint_list, list):
        try:
            msg = Polygon()
            for point in footprint_list:
                point_msg = Point32()
                point_msg.x = point[0]
                point_msg.y = point[1]
                point_msg.z = 0.0
                msg.points.append(point_msg)
            return msg
        except IndexError:
            pass
    return None


class ConfigurationException(BaseException):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)


class SkipException(BaseException):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)


class FootprintManager(Node):
    footprint_publishers: List[Publisher] = []
    footprint_stamped_publishers: List[Publisher] = []
    footprint_subscription: Subscription

    footprint_subscription_topic: str = ""
    footprint_publishers_topics: List[str] = []
    footprint_stamped_publishers_topics: List[str] = []

    tf_buffer: Buffer
    tf_listener: TransformListener

    last_footprint: PolygonStamped = None
    publish_frame: str

    def __init__(self, name: str):
        Node.__init__(self, name)

    def configure(self):
        self.footprint_subscription_topic = (
            self.declare_parameter("footprint_subscription_topic", "")
            .get_parameter_value()
            .string_value
        )
        self.footprint_publishers_topics = (
            self.declare_parameter("footprint_publishers_topics", [""])
            .get_parameter_value()
            .string_array_value
        )
        self.footprint_stamped_publishers_topics = (
            self.declare_parameter("footprint_stamped_publishers_topics", [""])
            .get_parameter_value()
            .string_array_value
        )
        self.publish_frame = (
            self.declare_parameter("publish_frame", "")
            .get_parameter_value()
            .string_value
        )
        first_value = (
            self.declare_parameter("first_value", "").get_parameter_value().string_value
        )
        first_value_frame = (
            self.declare_parameter("first_value_frame", "")
            .get_parameter_value()
            .string_value
        )
        frequency = (
            self.declare_parameter("frequency", 1.0).get_parameter_value().double_value
        )

        if self.footprint_subscription_topic == "":
            self.get_logger().error("Footprint Manager subscription topic is empty!")
            raise ConfigurationException

        self.get_logger().info(
            f"Creating subscription to topic {self.footprint_subscription_topic}"
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
        )

        self.footprint_subscription = self.create_subscription(
            PolygonStamped,
            self.footprint_subscription_topic,
            self.footprint_subscription_callback,
            qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        if self.footprint_publishers_topics == [
            ""
        ] and self.footprint_stamped_publishers_topics == [""]:
            self.get_logger().error(f"No footprint publishers topics defined.")
            raise ConfigurationException()

        for footprint_topic in self.footprint_publishers_topics:
            self.get_logger().info(f"Creating publisher at topic {footprint_topic}")
            self.footprint_publishers += [
                self.create_publisher(
                    Polygon,
                    footprint_topic,
                    qos,
                    callback_group=MutuallyExclusiveCallbackGroup(),
                )
            ]
        for footprint_topic in self.footprint_stamped_publishers_topics:
            self.get_logger().info(f"Creating publisher at topic {footprint_topic}")
            self.footprint_stamped_publishers += [
                self.create_publisher(
                    PolygonStamped,
                    footprint_topic,
                    qos,
                    callback_group=MutuallyExclusiveCallbackGroup(),
                )
            ]

        if first_value != "":
            polygon = parse_footprint_string(first_value)
            if not polygon is None:
                self.last_footprint = PolygonStamped()
                self.last_footprint.polygon = polygon
                self.last_footprint.header.frame_id = first_value_frame
            else:
                self.get_logger().error("First footprint value wrongly defined!")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(frequency, self.update)
        self.get_logger().info(f"Configured!")

    def update(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.publish_frame, self.last_footprint.header.frame_id, Time()
            )
            transformation: Transformation = Transformation.from_transformation_msg(
                tf.transform
            )
        except TransformException as ex:
            self.get_logger().error(
                f"Could not transform '{self.last_footprint.header.frame_id}' to frame '{self.publish_frame}': {ex}",
                throttle_duration_sec=2.0,
            )
            return
        polygon = PolygonStamped()
        polygon.header.stamp = self.get_clock().now().to_msg()
        polygon.header.frame_id = self.publish_frame
        for point in self.last_footprint.polygon.points:
            new_point = Point32()
            np_point = transformation.matrix.dot(
                np.array([point.x, point.y, point.z, 1])
            )
            new_point.x = np_point[0]
            new_point.y = np_point[1]
            new_point.z = np_point[2]
            polygon.polygon.points.append(new_point)

        # Publishing
        self.get_logger().info("Publishing ...", throttle_duration_sec=10.0)
        for publisher in self.footprint_publishers:
            publisher.publish(polygon.polygon)
        for publisher in self.footprint_stamped_publishers:
            publisher.publish(polygon)

        return

    def footprint_subscription_callback(self, msg: PolygonStamped):
        self.last_footprint = msg
        return


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = FootprintManager("footprint_manager")
    try:
        node.configure()
    except ConfigurationException:
        rclpy.shutdown()
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

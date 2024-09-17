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
import copy
import rclpy
from rclpy.node import Node, Subscription, Publisher
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist, TwistStamped


class ConfigurationException(BaseException):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)


class CmdVelConverter(Node):
    cmd_vel_publisher: Publisher
    cmd_vel_subscription: Subscription

    cmd_vel_subscription_topic: str = ""
    cmd_vel_publisher_topic: str = ""

    def __init__(self, name: str):
        Node.__init__(self, name)

    def configure(self):
        self.cmd_vel_subscription_topic = (
            self.declare_parameter("cmd_vel_input", "")
            .get_parameter_value()
            .string_value
        )
        self.cmd_vel_publisher_topic = (
            self.declare_parameter("cmd_vel_output", "")
            .get_parameter_value()
            .string_value
        )

        self.limit = (
            self.declare_parameter("limit", False).get_parameter_value().bool_value
        )
        self.x_limit = abs(
            self.declare_parameter("x_limit", 0.0).get_parameter_value().double_value
        )
        self.y_limit = abs(
            self.declare_parameter("y_limit", 0.0).get_parameter_value().double_value
        )
        self.theta_limit = abs(
            self.declare_parameter("theta_limit", 0.0)
            .get_parameter_value()
            .double_value
        )
        self.to_stamped = (
            self.declare_parameter("to_stamped", True).get_parameter_value().bool_value
        )

        if self.limit and all(
            [self.x_limit == 0.0, self.y_limit == 0.0, self.theta_limit == 0.0]
        ):
            self.get_logger().warn("All limits set to 0, velocity will not be limited!")

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            self.cmd_vel_subscription_topic,
            self.cmd_vel_callback,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        if not self.to_stamped:
            self.cmd_vel_publisher = self.create_publisher(
                Twist,
                self.cmd_vel_publisher_topic,
                1,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
        else:
            self.cmd_vel_publisher_stamped = self.create_publisher(
                TwistStamped,
                self.cmd_vel_publisher_topic,
                1,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
        self.get_logger().info(f"Cmd vel converter ready!")

    def cmd_vel_callback(self, msg: Twist):
        msg2 = copy.deepcopy(msg)
        self.get_logger().info(
            f"Converting input {self.cmd_vel_subscription_topic} to {self.cmd_vel_publisher_topic} ...",
            throttle_duration_sec=10.0,
        )
        if self.limit:
            if self.x_limit != 0.0:
                msg2.linear.x = min(msg.linear.x, self.x_limit)
                msg2.linear.x = max(msg2.linear.x, -self.x_limit)

            if self.y_limit != 0.0:
                msg2.linear.y = min(msg.linear.y, self.y_limit)
                msg2.linear.y = max(msg2.linear.y, -self.y_limit)

            if self.theta_limit != 0.0:
                msg2.angular.z = min(msg.angular.z, self.theta_limit)
                msg2.angular.z = max(msg2.angular.z, -self.theta_limit)

            if any(
                [
                    msg2.linear.y != msg.linear.y,
                    msg2.linear.x != msg.linear.x,
                    msg2.angular.z != msg.angular.z,
                ]
            ):
                self.get_logger().warn("Cmd Vel Limit Exceeded")

        if not self.to_stamped:
            self.cmd_vel_publisher.publish(msg2)
        else:
            msg3 = TwistStamped()
            msg3.twist = msg2
            msg3.header.stamp = self.get_clock().now().to_msg()
            self.cmd_vel_publisher_stamped.publish(msg3)

        return


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = CmdVelConverter("cmd_vel_converter")
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

    rclpy.shutdown()


if __name__ == "__main__":
    main()

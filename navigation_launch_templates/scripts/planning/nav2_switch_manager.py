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
import time
from typing import Any, Dict
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle


class ConfigurationException(BaseException):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)


class Nav2SwitchManager(Node):
    tf_following: Dict[str, Any] = {}

    def __init__(self, name: str):
        Node.__init__(self, name)

    def configure(self):
        self._configure_tf_following()

        self.get_logger().info(f"Configured!")

    def _configure_tf_following(self):
        self.tf_following = {}
        self.tf_following["tf_frame"] = (
            self.declare_parameter("tf_following.tf_frame", "nav2_tf_following_frame")
            .get_parameter_value()
            .string_value
        )
        self.tf_following["behavior_tree_path"] = (
            self.declare_parameter("tf_following.behavior_tree_path", "")
            .get_parameter_value()
            .string_value
        )
        if self.tf_following["behavior_tree_path"] == "":
            self.get_logger().info(
                f"{self.get_name()} param 'behavior_tree_path' not configured. Closing..."
            )
            raise ConfigurationException()
        self.tf_following["nav_to_action_topic"] = (
            self.declare_parameter(
                "tf_following.nav_to_action_topic", "navigate_to_pose"
            )
            .get_parameter_value()
            .string_value
        )
        self.tf_following["action_client"] = ActionClient(
            self,
            NavigateToPose,
            self.tf_following["nav_to_action_topic"],
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.tf_following["enable_service"] = self.create_service(
            Trigger,
            "~/tf_following/enable",
            self._tf_following_enable_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.tf_following["disable_service"] = self.create_service(
            Trigger,
            "~/tf_following/disable",
            self._tf_following_disable_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.tf_following["future"] = None
        self.tf_following["goal_handle"] = None

        return

    def _tf_following_enable_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        if not self.tf_following["future"] is None:
            response.message = "Tf following already going"
            response.success = True
            return response
        self.get_logger().info("Enabling Tf following")
        if not self.tf_following["action_client"].wait_for_server(5.0):
            self.get_logger().error(
                f"{self.tf_following['nav_to_action_topic']} action not ready."
            )
            response.message = f"Action server at topic {self.tf_following['nav_to_action_topic']} not available"
            response.success = False
            return response

        goal = NavigateToPose.Goal()
        goal.behavior_tree = self.tf_following["behavior_tree_path"]
        goal.pose.header.frame_id = self.tf_following["tf_frame"]
        future: Future = self.tf_following["action_client"].send_goal_async(
            goal, feedback_callback=self._tf_following_feeback
        )
        while rclpy.ok() and not future.done():
            self.get_logger().info(
                f"Starting tf following...", throttle_duration_sec=5.0
            )
            if future.cancelled():
                response.message = "Failed to start tf following action"
                response.success = False
                return response
            time.sleep(2)
        goal_handle: ClientGoalHandle = future.result()
        future: Future = goal_handle.get_result_async()
        future.add_done_callback(self._tf_following_done_callback)
        self.tf_following["future"] = future
        self.tf_following["goal_handle"] = goal_handle
        response.success = True
        response.message = "Tf following enabled"
        return response

    def _tf_following_disable_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        self.get_logger().info("Disabling Tf following")
        response.success = True
        response.message = "No tf following going"
        if (
            self.tf_following["future"] is None
            and self.tf_following["goal_handle"] is None
        ):
            return response

        if not self.tf_following["goal_handle"] is None:
            self.tf_following["goal_handle"].cancel_goal()
            self.tf_following["goal_handle"] = None
            response.message = "Tf following cancelled"
        return response

    def _tf_following_feeback(self, msg: NavigateToPose.Feedback):
        self.get_logger().info("Tf following going...", throttle_duration_sec=10.0)
        return

    def _tf_following_done_callback(self, future: Future):
        if future.cancelled():
            self.get_logger().error("Tf following cancelled.")
        else:
            self.get_logger().info("Tf following activated")
        self.tf_following["future"] = None
        return


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = Nav2SwitchManager("nav2_switch_manager")
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

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
import time
from typing import List
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node, Subscription, Publisher, SetParametersResult, Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from builtin_interfaces.msg import Time
from fhi_ros2.node_helpers.tf_manager import Transformation
from tf2_ros import TransformException
from rclpy.action.server import ServerGoalHandle, CancelResponse
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    ReliabilityPolicy,
)


class ConfigurationException(BaseException):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)


# converts the action pose command with respect to the another reference frame
class Nav2PlanTransformer(Node):
    plan_client: ActionClient
    plan_server: ActionServer

    plan_client_topic: str = ""
    plan_server_topic: str = ""

    plan_client: ActionClient
    plan_server: ActionServer

    client_goal_handle: ClientGoalHandle
    server_goal_handle: ServerGoalHandle

    def __init__(self, name: str):
        Node.__init__(self, name)

    def configure(self):
        self.plan_client_topic = (
            self.declare_parameter("plan_client_topic", "")
            .get_parameter_value()
            .string_value
        )
        self.plan_server_topic = (
            self.declare_parameter("plan_server_topic", "")
            .get_parameter_value()
            .string_value
        )
        self.goal_pose_subscription_topic = (
            self.declare_parameter("goal_pose_subscription_topic", "")
            .get_parameter_value()
            .string_value
        )
        self.goal_pose_publisher_topic = (
            self.declare_parameter("goal_pose_publisher_topic", "goal_pose")
            .get_parameter_value()
            .string_value
        )
        self.input_reference_frame = (
            self.declare_parameter("input_reference_frame", "base_link")
            .get_parameter_value()
            .string_value
        )
        self.output_reference_frame = (
            self.declare_parameter("output_reference_frame", "control_frame")
            .get_parameter_value()
            .string_value
        )

        self.path_publisher_topic = (
            self.declare_parameter("path_publisher_topic", "plan_frame")
            .get_parameter_value()
            .string_value
        )
        self.path_subscription_topic = (
            self.declare_parameter("path_subscription_topic", "plan")
            .get_parameter_value()
            .string_value
        )

        self.add_on_set_parameters_callback(self.change_parameters_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        client_created = False
        while rclpy.ok() and not client_created:
            try:
                self.plan_client = ActionClient(
                    self,
                    NavigateToPose,
                    self.plan_client_topic,
                    callback_group=MutuallyExclusiveCallbackGroup(),
                )
                client_created = True
            except ValueError:
                self.get_logger().warn(
                    f"Waiting for action server at topic {self.plan_client_topic}",
                    throttle_duration_sec=5.0,
                )
                pass

        self.plan_server = ActionServer(
            self,
            NavigateToPose,
            self.plan_server_topic,
            self.compute_goal,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
        )
        self.path_publisher = self.create_publisher(
            Path,
            self.path_publisher_topic,
            qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.path_subscription = self.create_subscription(
            Path,
            self.path_subscription_topic,
            self.compute_path_callback,
            qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.goal_pose_subscription = self.create_subscription(
            PoseStamped,
            self.goal_pose_subscription_topic,
            self.compute_goal_pose,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped,
            self.goal_pose_publisher_topic,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.client_goal_handle = None
        self.server_goal_handle = None

    def cancel_callback(self, cancel_request):
        if not self.client_goal_handle is None:
            self.client_goal_handle.cancel_goal()
            self.client_goal_handle = None
        return CancelResponse.ACCEPT

    def compute_goal(self, goal_handle: ServerGoalHandle) -> NavigateToPose.Result:
        self.server_goal_handle = goal_handle
        goal: NavigateToPose.Goal = goal_handle.request
        try:
            transformation = Transformation.from_pose_msg(goal.pose.pose)
        except ValueError:
            self.get_logger().error("Goal wrongly defined")
            goal_handle.abort()
            return
        xyzrpy = transformation.to_xyzrpy()
        self.get_logger().info(
            f"Received goal towards of frame {self.input_reference_frame} to position ({xyzrpy[0]}, {xyzrpy[1]}, {xyzrpy[5]})"
        )
        try:
            tf = self.tf_buffer.lookup_transform(
                self.input_reference_frame, self.output_reference_frame, Time()
            )
            transformation = transformation * Transformation.from_transformation_msg(
                tf.transform
            )
        except (TransformException, ValueError) as ex:
            self.get_logger().error(
                f"Failed to transform PLAN from {self.input_reference_frame} to {self.output_reference_frame}"
            )
            goal_handle.abort()
            return NavigateToPose.Result()

        transformed_goal = copy.deepcopy(goal)
        transformed_goal.pose.pose = transformation.to_pose_msg()

        if not self.plan_client.wait_for_server(5.0):
            self.get_logger().error("Navigation action not ready.")
            goal_handle.abort()
            return NavigateToPose.Result()

        future: Future = self.plan_client.send_goal_async(
            transformed_goal, feedback_callback=self.publish_feedback
        )
        while rclpy.ok() and not future.done():
            self.get_logger().info(f"Waiting for goal to be accepted by the server ...")
            if future.cancelled():
                goal_handle.canceled()
            elif goal_handle.is_cancel_requested or not goal_handle.is_active:
                future.cancel()
        self.client_goal_handle: ClientGoalHandle = future.result()
        future: Future = self.client_goal_handle.get_result_async()
        while rclpy.ok() and not future.done():
            self.get_logger().info(
                f"Navigating towards goal with frame {self.input_reference_frame}",
                throttle_duration_sec=2.0,
            )
            if future.cancelled():
                goal_handle.canceled()
            elif goal_handle.is_cancel_requested or not goal_handle.is_active:
                self.client_goal_handle.cancel_goal()
                future.cancel()
        result = future.result()
        self.client_goal_handle = None
        self.server_goal_handle = None
        if not result is None:
            goal_handle.succeed()
            self.get_logger().info("Goal Reached!")
            return result.result
        else:
            goal_handle.abort()
            self.get_logger().info("Goal Failed!")
            return NavigateToPose.Result()

    def compute_goal_pose(self, msg: PoseStamped):
        self.goal_pose_publisher.publish(msg)
        return

    def change_parameters_callback(
        self, parameters: List[Parameter]
    ) -> SetParametersResult:
        result = SetParametersResult()
        result.successful = True
        for parameter in parameters:
            if parameter.name == "input_reference_frame":
                self.input_reference_frame = (
                    parameter.get_parameter_value().string_value
                )
            elif parameter.name == "output_reference_frame":
                self.output_reference_frame = (
                    parameter.get_parameter_value().string_value
                )
            else:
                result.successful = False

        return result

    def compute_path_callback(self, msg: Path):
        msg2 = copy.deepcopy(msg)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.output_reference_frame, self.input_reference_frame, Time()
            )
            transformation = Transformation.from_transformation_msg(tf.transform)
        except (TransformException, ValueError) as ex:
            self.get_logger().error(
                f"Failed to transform PATH from {self.output_reference_frame} to {self.input_reference_frame}"
            )
            return
        self.get_logger().info(
            f"Republishing path from {self.output_reference_frame} to {self.input_reference_frame}",
            throttle_duration_sec=2.0,
        )

        for i, pose in enumerate(msg2.poses):
            msg2.poses[i].pose = (
                transformation * Transformation.from_pose_msg(pose.pose)
            ).to_pose_msg()

        self.path_publisher.publish(msg2)

        return

    def publish_feedback(self, msg: NavigateToPose.Feedback):
        self.server_goal_handle.publish_feedback(msg.feedback)


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = Nav2PlanTransformer("nav2_plan_transformer_node")
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


if __name__ == "__main__":
    main()

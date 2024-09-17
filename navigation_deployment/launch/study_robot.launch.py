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
from __future__ import annotations
import os


from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch_ros.actions import Node

from fhi_ros2.launch import get_absolute_file_path


def launch_setup(context, *args, **kwargs):
    robot_description = LaunchConfiguration("robot_description").perform(context)
    output_folder = LaunchConfiguration("output_folder").perform(context)
    automatic_navigation_file = LaunchConfiguration(
        "automatic_navigation_file"
    ).perform(context)

    if os.path.isfile(robot_description):
        study_robot = ExecuteProcess(
            cmd=[
                "python3",
                get_absolute_file_path("navigation_deployment/scripts/study_robot.py"),
                f"{robot_description}",
                f"{output_folder}",
                f"{automatic_navigation_file}",
            ],
            name="study_robot",
            output="screen",
        )
    else:
        study_robot = Node(
            parameters=[
                {
                    "robot_description_topic": robot_description,
                    "config_folder": output_folder,
                    "settings": automatic_navigation_file,
                }
            ],
            package="navigation_deployment",
            executable="automatic_navigation_node.py",
            name="automatic_navigation_node",
            output="screen",
        )

    return [study_robot]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_description",
                default_value="",
                description="Allow to select the robot_description file or topic",
            ),
            DeclareLaunchArgument(
                "output_folder",
                default_value="",
                description="Allow to specify the output config folder",
            ),
            DeclareLaunchArgument(
                "automatic_navigation_file",
                default_value="",
                description="Allow to select the automatic navigation settings file",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

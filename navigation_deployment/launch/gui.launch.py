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
from typing import List
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition
from fhi_ros2.launch import get_launcher
from navigation_deployment.utils import parse_yaml


def launch_setup(context, *args, **kwargs):
    config_folder = LaunchConfiguration("config_folder").perform(context)
    sim = LaunchConfiguration("sim").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    consider_namespace = LaunchConfiguration("consider_namespace").perform(context)

    automatic_navigation_file = os.path.join(config_folder, "automatic_navigation.yaml")
    try:
        configurations = parse_yaml(automatic_navigation_file)["automatic_navigation"][
            "tools"
        ]
    except (FileNotFoundError, KeyError) as e:
        print("Navigation cannot start. ", e)
        return []

    # ----- VISUALIZATION PART ----- #
    gui = []
    if namespace != "" and consider_namespace == "true":
        gui += [PushRosNamespace(namespace)]
    try:
        gui_config_file = configurations["visualization"]["config_file"]
        if (
            sim == "true"
            and "config_file_sim" in configurations["visualization"].keys()
        ):
            gui_config_file = configurations["visualization"]["config_file_sim"]
        gui += [
            IncludeLaunchDescription(
                get_launcher(
                    "navigation_launch_templates",
                    "visualization/" + configurations["visualization"]["tool"],
                ),
                launch_arguments={
                    "simulation": sim,
                    "visualization_params": config_folder + "/" + gui_config_file,
                }.items(),
            )
        ]
    except KeyError:
        print("Gui configurations not passed")

    return gui


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="config_folder",
                description="Allow to select the navigation config folder",
            ),
            DeclareLaunchArgument(
                name="sim",
                default_value="true",
                choices=["true", "false"],
                description="Allow to select use_sim_time value",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Nodes and topics namespace",
            ),
            DeclareLaunchArgument(
                "consider_namespace",
                default_value="true",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

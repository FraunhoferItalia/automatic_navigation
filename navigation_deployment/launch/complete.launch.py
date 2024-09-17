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
    TimerAction,
)
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from fhi_ros2.launch import get_launcher
from navigation_deployment.utils import parse_yaml


def launch_setup(context, *args, **kwargs):
    automatic_navigation_file = LaunchConfiguration(
        "automatic_navigation_file"
    ).perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    simulation = LaunchConfiguration("simulation").perform(context)
    world = LaunchConfiguration("world").perform(context)
    map = LaunchConfiguration("map").perform(context)
    urdf = LaunchConfiguration("urdf").perform(context)

    try:
        configurations = parse_yaml(automatic_navigation_file)["automatic_navigation"][
            "tools"
        ]
    except (FileNotFoundError, KeyError) as e:
        print("Automatic Navigation cannot start. ", e)
        return []

    config_folder = os.path.dirname(os.path.abspath(automatic_navigation_file))

    launchers: List[IncludeLaunchDescription] = []
    # ----- SIMULATION PART ----- #
    # Launches the simulation if required
    if simulation == "true":
        launchers += [
            IncludeLaunchDescription(
                get_launcher(
                    "navigation_launch_templates",
                    "simulation/" + configurations["simulation"]["tool"],
                ),
                launch_arguments={
                    "world": world,
                    "gui": gui,
                }.items(),
                condition=IfCondition(world),
            )
        ]
    # ----- ROBOT PART ----- #
    # Spawns the robot and loads the state publishers
    control_config_file = configurations["control"]["config_file"]
    if simulation == "true" and "config_file_sim" in configurations["control"].keys():
        control_config_file = configurations["control"]["config_file_sim"]
    launchers += [
        IncludeLaunchDescription(
            get_launcher("navigation_launch_templates", "robot"),
            launch_arguments={
                "urdf": urdf,
                "simulation": simulation,
                "controllers": config_folder + control_config_file,
            }.items(),
            condition=IfCondition(urdf),
        )
    ]
    # Activates the controller
    launchers += [
        IncludeLaunchDescription(
            get_launcher("navigation_launch_templates", "control/activate_controller"),
            launch_arguments={
                "controller": configurations["control"]["controller"],
            }.items(),
        )
    ]

    # ----- VISUALIZATION PART ----- #
    if gui == "true":
        gui_config_file = configurations["visualization"]["config_file"]
        if (
            simulation == "true"
            and "config_file_sim" in configurations["visualization"].keys()
        ):
            gui_config_file = configurations["visualization"]["config_file_sim"]
        launchers += [
            IncludeLaunchDescription(
                get_launcher(
                    "navigation_launch_templates",
                    "visualization/" + configurations["visualization"]["tool"],
                ),
                launch_arguments={
                    "visualization_params": config_folder + "/" + gui_config_file,
                    "map": map,
                }.items(),
            )
        ]

    return [launchers]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="automatic_navigation_file",
                default_value="",
                description="Allow to select the automatic navigate configuration file",
            ),
            DeclareLaunchArgument(
                name="gui",
                default_value="true",
                description="Allow to select the automatic navigate configuration file",
            ),
            DeclareLaunchArgument(
                name="simulation",
                default_value="true",
                description="Allow to select the automatic navigate configuration file",
            ),
            DeclareLaunchArgument(
                name="urdf",
                default_value="",
                description="Allow spawn the robot",
            ),
            DeclareLaunchArgument(
                name="world",
                default_value="",
                description="Allow to select the automatic navigate configuration file",
            ),
            DeclareLaunchArgument(
                name="map",
                default_value="",
                description="Allow to select the automatic navigate configuration file",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

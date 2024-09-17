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
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from fhi_ros2.launch import get_launcher
from navigation_deployment.utils import parse_yaml


def launch_setup(context, *args, **kwargs):
    config_folder = LaunchConfiguration("config_folder").perform(context)
    map = LaunchConfiguration("map").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    sim = LaunchConfiguration("sim").perform(context)

    automatic_navigation_file = os.path.join(config_folder, "automatic_navigation.yaml")
    try:
        configurations = parse_yaml(automatic_navigation_file)["automatic_navigation"][
            "tools"
        ]
    except (FileNotFoundError, KeyError) as e:
        print("Navigation cannot start. ", e)
        return []

    launchers: List[IncludeLaunchDescription] = []

    # ----- FILTERING PART ----- #
    # Activates the filtering
    try:
        filtering_config_file = configurations["filtering"]["config_file"]
        if sim == "true" and "config_file_sim" in configurations["filtering"].keys():
            filtering_config_file = configurations["filtering"]["config_file_sim"]
        launchers += [
            IncludeLaunchDescription(
                get_launcher(
                    "navigation_launch_templates",
                    "filtering/" + configurations["filtering"]["tool"],
                ),
                launch_arguments={
                    "simulation": sim,
                    "filtering_params": config_folder + "/" + filtering_config_file,
                }.items(),
            )
        ]
    except KeyError:
        pass

    # ----- MAPPING PART ----- #
    # Activates the mapping
    mapping = False
    if map == "":
        try:
            mapping_config_file = configurations["mapping"]["config_file"]
            if sim == "true" and "config_file_sim" in configurations["mapping"].keys():
                mapping_config_file = configurations["mapping"]["config_file_sim"]
            launchers += [
                IncludeLaunchDescription(
                    get_launcher(
                        "navigation_launch_templates",
                        "mapping/" + configurations["mapping"]["tool"],
                    ),
                    launch_arguments={
                        "mapping_params": config_folder + "/" + mapping_config_file,
                    }.items(),
                )
            ]
            mapping = True
        except KeyError:
            pass

    if not mapping or map != "":
        # ----- LOCALIZATION PART ----- #
        # Activates the localization
        try:
            localization_config_file = configurations["localization"]["config_file"]
            if (
                sim == "true"
                and "config_file_sim" in configurations["localization"].keys()
            ):
                localization_config_file = configurations["localization"][
                    "config_file_sim"
                ]
            launchers += [
                IncludeLaunchDescription(
                    get_launcher(
                        "navigation_launch_templates",
                        "localization/" + configurations["localization"]["tool"],
                    ),
                    launch_arguments={
                        "simulation": sim,
                        "localization_params": config_folder
                        + "/"
                        + localization_config_file,
                    }.items(),
                )
            ]
        except KeyError:
            pass

        # ----- MAP SERVER PART ----- #
        # Activates the map server
        if map != "":
            launchers += [
                IncludeLaunchDescription(
                    get_launcher(
                        "navigation_launch_templates",
                        "utils/map_server",
                    ),
                    launch_arguments={
                        "simulation": sim,
                        "map_file": map,
                    }.items(),
                )
            ]

    # ----- PLANNING PART ----- #
    # Activates the planner
    try:
        planning_config_file = configurations["planning"]["config_file"]
        if sim == "true" and "config_file_sim" in configurations["planning"].keys():
            planning_config_file = configurations["planning"]["config_file_sim"]
        launchers += [
            IncludeLaunchDescription(
                get_launcher(
                    "navigation_launch_templates",
                    "planning/" + configurations["planning"]["tool"],
                ),
                launch_arguments={
                    "planning_params": config_folder + "/" + planning_config_file,
                    "map": map,
                }.items(),
            )
        ]
    except KeyError:
        pass

    # ----- VISUALIZATION PART ----- #
    if gui == "true":
        try:
            launchers += [
                IncludeLaunchDescription(
                    get_launcher(
                        "navigation_deployment",
                        "gui",
                    ),
                    launch_arguments={
                        "sim": sim,
                        "config_folder": config_folder,
                        "consider_namespace": "false",
                    }.items(),
                )
            ]
        except KeyError:
            print("Gui configurations not passed")

    return launchers


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
                name="map",
                default_value="",
                description="Allow to select the map file",
            ),
            DeclareLaunchArgument(
                name="gui",
                default_value="true",
                choices=["true", "false"],
                description="Enables gui launch",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

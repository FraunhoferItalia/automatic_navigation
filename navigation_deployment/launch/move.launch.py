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
from fhi_ros2.launch import get_launcher
from navigation_deployment.utils import parse_yaml


def launch_setup(context, *args, **kwargs):
    automatic_navigation_file = LaunchConfiguration(
        "automatic_navigation_file"
    ).perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    simulation = LaunchConfiguration("simulation").perform(context)
    world = LaunchConfiguration("world").perform(context)
    urdf = LaunchConfiguration("urdf").perform(context)

    try:
        configuration = parse_yaml(automatic_navigation_file)["automatic_navigation"]
        tools_conf = configuration["tools"]
        vehicle_conf = configuration["vehicle"]
    except (FileNotFoundError, KeyError):
        print(
            "Automatic Navigation cannot start. Automatic Navigation File wrongly defined  "
        )
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
                    "simulation/" + tools_conf["simulation"]["tool"],
                ),
                # condition=IfCondition(world),
                launch_arguments={
                    "world": world,
                    "gui": gui,
                }.items(),
            )
        ]
    # ----- ROBOT PART ----- #
    # Spawns the robot and loads the state publishers
    control_config_file = tools_conf["control"]["config_file"]
    if simulation == "true" and "config_file_sim" in tools_conf["control"].keys():
        control_config_file = tools_conf["control"]["config_file_sim"]
    launchers += [
        IncludeLaunchDescription(
            get_launcher("navigation_launch_templates", "robot"),
            launch_arguments={
                "urdf": urdf,
                "simulation": simulation,
                "controllers": config_folder + control_config_file,
            }.items(),
            # condition=IfCondition(urdf),
        )
    ]
    # Activates the control frame manager
    control_frame_parent = vehicle_conf["root_frame"]
    chosen_kinematic_name = vehicle_conf["chosen_kinematic"]["name"]
    try:
        control_frame_x = vehicle_conf["possible_kinematics"][chosen_kinematic_name][
            "control_frame"
        ]["x"]
        control_frame_y = vehicle_conf["possible_kinematics"][chosen_kinematic_name][
            "control_frame"
        ]["y"]
        control_frame_theta = vehicle_conf["possible_kinematics"][
            chosen_kinematic_name
        ]["control_frame"]["theta"]
    except KeyError:
        print("The chosen kinematic does not exist between the possible kinematics!")
        return []
    launchers += [
        IncludeLaunchDescription(
            get_launcher(
                "navigation_launch_templates", "control/control_frame_manager"
            ),
            launch_arguments={
                "parent_frame": str(control_frame_parent),
                "x": str(control_frame_x),
                "y": str(control_frame_y),
                "theta": str(control_frame_theta),
            }.items(),
        )
    ]
    possible_kinematics = [
        key for key, _ in vehicle_conf["possible_kinematics"].items()
    ]
    # Activates the controller
    launchers += [
        IncludeLaunchDescription(
            get_launcher(
                "navigation_launch_templates", "control/switch_controller_manager"
            ),
            launch_arguments={
                "possible_kinematics": possible_kinematics.__str__(),
                "activate_kinematics": chosen_kinematic_name,
            }.items(),
        )
    ]

    # ----- VISUALIZATION PART ----- #
    if gui == "true":
        gui_config_file = tools_conf["visualization"]["config_file"]
        if (
            simulation == "true"
            and "config_file_sim" in tools_conf["visualization"].keys()
        ):
            gui_config_file = tools_conf["visualization"]["config_file_sim"]
        launchers += [
            IncludeLaunchDescription(
                get_launcher(
                    "navigation_launch_templates",
                    "visualization/" + tools_conf["visualization"]["tool"],
                ),
                launch_arguments={
                    "visualization_params": config_folder + "/" + gui_config_file,
                }.items(),
            )
        ]

    return launchers


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
            OpaqueFunction(function=launch_setup),
        ]
    )

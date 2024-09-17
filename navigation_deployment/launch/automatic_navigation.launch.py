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
from typing import Any, Dict

from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition

from fhi_ros2.launch import get_launcher, get_absolute_file_path
import yaml
from navigation_deployment.utils import parse_yaml


def check_config_folder(folder: str):
    automatic_navigation_file = os.path.join(folder, "automatic_navigation.yaml")
    if os.path.isfile(automatic_navigation_file):
        with open(automatic_navigation_file, "r") as f:
            try:
                automatic_navigation = yaml.safe_load(f)["automatic_navigation"]
                # some kinematic exist
                kinematics = automatic_navigation["vehicle"]["possible_kinematics"]
                if isinstance(kinematics, dict) and kinematics:
                    good = True
                    for tool_name, info in automatic_navigation["tools"].items():
                        if not os.path.isfile(
                            os.path.join(folder, info["config_file"])
                        ):
                            print(
                                f"ERROR: Missing config file {info['config_file']} specified!"
                            )
                            good = False
                            continue
                        if tool_name != "control" and not os.path.isfile(
                            get_absolute_file_path(
                                f"navigation_launch_templates/launch/{tool_name}/{info['tool']}.launch.py",
                            )
                        ):
                            print(
                                f"ERROR: Missing launch for {tool_name} tool specified!"
                            )
                            good = False
                        from navigation_configurators import get_configurator

                        if get_configurator(tool_name, info["tool"]) is None:
                            print(
                                f"ERROR: Missing configurator for {tool_name} tool specified!"
                            )
                            good = False

                    return good
                else:
                    print("ERROR: No kinematics available!")
            except:
                print("ERROR: Tools wrongly defined!")
                pass

    return False


def launch_setup(context, *args, **kwargs):
    # ----------------------------------------------------------------------------------------------
    # STUDY PHASE
    robot_description_topic = LaunchConfiguration("robot_description_topic").perform(
        context
    )
    automatic_navigation_settings = LaunchConfiguration(
        "automatic_navigation_settings"
    ).perform(context)
    config_folder = LaunchConfiguration("config_folder").perform(context)
    sim = LaunchConfiguration("sim").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)

    use_sim_time = sim == "true"

    if automatic_navigation_settings == "":
        print(
            f"No robot description given, robot study skipped. Folder {config_folder} will be used."
        )
        if not check_config_folder(config_folder):
            print("Configuration folder given is wrongly defined. Closing ...")
            return []
    else:
        if not os.path.exists(config_folder):
            os.mkdir(config_folder)

    study = Node(
        parameters=[
            {
                "robot_description_topic": robot_description_topic,
                "config_folder": config_folder,
                "settings": automatic_navigation_settings,
                "use_sim_time": use_sim_time,
            }
        ],
        package="navigation_deployment",
        executable="automatic_navigation_node.py",
        name="automatic_navigation_node",
        output="screen",
        condition=IfCondition(
            "true" if automatic_navigation_settings != "" else "false"
        ),
        namespace=namespace,
    )

    # ----------------------------------------------------------------------------------------------
    # CONTROL SWITCHER

    controller_manager = LaunchConfiguration("controller_manager").perform(context)
    initial_controller = LaunchConfiguration("initial_controller").perform(context)
    autostart = LaunchConfiguration("autostart").perform(context)
    controller_name = LaunchConfiguration("controller_name").perform(context)
    joy_topic = LaunchConfiguration("joy_topic").perform(context)
    kinematic_switch_manager = Node(
        parameters=[
            {
                "controller_manager": controller_manager,
                "initial_controller": initial_controller,
                "autostart": autostart == "true",
                "controller_name": controller_name,
                "config_folder": config_folder,
                "use_sim_time": use_sim_time,
            }
        ],
        package="navigation_launch_templates",
        executable="switch_kinematic_manager.py",
        name="switch_kinematic_manager",
        output="screen",
    )
    joy_switch_kinematic = Node(
        parameters=[
            {
                "switch_kinematic_manager_name": "switch_kinematic_manager",
                "sleep_time": 2.0,
                "joy_topic": joy_topic,
                "use_sim_time": use_sim_time,
            }
        ],
        package="navigation_launch_templates",
        executable="joy_switch_kinematic.py",
        name="joy_switch_kinematic",
        output="screen",
        condition=IfCondition("true" if joy_topic != "" else "false"),
    )

    # ----------------------------------------------------------------------------------------------
    # NAVIGATION
    gui = LaunchConfiguration("gui").perform(context)
    map = LaunchConfiguration("map").perform(context)

    navigation = IncludeLaunchDescription(
        get_launcher(
            "navigation_deployment",
            "navigate",
        ),
        launch_arguments={
            "config_folder": config_folder,
            "map": map,
            "gui": gui,
            "sim": sim,
        }.items(),
    )

    # Study and Navigate
    if automatic_navigation_settings != "":
        return [
            study,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=study,
                    on_exit=[
                        PushRosNamespace(namespace),
                        kinematic_switch_manager,
                        TimerAction(
                            period=5.0,
                            actions=[
                                joy_switch_kinematic,
                                navigation,
                            ],
                        ),
                    ],
                ),
            ),
        ]
    # Only Navigate
    else:
        return [
            PushRosNamespace(namespace),
            kinematic_switch_manager,
            TimerAction(
                period=5.0,
                actions=[
                    joy_switch_kinematic,
                    navigation,
                ],
            ),
        ]


def generate_launch_description():
    return LaunchDescription(
        [
            # study phase
            DeclareLaunchArgument(
                "robot_description_topic",
                description="Allow to select the robot_description topic",
            ),
            DeclareLaunchArgument(
                "automatic_navigation_settings",
                default_value="",
                description="Allow to select the automatic navigation settings file",
            ),
            DeclareLaunchArgument(
                "config_folder",
                description="Allow to select the automatic navigation output config folder",
            ),
            DeclareLaunchArgument(
                "controller_manager",
                default_value="controller_manager",
                description="Allow to select the robot_description topic",
            ),
            DeclareLaunchArgument(
                "controller_name",
                default_value="",
                description="Specification of the controller name",
            ),
            DeclareLaunchArgument(
                "initial_controller",
                default_value="",
                description="Allow to specify the initial already active controller",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                choices=["true", "false"],
                description="Deactivate initial controller at start",
            ),
            DeclareLaunchArgument(
                "joy_topic",
                default_value="",
                description="Allow to select the joy topic to listen for kinematic switch",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                choices=["true", "false"],
                description="Allow to select the joy topic to listen for kinematic switch",
            ),
            DeclareLaunchArgument(
                "sim",
                default_value="true",
                choices=["true", "false"],
                description="Allow to select if in simulation or not",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Allow to select the map file",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Nodes and topics namespace",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

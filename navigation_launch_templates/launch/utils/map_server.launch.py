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
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from fhi_ros2.launch import get_launcher


def launch_setup(context, *args, **kwargs):
    simulation = LaunchConfiguration("simulation").perform(context)
    map = LaunchConfiguration("map_file").perform(context)

    use_sim_time = simulation == "true"

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="both",
        parameters=[
            {"yaml_filename": map},
            {"use_sim_time": use_sim_time},
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="map_server_lifecycle_manager",
        output="screen",
        parameters=[
            {"autostart": True},
            {"node_names": ["map_server"]},
            {"use_sim_time": use_sim_time},
        ],
    )

    return [map_server, lifecycle_manager]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "simulation",
                default_value="true",
                choices=["true", "false"],
                description="Allow to use the simulation",
            ),
            DeclareLaunchArgument(
                name="map_file",
                default_value="",
                description="Allow to select the nav2 localization configuration file",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

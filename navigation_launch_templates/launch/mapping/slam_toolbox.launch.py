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
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    simulation = LaunchConfiguration("simulation").perform(context)
    params = LaunchConfiguration("mapping_params").perform(context)

    use_sim_time = simulation == "true"

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox_node",
        output="screen",
        parameters=[
            params,
            {"use_sim_time": use_sim_time},
        ],
    )

    return [slam_toolbox]


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
                name="mapping_params",
                default_value="",
                description="Allow to select the nao nav2 configuration file",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
